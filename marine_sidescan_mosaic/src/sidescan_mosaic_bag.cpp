// Copyright 2026 Center for Coastal and Ocean Mapping & NOAA-UNH Joint
// Hydrographic Center, University of New Hampshire
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

/// @file
/// @brief Offline sidescan bag importer → Tier-1 (ADR-0006 D2/D3, issue #184).
///
/// Reads a bag **directly** (rosbag2, not replay) in a **single interleaved pass**
/// over `/tf` + `/tf_static` + the port/starboard `RawSonarImage` channels + the
/// nadir `Range`, all in chronological order. `/tf` feeds a **bounded-window**
/// `tf2::BufferCore`; each ping waits in a short FIFO until the TF frontier has
/// advanced a guard interval past its stamp (so both bracketing transforms are
/// present), then resolves the **baked `earth`→transducer pose**, decodes the
/// backscatter, and writes one Tier-1 record. No bottom model is applied — that
/// is Tier-2's job.
///
/// The bounded window replaces an earlier two-pass design that buffered the whole
/// bag's `/tf` history (a 10-hour cache) and did a per-ping `lookupTransform`
/// against it — an O(n) linear walk of tf2's `std::list`-backed `TimeCache` that
/// made real survey bags take ~17 min each (issue #251; the same fix as
/// cube_bathymetry#63).
///
/// NOTE (issue #184): the decode/stamp helpers are replicated from `mosaic_node`
/// for the prototype; the shared per-ping engine extraction (ADR-0006 D12) is a
/// follow-up after #177 (PR #181) merges, to avoid touching the node in parallel.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "marine_acoustic_msgs/msg/raw_sonar_image.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/storage_filter.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "tf2/buffer_core.h"
#include "tf2/time.h"
#include "tf2_msgs/msg/tf_message.hpp"

#include "marine_sidescan_mosaic/decode.hpp"
#include "marine_sidescan_mosaic/tier1.hpp"

namespace
{
constexpr std::int64_t kNsPerS = 1000000000LL;

std::int64_t stampNs(const builtin_interfaces::msg::Time & t)
{
  return static_cast<std::int64_t>(t.sec) * kNsPerS + t.nanosec;
}

template<typename MsgT>
MsgT deserialize(const rosbag2_storage::SerializedBagMessageSharedPtr & bag_msg)
{
  rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
  MsgT out;
  rclcpp::Serialization<MsgT>().deserialize_message(&serialized, &out);
  return out;
}

std::string argValue(int argc, char ** argv, const std::string & flag, const std::string & dflt)
{
  for (int i = 1; i + 1 < argc; ++i) {
    if (flag == argv[i]) {
      return argv[i + 1];
    }
  }
  return dflt;
}

// Parse a numeric arg, exiting with a usage error rather than std::terminate on
// bad/empty/out-of-range input.
double toDouble(const std::string & s, const std::string & flag)
{
  try {
    return std::stod(s);
  } catch (const std::exception &) {
    std::cerr << "error: expected a number for " << flag << ", got '" << s << "'\n";
    std::exit(2);
  }
}

int toInt(const std::string & s, const std::string & flag)
{
  try {
    return std::stoi(s);
  } catch (const std::exception &) {
    std::cerr << "error: expected an integer for " << flag << ", got '" << s << "'\n";
    std::exit(2);
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::cerr <<
      "usage: sidescan_mosaic_bag <bag_uri> <out.sst1>\n"
      "       [--port-topic T] [--stbd-topic T] [--nadir-topic T]\n"
      "       [--sound-speed S] [--nadir-staleness S] [--earth-frame F]\n"
      "       [--bins N]   # require sample0+samples_per_beam==N (decode-bug gate; 0=off)\n";
    return 2;
  }
  const std::string bag_uri = argv[1];
  const std::string out_path = argv[2];
  const std::string base = "/bizzy/sensors/sidescan/garmin_sidescan/";
  const std::string port_topic = argValue(argc, argv, "--port-topic", base + "sonar_image_port");
  const std::string stbd_topic =
    argValue(argc, argv, "--stbd-topic", base + "sonar_image_starboard");
  const std::string nadir_topic = argValue(argc, argv, "--nadir-topic", base + "nadir_depth");
  const std::string earth_frame = argValue(argc, argv, "--earth-frame", "earth");
  const double sound_speed_fallback =
    toDouble(argValue(argc, argv, "--sound-speed", "1500.0"), "--sound-speed");
  const double nadir_staleness_s =
    toDouble(argValue(argc, argv, "--nadir-staleness", "5.0"), "--nadir-staleness");
  const int expected_bins = toInt(argValue(argc, argv, "--bins", "2048"), "--bins");

  // Bounded TF cache: the projection only needs the transforms bracketing each
  // ping's stamp, so a small rolling window keeps tf2's std::list-backed
  // TimeCache short. Whole-bag buffering (a 10-hour window) made every
  // lookupTransform an O(n) linear list walk over ~574k transforms → ~17 min/bag
  // (issue #251; same fix as cube_bathymetry#63). The guard is how far the TF
  // frontier must advance past a ping before we project it, so both bracketing
  // transforms are present (no frontier drops).
  constexpr double kCacheWindowSec = 60.0;
  constexpr double kGuardSec = 3.0;
  const std::int64_t kGuardNs = static_cast<std::int64_t>(kGuardSec * 1e9);
  // Hard cap on the pending FIFO. Normal backlog is tiny (~guard seconds of pings),
  // but if the TF frontier stalls — /tf stops early, a long dropout, or a chain
  // resolvable via static TF only (frontier never advances) — pings would
  // otherwise accumulate to end-of-stream and grow RAM without bound. Over the cap
  // we force-project the oldest (its bracketing TF is usually still in the 60s
  // buffer; if not it is counted as no-tf, never silently lost). ~20k pings caps
  // the backlog at a few hundred MB while sitting ~1000x above normal operation.
  constexpr std::size_t kMaxPending = 20000;
  tf2::BufferCore tf_buffer(tf2::durationFromSec(kCacheWindowSec));

  std::ofstream out(out_path, std::ios::binary);
  if (!out) {
    std::cerr << "error: cannot open output " << out_path << "\n";
    return 1;
  }
  marine_sidescan_mosaic::writeTier1Header(out);

  rosbag2_cpp::Reader reader;
  rosbag2_storage::StorageOptions so;
  so.uri = bag_uri;
  reader.open(so);
  rosbag2_storage::StorageFilter filter;
  filter.topics = {"/tf", "/tf_static", port_topic, stbd_topic, nadir_topic};
  reader.set_filter(filter);

  float held_nadir = -1.0F;
  std::int64_t held_nadir_ns = 0;
  std::size_t n_written = 0, n_no_tf = 0, n_no_rate = 0, n_bad_bins = 0, n_tf = 0;

  // How far the newest dynamic /tf stamp has advanced. Static transforms carry a
  // zero stamp and cover all time, so they do not move the frontier.
  std::int64_t tf_frontier_ns = std::numeric_limits<std::int64_t>::min();

  // Pings awaiting their bracketing TF. Everything but the earth→transducer pose
  // is filled at enqueue (including the nadir snapshot, so the last-nadir-within-
  // staleness semantics are evaluated at the ping's own time, not at drain time);
  // the pose lookup is deferred until the frontier passes ping_ns + guard.
  struct PendingPing
  {
    std::int64_t ping_ns;
    std::string frame_id;
    marine_sidescan_mosaic::Tier1Ping p;
  };
  std::deque<PendingPing> pending;

  // Project queued pings whose bracketing TF is now present (frontier advanced a
  // guard interval past the ping stamp). With flush=true, project whatever
  // remains at end-of-stream against the available coverage.
  // Project + write the oldest pending ping (front) and pop it. A ping whose
  // bracketing earth transform is not in the bounded buffer at its stamp is
  // counted (n_no_tf), not silently dropped.
  auto flush_front = [&]() {
      auto & front = pending.front();
      try {
        const auto pose = tf_buffer.lookupTransform(
          earth_frame, front.frame_id,
          tf2::TimePoint(std::chrono::nanoseconds(front.ping_ns)));
        front.p.tx = pose.transform.translation.x;
        front.p.ty = pose.transform.translation.y;
        front.p.tz = pose.transform.translation.z;
        front.p.qx = pose.transform.rotation.x;
        front.p.qy = pose.transform.rotation.y;
        front.p.qz = pose.transform.rotation.z;
        front.p.qw = pose.transform.rotation.w;
        marine_sidescan_mosaic::writeTier1Ping(out, front.p);
        ++n_written;
      } catch (const tf2::TransformException &) {
        // No earth transform in the bounded buffer at this stamp — before the
        // first fix, or a TF gap wider than the cache window. Counted, not
        // silently dropped.
        ++n_no_tf;
      }
      pending.pop_front();
    };

  auto drain_pending = [&](bool flush) {
      while (!pending.empty()) {
        if (!flush &&
          (tf_frontier_ns == std::numeric_limits<std::int64_t>::min() ||
          pending.front().ping_ns > tf_frontier_ns - kGuardNs))
        {
          break;
        }
        flush_front();
      }
    };

  // SINGLE INTERLEAVED PASS over the chronological message stream.
  while (reader.has_next()) {
    auto bag_msg = reader.read_next();
    const std::string & topic = bag_msg->topic_name;

    if (topic == "/tf" || topic == "/tf_static") {
      const bool is_static = topic == "/tf_static";
      auto m = deserialize<tf2_msgs::msg::TFMessage>(bag_msg);
      for (const auto & tr : m.transforms) {
        tf_buffer.setTransform(tr, "bag", is_static);
        ++n_tf;
        if (!is_static) {
          tf_frontier_ns = std::max(tf_frontier_ns, stampNs(tr.header.stamp));
        }
      }
      drain_pending(false);
      continue;
    }

    if (topic == nadir_topic) {
      auto r = deserialize<sensor_msgs::msg::Range>(bag_msg);
      if (std::isfinite(r.range) && r.range > 0.0F) {
        held_nadir = r.range;
        held_nadir_ns = stampNs(r.header.stamp);
      }
      continue;
    }

    const bool is_port = topic == port_topic;
    auto msg = deserialize<marine_acoustic_msgs::msg::RawSonarImage>(bag_msg);
    if (msg.sample_rate <= 0.0) {
      ++n_no_rate;
      continue;
    }
    // Decode-bug gate (#184): pre-fix bags mis-located sample values and show an
    // inconsistent bin count; well-formed pings have sample0+samples_per_beam == N.
    if (expected_bins > 0 &&
      static_cast<int>(msg.sample0) + static_cast<int>(msg.samples_per_beam) != expected_bins)
    {
      ++n_bad_bins;
      continue;
    }
    const std::int64_t ping_ns = stampNs(msg.header.stamp);

    PendingPing pp;
    pp.ping_ns = ping_ns;
    pp.frame_id = msg.header.frame_id;
    marine_sidescan_mosaic::Tier1Ping & p = pp.p;
    p.stamp_ns = ping_ns;
    p.channel = is_port ? marine_sidescan_mosaic::Tier1Channel::Port :
      marine_sidescan_mosaic::Tier1Channel::Starboard;
    p.sound_speed = msg.ping_info.sound_speed > 0.0 ? msg.ping_info.sound_speed :
      sound_speed_fallback;
    p.sample_rate = msg.sample_rate;
    p.sample0 = static_cast<std::int32_t>(msg.sample0);
    // Snapshot the held nadir at the ping's own time (not at drain), matching the
    // pre-single-pass semantics: the most recent nadir at/before the ping, within
    // the staleness bound.
    const double age_s = std::abs(ping_ns - held_nadir_ns) / 1e9;
    p.nadir_altitude_m = (held_nadir > 0.0F && age_s <= nadir_staleness_s) ? held_nadir : -1.0F;
    // Along-track tx −3 dB beamwidth (Tier-1 v2), so offline Tier-2 reproduces the
    // footprint without re-reading the bag; 0 when the driver didn't publish it.
    p.tx_beamwidth_rad =
      (!msg.ping_info.tx_beamwidths.empty() && msg.ping_info.tx_beamwidths[0] > 0.0F) ?
      msg.ping_info.tx_beamwidths[0] : 0.0F;
    const auto raw = marine_sidescan_mosaic::decodeSamples(msg);
    p.samples.assign(raw.begin(), raw.end());   // double -> float (lossless for GCV range).

    pending.push_back(std::move(pp));
    // Bound memory if the TF frontier stalls (see kMaxPending): force-project the
    // oldest so the FIFO cannot grow to end-of-stream on a static-only chain or a
    // long /tf dropout.
    if (pending.size() > kMaxPending) {
      flush_front();
    }
  }
  drain_pending(true);   // project the tail (within a guard of end-of-stream).

  out.flush();
  if (!out.good()) {
    std::cerr << "error: write/flush failed (disk full?); " << out_path
              << " may be truncated\n";
    return 1;
  }
  std::cerr << "wrote " << n_written << " Tier-1 pings to " << out_path
            << " (fed " << n_tf << " transforms; dropped: no-tf=" << n_no_tf
            << " no-rate=" << n_no_rate << " bad-bins=" << n_bad_bins << ")\n";
  return 0;
}
