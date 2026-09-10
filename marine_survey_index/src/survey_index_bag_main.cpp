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
/// @brief Offline survey indexer: bags → `survey_index.db` (#259, stage 1 of #258).
///
/// Reads each bag **directly** (rosbag2, not replay) in a **single interleaved
/// pass** over `/tf` + `/tf_static` + the MBES `SonarDetections` and sidescan
/// `RawSonarImage` channels, chronologically. `/tf` feeds a **bounded-window**
/// `tf2::BufferCore`; each ping waits in a short FIFO until the TF frontier has
/// advanced a guard interval past its stamp, then resolves the `earth`→sensor
/// pose, computes the ping's ground-footprint bounding box, and records every
/// touched GGGS tile in the pass-interval accumulator. The bounded window is
/// the proven fix for tf2's O(n) TimeCache walk (cube_bathymetry#63 /
/// marine_sidescan_mosaic#251) — reused, not re-derived.
///
/// The index answers "where did the sensor *look*", independent of what any
/// store accepted — pings rejected by CUBE or lost to store gaps still index.
/// The DB is a regenerable sidecar: bags remain the data of record.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geographic_msgs/msg/geo_point.hpp"
#include "geodesy/geodesics.h"
#include "marine_acoustic_msgs/msg/raw_sonar_image.hpp"
#include "marine_acoustic_msgs/msg/sonar_detections.hpp"
#include "marine_sidescan_mosaic/projection.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/storage_filter.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "tf2/buffer_core.h"
#include "tf2/time.h"
#include "tf2_msgs/msg/tf_message.hpp"

#include "marine_survey_index/bag_fingerprint.hpp"
#include "marine_survey_index/footprint.hpp"
#include "marine_survey_index/interval_accumulator.hpp"
#include "marine_survey_index/nav_decimation.hpp"
#include "marine_survey_index/schema.hpp"

namespace
{
constexpr std::int64_t kNsPerS = 1000000000LL;

// Extends the ADR-0005 D3 sensor_class vocabulary ('mbes-bathy' verbatim;
// 'sidescan' + channel suffix). The query CLI maps a plain 'sidescan' filter
// to both channels.
constexpr const char * kSensorMbes = "mbes-bathy";
constexpr const char * kSensorPort = "sidescan-port";
constexpr const char * kSensorStbd = "sidescan-stbd";

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

bool hasFlag(int argc, char ** argv, const std::string & flag)
{
  for (int i = 1; i < argc; ++i) {
    if (flag == argv[i]) {
      return true;
    }
  }
  return false;
}

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

// GGGS levels are 0..20 (gggs::Level throws std::out_of_range at >= 21, and it
// is constructed outside any try/catch → std::terminate). Reject out-of-range
// values here so the CLI exits cleanly instead of aborting.
int toLevel(const std::string & s, const std::string & flag)
{
  const int v = toInt(s, flag);
  if (v < 0 || v > 20) {
    std::cerr << "error: " << flag << " must be in [0, 20], got " << v << "\n";
    std::exit(2);
  }
  return v;
}

// True when an error_code means "this path definitively resolves to nothing"
// (a dangling symlink, or a name under a non-directory) rather than "the
// answer could not be obtained". The first hides nothing; the second does.
bool resolvesToNothing(const std::error_code & ec)
{
  return ec == std::errc::no_such_file_or_directory || ec == std::errc::not_a_directory;
}

// Recursively find rosbag2 bags (directories containing metadata.yaml) under a
// scan root. A bag directory itself is not descended into further.
//
// Anything that keeps the scan from seeing the whole tree is reported through
// @p problems, and the caller exits non-zero for it. A dropped subtree is
// strictly worse than the unreadable-bag case: those bags are absent from the
// index entirely rather than re-indexed needlessly, and downstream
// (cube_bathymetry's dirty-tile guard fires only when the dirty set is
// *entirely* empty) one missing bag yields an authoritative-looking marker
// over stale store tiles.
std::vector<std::filesystem::path> scanForBags(
  const std::filesystem::path & root, std::vector<std::string> & problems)
{
  namespace fs = std::filesystem;
  std::vector<fs::path> bags;
  std::error_code ec;
  if (fs::exists(root / "metadata.yaml", ec)) {
    bags.push_back(root);
    return bags;
  }
  // error_code overloads: a broken symlink or unreadable entry sets ec and
  // ends the walk cleanly instead of throwing and aborting the whole run.
  // Deliberately *not* `skip_permission_denied`, and no error_code is
  // discarded: skipping made an unreadable directory drop every bag beneath it
  // with no warning, no counter and exit 0.
  fs::recursive_directory_iterator it(root, ec), end;
  if (ec) {
    problems.push_back(
      "could not scan '" + root.string() + "': " + ec.message() +
      " - no bag anywhere under it is in this run");
    return bags;
  }
  while (it != end) {
    const fs::path current = it->path();
    std::error_code entry_ec;
    const bool dir = it->is_directory(entry_ec);
    if (entry_ec && !resolvesToNothing(entry_ec)) {
      problems.push_back(
        "could not determine the type of '" + current.string() + "': " + entry_ec.message() +
        " - if it is a bag, or holds one, it is missing from this run");
    } else if (!entry_ec && dir) {
      std::error_code meta_ec;
      if (fs::exists(current / "metadata.yaml", meta_ec)) {
        bags.push_back(current);
        it.disable_recursion_pending();
      } else if (meta_ec && !resolvesToNothing(meta_ec)) {
        problems.push_back(
          "could not tell whether '" + current.string() + "' is a bag: " + meta_ec.message() +
          " - it, and any bag beneath it, are missing from this run");
        // Reporting is not enough: descent has to be cancelled too. A failed
        // `increment()` ends the WHOLE walk (see below), so leaving recursion
        // pending on a directory we already know we cannot read would drop
        // every bag after it in readdir order, not just the ones beneath it.
        it.disable_recursion_pending();
      } else {
        std::error_code link_ec;
        // A symlinked directory is not descended into (a link can close a
        // cycle a recursive walk would never leave), so any bag beneath one
        // would be missed silently. Reported, not followed — name the real
        // path, or that bag, on the command line instead.
        if (it->is_symlink(link_ec) && !link_ec) {
          problems.push_back(
            "'" + current.string() +
            "' is a symlink to a directory, which is not scanned for bags"
            " - any bag beneath it is missing from this run");
        } else {
          // Ask whether this directory can be enumerated *before* the walk
          // descends into it. A traverse-only directory (mode 0111) probes
          // for `metadata.yaml` successfully yet cannot be listed, so it
          // reaches neither branch above and would abandon the walk in
          // `increment()` the same way. Same cost either way: the walk is
          // about to open this directory anyway.
          std::error_code enum_ec;
          fs::directory_iterator probe(current, enum_ec);
          if (enum_ec) {
            problems.push_back(
              "could not enumerate '" + current.string() + "': " + enum_ec.message() +
            " - any bag beneath it is missing from this run");
            it.disable_recursion_pending();
          }
        }
      }
    }
    it.increment(ec);
    if (ec) {
      problems.push_back(
        "scan of '" + root.string() + "' stopped at '" + current.string() + "': " +
        ec.message() + " - every bag after that point in the walk is missing from this run");
      break;
    }
  }
  std::sort(bags.begin(), bags.end());
  return bags;
}

// Thrown by the sqlite wrappers below so a failed write unwinds to the per-bag
// handler in main(), which ROLLBACKs — instead of silently COMMITting a partial
// index and marking the bag fully indexed.
class SqliteError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

void execOrThrow(sqlite3 * db, const char * sql)
{
  char * err = nullptr;
  if (sqlite3_exec(db, sql, nullptr, nullptr, &err) != SQLITE_OK) {
    const std::string msg = err ? err : "unknown";
    sqlite3_free(err);
    throw SqliteError("sqlite exec: " + msg);
  }
}

sqlite3_stmt * prepareOrThrow(sqlite3 * db, const char * sql)
{
  sqlite3_stmt * stmt = nullptr;
  if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    throw SqliteError(std::string("sqlite prepare: ") + sqlite3_errmsg(db));
  }
  return stmt;
}

// RAII for a prepared statement: finalized on scope exit, including when a
// SqliteError unwinds through the write path.
class StmtGuard
{
public:
  explicit StmtGuard(sqlite3_stmt * stmt)
  : stmt_(stmt) {}
  ~StmtGuard()
  {
    sqlite3_finalize(stmt_);
  }
  StmtGuard(const StmtGuard &) = delete;
  StmtGuard & operator=(const StmtGuard &) = delete;
  sqlite3_stmt * get() const
  {
    return stmt_;
  }

private:
  sqlite3_stmt * stmt_;
};

// Step a write statement expected to run to completion; throw on any non-DONE
// return so the caller's transaction is never COMMITted after a failed row.
void stepDoneOrThrow(sqlite3 * db, sqlite3_stmt * stmt)
{
  if (sqlite3_step(stmt) != SQLITE_DONE) {
    throw SqliteError(std::string("sqlite step: ") + sqlite3_errmsg(db));
  }
}

// Ledger lookup: 0 = not indexed; >0 = bag id (unchanged, skip); <0 = -bag id
// (changed, re-index: caller deletes old passes and updates the row).
std::int64_t ledgerState(
  sqlite3 * db, const std::string & path,
  const marine_survey_index::BagFingerprint & fp)
{
  sqlite3_stmt * stmt = prepareOrThrow(
    db, "SELECT id, size_bytes, mtime_ns FROM bags WHERE path = ?");
  sqlite3_bind_text(stmt, 1, path.c_str(), -1, SQLITE_TRANSIENT);
  std::int64_t result = 0;
  if (sqlite3_step(stmt) == SQLITE_ROW) {
    const std::int64_t id = sqlite3_column_int64(stmt, 0);
    const bool unchanged = marine_survey_index::fingerprintMatches(
      fp, sqlite3_column_int64(stmt, 1), sqlite3_column_int64(stmt, 2));
    result = unchanged ? id : -id;
  }
  sqlite3_finalize(stmt);
  return result;
}

geographic_msgs::msg::GeoPoint groundOrigin(const marine_sidescan_mosaic::GeoBeam & gb)
{
  geographic_msgs::msg::GeoPoint origin;
  origin.latitude = gb.latitude_deg;
  origin.longitude = gb.longitude_deg;
  origin.altitude = 0.0;  // geodesy::wgs84::direct precondition.
  return origin;
}

// Endpoint at signed across-track horizontal offset h (m): + = starboard of
// the sensor heading, - = port.
geographic_msgs::msg::GeoPoint acrossTrackPoint(
  const geographic_msgs::msg::GeoPoint & origin, double heading_rad, double h)
{
  if (h == 0.0 || !std::isfinite(h)) {
    return origin;
  }
  const double azimuth = heading_rad + (h >= 0.0 ? M_PI / 2.0 : -M_PI / 2.0);
  return geodesy::wgs84::direct(origin, azimuth, std::abs(h));
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2 || hasFlag(argc, argv, "--help")) {
    std::cerr <<
      "usage: survey_index_bag [bag_uri ...] [--scan DIR] [--db survey_index.db]\n"
      "       [--mbes-topic T] [--port-topic T] [--stbd-topic T]\n"
      "       [--mbes-level N=14] [--sidescan-level N=14] [--level N (overrides both)]\n"
      "       [--merge-gap S=5.0] [--nav-stride-m M=10.0]\n"
      "       [--earth-frame F=earth] [--sound-speed S=1500]\n"
      "\n"
      "Indexes where each sonar looked, per bag, into a regenerable SQLite\n"
      "sidecar. Unchanged already-indexed bags are skipped; changed bags are\n"
      "re-indexed. Default level 14 (~54 m tiles) — a target-inspection\n"
      "neighbourhood; rolls up to the stores' coarser native tiles (bathy L10,\n"
      "sidescan L13) via the GGGS parent hierarchy. Also records a decimated\n"
      "nav track (one point per >= --nav-stride-m metres) for the explorer map.\n";
    return 2;
  }

  const std::string db_path = argValue(argc, argv, "--db", "survey_index.db");
  const std::string base = "/bizzy/sensors/";
  const std::string mbes_topic = argValue(argc, argv, "--mbes-topic", base + "m3/detections");
  const std::string port_topic =
    argValue(argc, argv, "--port-topic", base + "sidescan/garmin_sidescan/sonar_image_port");
  const std::string stbd_topic =
    argValue(argc, argv, "--stbd-topic", base + "sidescan/garmin_sidescan/sonar_image_starboard");
  const std::string earth_frame = argValue(argc, argv, "--earth-frame", "earth");
  const double sound_speed_fallback =
    toDouble(argValue(argc, argv, "--sound-speed", "1500.0"), "--sound-speed");
  const double merge_gap_s = toDouble(argValue(argc, argv, "--merge-gap", "5.0"), "--merge-gap");
  if (!(merge_gap_s >= 0.0)) {  // also rejects NaN
    std::cerr << "error: --merge-gap must be >= 0, got " << merge_gap_s << "\n";
    return 2;
  }
  const double nav_stride_m =
    toDouble(argValue(argc, argv, "--nav-stride-m", "10.0"), "--nav-stride-m");
  // A zero/negative/NaN stride would keep every posed ping (millions of rows);
  // +inf would keep only the first point per bag. Reject both.
  if (!(nav_stride_m > 0.0) || !std::isfinite(nav_stride_m)) {
    std::cerr << "error: --nav-stride-m must be a finite value > 0, got "
              << nav_stride_m << "\n";
    return 2;
  }
  // Default L14 (~54 m tiles): the tile-of-interest scale for target review
  // (Roland, 2026-07-13) — small enough to select a neighbourhood, and it
  // rolls up to the coarser store-native tiles (bathy L10, sidescan L13)
  // through the GGGS quadtree when stage 2 joins against store tiling.
  int mbes_level_n = toLevel(argValue(argc, argv, "--mbes-level", "14"), "--mbes-level");
  int sidescan_level_n =
    toLevel(argValue(argc, argv, "--sidescan-level", "14"), "--sidescan-level");
  const std::string level_override = argValue(argc, argv, "--level", "");
  if (!level_override.empty()) {
    mbes_level_n = sidescan_level_n = toLevel(level_override, "--level");
  }
  const gggs::Level mbes_level(static_cast<std::uint8_t>(mbes_level_n));
  const gggs::Level sidescan_level(static_cast<std::uint8_t>(sidescan_level_n));

  // Collect bag list: positional URIs + --scan roots.
  std::vector<std::filesystem::path> bags;
  std::vector<std::string> scan_problems;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--scan") {
      if (i + 1 < argc) {
        const auto found = scanForBags(argv[i + 1], scan_problems);
        bags.insert(bags.end(), found.begin(), found.end());
      }
      ++i;
      continue;
    }
    if (arg.rfind("--", 0) == 0) {
      ++i;  // every other flag takes a value
      continue;
    }
    bags.emplace_back(arg);
  }
  // Each problem names its own consequence: a dropped subtree, an entry of
  // unknown type and a walk that stopped part-way lose different things, and a
  // blanket "any bag beneath it" was wrong for the last of those -- what is
  // missing there is every bag after that point in the walk.
  for (const std::string & problem : scan_problems) {
    std::cerr << "warning: " << problem << "\n";
  }
  if (bags.empty()) {
    // Precedence: a scan that could not be enumerated is an INCOMPLETE index
    // (1), not a usage error (2) — the command line was well formed, the tree
    // was not readable. Tested before the empty-bag return because that is the
    // worst instance of it: "the survey disk is not mounted" nominates nothing
    // at all, and reporting it as "you invoked me wrong" is exactly the
    // conclusion a scheduler must not draw. Summarised like any other run so
    // the zero is stated rather than inferred from an absent summary.
    if (!scan_problems.empty()) {
      std::cerr << "done: 0 bag(s) indexed, 0 unchanged skipped, 0 failed"
                << " (of 0 nominated); 0 not fully readable"
                << "; no bag could be nominated because the scan above could not"
                << " be enumerated -> " << db_path << " (unchanged)\n";
      return 1;
    }
    std::cerr << "error: no bags given (positional URIs and/or --scan DIR)\n";
    return 2;
  }

  sqlite3 * db = nullptr;
  try {
    db = marine_survey_index::openIndexDb(db_path);
  } catch (const std::exception & e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }

  // Same bounded-TF-window constants as the proven importers (#251 / cube#63).
  constexpr double kCacheWindowSec = 60.0;
  constexpr double kGuardSec = 3.0;
  const std::int64_t kGuardNs = static_cast<std::int64_t>(kGuardSec * 1e9);
  constexpr std::size_t kMaxPending = 20000;
  const std::int64_t merge_gap_ns = static_cast<std::int64_t>(merge_gap_s * 1e9);

  std::size_t n_bags_indexed = 0, n_bags_skipped = 0;
  // Durable signals for the exit status: a bag that cannot be fingerprinted
  // authoritatively re-indexes forever, and a bag that failed mid-index leaves
  // the ledger without it. Neither may report success.
  std::size_t n_bags_unreadable = 0, n_bags_failed = 0;

  for (const auto & bag : bags) {
    try {
      // Symlinks are resolved out of the ledger key: `lexically_normal()`
      // alone does not, while `is_directory()` does, so a bag reached through
      // a symlinked path used to become a SECOND bag under a second `bag_id`
      // -- every pass interval and nav point inserted twice. With the link
      // resolved, the second nomination finds the first bag's ledger row and
      // skips it. `weakly_canonical` falls back to the plain absolute path if
      // the resolution fails (the fingerprint reports the same cause).
      std::error_code key_ec;
      const std::filesystem::path resolved = std::filesystem::weakly_canonical(bag, key_ec);
      const std::string bag_key = key_ec ?
        std::filesystem::absolute(bag).lexically_normal().string() :
        resolved.lexically_normal().string();
      std::string fingerprint_problem;
      const marine_survey_index::BagFingerprint fp =
        marine_survey_index::bagFingerprint(bag, &fingerprint_problem);
      if (!fp.authoritative()) {
        // Loud, not silent: this bag cannot be judged unchanged, so it will
        // re-index on every run until the cause is fixed. The counter and the
        // exit status key on the fingerprint's own flags — the string is the
        // message, never the condition, so no empty-string convention has to
        // hold across the library boundary for the exit code to be right.
        ++n_bags_unreadable;
        // The library always sets a reason here (asserted by
        // ProblemStringAndTrustFlagsAlwaysAgree); the fallback keeps the
        // warning meaningful rather than blank if that ever drifts.
        const std::string why = fingerprint_problem.empty() ?
          "'" + bag_key + "' could not be fingerprinted authoritatively" : fingerprint_problem;
        std::cerr << "warning: " << why
                  << " - treating this bag as changed, so it re-indexes every run\n";
      }
      const std::int64_t state = ledgerState(db, bag_key, fp);
      if (state > 0) {
        ++n_bags_skipped;
        std::cerr << "skip (unchanged): " << bag_key << "\n";
        continue;
      }

      tf2::BufferCore tf_buffer(tf2::durationFromSec(kCacheWindowSec));
      marine_survey_index::IntervalAccumulator accumulator(merge_gap_ns);
      std::int64_t tf_frontier_ns = std::numeric_limits<std::int64_t>::min();
      std::size_t n_pings = 0, n_no_tf = 0, n_bad_pose = 0;

      // Decimated nav track (#265): every posed ping's ground origin feeds
      // the distance gate; accepted points land in nav_track alongside the
      // passes. Provenance: sensor ground origins interleaved across the
      // indexed topics — not a single vehicle frame (see the schema doc).
      marine_survey_index::NavDecimator nav_decimator(nav_stride_m);
      struct NavTrackPoint
      {
        std::int64_t t_ns;
        double latitude;
        double longitude;
      };
      std::vector<NavTrackPoint> nav_points;

      // A ping queued for TF resolution: everything except the earth pose is
      // computed at enqueue. `extent_port`/`extent_stbd` are the signed
      // across-track horizontal extents (m, + = starboard). For sidescan the
      // slant range bounds the ground range (conservative overestimate — the
      // index answers "could this sensor have seen here").
      struct PendingPing
      {
        std::int64_t ping_ns;
        std::string frame_id;
        const char * sensor_type;
        const std::string * topic;
        const gggs::Level * level;
        double extent_port;
        double extent_stbd;
      };
      std::deque<PendingPing> pending;

      auto flush_front = [&]() {
          auto & front = pending.front();
          try {
            const auto pose = tf_buffer.lookupTransform(
              earth_frame, front.frame_id,
              tf2::TimePoint(std::chrono::nanoseconds(front.ping_ns)));
            const auto gb = marine_sidescan_mosaic::ecefPoseToGeoBeam(
              pose.transform.translation.x, pose.transform.translation.y,
              pose.transform.translation.z,
              pose.transform.rotation.x, pose.transform.rotation.y,
              pose.transform.rotation.z, pose.transform.rotation.w);
            if (!gb.valid) {
              ++n_bad_pose;
            } else {
              const auto origin = groundOrigin(gb);
              if (nav_decimator.accept(origin.latitude, origin.longitude)) {
                nav_points.push_back({front.ping_ns, origin.latitude, origin.longitude});
              }
              const auto p1 = acrossTrackPoint(origin, gb.heading_rad, front.extent_port);
              const auto p2 = acrossTrackPoint(origin, gb.heading_rad, front.extent_stbd);
              const double lat_min =
                std::min({origin.latitude, p1.latitude, p2.latitude});
              const double lat_max =
                std::max({origin.latitude, p1.latitude, p2.latitude});
              const double lon_min =
                std::min({origin.longitude, p1.longitude, p2.longitude});
              const double lon_max =
                std::max({origin.longitude, p1.longitude, p2.longitude});
              for (const auto & tile : marine_survey_index::tilesForBoundingBox(
                  lat_min, lon_min, lat_max, lon_max, *front.level))
              {
                marine_survey_index::TileKey key;
                key.level = tile.level();
                key.tile_row = tile.row();
                key.tile_col = tile.column();
                key.sensor_type = front.sensor_type;
                key.topic = *front.topic;
                accumulator.addPing(key, front.ping_ns);
              }
              ++n_pings;
            }
          } catch (const tf2::TransformException &) {
            // No earth transform in the bounded buffer at this stamp — before
            // the first fix, or a TF gap wider than the window. Counted, not
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

      rosbag2_cpp::Reader reader;
      rosbag2_storage::StorageOptions so;
      so.uri = bag.string();
      try {
        reader.open(so);
      } catch (const std::exception & e) {
        // Counted, not just printed: this bag is absent from the index, which
        // is the same durable condition as a mid-index failure. Landing in no
        // summary bucket at all used to exit 0 on a run that indexed nothing.
        ++n_bags_failed;
        std::cerr << "error: cannot open bag " << bag_key << ": " << e.what()
                  << "; skipping\n";
        continue;
      }
      rosbag2_storage::StorageFilter filter;
      filter.topics = {"/tf", "/tf_static", mbes_topic, port_topic, stbd_topic};
      reader.set_filter(filter);

      // SINGLE INTERLEAVED PASS over the chronological message stream.
      while (reader.has_next()) {
        auto bag_msg = reader.read_next();
        const std::string & topic = bag_msg->topic_name;

        if (topic == "/tf" || topic == "/tf_static") {
          const bool is_static = topic == "/tf_static";
          auto m = deserialize<tf2_msgs::msg::TFMessage>(bag_msg);
          for (const auto & tr : m.transforms) {
            tf_buffer.setTransform(tr, "bag", is_static);
            if (!is_static) {
              tf_frontier_ns = std::max(tf_frontier_ns, stampNs(tr.header.stamp));
            }
          }
          drain_pending(false);
          continue;
        }

        PendingPing pp;
        if (topic == mbes_topic) {
          auto msg = deserialize<marine_acoustic_msgs::msg::SonarDetections>(bag_msg);
          pp.ping_ns = stampNs(msg.header.stamp);
          pp.frame_id = msg.header.frame_id;
          pp.sensor_type = kSensorMbes;
          pp.topic = &mbes_topic;
          pp.level = &mbes_level;
          // Across-track extents from the outermost good detections:
          // horizontal ≈ slant · sin(rx_angle) (+ starboard). No good beams →
          // extents 0, the sensor-position tile still indexes.
          const double c = msg.ping_info.sound_speed > 0.0F ?
            msg.ping_info.sound_speed : sound_speed_fallback;
          double min_h = 0.0, max_h = 0.0;
          const std::size_t n = msg.two_way_travel_times.size();
          for (std::size_t i = 0; i < n; ++i) {
            if (i < msg.flags.size() &&
              msg.flags[i].flag != marine_acoustic_msgs::msg::DetectionFlag::DETECT_OK)
            {
              continue;
            }
            const double twtt = msg.two_way_travel_times[i];
            if (!std::isfinite(twtt) || twtt <= 0.0) {
              continue;
            }
            const double rx = i < msg.rx_angles.size() ? msg.rx_angles[i] : 0.0;
            const double h = (twtt * c / 2.0) * std::sin(rx);
            min_h = std::min(min_h, h);
            max_h = std::max(max_h, h);
          }
          pp.extent_port = min_h;
          pp.extent_stbd = max_h;
        } else {
          const bool is_port = topic == port_topic;
          auto msg = deserialize<marine_acoustic_msgs::msg::RawSonarImage>(bag_msg);
          // A zero sample count drives slantRange() negative, which flips the
          // sidescan footprint to the wrong side; skip like sample_rate<=0.
          if (msg.sample_rate <= 0.0 || msg.samples_per_beam == 0) {
            continue;
          }
          pp.ping_ns = stampNs(msg.header.stamp);
          pp.frame_id = msg.header.frame_id;
          pp.sensor_type = is_port ? kSensorPort : kSensorStbd;
          pp.topic = is_port ? &port_topic : &stbd_topic;
          pp.level = &sidescan_level;
          const double c = msg.ping_info.sound_speed > 0.0 ?
            msg.ping_info.sound_speed : sound_speed_fallback;
          // Max slant range bounds the ground range (flat-earth-free
          // conservative footprint; the drill-down stages do the exact math).
          const double max_slant = marine_sidescan_mosaic::slantRange(
            static_cast<int>(msg.samples_per_beam) - 1,
            static_cast<int>(msg.sample0), c, msg.sample_rate);
          pp.extent_port = is_port ? -max_slant : 0.0;
          pp.extent_stbd = is_port ? 0.0 : max_slant;
        }

        pending.push_back(std::move(pp));
        if (pending.size() > kMaxPending) {
          // Prefer flushing pings the TF frontier has already passed (this
          // honours the guard interval); force the oldest out — accepting a
          // possible no-tf — only if TF has genuinely stalled and we are still
          // over the memory cap.
          drain_pending(false);
          if (pending.size() > kMaxPending) {
            flush_front();
          }
        }
      }
      drain_pending(true);

      // Persist this bag atomically: ledger row + all pass intervals.
      const auto intervals = accumulator.flushAll();
      const std::int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
      execOrThrow(db, "BEGIN TRANSACTION;");
      std::int64_t bag_id = 0;
      if (state < 0) {
        bag_id = -state;
        {
          StmtGuard del(prepareOrThrow(db, "DELETE FROM passes WHERE bag_id = ?"));
          sqlite3_bind_int64(del.get(), 1, bag_id);
          stepDoneOrThrow(db, del.get());
        }
        {
          // The re-index path keeps the bag row, so the CASCADE never fires —
          // clear the old track explicitly like the passes above.
          StmtGuard del(prepareOrThrow(db, "DELETE FROM nav_track WHERE bag_id = ?"));
          sqlite3_bind_int64(del.get(), 1, bag_id);
          stepDoneOrThrow(db, del.get());
        }
        {
          const char * upd_sql =
            "UPDATE bags SET size_bytes = ?, mtime_ns = ?, indexed_at_ns = ? WHERE id = ?";
          StmtGuard upd(prepareOrThrow(db, upd_sql));
          sqlite3_bind_int64(upd.get(), 1, fp.size_bytes);
          // `mtime_ns` is 0 whenever the fingerprint is not valid (struct
          // invariant), and is never load-bearing: `fingerprintMatches()`
          // rejects an untrustworthy fingerprint before reading it.
          sqlite3_bind_int64(upd.get(), 2, fp.mtime_ns);
          sqlite3_bind_int64(upd.get(), 3, now_ns);
          sqlite3_bind_int64(upd.get(), 4, bag_id);
          stepDoneOrThrow(db, upd.get());
        }
      } else {
        const char * ins_sql =
          "INSERT INTO bags (path, size_bytes, mtime_ns, indexed_at_ns) VALUES (?, ?, ?, ?)";
        StmtGuard ins(prepareOrThrow(db, ins_sql));
        sqlite3_bind_text(ins.get(), 1, bag_key.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_int64(ins.get(), 2, fp.size_bytes);
        sqlite3_bind_int64(ins.get(), 3, fp.mtime_ns);
        sqlite3_bind_int64(ins.get(), 4, now_ns);
        stepDoneOrThrow(db, ins.get());
        bag_id = sqlite3_last_insert_rowid(db);
      }
      {
        const char * pass_sql =
          "INSERT INTO passes (bag_id, level, tile_row, tile_col, sensor_type, topic,"
          " t_start_ns, t_end_ns, ping_count) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)";
        StmtGuard ins_pass(prepareOrThrow(db, pass_sql));
        for (const auto & pass : intervals) {
          sqlite3_bind_int64(ins_pass.get(), 1, bag_id);
          sqlite3_bind_int(ins_pass.get(), 2, pass.key.level);
          sqlite3_bind_int64(ins_pass.get(), 3, pass.key.tile_row);
          sqlite3_bind_int64(ins_pass.get(), 4, pass.key.tile_col);
          sqlite3_bind_text(ins_pass.get(), 5, pass.key.sensor_type.c_str(), -1, SQLITE_TRANSIENT);
          sqlite3_bind_text(ins_pass.get(), 6, pass.key.topic.c_str(), -1, SQLITE_TRANSIENT);
          sqlite3_bind_int64(ins_pass.get(), 7, pass.t_start_ns);
          sqlite3_bind_int64(ins_pass.get(), 8, pass.t_end_ns);
          sqlite3_bind_int64(ins_pass.get(), 9, pass.ping_count);
          stepDoneOrThrow(db, ins_pass.get());
          sqlite3_reset(ins_pass.get());
        }
      }
      {
        // Points were gated in flush (read) order; store them time-ordered so
        // row order matches the accessors' ORDER BY t_ns.
        std::stable_sort(
          nav_points.begin(), nav_points.end(),
          [](const NavTrackPoint & a, const NavTrackPoint & b) {return a.t_ns < b.t_ns;});
        const char * nav_sql =
          "INSERT INTO nav_track (bag_id, t_ns, latitude, longitude) VALUES (?, ?, ?, ?)";
        StmtGuard ins_nav(prepareOrThrow(db, nav_sql));
        for (const auto & p : nav_points) {
          sqlite3_bind_int64(ins_nav.get(), 1, bag_id);
          sqlite3_bind_int64(ins_nav.get(), 2, p.t_ns);
          sqlite3_bind_double(ins_nav.get(), 3, p.latitude);
          sqlite3_bind_double(ins_nav.get(), 4, p.longitude);
          stepDoneOrThrow(db, ins_nav.get());
          sqlite3_reset(ins_nav.get());
        }
      }
      execOrThrow(db, "COMMIT;");

      ++n_bags_indexed;
      std::cerr << (state < 0 ? "re-indexed: " : "indexed: ") << bag_key
                << " (" << n_pings << " pings -> " << intervals.size()
                << " pass intervals, " << nav_points.size()
                << " nav points; no-tf=" << n_no_tf
                << " bad-pose=" << n_bad_pose << ")\n";
    } catch (const std::exception & e) {
      // A bad message or a SQLite failure aborts this bag only: roll back
      // its (possibly open) transaction and move on so the remaining bags
      // still index and the db is closed cleanly at the end.
      sqlite3_exec(db, "ROLLBACK;", nullptr, nullptr, nullptr);
      ++n_bags_failed;
      std::cerr << "error: failed to index " << bag.string() << ": "
                << e.what() << "; skipping\n";
    }
  }

  sqlite3_close(db);
  // `indexed + skipped + failed` partitions the nominated bags (hence the
  // "of N" — a mismatch is a bug), while `not fully readable` cuts across
  // them: such a bag is never skipped, so it is already counted as indexed or
  // failed. Reported separately rather than folded in, because it is the one
  // condition that persists across runs.
  std::cerr << "done: " << n_bags_indexed << " bag(s) indexed, "
            << n_bags_skipped << " unchanged skipped, "
            << n_bags_failed << " failed (of " << bags.size() << " nominated); "
            << n_bags_unreadable << " not fully readable, so re-indexing every run"
            << " -> " << db_path << "\n";
  // Exit status contract (also stated in the README):
  //   0  every nominated bag is in the index, and every fingerprint is
  //      authoritative
  //   1  the index is INCOMPLETE: a bag failed mid-index or could not be
  //      opened, or a --scan tree could not be fully enumerated (so bags may
  //      be missing outright). Also returned earlier when the DB itself could
  //      not be opened, in which case nothing was done at all.
  //   2  usage error
  //   3  the index is complete, but at least one bag cannot be fingerprinted
  //      authoritatively and so re-indexes on every run until the cause is
  //      fixed -- durable and worth a signal, but not a missing bag.
  // A run that only said so on stderr left no signal a scheduler or script
  // could see; conflating the two left a caller unable to tell a permanent
  // permission wart from an index it cannot rely on.
  if (n_bags_failed > 0 || !scan_problems.empty()) {
    return 1;
  }
  return n_bags_unreadable > 0 ? 3 : 0;
}
