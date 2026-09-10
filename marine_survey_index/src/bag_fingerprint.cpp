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

#include "marine_survey_index/bag_fingerprint.hpp"

#include <sys/stat.h>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <string>
#include <system_error>

namespace marine_survey_index
{
namespace
{
constexpr std::int64_t kNsPerS = 1000000000LL;

// The tv_sec range whose nanosecond conversion still fits in int64_t. Outside
// it, `tv_sec * kNsPerS` is signed-overflow undefined behaviour (roughly past
// the year 2262, or before 1678) and the wrapped value would be indexed as a
// perfectly ordinary timestamp. That is reachable from a corrupt inode or a
// host with a wrong clock, not only from a deliberate `touch -d 2500-01-01`.
constexpr std::int64_t kMaxMtimeSec =
  (std::numeric_limits<std::int64_t>::max() - (kNsPerS - 1)) / kNsPerS;
constexpr std::int64_t kMinMtimeSec = std::numeric_limits<std::int64_t>::min() / kNsPerS;

// One ::stat per file: size and mtime together, so a file whose timestamp
// cannot be read contributes neither. Returns false if the stat failed or the
// timestamp is outside the representable range, in which case the caller
// treats the file as unreadable rather than trusting a wrapped value.
bool statFile(const std::filesystem::path & f, std::int64_t & size, std::int64_t & mtime_ns)
{
  struct ::stat st {};
  if (::stat(f.c_str(), &st) != 0) {
    return false;
  }
  const auto sec = static_cast<std::int64_t>(st.st_mtim.tv_sec);
  const auto nsec = static_cast<std::int64_t>(st.st_mtim.tv_nsec);
  if (sec > kMaxMtimeSec || sec < kMinMtimeSec || nsec < 0 || nsec >= kNsPerS) {
    return false;
  }
  size = static_cast<std::int64_t>(st.st_size);
  mtime_ns = sec * kNsPerS + nsec;
  return true;
}
}  // namespace

BagFingerprint bagFingerprint(const std::filesystem::path & bag, std::string * problem)
{
  namespace fs = std::filesystem;
  BagFingerprint fp;
  std::string first_problem;

  // Any way the walk fails to see the whole bag. Keeps the first reason: it is
  // the one closest to the cause, and the operator only needs one path to go
  // look at.
  auto incomplete = [&fp, &first_problem](const std::string & why) {
      fp.scan_complete = false;
      if (first_problem.empty()) {
        first_problem = why;
      }
    };

  auto consider = [&fp, &incomplete](const fs::path & f) {
      std::int64_t size = 0;
      std::int64_t mtime_ns = 0;
      if (!statFile(f, size, mtime_ns)) {
        incomplete("could not read size and timestamp of '" + f.string() + "'");
        return;
      }
      fp.size_bytes += size;
      // Seed from the first successful reading rather than from any in-range
      // constant, so no initializer can swallow a legitimate value.
      fp.mtime_ns = fp.mtime_valid ? std::max(fp.mtime_ns, mtime_ns) : mtime_ns;
      fp.mtime_valid = true;
    };

  // Classify one entry of the walk. `directory_entry::is_regular_file()` alone
  // is not enough: it follows symlinks, so a symlink to a directory answers
  // "not a regular file" with no error at all — and because the walk
  // deliberately does not follow directory symlinks, every byte behind such a
  // link used to be both invisible and *stable*, presenting as an
  // authoritative fingerprint. That is the #375 failure class one level up,
  // so it is reported rather than followed (a link can close a cycle a
  // recursive walk would never leave).
  //
  // The dividing line is not "regular file or not" but *what the walk knows*:
  // an entry definitively known not to be a regular file (a FIFO, socket,
  // device node, or a symlink that resolves to nothing) carries no bag bytes,
  // so leaving it out of the reading hides nothing and the walk is still
  // complete. Only content the walk could not see — an unresolvable type, a
  // directory it will not enumerate — clears `scan_complete`.
  auto classify = [&consider, &incomplete](const fs::path & p, const fs::directory_entry & entry) {
      std::error_code sym_ec;
      const fs::file_status sym = entry.symlink_status(sym_ec);
      if (sym_ec) {
        incomplete("could not determine the type of '" + p.string() + "': " + sym_ec.message());
        return;
      }
      if (fs::is_directory(sym)) {
        // A real subdirectory: the recursive walk descends into it, and a
        // failure to do so is reported by the increment below.
        return;
      }
      if (fs::is_regular_file(sym)) {
        consider(p);
        return;
      }
      if (!fs::is_symlink(sym)) {
        // A FIFO, socket or device node. Definitively not a regular file, so
        // it holds no bag bytes; the single-path branch below still rejects
        // one *nominated as* the bag, where there is nothing else to read.
        return;
      }
      std::error_code target_ec;
      const fs::file_status target = entry.status(target_ec);
      if (target_ec) {
        if (target_ec == std::errc::no_such_file_or_directory ||
          target_ec == std::errc::not_a_directory)
        {
          // Resolves to nothing, definitively: a dangling symlink contributes
          // no bytes and hides none, so the walk still saw the whole bag.
          // Should its target ever appear, `::stat` would count it and the
          // fingerprint would move — a re-index, exactly as wanted.
          return;
        }
        // Anything else (a symlink loop, an unreadable path component) leaves
        // the target's very existence unknown.
        incomplete(
          "could not resolve the symlink '" + p.string() + "': " + target_ec.message());
        return;
      }
      if (fs::is_directory(target)) {
        incomplete(
          "'" + p.string() + "' is a symlink to a directory, whose contents are not walked");
        return;
      }
      if (fs::is_regular_file(target)) {
        // `::stat` follows the link, so the target's size and mtime count.
        consider(p);
      }
      // Otherwise a symlink to a FIFO, socket or device: nothing to read, and
      // nothing hidden.
    };

  std::error_code ec;
  const bool is_dir = fs::is_directory(bag, ec);
  if (ec) {
    if (ec == std::errc::no_such_file_or_directory || ec == std::errc::not_a_directory) {
      // The commonest operator error, and it must read as one: "could not
      // determine what this is" sent whoever mistyped a bag URI looking for a
      // permission problem. Still not authoritative -- there is nothing to
      // fingerprint -- and the bag will fail to open a moment later anyway.
      incomplete("there is no bag at '" + bag.string() + "'");
    } else {
      // The type itself is unknown, so neither branch below can be trusted.
      incomplete("could not determine what '" + bag.string() + "' is: " + ec.message());
    }
  } else if (is_dir) {
    // error_code overloads throughout (#259): a broken symlink or unreadable
    // entry ends the walk cleanly instead of throwing and aborting the whole
    // run. Deliberately *not* `skip_permission_denied`: skipping made an
    // unreadable subdirectory vanish with no trace, leaving a stable partial
    // fingerprint that presented as authoritative and skipped the bag forever.
    fs::recursive_directory_iterator it(bag, ec), end;
    if (ec) {
      incomplete("could not open directory '" + bag.string() + "': " + ec.message());
    }
    while (!ec && it != end) {
      // Kept before the increment: when descending into an unreadable
      // subdirectory fails, this is the path that names the actual cause —
      // reporting the bag root instead would send the operator looking in the
      // wrong place.
      const fs::path current = it->path();
      classify(current, *it);
      it.increment(ec);
      if (ec) {
        // The walk stopped here — everything past this point is unseen.
        incomplete("directory walk stopped at '" + current.string() + "': " + ec.message());
      }
    }
  } else {
    std::error_code file_ec;
    const bool regular = fs::is_regular_file(bag, file_ec);
    if (file_ec) {
      // Checked before the answer: `file_ec` short-circuited away used to
      // report a failed check as a definitive "not a regular file".
      incomplete("could not determine what '" + bag.string() + "' is: " + file_ec.message());
    } else if (!regular) {
      // A FIFO, socket or device would otherwise fingerprint as size 0 with a
      // timestamp that moves every run.
      incomplete("'" + bag.string() + "' is not a regular file or a directory");
    } else {
      consider(bag);
    }
  }

  if (!fp.mtime_valid && first_problem.empty()) {
    // Nothing failed, so the walk really did see everything there is: the bag
    // simply holds no regular file to time. Saying "no readable timestamp"
    // here described a failure that did not happen.
    first_problem = "'" + bag.string() + "' holds no regular files, so it has no timestamp to read";
  }
  if (problem != nullptr) {
    *problem = first_problem;
  }
  return fp;
}

bool fingerprintMatches(
  const BagFingerprint & current, std::int64_t stored_size_bytes,
  std::int64_t stored_mtime_ns)
{
  // Trust first. Deliberately not expressible as a magic mtime value: the
  // ledger persists whatever the fingerprint carries, so an in-band sentinel
  // would compare equal to itself on the next run and skip the bag. A partial
  // walk is rejected here too — its size and mtime are stable, so they would
  // otherwise match their own stored copy for as long as the cause persists.
  if (!current.authoritative()) {
    return false;
  }
  return stored_size_bytes == current.size_bytes && stored_mtime_ns == current.mtime_ns;
}

}  // namespace marine_survey_index
