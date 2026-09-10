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
#include <string>
#include <system_error>

namespace marine_survey_index
{
namespace
{
constexpr std::int64_t kNsPerS = 1000000000LL;

// One ::stat per file: size and mtime together, so a file whose timestamp
// cannot be read contributes neither. Returns false if the stat failed.
bool statFile(const std::filesystem::path & f, std::int64_t & size, std::int64_t & mtime_ns)
{
  struct ::stat st {};
  if (::stat(f.c_str(), &st) != 0) {
    return false;
  }
  size = static_cast<std::int64_t>(st.st_size);
  mtime_ns = static_cast<std::int64_t>(st.st_mtim.tv_sec) * kNsPerS +
    static_cast<std::int64_t>(st.st_mtim.tv_nsec);
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

  std::error_code ec;
  const bool is_dir = fs::is_directory(bag, ec);
  if (ec) {
    // The type itself is unknown, so neither branch below can be trusted.
    incomplete("could not determine what '" + bag.string() + "' is: " + ec.message());
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
      std::error_code entry_ec;
      const bool regular = it->is_regular_file(entry_ec);
      if (entry_ec) {
        incomplete(
          "could not determine the type of '" + current.string() + "': " + entry_ec.message());
      } else if (regular) {
        consider(current);
      }
      it.increment(ec);
      if (ec) {
        // The walk stopped here — everything past this point is unseen.
        incomplete("directory walk stopped at '" + current.string() + "': " + ec.message());
      }
    }
  } else {
    std::error_code file_ec;
    if (!fs::is_regular_file(bag, file_ec) || file_ec) {
      // A FIFO, socket or device would otherwise fingerprint as size 0 with a
      // timestamp that moves every run.
      incomplete("'" + bag.string() + "' is not a regular file or a directory");
    } else {
      consider(bag);
    }
  }

  if (!fp.mtime_valid && first_problem.empty()) {
    first_problem = "no readable timestamp under '" + bag.string() + "'";
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
  if (!current.mtime_valid || !current.scan_complete) {
    return false;
  }
  return stored_size_bytes == current.size_bytes && stored_mtime_ns == current.mtime_ns;
}

}  // namespace marine_survey_index
