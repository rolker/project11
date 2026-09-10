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
#include <iostream>
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

BagFingerprint fingerprint(const std::filesystem::path & bag)
{
  namespace fs = std::filesystem;
  BagFingerprint fp;

  auto consider = [&fp](const fs::path & f) {
      std::int64_t size = 0;
      std::int64_t mtime_ns = 0;
      if (!statFile(f, size, mtime_ns)) {
        return;
      }
      fp.size_bytes += size;
      // Seed from the first successful reading rather than from any in-range
      // constant, so no initializer can swallow a legitimate value.
      fp.mtime_ns = fp.mtime_valid ? std::max(fp.mtime_ns, mtime_ns) : mtime_ns;
      fp.mtime_valid = true;
    };

  std::error_code ec;
  if (fs::is_directory(bag, ec)) {
    // error_code overloads: a broken symlink or unreadable entry sets ec and
    // ends the walk cleanly instead of throwing and aborting the whole run.
    for (fs::recursive_directory_iterator it(
        bag, fs::directory_options::skip_permission_denied, ec), end;
      !ec && it != end; it.increment(ec))
    {
      std::error_code entry_ec;
      if (it->is_regular_file(entry_ec)) {
        consider(it->path());
      }
    }
  } else {
    consider(bag);
  }

  if (!fp.mtime_valid) {
    // Loud, not silent: this bag will re-index on every run until the cause is
    // fixed, and the operator needs to know which bag and why.
    std::cerr << "warning: no readable timestamp under '" << bag.string()
              << "' - treating it as changed, so it re-indexes every run\n";
  }
  return fp;
}

bool fingerprintMatches(
  const BagFingerprint & current, std::int64_t stored_size_bytes,
  std::int64_t stored_mtime_ns)
{
  // Validity first. Deliberately not expressible as a magic mtime value: the
  // ledger persists whatever the fingerprint carries, so an in-band sentinel
  // would compare equal to itself on the next run and skip the bag.
  if (!current.mtime_valid) {
    return false;
  }
  return stored_size_bytes == current.size_bytes && stored_mtime_ns == current.mtime_ns;
}

std::int64_t fingerprintStoredMtime(const BagFingerprint & current)
{
  return current.mtime_valid ? current.mtime_ns : 0;
}

}  // namespace marine_survey_index
