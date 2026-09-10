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

#ifndef MARINE_SURVEY_INDEX__BAG_FINGERPRINT_HPP_
#define MARINE_SURVEY_INDEX__BAG_FINGERPRINT_HPP_

/// @file
/// @brief Bag identity for the incremental-skip ledger.
///
/// A bag is identified by its total regular-file bytes and the newest mtime
/// under its directory (or of the single file). A re-run over an unchanged bag
/// is a no-op; a changed bag is re-indexed.
///
/// The timestamp is read through `::stat` rather than
/// `std::filesystem::last_write_time`. `file_time_type`'s epoch is not the
/// Unix epoch on libstdc++ (it is 2174-01-01), so reading
/// `time_since_epoch()` raw yields a large negative value for any real file,
/// which is neither comparable to anything else on the system nor survivable
/// through a max accumulator seeded at zero. That combination is what made
/// every stored `mtime_ns` zero and the skip test size-only (issue #375).

#include <cstdint>
#include <filesystem>

namespace marine_survey_index
{

/// Bag identity: total regular-file bytes plus the newest mtime beneath it.
///
/// `mtime_valid` carries whether a timestamp was read at all. It is a separate
/// field on purpose: an in-band sentinel would be persisted by the ledger's
/// write path and then compare equal to itself on the next run, silently
/// restoring the skip it was meant to prevent.
struct BagFingerprint
{
  std::int64_t size_bytes = 0;
  std::int64_t mtime_ns = 0;
  bool mtime_valid = false;
};

/// Fingerprint a bag directory (recursively) or a single bag file.
///
/// Size and mtime come from the same `::stat` call per file, so a file whose
/// timestamp cannot be read does not contribute its bytes either. When no
/// timestamp could be read anywhere under `bag` — every `::stat` failed, or
/// there are no regular files — `mtime_valid` is false and a warning naming
/// `bag` is written to stderr.
BagFingerprint fingerprint(const std::filesystem::path & bag);

/// The ledger's unchanged test: does `current` match what the ledger stored?
///
/// Returns false whenever `current.mtime_valid` is false, **before** comparing
/// anything — an unreadable timestamp forces a re-index rather than passing the
/// test. Otherwise both size and mtime must match.
bool fingerprintMatches(
  const BagFingerprint & current, std::int64_t stored_size_bytes,
  std::int64_t stored_mtime_ns);

/// The value to persist for `current`: its mtime, or 0 when none was readable.
///
/// The persisted value is never load-bearing — `fingerprintMatches` rejects an
/// invalid fingerprint before reading it — but it must not be an out-of-range
/// sentinel that could later compare equal to a fresh reading.
std::int64_t fingerprintStoredMtime(const BagFingerprint & current);

}  // namespace marine_survey_index

#endif  // MARINE_SURVEY_INDEX__BAG_FINGERPRINT_HPP_
