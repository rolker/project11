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
///
/// `::stat`'s `st_mtim` field is POSIX.1-2008; macOS spells the same field
/// `st_mtimespec`. This package targets Linux (ROS 2 Jazzy on Ubuntu), so no
/// portability shim is carried — a macOS port would need one here, and that is
/// the whole of the trade-off `::stat` buys over `last_write_time`.

#include <cstdint>
#include <filesystem>
#include <string>

namespace marine_survey_index
{

/// @brief Bag identity: total regular-file bytes plus the newest mtime beneath
///   it, together with whether the walk that produced them saw everything.
///
/// Two independent trust flags, because they have independent causes and both
/// must hold before a skip decision may be made:
///
/// - `mtime_valid` — at least one timestamp was read. It is a separate field
///   rather than a magic `mtime_ns` value on purpose: an in-band sentinel would
///   be persisted by the ledger's write path and then compare equal to itself
///   on the next run, silently restoring the skip it was meant to prevent.
/// - `scan_complete` — every entry beneath the bag was either read, or is
///   *definitively* not a regular file and so holds no bag bytes. A partial
///   walk (an unreadable subdirectory, a symlinked subdirectory the walk will
///   not follow, an entry whose type cannot be resolved, a file that vanished
///   mid-walk, an unrepresentable timestamp) yields a *stable* size and mtime
///   that would otherwise present as authoritative and skip a changed bag
///   forever — the same failure class as #375, one level up.
///
/// Invariant: `mtime_ns == 0` whenever `mtime_valid` is false. Nothing carries
/// an out-of-range sentinel, so the ledger can persist `mtime_ns` verbatim.
struct BagFingerprint
{
  std::int64_t size_bytes = 0;
  std::int64_t mtime_ns = 0;
  bool mtime_valid = false;
  bool scan_complete = true;

  /// @brief May this fingerprint be compared against a stored one at all?
  ///
  /// The one place the two flags are conjuncted. Callers ask this instead of
  /// assembling `mtime_valid && scan_complete` themselves: getting that
  /// conjunction wrong reads as "authoritative" — the silent-skip direction —
  /// and a dropped conjunct is invisible at the call site.
  bool authoritative() const {return mtime_valid && scan_complete;}
};

/// @brief Fingerprint a bag directory (recursively) or a single bag file.
///
/// Size and mtime come from the same `::stat` call per file, so a file whose
/// timestamp cannot be read contributes neither. Every way the walk can fail
/// to see the whole bag — an unreadable directory, a symlink to a directory
/// (not followed: a link can close a cycle a recursive walk would never
/// leave), a failed `::stat`, a symlink whose target's existence cannot be
/// established, an entry whose type cannot be determined, a non-regular single
/// path, or an mtime outside the range `int64_t` nanoseconds can represent —
/// clears `scan_complete`; reading no timestamp at all also leaves
/// `mtime_valid` false. Either way `fingerprintMatches()` then refuses to call
/// the bag unchanged.
///
/// An entry the walk knows *definitively* carries no bag bytes — a FIFO,
/// socket or device node, or a symlink that resolves to nothing — is skipped
/// without clearing `scan_complete`: nothing is hidden by leaving it out, and
/// a dangling symlink is not worth a permanent re-index.
///
/// This function is silent by design: it is exported from the core library and
/// linked into GUI processes (`marine_perception_tools`) where stderr is
/// invisible. It reports the fact through the returned flags, and the reason
/// through @p problem, leaving the diagnostic to the call site.
///
/// @param bag Bag directory or single bag file.
/// @param problem Optional out-param. Set to a human-readable description of
///   the first reason the fingerprint is not authoritative (naming the
///   offending path), or cleared to the empty string when it is.
/// @return The fingerprint, always — failures are reported in its flags, never
///   thrown, so one bad bag cannot abort a whole indexing run.
BagFingerprint bagFingerprint(const std::filesystem::path & bag, std::string * problem = nullptr);

/// @brief The ledger's unchanged test: does @p current match what was stored?
///
/// @param current Freshly computed fingerprint.
/// @param stored_size_bytes The ledger row's `size_bytes`.
/// @param stored_mtime_ns The ledger row's `mtime_ns`.
/// @return True only if the fingerprint is `authoritative()` *and* both stored
///   values match it. An untrustworthy
///   fingerprint is rejected **before** anything is compared, so a bag whose
///   timestamp or whose full contents could not be read is re-indexed rather
///   than skipped on a partial reading.
bool fingerprintMatches(
  const BagFingerprint & current, std::int64_t stored_size_bytes,
  std::int64_t stored_mtime_ns);

}  // namespace marine_survey_index

#endif  // MARINE_SURVEY_INDEX__BAG_FINGERPRINT_HPP_
