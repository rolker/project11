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

#include <gtest/gtest.h>

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>

#include "marine_survey_index/bag_fingerprint.hpp"

namespace
{

// The incremental-skip ledger's identity function (issue #375). Before the fix
// every stored mtime was 0, because file_time_type's epoch is not the Unix
// epoch on libstdc++ and a negative reading never survived a max seeded at
// zero — so the unchanged test compared size alone and an in-place rewrite at
// an identical byte count was silently skipped.

constexpr std::int64_t kNsPerS = 1000000000LL;

class BagFingerprintTest : public testing::Test
{
protected:
  void SetUp() override
  {
    // Unique per test so a parallel or repeated run cannot collide.
    dir_ = std::filesystem::temp_directory_path() /
      ("msi_fingerprint_" + std::to_string(::getpid()) + "_" +
      testing::UnitTest::GetInstance()->current_test_info()->name());
    std::filesystem::remove_all(dir_);
    ASSERT_TRUE(std::filesystem::create_directories(dir_));
  }

  // Runs even when an ASSERT_* returned early, so the temp tree never leaks.
  // Permissions are restored first: a test that made a directory unreadable
  // would otherwise leave a tree `remove_all` cannot descend into.
  void TearDown() override
  {
    restorePermissions(dir_);
    std::error_code ec;
    std::filesystem::remove_all(dir_, ec);
  }

  static void restorePermissions(const std::filesystem::path & p)
  {
    std::error_code ec;
    if (!std::filesystem::is_directory(p, ec)) {
      return;
    }
    ::chmod(p.c_str(), 0700);
    for (std::filesystem::directory_iterator it(p, ec), end; !ec && it != end; it.increment(ec)) {
      restorePermissions(it->path());
    }
  }

  // Root ignores the permission bits these tests rely on, so the unreadable
  // routes are unobservable there. Skip rather than fail.
  static bool runningAsRoot() {return ::geteuid() == 0;}

  void write(const std::string & name, const std::string & bytes) const
  {
    std::ofstream out(dir_ / name, std::ios::binary | std::ios::trunc);
    out << bytes;
  }

  // Force an mtime instead of relying on filesystem timestamp granularity.
  // The two-argument setter is C++17 and converts the epoch correctly inside
  // libstdc++, so it is safe to use in a test policing the reader's epoch bug.
  void setMtime(const std::string & name, std::chrono::seconds ago) const
  {
    std::filesystem::last_write_time(
      dir_ / name, std::filesystem::file_time_type::clock::now() - ago);
  }

  static std::int64_t statMtimeNs(const std::filesystem::path & f)
  {
    struct ::stat st {};
    EXPECT_EQ(::stat(f.c_str(), &st), 0);
    return static_cast<std::int64_t>(st.st_mtim.tv_sec) * kNsPerS +
           static_cast<std::int64_t>(st.st_mtim.tv_nsec);
  }

  std::filesystem::path dir_;
};

// The bug itself: the stored value was 0 for every real file. Assert the exact
// value ::stat reports, not a tolerance window — a window would also accept a
// seconds-versus-nanoseconds scaling slip.
TEST_F(BagFingerprintTest, MtimeMatchesStatExactly)
{
  write("a.mcap", "0123456789");
  const auto fp = marine_survey_index::bagFingerprint(dir_);

  EXPECT_TRUE(fp.mtime_valid);
  EXPECT_EQ(fp.size_bytes, 10);
  EXPECT_EQ(fp.mtime_ns, statMtimeNs(dir_ / "a.mcap"));
  EXPECT_GT(fp.mtime_ns, 0) << "a real file's mtime is a positive Unix ns count";
}

// The actual stale-data scenario: same byte count, different content. The
// pre-fix fingerprint could not tell these apart, so the bag was never
// re-indexed.
TEST_F(BagFingerprintTest, InPlaceRewriteAtIdenticalSizeChangesFingerprint)
{
  write("a.mcap", "AAAAAAAAAA");
  setMtime("a.mcap", std::chrono::seconds(120));
  const auto before = marine_survey_index::bagFingerprint(dir_);

  write("a.mcap", "BBBBBBBBBB");
  setMtime("a.mcap", std::chrono::seconds(10));
  const auto after = marine_survey_index::bagFingerprint(dir_);

  EXPECT_EQ(after.size_bytes, before.size_bytes) << "the rewrite kept the size";
  EXPECT_NE(after.mtime_ns, before.mtime_ns);
  EXPECT_FALSE(
    marine_survey_index::fingerprintMatches(after, before.size_bytes, before.mtime_ns))
    << "a same-size rewrite must not read as unchanged";
}

// The newest mtime under the directory wins, and one unreadable file must not
// pin the result to an initializer.
TEST_F(BagFingerprintTest, TakesNewestMtimeAcrossMembers)
{
  write("a.mcap", "aaa");
  write("b.mcap", "bbbb");
  setMtime("a.mcap", std::chrono::seconds(600));
  setMtime("b.mcap", std::chrono::seconds(5));

  const auto fp = marine_survey_index::bagFingerprint(dir_);
  EXPECT_TRUE(fp.mtime_valid);
  EXPECT_EQ(fp.size_bytes, 7);
  EXPECT_EQ(fp.mtime_ns, statMtimeNs(dir_ / "b.mcap"));
}

// The ledger round-trip, at the level the decision is actually made.
TEST_F(BagFingerprintTest, MatchesOnlyWhenBothSizeAndMtimeAgree)
{
  write("a.mcap", "0123456789");
  const auto fp = marine_survey_index::bagFingerprint(dir_);

  EXPECT_TRUE(marine_survey_index::fingerprintMatches(fp, fp.size_bytes, fp.mtime_ns));
  EXPECT_FALSE(marine_survey_index::fingerprintMatches(fp, fp.size_bytes, fp.mtime_ns - 1))
    << "same size, different mtime: the case the pre-fix code could not see";
  EXPECT_FALSE(marine_survey_index::fingerprintMatches(fp, fp.size_bytes + 1, fp.mtime_ns));
  EXPECT_FALSE(marine_survey_index::fingerprintMatches(fp, 0, 0))
    << "an unindexed-looking row must not match a real fingerprint";
}

// The unreadable-mtime policy, including the second run. A sentinel carried in
// mtime_ns would be persisted by the write path and then compare equal to
// itself here, which is the hole the plan review caught.
TEST_F(BagFingerprintTest, UnreadableMtimeForcesReindexOnEveryRun)
{
  // No regular files, so no timestamp is readable anywhere under the bag.
  std::string problem;
  const auto fp = marine_survey_index::bagFingerprint(dir_, &problem);
  ASSERT_FALSE(fp.mtime_valid);
  EXPECT_TRUE(fp.scan_complete) << "nothing failed: there is simply nothing to time";
  EXPECT_NE(problem.find("no regular files"), std::string::npos)
    << "the reason must not describe a failure that did not happen: " << problem;

  // What the ledger writes is the fingerprint's own field: the struct's
  // invariant keeps it 0 while `mtime_valid` is false, so nothing out of range
  // can be persisted and later compare equal to a fresh reading.
  const std::int64_t stored = fp.mtime_ns;

  // Run 1: whatever is stored, the bag must not read as unchanged.
  EXPECT_FALSE(marine_survey_index::fingerprintMatches(fp, fp.size_bytes, stored));
  // Run 2: the ledger now holds exactly what run 1 wrote. Still not unchanged.
  const auto again = marine_survey_index::bagFingerprint(dir_);
  EXPECT_FALSE(marine_survey_index::fingerprintMatches(again, again.size_bytes, stored))
    << "an in-band sentinel would round-trip and skip the bag on the second run";

  // The persisted value is a perfectly reachable real mtime (`touch -d @0`
  // produces exactly it), so it is *not* what makes this safe -- only the
  // validity gate in fingerprintMatches() is. Asserted because the struct
  // invariant the write path relies on is that mtime_ns stays 0 here.
  EXPECT_EQ(stored, 0);
}

// An unreadable subdirectory: one route to a partial walk. Permission-based,
// so it cannot run as root — the *gate* it exercises is guarded root-observably
// by SymlinkedSubdirectoryHidesContentButIsNotTrusted and
// IncompleteWalkIsRejectedBeforeAnythingIsCompared below, which is what keeps
// `scan_complete` defended in CI (both hosted CI and `ci_local.sh` run as
// root, so a test that skips there defends nothing).
//
// Note this route cannot assert non-degeneracy: whether the visible file is
// seen at all depends on readdir order, since the failed descent ends the
// walk. That is exactly why it cannot be the only guard.
TEST_F(BagFingerprintTest, PartialWalkIsNotAuthoritative)
{
  if (runningAsRoot()) {
    GTEST_SKIP() << "root ignores the directory permissions this test relies on";
  }
  write("a.mcap", "0123456789");
  const auto sub = dir_ / "sub";
  ASSERT_TRUE(std::filesystem::create_directory(sub));
  {
    std::ofstream out(sub / "b.mcap", std::ios::binary | std::ios::trunc);
    out << "AAAA";
  }
  ASSERT_EQ(::chmod(sub.c_str(), 0000), 0);

  std::string problem;
  const auto before = marine_survey_index::bagFingerprint(dir_, &problem);
  EXPECT_FALSE(before.scan_complete) << "an unreadable subdirectory is not a complete walk";
  EXPECT_NE(problem.find("sub"), std::string::npos) << "the reason must name the offending path";
  EXPECT_FALSE(marine_survey_index::fingerprintMatches(before, before.size_bytes, before.mtime_ns))
    << "a partial walk must never satisfy the unchanged test, not even against itself";

  // Change the content hidden behind the unreadable directory. This is the
  // reproduction: the visible size and mtime do not move at all.
  ASSERT_EQ(::chmod(sub.c_str(), 0700), 0);
  {
    std::ofstream out(sub / "b.mcap", std::ios::binary | std::ios::trunc);
    out << "BBBBBBBB";
  }
  ASSERT_EQ(::chmod(sub.c_str(), 0000), 0);

  const auto after = marine_survey_index::bagFingerprint(dir_);
  EXPECT_EQ(after.size_bytes, before.size_bytes) << "the change is invisible to the walk";
  EXPECT_EQ(after.mtime_ns, before.mtime_ns) << "the change is invisible to the walk";
  EXPECT_FALSE(marine_survey_index::fingerprintMatches(after, before.size_bytes, before.mtime_ns))
    << "the bag changed; only scan_complete can tell, so it must force a re-index";
}

// The production-reachable route: `scanForBags` nominates a bag directory it
// can see the name of but not descend into (mode 0111 — traversable, not
// listable), so the walk cannot enumerate a single member.
TEST_F(BagFingerprintTest, UnlistableBagDirectoryIsNotAuthoritative)
{
  if (runningAsRoot()) {
    GTEST_SKIP() << "root ignores the directory permissions this test relies on";
  }
  write("a.mcap", "0123456789");
  ASSERT_EQ(::chmod(dir_.c_str(), 0111), 0);

  std::string problem;
  const auto fp = marine_survey_index::bagFingerprint(dir_, &problem);
  ASSERT_EQ(::chmod(dir_.c_str(), 0700), 0);

  EXPECT_FALSE(fp.scan_complete);
  EXPECT_FALSE(fp.mtime_valid) << "nothing was enumerated, so nothing was timed";
  EXPECT_FALSE(problem.empty());
  EXPECT_FALSE(marine_survey_index::fingerprintMatches(fp, fp.size_bytes, fp.mtime_ns));
}

// An mtime past ~2262 overflows `tv_sec * 1e9` in int64_t — signed-overflow UB,
// and the wrapped value would index as an ordinary timestamp. Reachable from a
// corrupt inode or a wrong host clock, not only a deliberate `touch`.
TEST_F(BagFingerprintTest, UnrepresentableMtimeIsNotTrusted)
{
  write("a.mcap", "0123456789");
  const auto f = dir_ / "a.mcap";

  // 2500-01-01T00:00:00Z. Not every filesystem can store it; if this one
  // clamped or refused, there is nothing to police here.
  constexpr std::int64_t kYear2500Sec = 16725225600LL;
  const struct ::timespec times[2] = {{kYear2500Sec, 0}, {kYear2500Sec, 0}};
  if (::utimensat(AT_FDCWD, f.c_str(), times, 0) != 0) {
    GTEST_SKIP() << "this filesystem refuses a year-2500 mtime";
  }
  struct ::stat st {};
  ASSERT_EQ(::stat(f.c_str(), &st), 0);
  constexpr std::int64_t kMaxMtimeSec = (INT64_MAX - (kNsPerS - 1)) / kNsPerS;
  if (static_cast<std::int64_t>(st.st_mtim.tv_sec) <= kMaxMtimeSec) {
    GTEST_SKIP() << "this filesystem clamped the mtime back into range";
  }

  std::string problem;
  const auto fp = marine_survey_index::bagFingerprint(dir_, &problem);
  EXPECT_FALSE(fp.mtime_valid) << "an unrepresentable timestamp is not a reading";
  EXPECT_FALSE(fp.scan_complete);
  EXPECT_EQ(fp.mtime_ns, 0) << "no wrapped value may reach the ledger";
  EXPECT_EQ(fp.size_bytes, 0) << "a file whose timestamp is unusable contributes no bytes either";
  EXPECT_FALSE(problem.empty());
  EXPECT_FALSE(marine_survey_index::fingerprintMatches(fp, fp.size_bytes, fp.mtime_ns));
}

// A single-path bag that is not a regular file would fingerprint as size 0
// with a timestamp that moves on every read.
TEST_F(BagFingerprintTest, NonRegularSinglePathIsNotAuthoritative)
{
  const auto fifo = dir_ / "a.fifo";
  if (::mkfifo(fifo.c_str(), 0600) != 0) {
    GTEST_SKIP() << "this filesystem does not support FIFOs";
  }

  std::string problem;
  const auto fp = marine_survey_index::bagFingerprint(fifo, &problem);
  EXPECT_FALSE(fp.scan_complete);
  EXPECT_FALSE(fp.mtime_valid);
  EXPECT_FALSE(problem.empty());
  EXPECT_FALSE(marine_survey_index::fingerprintMatches(fp, fp.size_bytes, fp.mtime_ns));
}

// The migration case this PR creates: every pre-existing ledger row holds
// `mtime_ns = 0` (the bug), so each bag must re-index exactly once.
TEST_F(BagFingerprintTest, LegacyZeroMtimeRowForcesOneReindex)
{
  write("a.mcap", "0123456789");
  const auto fp = marine_survey_index::bagFingerprint(dir_);
  ASSERT_TRUE(fp.mtime_valid);

  EXPECT_FALSE(marine_survey_index::fingerprintMatches(fp, fp.size_bytes, 0))
    << "a legacy mtime_ns = 0 row must not read as unchanged";
  // ...and after that one re-index writes the real mtime, it settles.
  EXPECT_TRUE(marine_survey_index::fingerprintMatches(fp, fp.size_bytes, fp.mtime_ns));
}

// A trustworthy fingerprint reports no problem at all — the CLI's warning (and
// its non-zero exit) keys on this string being empty.
TEST_F(BagFingerprintTest, ReadableBagReportsNoProblem)
{
  write("a.mcap", "0123456789");
  std::string problem = "not cleared";
  const auto fp = marine_survey_index::bagFingerprint(dir_, &problem);
  EXPECT_TRUE(fp.mtime_valid);
  EXPECT_TRUE(fp.scan_complete);
  EXPECT_TRUE(problem.empty()) << "unexpected problem: " << problem;
}


// The round-2 must-fix, and the root-observable guard on `scan_complete`.
//
// A symlink to a directory inside a bag is not followed by the walk (a link can
// close a cycle a recursive walk would never leave), and
// `directory_entry::is_regular_file()` follows the link, so it answered "not a
// regular file" with **no error at all**: every byte behind the link was
// invisible *and* stable, and the bag fingerprinted as fully authoritative.
// The #375 failure class one level up.
//
// This is also the test that makes the `scan_complete` conjunct in
// `fingerprintMatches()` load-bearing: it reaches the genuinely interesting
// flag combination (`mtime_valid` true, `scan_complete` false) with a non-zero
// size and a real timestamp, so nothing here compares 0 to 0.
TEST_F(BagFingerprintTest, SymlinkedSubdirectoryHidesContentButIsNotTrusted)
{
  const auto bag = dir_ / "bag";
  const auto outside = dir_ / "outside";
  ASSERT_TRUE(std::filesystem::create_directory(bag));
  ASSERT_TRUE(std::filesystem::create_directory(outside));
  {
    std::ofstream out(bag / "a.mcap", std::ios::binary | std::ios::trunc);
    out << "0123456789";
  }
  {
    std::ofstream out(outside / "b.mcap", std::ios::binary | std::ios::trunc);
    out << "AAAA";
  }
  std::error_code link_ec;
  std::filesystem::create_directory_symlink(outside, bag / "sub", link_ec);
  ASSERT_FALSE(link_ec) << "this filesystem refuses directory symlinks: " << link_ec.message();

  std::string problem;
  const auto before = marine_survey_index::bagFingerprint(bag, &problem);
  EXPECT_TRUE(before.mtime_valid) << "the visible member was timed";
  EXPECT_EQ(before.size_bytes, 10) << "only the visible member is counted";
  EXPECT_GT(before.mtime_ns, 0);
  EXPECT_FALSE(before.scan_complete) << "content behind the symlink was never walked";
  EXPECT_FALSE(before.authoritative());
  EXPECT_NE(problem.find("sub"), std::string::npos) << "the reason must name the offending path";
  EXPECT_FALSE(marine_survey_index::fingerprintMatches(before, before.size_bytes, before.mtime_ns))
    << "a partial walk must never satisfy the unchanged test, not even against itself";

  // The reproduction: change the hidden content, at a different byte count and
  // a newer timestamp. Neither moves the fingerprint at all.
  {
    std::ofstream out(outside / "b.mcap", std::ios::binary | std::ios::trunc);
    out << "BBBBBBBBBBBBBBBBBBBB";
  }
  const auto after = marine_survey_index::bagFingerprint(bag);
  EXPECT_EQ(after.size_bytes, before.size_bytes) << "the change is invisible to the walk";
  EXPECT_EQ(after.mtime_ns, before.mtime_ns) << "the change is invisible to the walk";
  EXPECT_TRUE(after.mtime_valid) << "so the mtime gate cannot be what rejects this";
  EXPECT_FALSE(marine_survey_index::fingerprintMatches(after, before.size_bytes, before.mtime_ns))
    << "the bag changed; only scan_complete can tell, so it must force a re-index";
}

// The gate itself, over the flag combinations, with no filesystem involved:
// `mtime_valid` and `scan_complete` are independent, and each on its own must
// be enough to refuse the comparison. Constructed directly, because the
// combination that matters is otherwise reachable only through a partial walk.
TEST_F(BagFingerprintTest, IncompleteWalkIsRejectedBeforeAnythingIsCompared)
{
  marine_survey_index::BagFingerprint fp;
  fp.size_bytes = 4096;
  fp.mtime_ns = 1700000000123456789LL;

  fp.mtime_valid = true;
  fp.scan_complete = true;
  EXPECT_TRUE(fp.authoritative());
  ASSERT_TRUE(marine_survey_index::fingerprintMatches(fp, fp.size_bytes, fp.mtime_ns))
    << "the control: matching values on a trustworthy fingerprint do read as unchanged";

  // A real timestamp, read from a walk that did not see the whole bag: the
  // values match their stored copy exactly, and must still be refused.
  fp.scan_complete = false;
  EXPECT_FALSE(fp.authoritative());
  EXPECT_FALSE(marine_survey_index::fingerprintMatches(fp, fp.size_bytes, fp.mtime_ns))
    << "an incomplete walk is stable, so matching its own stored copy proves nothing";

  fp.mtime_valid = false;
  fp.scan_complete = true;
  EXPECT_FALSE(fp.authoritative());
  EXPECT_FALSE(marine_survey_index::fingerprintMatches(fp, fp.size_bytes, fp.mtime_ns))
    << "no timestamp was read, so the mtime half of the comparison is meaningless";

  fp.mtime_valid = false;
  fp.scan_complete = false;
  EXPECT_FALSE(fp.authoritative());
  EXPECT_FALSE(marine_survey_index::fingerprintMatches(fp, fp.size_bytes, fp.mtime_ns));
}

// A symlink whose target's existence cannot be established at all (a loop:
// ELOOP, which root is not exempt from either) hides whatever it points at, so
// it is a partial walk. This is also the root-observable case of a member the
// directory can list but the walk cannot read.
TEST_F(BagFingerprintTest, UnresolvableSymlinkIsNotAuthoritative)
{
  write("a.mcap", "0123456789");
  std::error_code link_ec;
  std::filesystem::create_symlink("loop", dir_ / "loop", link_ec);
  ASSERT_FALSE(link_ec) << "this filesystem refuses symlinks: " << link_ec.message();

  std::string problem;
  const auto fp = marine_survey_index::bagFingerprint(dir_, &problem);
  EXPECT_FALSE(fp.scan_complete);
  EXPECT_FALSE(fp.authoritative());
  EXPECT_NE(problem.find("loop"), std::string::npos);
  EXPECT_FALSE(marine_survey_index::fingerprintMatches(fp, fp.size_bytes, fp.mtime_ns));
}

// The other side of that line, and the reason it is drawn where it is: an entry
// the walk knows *definitively* carries no bag bytes hides nothing, so it must
// not cost the bag a permanent re-index. A dangling symlink is the common one
// (a moved external drive, a pruned scratch tree).
TEST_F(BagFingerprintTest, DanglingSymlinkAndFifoDoNotSpoilTheWalk)
{
  write("a.mcap", "0123456789");
  std::error_code link_ec;
  std::filesystem::create_symlink(dir_ / "gone.mcap", dir_ / "dangling.mcap", link_ec);
  ASSERT_FALSE(link_ec) << "this filesystem refuses symlinks: " << link_ec.message();
  const bool have_fifo = ::mkfifo((dir_ / "a.fifo").c_str(), 0600) == 0;

  std::string problem = "not cleared";
  const auto fp = marine_survey_index::bagFingerprint(dir_, &problem);
  EXPECT_TRUE(fp.scan_complete) << "unexpected problem: " << problem;
  EXPECT_TRUE(fp.mtime_valid);
  EXPECT_EQ(fp.size_bytes, 10) << "neither entry contributes bytes";
  EXPECT_TRUE(problem.empty()) << "unexpected problem: " << problem;
  EXPECT_TRUE(marine_survey_index::fingerprintMatches(fp, fp.size_bytes, fp.mtime_ns))
    << "nothing is hidden, so this bag is skippable when it has not changed";
  if (!have_fifo) {
    GTEST_SUCCEED() << "this filesystem does not support FIFOs; the symlink half still ran";
  }
}

// A symlink to a regular file is followed by ::stat, so the target's bytes and
// timestamp count -- and a change to the target therefore *is* visible, which
// is why such a link needs no `scan_complete` flag.
TEST_F(BagFingerprintTest, SymlinkToRegularFileIsFollowed)
{
  const auto bag = dir_ / "bag";
  const auto outside = dir_ / "outside";
  ASSERT_TRUE(std::filesystem::create_directory(bag));
  ASSERT_TRUE(std::filesystem::create_directory(outside));
  {
    std::ofstream out(outside / "b.mcap", std::ios::binary | std::ios::trunc);
    out << "AAAA";
  }
  std::error_code link_ec;
  std::filesystem::create_symlink(outside / "b.mcap", bag / "b.mcap", link_ec);
  ASSERT_FALSE(link_ec) << "this filesystem refuses symlinks: " << link_ec.message();

  std::string problem = "not cleared";
  const auto before = marine_survey_index::bagFingerprint(bag, &problem);
  EXPECT_TRUE(before.scan_complete) << "unexpected problem: " << problem;
  EXPECT_EQ(before.size_bytes, 4) << "the target's size, through the link";

  {
    std::ofstream out(outside / "b.mcap", std::ios::binary | std::ios::trunc);
    out << "BBBBBBBB";
  }
  const auto after = marine_survey_index::bagFingerprint(bag);
  EXPECT_EQ(after.size_bytes, 8);
  EXPECT_FALSE(marine_survey_index::fingerprintMatches(after, before.size_bytes, before.mtime_ns));
}

// The documented policy for a member that can be listed but never read: the bag
// re-indexes on every run until the cause is fixed. Permission-based, so it
// skips as root; the same `statFile` failure branch is reached root-observably
// by UnrepresentableMtimeIsNotTrusted above.
TEST_F(BagFingerprintTest, UnreadableMemberOfAListableDirectoryReindexesEveryRun)
{
  if (runningAsRoot()) {
    GTEST_SKIP() << "root ignores the directory permissions this test relies on";
  }
  const auto sub = dir_ / "sub";
  ASSERT_TRUE(std::filesystem::create_directory(sub));
  {
    std::ofstream out(sub / "b.mcap", std::ios::binary | std::ios::trunc);
    out << "AAAA";
  }
  // Readable, so the member is enumerated -- but not traversable, so ::stat of
  // it fails: the walk sees the name and nothing else.
  ASSERT_EQ(::chmod(sub.c_str(), 0444), 0);

  std::string problem;
  const auto fp = marine_survey_index::bagFingerprint(dir_, &problem);
  ASSERT_EQ(::chmod(sub.c_str(), 0700), 0);

  EXPECT_FALSE(fp.scan_complete);
  EXPECT_NE(problem.find("b.mcap"), std::string::npos)
    << "the reason must name the member, not the bag root: " << problem;
  EXPECT_FALSE(marine_survey_index::fingerprintMatches(fp, fp.size_bytes, fp.mtime_ns));
}


// The invariant the CLI would otherwise have to trust: the reported string and
// the trust flags always agree. The CLI branches on `authoritative()` and uses
// the string only for the message, so no empty-string convention is
// load-bearing across the library boundary — but the two must not disagree
// either, or an operator gets a warning with nothing to act on (or a silent
// permanent re-index).
TEST_F(BagFingerprintTest, ProblemStringAndTrustFlagsAlwaysAgree)
{
  const auto check = [](const std::filesystem::path & bag, const char * what) {
      std::string problem = "not cleared";
      const auto fp = marine_survey_index::bagFingerprint(bag, &problem);
      EXPECT_EQ(fp.authoritative(), problem.empty())
        << what << ": authoritative=" << fp.authoritative() << " problem='" << problem << "'";
    };

  const auto clean = dir_ / "clean";
  ASSERT_TRUE(std::filesystem::create_directory(clean));
  {
    std::ofstream out(clean / "a.mcap", std::ios::binary | std::ios::trunc);
    out << "0123456789";
  }
  check(clean, "a readable bag");

  const auto empty = dir_ / "empty";
  ASSERT_TRUE(std::filesystem::create_directory(empty));
  check(empty, "a bag with no regular files");

  check(dir_ / "absent", "a bag that does not exist");

  std::error_code link_ec;
  std::filesystem::create_symlink("loop", clean / "loop", link_ec);
  if (!link_ec) {
    check(clean, "a bag holding an unresolvable symlink");
    std::filesystem::create_directory_symlink(empty, clean / "sub", link_ec);
    if (!link_ec) {
      check(clean, "a bag holding a symlinked subdirectory");
    }
  }
}

}  // namespace
