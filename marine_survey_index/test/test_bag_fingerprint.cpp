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

#include <sys/stat.h>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>

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
  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(dir_, ec);
  }

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
  const auto fp = marine_survey_index::fingerprint(dir_);

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
  const auto before = marine_survey_index::fingerprint(dir_);

  write("a.mcap", "BBBBBBBBBB");
  setMtime("a.mcap", std::chrono::seconds(10));
  const auto after = marine_survey_index::fingerprint(dir_);

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

  const auto fp = marine_survey_index::fingerprint(dir_);
  EXPECT_TRUE(fp.mtime_valid);
  EXPECT_EQ(fp.size_bytes, 7);
  EXPECT_EQ(fp.mtime_ns, statMtimeNs(dir_ / "b.mcap"));
}

// The ledger round-trip, at the level the decision is actually made.
TEST_F(BagFingerprintTest, MatchesOnlyWhenBothSizeAndMtimeAgree)
{
  write("a.mcap", "0123456789");
  const auto fp = marine_survey_index::fingerprint(dir_);

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
  const auto fp = marine_survey_index::fingerprint(dir_);
  ASSERT_FALSE(fp.mtime_valid);

  const std::int64_t stored = marine_survey_index::fingerprintStoredMtime(fp);

  // Run 1: whatever is stored, the bag must not read as unchanged.
  EXPECT_FALSE(marine_survey_index::fingerprintMatches(fp, fp.size_bytes, stored));
  // Run 2: the ledger now holds exactly what run 1 wrote. Still not unchanged.
  const auto again = marine_survey_index::fingerprint(dir_);
  EXPECT_FALSE(marine_survey_index::fingerprintMatches(again, again.size_bytes, stored))
    << "an in-band sentinel would round-trip and skip the bag on the second run";

  // And the persisted value must stay inside the range a real mtime occupies,
  // so it can never be mistaken for one.
  EXPECT_EQ(stored, 0);
}

}  // namespace
