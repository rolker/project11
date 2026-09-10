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
#include <sys/wait.h>
#include <unistd.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>

#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/storage_options.hpp"

namespace
{

// The indexer's exit-status contract, exercised by running the real binary.
//
// This exists because the contract lives in `main()`: no library call can
// reach it, and the one condition it most needs to police -- a bag that could
// not be opened at all -- used to land in no summary bucket and exit 0, which
// the contract's own comment says must not happen. A scheduler or a
// `set -euo pipefail` store build is the consumer, so the status is the
// product here, not a detail.
//
//   0  every nominated bag indexed (or skipped), every fingerprint trustworthy
//   1  the index is incomplete (a bag failed, or a --scan tree was not fully
//      enumerated)
//   2  usage error
//   3  index complete, but a bag cannot be fingerprinted authoritatively

struct RunResult
{
  int status = -1;
  std::string output;
};

class IndexerExitStatusTest : public testing::Test
{
protected:
  void SetUp() override
  {
    dir_ = std::filesystem::temp_directory_path() /
      ("msi_exit_" + std::to_string(::getpid()) + "_" +
      testing::UnitTest::GetInstance()->current_test_info()->name());
    std::filesystem::remove_all(dir_);
    ASSERT_TRUE(std::filesystem::create_directories(dir_));
  }

  void TearDown() override
  {
    // Permissions first: a test that made a directory unreadable must not
    // leave it that way, or the cleanup silently fails and the next run
    // inherits it.
    restorePermissions(dir_);
    std::error_code ec;
    std::filesystem::remove_all(dir_, ec);
  }

  static void restorePermissions(const std::filesystem::path & p)
  {
    std::error_code ec;
    if (!std::filesystem::is_directory(std::filesystem::symlink_status(p, ec)) || ec) {
      return;
    }
    ::chmod(p.c_str(), 0700);
    std::filesystem::directory_iterator it(p, ec), end;
    for (; !ec && it != end; it.increment(ec)) {
      restorePermissions(it->path());
    }
  }

  // Root ignores the directory permission bits the walk-abandonment tests
  // rely on: only a failed `opendir` abandons the walk, and root is not
  // denied one on a local filesystem. The same fold is covered
  // root-observably by the symlinked-subtree and ELOOP tests below.
  static bool runningAsRoot() {return ::geteuid() == 0;}

  // Single-quote for the shell that `popen` runs, so a space or metacharacter
  // in a temp or build path cannot become argv structure.
  static std::string quote(const std::string & s)
  {
    std::string out = "'";
    for (const char c : s) {
      if (c == '\'') {
        out += "'\\''";
      } else {
        out += c;
      }
    }
    return out + "'";
  }

  RunResult runIndexer(const std::string & args) const
  {
    return runIndexerWithDb((dir_ / "index.db").string(), args);
  }

  RunResult runIndexerWithDb(const std::string & db, const std::string & args) const
  {
    // stderr merged in: the summary line and the warnings both go there.
    // Everything the test interpolates is quoted: a build path or a TMPDIR
    // containing a space would otherwise mis-parse into extra argv words and
    // the test would be measuring the wrong invocation.
    const std::string cmd =
      quote(SURVEY_INDEX_BAG_EXE) + " --db " + quote(db) + " " + args + " 2>&1";
    RunResult result;
    FILE * pipe = ::popen(cmd.c_str(), "r");
    if (pipe == nullptr) {
      ADD_FAILURE() << "could not run " << cmd;
      return result;
    }
    char buffer[512];
    while (::fgets(buffer, sizeof(buffer), pipe) != nullptr) {
      result.output += buffer;
    }
    const int rc = ::pclose(pipe);
    // 127 is the shell's "could not execute": a real failure, never a skip --
    // a test that shrugged at that would defend nothing.
    EXPECT_TRUE(WIFEXITED(rc)) << "the indexer did not exit normally: " << result.output;
    result.status = WIFEXITED(rc) ? WEXITSTATUS(rc) : -1;
    return result;
  }

  // A directory that looks like a bag to the fingerprint and the scanner but
  // that no storage plugin can open.
  std::filesystem::path makeUnopenableBag(const std::string & name) const
  {
    const auto bag = dir_ / name;
    std::filesystem::create_directories(bag);
    std::ofstream out(bag / "metadata.yaml", std::ios::trunc);
    out << "this is not: [valid rosbag2 metadata\n";
    return bag;
  }

  // A real, empty rosbag2 bag: openable, indexable, zero pings.
  std::filesystem::path makeEmptyBag(const std::string & name) const
  {
    const auto bag = dir_ / name;
    rosbag2_cpp::Writer writer;
    rosbag2_storage::StorageOptions so;
    so.uri = bag.string();
    writer.open(so);
    writer.close();
    return bag;
  }

  std::filesystem::path dir_;
};

// The must-fix: a bag the reader cannot open is a durable condition -- that bag
// is absent from the index -- so it must be counted and must not exit 0.
TEST_F(IndexerExitStatusTest, UnopenableBagIsCountedAndExitsNonZero)
{
  const auto a = makeUnopenableBag("bag_a");
  const auto b = makeUnopenableBag("bag_b");

  const auto run = runIndexer(quote(a.string()) + " " + quote(b.string()));
  EXPECT_EQ(run.status, 1) << run.output;
  EXPECT_NE(run.output.find("2 failed (of 2 nominated)"), std::string::npos)
    << "the counters must account for every nominated bag: " << run.output;
}

// The other new code: a bag that indexes fine but cannot be fingerprinted
// authoritatively is a permanent re-index, not a missing bag -- reported with
// its own status so a caller can tell the two apart.
TEST_F(IndexerExitStatusTest, IndexedButUntrustworthyBagExitsWithItsOwnCode)
{
  const auto bag = makeEmptyBag("bag_ok");
  ASSERT_TRUE(std::filesystem::exists(bag / "metadata.yaml"))
    << "rosbag2 wrote no metadata for an empty bag";

  const auto clean = runIndexer(quote(bag.string()));
  ASSERT_EQ(clean.status, 0) << "a readable empty bag is a clean run: " << clean.output;

  // Hide content behind a symlinked subdirectory: the walk will not follow it,
  // so the fingerprint is not authoritative -- but the bag still indexes.
  const auto outside = dir_ / "outside";
  ASSERT_TRUE(std::filesystem::create_directory(outside));
  std::error_code link_ec;
  std::filesystem::create_directory_symlink(outside, bag / "sub", link_ec);
  ASSERT_FALSE(link_ec) << "this filesystem refuses directory symlinks: " << link_ec.message();

  const auto run = runIndexer(quote(bag.string()));
  EXPECT_EQ(run.status, 3) << run.output;
  EXPECT_NE(run.output.find("1 not fully readable"), std::string::npos) << run.output;
  EXPECT_NE(run.output.find("0 failed (of 1 nominated)"), std::string::npos) << run.output;
}

// A --scan tree the walk cannot fully enumerate drops whole bags from the run,
// which is worse than re-indexing one needlessly. It used to be silent.
//
// A good bag is nominated alongside the dropped subtree deliberately: this is
// the test that makes the `!scan_problems.empty()` conjunct in the exit-1 fold
// load-bearing. With nothing nominated the run returns before the fold is ever
// reached, so the old version of this test asserted an exit code that came from
// the empty bag list and would have passed with the conjunct deleted.
TEST_F(IndexerExitStatusTest, ScanReportsASubtreeItCannotEnumerate)
{
  const auto root = dir_ / "root";
  const auto elsewhere = dir_ / "elsewhere";
  ASSERT_TRUE(std::filesystem::create_directories(root));
  ASSERT_TRUE(std::filesystem::create_directories(elsewhere / "bag_hidden"));
  {
    std::ofstream out(elsewhere / "bag_hidden" / "metadata.yaml", std::ios::trunc);
    out << "this is not: [valid rosbag2 metadata\n";
  }
  const auto good = makeEmptyBag("root/bag_good");
  ASSERT_TRUE(std::filesystem::exists(good / "metadata.yaml"))
    << "rosbag2 wrote no metadata for an empty bag";
  std::error_code link_ec;
  std::filesystem::create_directory_symlink(elsewhere, root / "linked", link_ec);
  ASSERT_FALSE(link_ec) << "this filesystem refuses directory symlinks: " << link_ec.message();

  const auto run = runIndexer("--scan " + quote(root.string()));
  EXPECT_NE(run.output.find("is not scanned for bags"), std::string::npos)
    << "a dropped subtree must not be silent: " << run.output;
  EXPECT_NE(run.output.find("0 failed (of 1 nominated)"), std::string::npos)
    << "the nominated bag indexed cleanly, so only the scan can be what fails: " << run.output;
  EXPECT_NE(run.output.find("0 not fully readable"), std::string::npos) << run.output;
  // Nothing failed and nothing is untrustworthy: the incomplete *scan* is the
  // whole reason this run cannot report success.
  EXPECT_EQ(run.status, 1) << run.output;
}

// The other half of that contract, and the case that reads worst in the field:
// a --scan tree that could not be enumerated at all nominates nothing, and used
// to exit 2 -- so a scheduler could not tell an unmounted survey disk from a
// mistyped command line. A missing root is the same condition as an unmounted
// mountpoint, and needs no permission bits, so this runs as root too.
TEST_F(IndexerExitStatusTest, ScanTreeThatCannotBeEnumeratedIsIncompleteNotAUsageError)
{
  const auto run = runIndexer("--scan " + quote((dir_ / "not_mounted").string()));
  EXPECT_NE(run.output.find("could not scan"), std::string::npos) << run.output;
  EXPECT_EQ(run.status, 1) << "an unreadable scan tree is an incomplete index: " << run.output;
  EXPECT_NE(run.output.find("of 0 nominated"), std::string::npos)
    << "a run that nominated nothing still has to say so: " << run.output;
}

// The regression this round exists for. Removing `skip_permission_denied`
// (round 2) made the report loud but left recursion pending on a directory the
// walk cannot read, and a failed `increment()` ends the whole walk -- so ONE
// unreadable directory dropped every bag after it in readdir order, and the
// "of N nominated" summary could not reveal it because those bags were never
// nominated. The invariant asserted here is order-independent: whatever readdir
// order the filesystem gives, the walk must reach every bag and must never
// report having stopped.
TEST_F(IndexerExitStatusTest, ScanWalkContinuesPastADirectoryItCannotProbe)
{
  if (runningAsRoot()) {
    GTEST_SKIP() << "root ignores the directory permissions this test relies on";
  }
  const auto root = dir_ / "root";
  ASSERT_TRUE(std::filesystem::create_directories(root));
  for (const std::string name : {"bag_1", "bag_2", "bag_3", "bag_4"}) {
    makeUnopenableBag("root/" + name);
  }
  const auto locked = root / "locked";
  ASSERT_TRUE(std::filesystem::create_directories(locked / "inner"));
  ASSERT_EQ(::chmod(locked.c_str(), 0000), 0);

  const auto run = runIndexer("--scan " + quote(root.string()));
  EXPECT_NE(run.output.find("is a bag"), std::string::npos)
    << "the unreadable directory must still be reported: " << run.output;
  EXPECT_EQ(run.output.find("stopped at"), std::string::npos)
    << "one unreadable directory must not abandon the walk: " << run.output;
  EXPECT_NE(run.output.find("(of 4 nominated)"), std::string::npos)
    << "every bag outside the unreadable directory must still be nominated: " << run.output;
  EXPECT_EQ(run.status, 1) << run.output;
}

// The same abandonment by the other route: a traverse-only directory (mode
// 0111) answers the `metadata.yaml` probe without error, so it reaches none of
// the reporting branches -- and then cannot be listed. Enumerability is asked
// about up front for exactly this case.
TEST_F(IndexerExitStatusTest, ScanWalkContinuesPastADirectoryItCannotList)
{
  if (runningAsRoot()) {
    GTEST_SKIP() << "root ignores the directory permissions this test relies on";
  }
  const auto root = dir_ / "root";
  ASSERT_TRUE(std::filesystem::create_directories(root));
  for (const std::string name : {"bag_1", "bag_2", "bag_3", "bag_4"}) {
    makeUnopenableBag("root/" + name);
  }
  const auto locked = root / "listless";
  ASSERT_TRUE(std::filesystem::create_directories(locked / "inner"));
  ASSERT_EQ(::chmod(locked.c_str(), 0111), 0);

  const auto run = runIndexer("--scan " + quote(root.string()));
  EXPECT_NE(run.output.find("could not enumerate"), std::string::npos)
    << "a directory the walk cannot list must be reported: " << run.output;
  EXPECT_EQ(run.output.find("stopped at"), std::string::npos)
    << "it must not abandon the walk either: " << run.output;
  EXPECT_NE(run.output.find("(of 4 nominated)"), std::string::npos) << run.output;
  EXPECT_EQ(run.status, 1) << run.output;
}

// An entry whose type cannot be established at all (a symlink loop: ELOOP,
// which root is not exempt from) is the third scan report, and the
// root-observable one. It costs the run its clean exit, and must not cost it
// the rest of the walk.
TEST_F(IndexerExitStatusTest, ScanReportsAnEntryOfUndeterminableType)
{
  const auto root = dir_ / "root";
  ASSERT_TRUE(std::filesystem::create_directories(root));
  const auto good = makeEmptyBag("root/bag_good");
  ASSERT_TRUE(std::filesystem::exists(good / "metadata.yaml"));
  std::error_code link_ec;
  std::filesystem::create_symlink("loop", root / "loop", link_ec);
  ASSERT_FALSE(link_ec) << "this filesystem refuses symlinks: " << link_ec.message();

  const auto run = runIndexer("--scan " + quote(root.string()));
  EXPECT_NE(run.output.find("could not determine the type"), std::string::npos) << run.output;
  EXPECT_EQ(run.output.find("stopped at"), std::string::npos) << run.output;
  EXPECT_NE(run.output.find("0 failed (of 1 nominated)"), std::string::npos) << run.output;
  EXPECT_EQ(run.status, 1) << run.output;
}

// The line the dangling-symlink exemption draws, from the CLI's side: an entry
// that definitively resolves to nothing hides nothing, so it must cost the run
// neither a warning nor its clean exit. Without `resolvesToNothing()` one
// broken link under a scan root would force a non-zero exit every run.
TEST_F(IndexerExitStatusTest, DanglingSymlinkUnderAScanRootIsACleanRun)
{
  const auto root = dir_ / "root";
  ASSERT_TRUE(std::filesystem::create_directories(root));
  const auto good = makeEmptyBag("root/bag_good");
  ASSERT_TRUE(std::filesystem::exists(good / "metadata.yaml"));
  std::error_code link_ec;
  std::filesystem::create_symlink(dir_ / "gone", root / "dangling", link_ec);
  ASSERT_FALSE(link_ec) << "this filesystem refuses symlinks: " << link_ec.message();

  const auto run = runIndexer("--scan " + quote(root.string()));
  EXPECT_EQ(run.output.find("warning:"), std::string::npos)
    << "a broken link hides nothing, so it must not be reported: " << run.output;
  EXPECT_NE(run.output.find("1 bag(s) indexed"), std::string::npos) << run.output;
  EXPECT_EQ(run.status, 0) << run.output;
}

// The index DB itself: named in the contract table as a cause of exit 1
// (nothing was done at all), and previously untested. A path under a
// non-existent directory cannot be created by sqlite even as root.
TEST_F(IndexerExitStatusTest, UnopenableIndexDbExitsOneWithNothingDone)
{
  const auto bag = makeEmptyBag("bag_ok");
  const auto db = (dir_ / "no_such_dir" / "index.db").string();

  const auto run = runIndexerWithDb(db, quote(bag.string()));
  EXPECT_EQ(run.status, 1) << run.output;
  EXPECT_NE(run.output.find("cannot open"), std::string::npos) << run.output;
  EXPECT_EQ(run.output.find("done:"), std::string::npos)
    << "nothing was indexed, so there is no run to summarise: " << run.output;
}

TEST_F(IndexerExitStatusTest, NoBagsIsAUsageError)
{
  const auto run = runIndexer("");
  EXPECT_EQ(run.status, 2) << run.output;
}

}  // namespace
