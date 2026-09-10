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

#include <sqlite3.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>
#include <thread>

#include "marine_survey_index/schema.hpp"
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
    return runIndexerUnderShell("", db, args);
  }

  // @p shell_prefix runs in the same shell before the binary, for the one
  // resource limit that reproduces a real failure root is not exempt from
  // (`ulimit -n`). It is test-authored text, never interpolated input.
  RunResult runIndexerUnderShell(
    const std::string & shell_prefix, const std::string & db, const std::string & args) const
  {
    // stderr merged in: the summary line and the warnings both go there.
    // Everything the test interpolates is quoted: a build path or a TMPDIR
    // containing a space would otherwise mis-parse into extra argv words and
    // the test would be measuring the wrong invocation.
    const std::string cmd =
      shell_prefix + quote(SURVEY_INDEX_BAG_EXE) + " --db " + quote(db) + " " + args + " 2>&1";
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

  // Direct SQL against the index, for the states only a *previous* indexer
  // could have left behind (a ledger row keyed through a symlink) and for the
  // assertions the summary line cannot make (how many rows there are).
  static void execSql(const std::string & db_path, const std::string & sql)
  {
    sqlite3 * db = marine_survey_index::openIndexDb(db_path);
    char * err = nullptr;
    const int rc = sqlite3_exec(db, sql.c_str(), nullptr, nullptr, &err);
    if (rc != SQLITE_OK) {
      ADD_FAILURE() << "sql failed: " << sql << ": " << (err ? err : "?");
    }
    sqlite3_free(err);
    sqlite3_close(db);
  }

  static std::string queryScalar(const std::string & db_path, const std::string & sql)
  {
    sqlite3 * db = marine_survey_index::openIndexDb(db_path);
    sqlite3_stmt * stmt = nullptr;
    std::string out;
    if (sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
      ADD_FAILURE() << "prepare failed: " << sql << ": " << sqlite3_errmsg(db);
    } else if (sqlite3_step(stmt) == SQLITE_ROW) {
      const unsigned char * text = sqlite3_column_text(stmt, 0);
      out = text != nullptr ? reinterpret_cast<const char *>(text) : "";
    }
    sqlite3_finalize(stmt);
    sqlite3_close(db);
    return out;
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
  // whole reason this run cannot report success -- so the summary line has to
  // say so, or it reads as a clean run with an inexplicable exit code.
  EXPECT_NE(run.output.find("1 scan problem(s) reported above"), std::string::npos)
    << "the summary must name the cause of its own exit status: " << run.output;
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

// The enumerability probe's guard, root-observably: a directory that cannot be
// opened because the process is out of file descriptors. The recursive walk
// holds one open directory per level, so a chain deeper than the descriptor
// limit exhausts them part way down -- EMFILE, which root is not exempt from,
// unlike the mode bits the two tests above need.
//
// This matters because `ci_local.sh` (ADR-0018's merge verification, and the
// only one that reaches this package -- no hosted workflow builds it) runs as
// root: without a route like this one, the invariant that a walk survives a
// directory it cannot open would execute in no merge-gating path at all.
//
// The assertion is the same order- and depth-independent invariant: every bag
// outside the unopenable subtree is still nominated, and the walk never
// reports having stopped.
TEST_F(IndexerExitStatusTest, ScanWalkContinuesPastADirectoryItHasNoDescriptorFor)
{
  const auto root = dir_ / "root";
  ASSERT_TRUE(std::filesystem::create_directories(root));
  for (const std::string name : {"bag_1", "bag_2", "bag_3", "bag_4"}) {
    makeUnopenableBag("root/" + name);
  }
  std::filesystem::path deep = root / "deep";
  for (int i = 0; i < 400; ++i) {
    deep /= "d";
  }
  std::error_code deep_ec;
  std::filesystem::create_directories(deep, deep_ec);
  ASSERT_FALSE(deep_ec) << "could not build the deep chain: " << deep_ec.message();

  // Lowering a soft limit is always permitted, so this is not a flaky
  // privilege test -- but a host whose *hard* limit is already below 256 could
  // not raise it back, and `ulimit` would fail rather than lie. 111 says so.
  const auto run = runIndexerUnderShell(
    "ulimit -n 256 || exit 111; ", (dir_ / "index.db").string(),
    "--scan " + quote(root.string()));
  if (run.status == 111) {
    GTEST_SKIP() << "this host's hard descriptor limit is below 256";
  }
  EXPECT_NE(run.output.find("could not enumerate"), std::string::npos)
    << "a directory the walk has no descriptor for must be reported: " << run.output;
  EXPECT_EQ(run.output.find("stopped at"), std::string::npos)
    << "and must not abandon the walk: " << run.output;
  EXPECT_NE(run.output.find("(of 4 nominated)"), std::string::npos)
    << "every bag outside the deep chain must still be nominated: " << run.output;
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

// A bag reached through a symlinked path is the SAME bag: the ledger key
// resolves links, so it cannot become a second bag under a second `bag_id`
// with every pass interval and nav point inserted twice. `--scan` nominates
// such a link as a bag (the metadata probe follows it), so this is reachable
// without anyone naming both paths by hand.
//
// Within one run that is a matter of nomination -- two paths for one bag are
// one nomination, not a bag plus a bag that is mysteriously already up to
// date. Across runs it is the ledger key that recognises it, which is the case
// that matters: the row must be found, not inserted beside.
TEST_F(IndexerExitStatusTest, ABagReachedThroughASymlinkIsNotASecondBag)
{
  const auto bag = makeEmptyBag("bag_ok");
  const auto link = dir_ / "link_to_bag";
  std::error_code link_ec;
  std::filesystem::create_directory_symlink(bag, link, link_ec);
  ASSERT_FALSE(link_ec) << "this filesystem refuses directory symlinks: " << link_ec.message();
  const auto db = (dir_ / "index.db").string();

  const auto both = runIndexerWithDb(db, quote(bag.string()) + " " + quote(link.string()));
  EXPECT_EQ(both.status, 0) << both.output;
  EXPECT_NE(
    both.output.find("1 bag(s) indexed, 0 unchanged skipped, 0 failed (of 1 nominated)"),
    std::string::npos)
    << "two paths to one bag are one nomination: " << both.output;
  const std::string id_after_first = queryScalar(db, "SELECT id FROM bags");
  EXPECT_FALSE(id_after_first.empty());

  // A second run, nominating only the link: the ledger key is now the only
  // thing that can recognise the bag.
  const auto again = runIndexerWithDb(db, quote(link.string()));
  EXPECT_EQ(again.status, 0) << again.output;
  EXPECT_NE(again.output.find("0 bag(s) indexed, 1 unchanged skipped"), std::string::npos)
    << "the link must find the row the real path wrote: " << again.output;
  EXPECT_EQ(queryScalar(db, "SELECT COUNT(*) FROM bags"), "1");
  EXPECT_EQ(queryScalar(db, "SELECT id FROM bags"), id_after_first)
    << "a new id means a second row was written for the same bag";
}

// The migration half of the resolved ledger key, and the one with reach beyond
// this machine: a row a PRE-FIX indexer wrote through a symlinked path can
// never be matched again, so the lookup misses, a second row is inserted, and
// the stale row's passes double-report that bag through the query join
// forever. The row must be re-keyed, not orphaned.
TEST_F(IndexerExitStatusTest, APreFixRowKeyedThroughASymlinkIsRekeyedNotOrphaned)
{
  const auto bag = makeEmptyBag("bag_ok");
  const auto link = dir_ / "link_to_bag";
  std::error_code link_ec;
  std::filesystem::create_directory_symlink(bag, link, link_ec);
  ASSERT_FALSE(link_ec) << "this filesystem refuses directory symlinks: " << link_ec.message();
  const auto db = (dir_ / "index.db").string();

  // Exactly what a pre-#375 indexer left: the ledger keyed by the path it was
  // given, plus the passes that key owns.
  execSql(
    db,
    "INSERT INTO bags (path, size_bytes, mtime_ns, indexed_at_ns) VALUES ('" +
    link.string() + "', 1, 0, 0);"
    "INSERT INTO passes (bag_id, level, tile_row, tile_col, sensor_type, topic,"
    " t_start_ns, t_end_ns, ping_count)"
    " SELECT id, 14, 1, 1, 'mbes-bathy', '/t', 0, 1, 1 FROM bags;");

  const auto run = runIndexerWithDb(db, quote(bag.string()));
  EXPECT_EQ(run.status, 0) << run.output;
  EXPECT_NE(run.output.find("re-keyed the ledger row"), std::string::npos)
    << "a row that can never be matched again must be migrated, not orphaned: " << run.output;
  EXPECT_EQ(queryScalar(db, "SELECT COUNT(*) FROM bags"), "1")
    << "one bag is one row: the pre-fix row was re-keyed, not inserted beside";
  EXPECT_EQ(queryScalar(db, "SELECT path FROM bags"), bag.string());
  // The re-key hands the bag its old row, so re-indexing it clears the stale
  // passes -- nothing is left to double-report.
  EXPECT_EQ(queryScalar(db, "SELECT COUNT(*) FROM passes"), "0");
}

// The same migration where the resolved key is already taken: that pair IS the
// double report. The canonical row is the one every future run uses, so the
// stale duplicate goes, and its passes with it.
TEST_F(IndexerExitStatusTest, APreFixDuplicateRowIsRemovedRatherThanLeftDoubleReporting)
{
  const auto bag = makeEmptyBag("bag_ok");
  const auto link = dir_ / "link_to_bag";
  std::error_code link_ec;
  std::filesystem::create_directory_symlink(bag, link, link_ec);
  ASSERT_FALSE(link_ec) << "this filesystem refuses directory symlinks: " << link_ec.message();
  const auto db = (dir_ / "index.db").string();

  // Index the bag properly first, then add the pre-fix symlink-keyed row
  // beside it -- the state a pre-fix index plus one post-fix run leaves.
  ASSERT_EQ(runIndexerWithDb(db, quote(bag.string())).status, 0);
  execSql(
    db,
    "INSERT INTO bags (path, size_bytes, mtime_ns, indexed_at_ns) VALUES ('" +
    link.string() + "', 1, 0, 0);"
    "INSERT INTO passes (bag_id, level, tile_row, tile_col, sensor_type, topic,"
    " t_start_ns, t_end_ns, ping_count)"
    " SELECT id, 14, 1, 1, 'mbes-bathy', '/t', 0, 1, 1 FROM bags WHERE path = '" +
    link.string() + "';");
  ASSERT_EQ(queryScalar(db, "SELECT COUNT(*) FROM bags"), "2");

  const auto run = runIndexerWithDb(db, quote(bag.string()));
  EXPECT_EQ(run.status, 0) << run.output;
  EXPECT_NE(run.output.find("removed the stale duplicate row"), std::string::npos) << run.output;
  EXPECT_EQ(queryScalar(db, "SELECT COUNT(*) FROM bags"), "1")
    << "the duplicate keyed through the symlink must not survive the run";
  EXPECT_EQ(queryScalar(db, "SELECT path FROM bags"), bag.string());
  EXPECT_EQ(queryScalar(db, "SELECT COUNT(*) FROM passes"), "0")
    << "CASCADE must take the stale row's passes: they are the double report";
}

// The mid-index failure handler, in the shape that actually defends the
// `ROLLBACK`: a bag that failed with its transaction open must not leave that
// transaction open, or the NEXT bag's `BEGIN` fails and one bad bag takes the
// rest of the run with it. SQLite rolls a transaction back at `sqlite3_close`
// anyway, so nothing a single-bag run can observe distinguishes the statement
// being there from it being gone -- which is why the read-only-DB test below
// left it undefended, and why the honesty note there used to give the wrong
// reason for that.
//
// The failure has to be per-bag selective for a second bag to reach `BEGIN` at
// all. A trigger keyed on the bag's own path is exactly that, and needs no
// permission trick, so unlike the sibling below it runs as root -- which is
// the only environment that gates a merge here (ADR-0018's `ci_local.sh` runs
// as root, and this package is in no hosted workflow's build or test list).
TEST_F(IndexerExitStatusTest, ABagThatFailsMidTransactionRollsBackSoTheNextBagStillIndexes)
{
  const auto bad = makeEmptyBag("bag_bad");
  const auto good = makeEmptyBag("bag_good");
  const auto db = (dir_ / "index.db").string();
  execSql(
    db,
    "CREATE TRIGGER fail_one_bag BEFORE INSERT ON bags WHEN NEW.path LIKE '%bag_bad%'"
    " BEGIN SELECT RAISE(ABORT, 'synthetic mid-index write failure'); END;");

  // Order matters: the failing bag first, so the good one has to survive it.
  const auto run = runIndexerWithDb(db, quote(bad.string()) + " " + quote(good.string()));
  EXPECT_NE(run.output.find("failed to index"), std::string::npos) << run.output;
  EXPECT_NE(
    run.output.find("1 bag(s) indexed, 0 unchanged skipped, 1 failed (of 2 nominated)"),
    std::string::npos)
    << "the failed bag must cost the run exactly itself: " << run.output;
  EXPECT_EQ(run.status, 1) << run.output;
  EXPECT_EQ(queryScalar(db, "SELECT COUNT(*) FROM bags"), "1")
    << "the rolled-back bag must leave no ledger row, and the good bag must have one";
  EXPECT_EQ(queryScalar(db, "SELECT path FROM bags"), good.string());
}

// The same counter by the route a field operator hits (a DB that cannot be
// written), reached by re-indexing a changed bag into a read-only index DB --
// the write fails at the first statement, with the transaction already open.
//
// Permission-based, so it skips as root. Its guard is defended
// root-observably by the trigger test above; what this one adds is the real
// sqlite write error, and it is kept for that.
TEST_F(IndexerExitStatusTest, BagThatFailsMidIndexIsRolledBackAndCounted)
{
  if (runningAsRoot()) {
    GTEST_SKIP() << "root writes a read-only database, so nothing would fail";
  }
  const auto bag = makeEmptyBag("bag_ok");
  const auto db = (dir_ / "index.db").string();
  ASSERT_EQ(runIndexerWithDb(db, quote(bag.string())).status, 0);

  // Change the bag so the next run re-indexes it instead of skipping it: the
  // ledger row is what makes this the *re-index* path, and a write is what has
  // to fail.
  std::filesystem::path member;
  for (const auto & entry : std::filesystem::directory_iterator(bag)) {
    if (entry.path().extension() != ".yaml") {
      member = entry.path();
    }
  }
  ASSERT_FALSE(member.empty()) << "the empty bag has no storage member to touch";
  {
    std::ofstream out(member, std::ios::binary | std::ios::app);
    out << "x";
  }
  ASSERT_EQ(::chmod(db.c_str(), 0444), 0);

  const auto run = runIndexerWithDb(db, quote(bag.string()));
  EXPECT_NE(run.output.find("failed to index"), std::string::npos) << run.output;
  EXPECT_NE(run.output.find("1 failed (of 1 nominated)"), std::string::npos)
    << "a bag that failed mid-index is neither indexed nor skipped: " << run.output;
  EXPECT_EQ(run.status, 1) << run.output;
}

// Argument parsing, and the reason it belongs in the exit-status file: the
// contract says 0 means every nominated bag is in the index. A `--scan` with
// nothing behind it walked no tree, reported no problem and exited 0 -- the
// exact shape of `--scan $ROOT` with `ROOT` unset and unquoted, where the
// quoted spelling (`--scan ""`) already exited 1. Both spellings must now
// refuse to run.
TEST_F(IndexerExitStatusTest, ValuelessScanIsAUsageErrorNotASilentlyEmptyRun)
{
  const auto run = runIndexer("--scan");
  EXPECT_EQ(run.status, 2) << run.output;
  EXPECT_NE(run.output.find("--scan requires a value"), std::string::npos) << run.output;
  EXPECT_EQ(run.output.find("done:"), std::string::npos)
    << "nothing was walked, so there is no run to summarise: " << run.output;

  // The quoted spelling of the same slip: an empty root is a root that cannot
  // be enumerated, which is an incomplete index (1), not a usage error.
  const auto quoted = runIndexer("--scan ''");
  EXPECT_EQ(quoted.status, 1) << quoted.output;
}

// The other half: an unknown flag used to be assumed to take a value, so the
// positional bag behind it was consumed as that value and the run exited 0
// having indexed nothing.
TEST_F(IndexerExitStatusTest, UnrecognisedFlagDoesNotSwallowTheBagBehindIt)
{
  const auto bag = makeEmptyBag("bag_ok");
  const auto run = runIndexer("--verbose " + quote(bag.string()));
  EXPECT_EQ(run.status, 2) << run.output;
  EXPECT_NE(run.output.find("unrecognised flag '--verbose'"), std::string::npos) << run.output;
  EXPECT_EQ(run.output.find("done:"), std::string::npos)
    << "the bag was never indexed, so no summary may claim a run: " << run.output;
}

// A known flag where a value should be is a missing value, not a value: this
// would otherwise scan a directory literally called "--db".
TEST_F(IndexerExitStatusTest, AFlagFollowedByAnotherFlagIsAMissingValue)
{
  const auto run = runIndexer("--scan --level 14");
  EXPECT_EQ(run.status, 2) << run.output;
  EXPECT_NE(run.output.find("--scan requires a value"), std::string::npos) << run.output;
}

// The indexer opts into waiting for someone else's lock (`openIndexDb`'s
// `busy_timeout_ms`), because the explorer GUI holds a write-capable handle on
// the same file and a moment's contention would otherwise become a *failed*
// bag -- an exit-1 incomplete index, with no retry. The default is not to
// wait, so this asserts the opt-in from the outside: a lock held for a second
// and a half must cost the run nothing.
TEST_F(IndexerExitStatusTest, IndexerWaitsOutALockInsteadOfFailingTheBag)
{
  const auto bag = makeEmptyBag("bag_ok");
  const auto db = (dir_ / "index.db").string();

  sqlite3 * holder = marine_survey_index::openIndexDb(db);
  ASSERT_NE(holder, nullptr);
  ASSERT_EQ(
    sqlite3_exec(holder, "BEGIN EXCLUSIVE;", nullptr, nullptr, nullptr), SQLITE_OK)
    << sqlite3_errmsg(holder);
  std::thread releaser(
    [holder]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(1500));
      sqlite3_exec(holder, "COMMIT;", nullptr, nullptr, nullptr);
      sqlite3_close(holder);
    });

  const auto run = runIndexerWithDb(db, quote(bag.string()));
  releaser.join();
  EXPECT_EQ(run.status, 0)
    << "a lock held far inside the timeout must not fail a bag: " << run.output;
  EXPECT_NE(run.output.find("1 bag(s) indexed"), std::string::npos) << run.output;
}

TEST_F(IndexerExitStatusTest, NoBagsIsAUsageError)
{
  const auto run = runIndexer("");
  EXPECT_EQ(run.status, 2) << run.output;
}

}  // namespace
