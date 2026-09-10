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
    std::error_code ec;
    std::filesystem::remove_all(dir_, ec);
  }

  RunResult runIndexer(const std::string & args) const
  {
    // stderr merged in: the summary line and the warnings both go there.
    const std::string cmd =
      std::string(SURVEY_INDEX_BAG_EXE) + " --db " + (dir_ / "index.db").string() + " " +
      args + " 2>&1";
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

  const auto run = runIndexer(a.string() + " " + b.string());
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

  const auto clean = runIndexer(bag.string());
  ASSERT_EQ(clean.status, 0) << "a readable empty bag is a clean run: " << clean.output;

  // Hide content behind a symlinked subdirectory: the walk will not follow it,
  // so the fingerprint is not authoritative -- but the bag still indexes.
  const auto outside = dir_ / "outside";
  ASSERT_TRUE(std::filesystem::create_directory(outside));
  std::error_code link_ec;
  std::filesystem::create_directory_symlink(outside, bag / "sub", link_ec);
  ASSERT_FALSE(link_ec) << "this filesystem refuses directory symlinks: " << link_ec.message();

  const auto run = runIndexer(bag.string());
  EXPECT_EQ(run.status, 3) << run.output;
  EXPECT_NE(run.output.find("1 not fully readable"), std::string::npos) << run.output;
  EXPECT_NE(run.output.find("0 failed (of 1 nominated)"), std::string::npos) << run.output;
}

// A --scan tree the walk cannot fully enumerate drops whole bags from the run,
// which is worse than re-indexing one needlessly. It used to be silent.
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
  std::error_code link_ec;
  std::filesystem::create_directory_symlink(elsewhere, root / "linked", link_ec);
  ASSERT_FALSE(link_ec) << "this filesystem refuses directory symlinks: " << link_ec.message();

  const auto run = runIndexer("--scan " + root.string());
  EXPECT_NE(run.output.find("is not scanned for bags"), std::string::npos)
    << "a dropped subtree must not be silent: " << run.output;
  // Nothing was nominated, so this is a usage error -- but a loud one now.
  EXPECT_EQ(run.status, 2) << run.output;
}

TEST_F(IndexerExitStatusTest, NoBagsIsAUsageError)
{
  const auto run = runIndexer("");
  EXPECT_EQ(run.status, 2) << run.output;
}

}  // namespace
