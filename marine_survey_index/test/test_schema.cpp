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

#include <cstdio>
#include <string>

#include "marine_survey_index/schema.hpp"

namespace
{

// The DB-open contract that #258 stages 2-5 depend on: a fresh open creates
// the tables and version row; a re-open of a valid DB succeeds; a version
// mismatch fails loudly with the regenerate hint.

// Returns -1 after ADD_FAILURE on any sqlite error, so a failed prepare
// fails the test cleanly instead of stepping a null statement (ASSERT_* is
// unavailable in a value-returning helper).
int singleInt(sqlite3 * db, const char * sql)
{
  sqlite3_stmt * stmt = nullptr;
  if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    ADD_FAILURE() << "prepare failed for '" << sql << "': " << sqlite3_errmsg(db);
    return -1;
  }
  if (sqlite3_step(stmt) != SQLITE_ROW) {
    ADD_FAILURE() << "no row for '" << sql << "': " << sqlite3_errmsg(db);
    sqlite3_finalize(stmt);
    return -1;
  }
  const int value = sqlite3_column_int(stmt, 0);
  sqlite3_finalize(stmt);
  return value;
}

TEST(Schema, FreshOpenCreatesTablesAndVersion)
{
  sqlite3 * db = marine_survey_index::openIndexDb(":memory:");
  ASSERT_NE(db, nullptr);
  EXPECT_EQ(singleInt(db, "SELECT version FROM schema_version"),
    marine_survey_index::kSchemaVersion);
  EXPECT_EQ(
    singleInt(
      db,
      "SELECT count(*) FROM sqlite_master WHERE type='table'"
      " AND name IN ('schema_version','bags','passes','nav_track')"),
    4);
  // The v2 nav_track shape: id, bag_id, t_ns, latitude, longitude.
  EXPECT_EQ(singleInt(db, "SELECT count(*) FROM pragma_table_info('nav_track')"), 5);
  sqlite3_close(db);
}

// A GUI holds this DB open (marine_perception_tools' survey_index_bridge), and
// an indexer run writes to it. With sqlite's default zero busy timeout, a lock
// held for an instant turns a bag into a *failed* bag -- an exit-1 incomplete
// index, with no retry. So the open must arrive already willing to wait.
TEST(Schema, OpenWaitsForAWriterInsteadOfFailingOnOne)
{
  sqlite3 * db = marine_survey_index::openIndexDb(":memory:");
  ASSERT_NE(db, nullptr);
  EXPECT_GE(singleInt(db, "PRAGMA busy_timeout"), 5000)
    << "a zero (default) busy timeout makes a concurrent reader a failed bag";
  sqlite3_close(db);
}

TEST(Schema, ReopenExistingSucceeds)
{
  const std::string path = testing::TempDir() + "/survey_index_schema_test.db";
  std::remove(path.c_str());

  sqlite3 * db = marine_survey_index::openIndexDb(path);
  ASSERT_NE(db, nullptr);
  sqlite3_close(db);

  // Second open must validate, not re-stamp or fail.
  db = marine_survey_index::openIndexDb(path);
  ASSERT_NE(db, nullptr);
  EXPECT_EQ(singleInt(db, "SELECT count(*) FROM schema_version"), 1);
  sqlite3_close(db);
  std::remove(path.c_str());
}

TEST(Schema, VersionMismatchThrowsWithRegenerateHint)
{
  const std::string path = testing::TempDir() + "/survey_index_version_test.db";
  std::remove(path.c_str());

  sqlite3 * db = marine_survey_index::openIndexDb(path);
  ASSERT_NE(db, nullptr);
  ASSERT_EQ(
    sqlite3_exec(db, "UPDATE schema_version SET version = 9999", nullptr, nullptr, nullptr),
    SQLITE_OK);
  sqlite3_close(db);

  try {
    marine_survey_index::openIndexDb(path);
    FAIL() << "expected a schema-version mismatch to throw";
  } catch (const std::runtime_error & e) {
    EXPECT_NE(std::string(e.what()).find("re-run the indexer"), std::string::npos);
  }
  std::remove(path.c_str());
}

}  // namespace
