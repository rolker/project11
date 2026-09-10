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

#include "marine_survey_index/schema.hpp"

#include <stdexcept>
#include <string>

namespace marine_survey_index
{
namespace
{

// The schema DDL. `sensor_type` extends the ADR-0005 D3 `sensor_class`
// vocabulary: 'mbes-bathy' verbatim; 'sidescan-port' / 'sidescan-stbd' are
// D3 'sidescan' + a channel suffix (the query CLI maps a 'sidescan' filter to
// both). See docs/survey_index_schema.md for the contract description.
constexpr const char * kSchemaDdl =
  R"sql(
CREATE TABLE IF NOT EXISTS schema_version (
  version INTEGER NOT NULL
);

CREATE TABLE IF NOT EXISTS bags (
  id            INTEGER PRIMARY KEY,
  path          TEXT    NOT NULL UNIQUE,
  size_bytes    INTEGER NOT NULL,
  mtime_ns      INTEGER NOT NULL,
  indexed_at_ns INTEGER NOT NULL
);

CREATE TABLE IF NOT EXISTS passes (
  id          INTEGER PRIMARY KEY,
  bag_id      INTEGER NOT NULL REFERENCES bags(id) ON DELETE CASCADE,
  level       INTEGER NOT NULL,
  tile_row    INTEGER NOT NULL,
  tile_col    INTEGER NOT NULL,
  sensor_type TEXT    NOT NULL,
  topic       TEXT    NOT NULL,
  t_start_ns  INTEGER NOT NULL,
  t_end_ns    INTEGER NOT NULL,
  ping_count  INTEGER NOT NULL
);
CREATE INDEX IF NOT EXISTS passes_tile ON passes(level, tile_row, tile_col);
CREATE INDEX IF NOT EXISTS passes_bag  ON passes(bag_id);

CREATE TABLE IF NOT EXISTS nav_track (
  id         INTEGER PRIMARY KEY,
  bag_id     INTEGER NOT NULL REFERENCES bags(id) ON DELETE CASCADE,
  t_ns       INTEGER NOT NULL,
  latitude   REAL    NOT NULL,
  longitude  REAL    NOT NULL
);
CREATE INDEX IF NOT EXISTS nav_track_bag ON nav_track(bag_id, t_ns);
CREATE INDEX IF NOT EXISTS nav_track_geo ON nav_track(latitude, longitude);
)sql";

void execOrThrow(sqlite3 * db, const char * sql)
{
  char * err = nullptr;
  if (sqlite3_exec(db, sql, nullptr, nullptr, &err) != SQLITE_OK) {
    std::string message = err ? err : "unknown sqlite error";
    sqlite3_free(err);
    sqlite3_close(db);
    throw std::runtime_error("survey index: " + message);
  }
}

}  // namespace

sqlite3 * openIndexDb(const std::string & path)
{
  sqlite3 * db = nullptr;
  if (sqlite3_open(path.c_str(), &db) != SQLITE_OK) {
    const std::string message = db ? sqlite3_errmsg(db) : "out of memory";
    sqlite3_close(db);
    throw std::runtime_error("survey index: cannot open '" + path + "': " + message);
  }
  execOrThrow(db, "PRAGMA foreign_keys = ON;");
  // Wait for a writer's lock instead of failing on contact with one. The index
  // is opened read-only by the explorer GUI (marine_perception_tools'
  // survey_index_bridge opens it in its constructor), and opening it here
  // executes DDL, so even opening takes a write lock. Without a timeout a GUI
  // holding the DB for a moment turns an indexer run into failed bags -- and a
  // failed bag is an exit-1 incomplete index, not a retry. Ten seconds is far
  // longer than any read this DB serves and far shorter than a survey-tree
  // indexing run.
  execOrThrow(db, "PRAGMA busy_timeout = 10000;");
  execOrThrow(db, kSchemaDdl);

  // Version gate: stamp a fresh DB, verify an existing one. Regeneration is
  // the migration path — the index is a derived cache over the bags.
  sqlite3_stmt * stmt = nullptr;
  if (sqlite3_prepare_v2(db, "SELECT version FROM schema_version", -1, &stmt, nullptr) !=
    SQLITE_OK)
  {
    const std::string message = sqlite3_errmsg(db);
    sqlite3_close(db);
    throw std::runtime_error("survey index: " + message);
  }
  const int step = sqlite3_step(stmt);
  if (step == SQLITE_ROW) {
    const int version = sqlite3_column_int(stmt, 0);
    sqlite3_finalize(stmt);
    if (version != kSchemaVersion) {
      sqlite3_close(db);
      throw std::runtime_error(
        "survey index: '" + path + "' has schema version " + std::to_string(version) +
        " but this tool expects " + std::to_string(kSchemaVersion) +
        " — delete the file and re-run the indexer to regenerate it (bags are the "
        "data of record; nothing is lost)");
    }
  } else if (step == SQLITE_DONE) {
    // Empty schema_version table → genuinely fresh DB: stamp the version.
    sqlite3_finalize(stmt);
    execOrThrow(
      db, ("INSERT INTO schema_version (version) VALUES (" +
      std::to_string(kSchemaVersion) + ");").c_str());
  } else {
    // Any other step result is a read error (corrupt/unreadable DB) — do not
    // mask it as a fresh DB and write to it.
    sqlite3_finalize(stmt);
    const std::string message = sqlite3_errmsg(db);
    sqlite3_close(db);
    throw std::runtime_error(
      "survey index: reading schema_version from '" + path + "': " + message);
  }
  return db;
}

}  // namespace marine_survey_index
