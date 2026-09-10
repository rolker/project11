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

#ifndef MARINE_SURVEY_INDEX__SCHEMA_HPP_
#define MARINE_SURVEY_INDEX__SCHEMA_HPP_

/// @file
/// @brief SQLite survey-index schema: open/create + version gate.
///
/// The index is a **regenerable sidecar** (`survey_index.db`) — the bags remain
/// the data of record; deleting the index and re-running the indexer always
/// reproduces it. The schema is the cross-stage contract for #258 stages 2–5;
/// the durable description lives in `docs/survey_index_schema.md`.

#include <sqlite3.h>

#include <string>

namespace marine_survey_index
{

/// Schema version stamped into a fresh DB and checked on every open. Bump on
/// any incompatible change; the open then fails with a "regenerate" hint
/// (never silently migrate — regeneration is the migration).
constexpr int kSchemaVersion = 2;

/// @brief Open (creating and initializing if needed) a survey index database.
///
/// Creates the `schema_version`, `bags`, `passes`, and `nav_track` tables
/// when absent, and verifies the stored schema version matches
/// ::kSchemaVersion.
///
/// Opening **writes**: it executes the schema DDL, so it takes a write lock on
/// the file even when the caller only intends to read. With
/// @p busy_timeout_ms > 0 this call, and every later statement on the returned
/// handle, may therefore **block** for up to that long while another process
/// holds the lock (the indexer holds it for one bag's transaction). Leave it at
/// the default in anything that must stay responsive — a GUI thread — and take
/// the error instead; a batch writer wants the wait.
///
/// @param path Filesystem path (or ":memory:" for tests).
/// @param busy_timeout_ms `PRAGMA busy_timeout` for the returned handle, in
///        milliseconds. `0` (the default) is sqlite's own behaviour: fail
///        immediately on a lock held by someone else.
/// @return An open handle; the caller owns it (close with `sqlite3_close`).
/// @throws std::runtime_error on open failure or schema-version mismatch.
sqlite3 * openIndexDb(const std::string & path, int busy_timeout_ms = 0);

}  // namespace marine_survey_index

#endif  // MARINE_SURVEY_INDEX__SCHEMA_HPP_
