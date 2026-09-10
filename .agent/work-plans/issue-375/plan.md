# Plan: marine_survey_index: bags.mtime_ns is always 0, so the incremental-skip ledger compares size only and stale bags are never re-indexed

## Issue

https://github.com/rolker/unh_marine_autonomy/issues/375

## Context

`fingerprint()` in `marine_survey_index/src/survey_index_bag_main.cpp` (line
160) computes a `BagFingerprint{size_bytes, mtime_ns}` used by `ledgerState()`
(line ~284) to decide whether a bag is unchanged (skip) or changed
(re-index). `mtime_ns` is derived from `fs::last_write_time()`'s raw
`time_since_epoch()`. libstdc++'s `file_clock` epoch is 2174-01-01, not the
Unix epoch, so that raw count is large and negative for any real file — and
the `std::max(fp.mtime_ns, ...)` accumulator, seeded at `0`, never accepts a
negative candidate. `mtime_ns` therefore stays `0` for every bag, silently.
The `ec` check passes, so nothing fails loudly. Confirmed live: the dev
host's index carries `mtime_ns = 0` for all 177 rows. Consequence: the
unchanged-test at line ~292 (`stored.size == fp.size && stored.mtime_ns ==
fp.mtime_ns`) degrades to size-only — a bag rewritten in place at an
unchanged byte count is silently treated as unchanged and never re-indexed,
leaving its `passes`/`nav_track` rows stale.

`fingerprint()` and `ledgerState()` currently live in
`survey_index_bag_main.cpp`, which is compiled only into the
`survey_index_bag` executable — not into `marine_survey_index_core`, the
library the existing unit tests (`test_schema.cpp`,
`test_interval_merge.cpp`, etc.) link against. The core lib is deliberately
free of `rosbag2`/`tf2`/`rclcpp` (per the comment at `CMakeLists.txt:30`) so
tests can run without bag I/O — but `fingerprint()`/`ledgerState()` have no
such dependency themselves (`<filesystem>`, `<chrono>`, `<cstdint>`, and
`SQLite3`, which the core lib already links). Testing them in place would
require a new test binary pulling in the full `rosbag2`/`tf2` dependency
chain just to construct one function's inputs. Moving both functions into
the core lib is the natural fix and matches the existing library/executable
split in this package.

## Approach

1. **Add `marine_survey_index/include/marine_survey_index/bag_fingerprint.hpp`
   and `marine_survey_index/src/bag_fingerprint.cpp`.** Move `BagFingerprint`,
   `fingerprint()`, and `ledgerState()` out of `survey_index_bag_main.cpp`
   into this new pair (same license header and doc-comment style as
   `footprint.hpp`/`schema.hpp`). `bag_fingerprint.cpp` includes
   `<sys/stat.h>` and `sqlite3.h` (already a core-lib dependency via
   `schema.hpp`); no new `find_package`/`ament_target_dependencies` needed.

2. **Fix the mtime conversion.** Stay at C++17 (`CMAKE_CXX_STANDARD 17` is
   not otherwise triggered for a bump) and take the mtime from `::stat()`
   directly instead of `fs::last_write_time()`:
   ```cpp
   struct ::stat st{};
   if (::stat(f.c_str(), &st) == 0) {
     const auto ns = static_cast<std::int64_t>(st.st_mtim.tv_sec) * 1'000'000'000LL +
       st.st_mtim.tv_nsec;
     ...
   }
   ```
   This sidesteps the `file_clock` epoch entirely and gives a value directly
   comparable to `stat`-based tooling elsewhere (as the issue requests).

3. **Fix the accumulator so a legitimate value can never be swallowed by the
   initializer.** Track `bool has_mtime = false` alongside `mtime_ns` (or
   seed with `std::numeric_limits<std::int64_t>::min()` — `has_value` flag is
   clearer at the call site and at the "unreadable" decision in step 4, so
   use that). Update `fp.mtime_ns = std::max(fp.mtime_ns, ns)` only after
   setting `has_mtime = true` on the first successful reading, and initialize
   `fp.mtime_ns` to `std::numeric_limits<std::int64_t>::min()` so the
   `std::max` is correct even before `has_mtime` toggles.

4. **Decide and implement the unreadable-mtime policy.** Per the issue and
   `review-issue`'s Principle Alignment note (Human control and
   transparency: "the implementer should make the unreadable-mtime failure
   mode loud"): if `fingerprint()` walks a bag and finds regular files but
   `has_mtime` stays `false` at the end (every `::stat` call failed), or the
   bag has zero regular files, the fingerprint must **fail the unchanged
   test**, not pass it — an index bug must not re-hide as a skip. Represent
   this as `fp.mtime_ns = std::numeric_limits<std::int64_t>::min()` staying
   in place (already guaranteed distinct from any real stored value, since
   real mtimes are non-negative Unix nanosecond counts) **and** log a
   `std::cerr` warning naming the bag path when this happens, so the
   operator sees it instead of it silently forcing a re-index every run.
   This satisfies "fail rather than silently pass" without adding a new
   sentinel column.

5. **Update `survey_index_bag_main.cpp`** to `#include
   "marine_survey_index/bag_fingerprint.hpp"` and drop the now-moved
   definitions; call sites (`fingerprint(bag)`, `ledgerState(db, path, fp)`)
   are unchanged.

6. **Update `marine_survey_index/CMakeLists.txt`**: add `src/bag_fingerprint.cpp`
   to the `${PROJECT_NAME}_core` library's source list (alongside
   `schema.cpp`, `footprint.cpp`, `query.cpp`); add a new
   `ament_add_gtest(test_bag_fingerprint test/test_bag_fingerprint.cpp)` +
   `target_link_libraries(test_bag_fingerprint ${PROJECT_NAME}_core)` in the
   existing test block (after `test_query_join`).

7. **Add `marine_survey_index/test/test_bag_fingerprint.cpp`** covering the
   three cases the issue names, using `std::filesystem::temp_directory_path()`
   + a per-test unique subdirectory (created/removed in the test, mirroring
   the temp-file pattern in `test_schema.cpp`):
   - **mtime accuracy**: fingerprint a temp file; assert `fp.mtime_ns` is
     within ~2 seconds of `::stat`'s own reading of the same file (not
     `fs::last_write_time`, to keep the test independent of the bug it's
     catching).
   - **in-place rewrite at identical size** (the actual stale-data
     scenario): write a file, fingerprint it, sleep briefly (or force a
     distinct mtime — see Open Questions), overwrite its content with
     different bytes of the *same length*, re-fingerprint, and assert
     `size_bytes` is unchanged but `mtime_ns` changed (fingerprint differs).
   - **ledger round-trip**: exercise `ledgerState()` directly against an
     in-memory/temp sqlite DB with the `bags` table (reuse the schema via
     `marine_survey_index::openOrCreate`/`schema.hpp` or a minimal inline
     `CREATE TABLE`) — insert a row with a real fingerprint's
     `(size_bytes, mtime_ns)`, call `ledgerState()` with the same fingerprint
     (expect unchanged/positive id), then with a fingerprint from the
     rewritten-file case (expect changed/negative id).
   - **unreadable-mtime policy**: fingerprint an empty directory (or a
     directory whose only entries are unreadable) and assert the result
     never equals a previously-stored `(0, 0)`-style row — i.e. it does not
     silently satisfy `unchanged`.

## Files to Change

| File | Change |
|------|--------|
| `marine_survey_index/include/marine_survey_index/bag_fingerprint.hpp` | New. Declares `BagFingerprint`, `fingerprint()`, `ledgerState()`. |
| `marine_survey_index/src/bag_fingerprint.cpp` | New. Moves + fixes the implementations (stat-based mtime, has-value accumulator, loud unreadable-mtime policy). |
| `marine_survey_index/src/survey_index_bag_main.cpp` | Remove the moved struct/functions; include the new header. |
| `marine_survey_index/CMakeLists.txt` | Add `bag_fingerprint.cpp` to the core lib; add the new gtest target. |
| `marine_survey_index/test/test_bag_fingerprint.cpp` | New. The four test cases above. |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Test what breaks | The four new tests target the exact regressions named in the issue (mtime accuracy, same-size rewrite, ledger round-trip) plus the unreadable-mtime policy this plan adds — not framework glue. |
| A change includes its consequences | PR description will state the one-time full re-index of all 177 existing (`mtime_ns=0`) rows on the dev host, per the operator's note — this is expected derived-cache-rebuild cost, not a regression, and no data migration is needed. |
| Human control and transparency | The unreadable-mtime path now fails loud (stderr warning + guaranteed-mismatching sentinel) instead of silently contributing nothing to a max, per `review-issue`'s flag. |
| Only what's needed | No ledger schema change, no new DB column, no speculative generalization beyond moving the two functions needed for testability. |
| Improve incrementally | Single PR, single package, bounded diff. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| None of 0001–0013 (workspace or project) | No | Offline-indexer bug fix; no vehicle control, hardware interface, store schema, or transport change. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `mtime_ns` computation becomes non-zero and correct | Existing on-disk indexes (177 rows, `mtime_ns=0`) will compare unequal on next run and each bag re-indexes once | Yes — called out in PR description per the operator's note; no code change needed, the index is a derived cache and self-heals on the next `survey_index_bag` run |
| `fingerprint()`/`ledgerState()` move to the core lib | `survey_index_bag_main.cpp`'s includes and any other in-package caller | Yes — only caller is `main()`, which is updated in step 5 |
| New test binary in CMakeLists | `docs/survey_index_schema.md` build/test instructions | No — that doc doesn't enumerate test binaries by name, only the schema/contract, which is unchanged (the doc already describes the *intended* `mtime_ns` contract correctly; this PR makes the code match it, not the doc) |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): None — `docs/survey_index_schema.md`
  and the `BagFingerprint` doc-comment already describe the intended
  contract ("newest mtime under the bag directory") correctly; this PR
  makes the implementation match that existing description rather than
  changing what any doc claims.
- **Agent-instruction candidates** (proposals only): None in this plan. The
  general libstdc++ `file_clock` epoch pitfall was already split out by
  operator decision to
  [rolker/ros2_agent_workspace#623](https://github.com/rolker/ros2_agent_workspace/issues/623)
  and is out of scope here.

## Open Questions

- Test timing precision: the in-place-rewrite test needs two distinct
  `mtime_ns` values from `::stat` across a rewrite. Filesystem mtime
  resolution varies (ext4 is typically sub-ms via `st_mtim`, but CI
  filesystems can differ). Plan default: no artificial `sleep` — write,
  fingerprint, rewrite, fingerprint, and assert `mtime_ns` values differ;
  if this proves flaky in CI, fall back to `utimensat`/explicit
  `std::filesystem::last_write_time(path, new_time)` to force a distinct
  timestamp deterministically instead of relying on wall-clock granularity.
  Not resolved here because it depends on observed CI behavior during
  implementation — implementer should verify locally first and only add the
  explicit-timestamp fallback if the wall-clock version is flaky.

## Estimated Scope

Single PR, single project repo (`unh_marine_autonomy`), single package
(`marine_survey_index`). Five files touched (two new).
