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
   and `marine_survey_index/src/bag_fingerprint.cpp`.** Move `BagFingerprint`
   and `fingerprint()` out of `survey_index_bag_main.cpp` into this new pair
   (same license header and doc-comment style as
   `footprint.hpp`/`schema.hpp`), and add one pure decision function,
   `fingerprintMatches(current, stored_size, stored_mtime)`. `bag_fingerprint.cpp`
   needs only `<sys/stat.h>` and the standard library — no sqlite3, no new
   `find_package`/`ament_target_dependencies`.

   **Amended after plan review (finding 2): `ledgerState()` does NOT move.** It
   calls `prepareOrThrow()`/`SqliteError`, which live in
   `survey_index_bag_main.cpp`'s anonymous namespace and are still needed by
   main's re-index path — moving it would drag those into a second installed
   header for no gain. Instead `ledgerState()` stays where it is and delegates
   its comparison to `fingerprintMatches()`. The skip *decision* is what needed
   to become testable, and that is what moves; the SQL row lookup around it is
   not where the bug lives.

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
   initializer.** Carry an explicit `bool mtime_valid = false` in
   `BagFingerprint` alongside `mtime_ns`, and take the max only over readings
   that actually succeeded — `mtime_valid` becomes `true` on the first success
   and the max is seeded from that first value rather than from any in-range
   constant. No `0` initializer and no `INT64_MIN` sentinel: the validity of
   the value is carried as its own field, not encoded in the value.

   Take **size and mtime from the same `::stat()` call** (plan-review
   suggestion): today a file whose timestamp read fails still contributes its
   bytes to `size_bytes`, which is exactly the half-readable state the ledger
   should not treat as authoritative.

4. **Decide and implement the unreadable-mtime policy.** Per the issue and
   `review-issue`'s Principle Alignment note (Human control and
   transparency: "the implementer should make the unreadable-mtime failure
   mode loud"): if `fingerprint()` walks a bag and no timestamp could be read
   at all — every `::stat` failed, or the bag holds no regular files — the
   fingerprint must **fail the unchanged test**, not pass it. An index bug
   must not re-hide as a skip. Also log a `std::cerr` warning naming the bag
   path, consistent with how `main()` already reports errors.

   **Amended after plan review (finding 1, HIGH). The sentinel design was
   wrong and is dropped.** The write path binds the fingerprint's `mtime_ns`
   into `bags` on both UPDATE and INSERT
   (`survey_index_bag_main.cpp:651-667`), so a sentinel would be *persisted*:
   run 1 re-indexes and stores `INT64_MIN`, then on run 2
   `stored == fp.mtime_ns` is `INT64_MIN == INT64_MIN` and the bag is
   **skipped** — the loud policy defeating itself on the second run, and the
   same failure class as the bug being fixed.

   The fix is to make validity, not a magic value, drive the decision:
   `fingerprintMatches()` returns `false` whenever `!current.mtime_valid`,
   **before** comparing anything. Then no in-band value can round-trip through
   the database and re-create the hole, whatever gets stored. The write path
   persists a plain `0` for an unknown mtime, and that stored value is never
   load-bearing: if the timestamp becomes readable later, `0` versus a real
   reading mismatches and the bag re-indexes, which is correct.

   The test for this must assert the **second run also re-indexes**, not
   merely that the first one does. The pre-amendment test — asserting the
   result does not match a `(0, 0)` row — passes with the hole wide open, as
   the plan review pointed out.

5. **Update `survey_index_bag_main.cpp`** to `#include
   "marine_survey_index/bag_fingerprint.hpp"`, drop the moved
   `BagFingerprint`/`fingerprint()` definitions, and rewrite `ledgerState()`'s
   comparison as a `fingerprintMatches()` call.

   **Amended after plan review (finding 2):** the claim that call sites are
   unchanged was false. Leaving the anonymous namespace for
   `namespace marine_survey_index` makes them `marine_survey_index::fingerprint(...)`
   and `marine_survey_index::fingerprintMatches(...)`, matching every other
   core-lib call already in this file (lines 405, 432, 440, 494). The write
   path also changes, to persist `0` rather than an in-band sentinel when
   `mtime_valid` is false.

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
| `marine_survey_index/include/marine_survey_index/bag_fingerprint.hpp` | New. Declares `BagFingerprint` (with `mtime_valid`), `fingerprint()`, `fingerprintMatches()`. `ledgerState()` stays in the executable — plan-review finding 2. |
| `marine_survey_index/src/bag_fingerprint.cpp` | New. Moves + fixes the implementations (one `::stat()` for size and mtime, explicit `mtime_valid`, no sentinel, loud unreadable-mtime policy). |
| `marine_survey_index/src/survey_index_bag_main.cpp` | Remove the moved struct/function; include the new header; `ledgerState()` delegates to `fingerprintMatches()`; qualify call sites; write path stores `0` for an unknown mtime. |
| `marine_survey_index/CMakeLists.txt` | Add `bag_fingerprint.cpp` to the core lib; add the new gtest target; refresh the stale core-lib contents comment at lines 29-31 — plan-review finding 3. |
| `marine_survey_index/test/test_bag_fingerprint.cpp` | New. The four test cases above, with a `SetUp`/`TearDown` fixture for the temp tree modelled on `test_query_join.cpp` — a failing `ASSERT_*` returns early and would otherwise leak it. |
| `docs/survey_index_schema.md` | Document the new unreadable-mtime rule in the "Incremental re-runs" contract — plan-review finding 3. |

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
| New test binary in CMakeLists | `docs/survey_index_schema.md` build/test instructions | N/A — that doc doesn't enumerate test binaries by name |
| An unreadable mtime now forces a re-index | `docs/survey_index_schema.md`'s "Incremental re-runs" contract | **Yes — amended after plan review (finding 3).** This is a *new* rule, not the code catching up to an existing description: the doc's contract says a bag whose `path`, `size_bytes` and `mtime_ns` all match is skipped, and says nothing about a timestamp that cannot be read. One sentence, this PR. |
| The fingerprint carries a validity flag rather than an in-band value | The write path, which persists `mtime_ns` on both UPDATE and INSERT | **Yes — amended after plan review (finding 1).** This is the consequence the original plan missed: any sentinel the fingerprint carries gets stored and then compares equal to itself on the next run, restoring the silent skip. Validity is therefore checked before the comparison, and the persisted value is never load-bearing. |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR), **amended after plan review
  (finding 3)**:
  - `docs/survey_index_schema.md`, "Incremental re-runs" —
    the unreadable-mtime rule is genuinely **new**. The existing sentence says
    a bag whose `path`, `size_bytes` and `mtime_ns` all match its ledger row is
    skipped, and is silent on a timestamp that cannot be read; that case now
    forces a re-index and warns. One sentence.
  - `marine_survey_index/CMakeLists.txt:29-31` — the comment enumerating the
    core library's contents goes stale when `bag_fingerprint.cpp` joins it.
  - The `BagFingerprint` doc-comment does correctly describe the intended
    contract already, so it moves across as-is apart from naming the new
    validity field.
- **Agent-instruction candidates** (proposals only): None in this plan. The
  general libstdc++ `file_clock` epoch pitfall was already split out by
  operator decision to
  [rolker/ros2_agent_workspace#623](https://github.com/rolker/ros2_agent_workspace/issues/623)
  and is out of scope here.

## Open Questions

- ~~Test timing precision~~ — **settled at plan review, no longer open.** The
  in-place-rewrite test **forces** the timestamp rather than relying on
  wall-clock granularity. Use the two-argument
  `std::filesystem::last_write_time(path, t)` **setter**, which is C++17 and
  converts the epoch correctly inside libstdc++, so it is safe to use even in a
  test policing this very bug (`utimensat` is the alternative if touching
  `<filesystem>` in this test is unwelcome). Same line count as asserting the
  values merely differ, it removes an if-it-flakes-in-CI branch from the
  implementer, and it lets the accuracy test assert an **exact** value instead
  of a ±2 s window — which would catch a future seconds-versus-nanoseconds
  scaling slip that the window would not.

No open questions remain.

## Estimated Scope

Single PR, single project repo (`unh_marine_autonomy`), single package
(`marine_survey_index`). Five files touched (two new).
