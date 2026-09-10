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
chain just to construct one function's inputs. Moving the skip *decision*
into the core lib is the natural fix and matches the existing
library/executable split in this package. (`ledgerState()` itself stays in the
executable — see the amendment under Approach step 1.)

## Approach

1. **Add `marine_survey_index/include/marine_survey_index/bag_fingerprint.hpp`
   and `marine_survey_index/src/bag_fingerprint.cpp`.** Move `BagFingerprint`
   and `fingerprint()` out of `survey_index_bag_main.cpp` into this new pair
   (renamed `bagFingerprint()` at pre-push review round 1 — `fingerprint()` is
   too generic a name for an exported symbol)
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

   **Amended after pre-push review round 1 (must-fix 2): the multiply is
   range-checked.** `tv_sec * 1e9` is signed-overflow UB past roughly the year
   2262 (or before 1678), and the wrapped result would be stored and compared
   as an ordinary mtime — reachable from a corrupt inode or a host whose clock
   came up wrong, not only from a deliberate `touch`. As shipped, `statFile()`
   validates `tv_sec` against the largest/smallest values whose ns conversion
   fits in `int64_t` and `tv_nsec` into [0, 1e9) *before* multiplying, and
   treats an out-of-range timestamp exactly like an unreadable one.

   The `st_mtim` field is POSIX.1-2008; macOS spells it `st_mtimespec`. This
   package targets Linux (ROS 2 Jazzy on Ubuntu), so no shim is carried — the
   trade-off is recorded in the new header (plan-review finding 6).

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
   (`survey_index_bag_main.cpp:651-667` **as the file stood before this
   branch** — every line citation in this plan is pre-change), so a sentinel
   would be *persisted*:
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

   **Amended after pre-push review round 1 (must-fix 1): "no timestamp at all"
   was too narrow a policy.** The reviewer reproduced the same silent-skip bug
   one level up: an unreadable *subdirectory* was swallowed by
   `skip_permission_denied`, `is_regular_file`'s `error_code` was discarded and
   a failed per-file `::stat` returned silently, so a **partial** walk produced
   a *stable* size and mtime that matched its own stored copy run after run —
   a permanent silent skip of a changed bag.

   As shipped, `BagFingerprint` carries a second trust flag, `scan_complete`,
   cleared by every way the walk can come up short: an undeterminable bag type,
   a directory that cannot be opened, a walk that stops early (the diagnostic
   names the entry it stopped at, not the bag root), an entry whose type cannot
   be read, a failed or out-of-range `::stat`, or a single path that is not a
   regular file (a FIFO used to fingerprint as size 0 with a moving timestamp).
   `skip_permission_denied` is gone — the iteration is still non-throwing
   (#259's requirement), but an unreadable entry is now recorded instead of
   hidden. `fingerprintMatches()` requires `mtime_valid` **and**
   `scan_complete` before comparing anything.

   The diagnostic moved **out of the library** to the CLI call site: the core
   lib is linked into `marine_perception_tools`' Qt GUI where stderr is
   invisible, so `bagFingerprint()` reports the reason through an optional
   `problem` out-param and stays silent itself. The CLI counts
   not-fully-readable and failed bags, reports both in the run summary, and
   **exits non-zero** on either — a bag re-indexing forever previously left no
   signal a script or scheduler could see.

   The test that matters most is the subdirectory route (identical size and
   mtime across a content change behind it) plus the production-reachable
   unlistable bag directory that `scanForBags` does nominate.

   **Amended after pre-push review round 2 (must-fixes 1-3).** Three things
   changed here as shipped:

   - **What clears `scan_complete`.** `directory_entry::is_regular_file()`
     follows symlinks, so a symlink *to a directory* answered "not a regular
     file" with no error and the walk (which does not follow directory
     symlinks) never descended: the whole subtree behind it was invisible and
     *stable*, i.e. the #375 failure class one level up, reachable through
     `--scan`. Entries are now classified through `symlink_status()` first, and
     the line is drawn at what the walk *knows*: content it could not see
     clears the flag; an entry definitively carrying no bag bytes (FIFO,
     socket, device node, a symlink resolving to nothing) does not — a
     dangling symlink hides nothing and must not cost a permanent re-index.
   - **The exit status gained causes**, since a `set -euo pipefail` store build
     is the consumer: 1 = the index is incomplete (a bag failed or could not be
     opened, or a `--scan` tree was not fully enumerated), 2 = usage,
     3 = complete but a bag re-indexes every run. A bag whose reader could not
     be opened at all was previously counted nowhere and exited 0.
     `scanForBags` no longer passes `skip_permission_denied` and no longer
     discards its `error_code`s.
   - **Both trust flags are now guarded by tests that do not need a non-root
     user**, because the verification that gates a merge here runs as root — a
     permission-based test that skips there defends nothing (mutation-proved:
     the `scan_complete` conjunct broke zero tests). The symlinked-subdirectory
     route reaches `mtime_valid && !scan_complete` directly, and the flag truth
     table is also asserted on a directly constructed fingerprint.

     **Corrected at round 4.** This repo *does* have a hosted workflow
     (`.github/workflows/ros-base-docker.yml`), but it lists
     `marine_survey_index` in neither its build nor its test set and nothing
     depends on the package — so the root-run `ci_local.sh` attestation
     (ADR-0018) is not merely the *preferred* gate here, it is the **only**
     verification this package's tests ever run under. That is what makes a
     root skip total rather than partial, and it is why round 4 added two
     root-observable routes (below).

5. **Update `survey_index_bag_main.cpp`** to `#include
   "marine_survey_index/bag_fingerprint.hpp"`, drop the moved
   `BagFingerprint`/`fingerprint()` definitions, and rewrite `ledgerState()`'s
   comparison as a `fingerprintMatches()` call.

   **Amended after plan review (finding 2):** the claim that call sites are
   unchanged was false. Leaving the anonymous namespace for
   `namespace marine_survey_index` makes them `marine_survey_index::fingerprint(...)`
   and `marine_survey_index::fingerprintMatches(...)`, matching every other
   core-lib call already in this file (pre-change lines 405, 432, 440, 494).
   The write
   path also changes, to persist `0` rather than an in-band sentinel when
   `mtime_valid` is false — as shipped it binds `fp.mtime_ns` directly, since
   the struct's invariant keeps that field `0` while `mtime_valid` is false.
   The exported helper that wrapped this (`fingerprintStoredMtime()`) was
   dropped at pre-push review round 1: it was provably an identity given that
   invariant, which is now stated on the struct instead.

6. **Update `marine_survey_index/CMakeLists.txt`**: add `src/bag_fingerprint.cpp`
   to the `${PROJECT_NAME}_core` library's source list (alongside
   `schema.cpp`, `footprint.cpp`, `query.cpp`); add a new
   `ament_add_gtest(test_bag_fingerprint test/test_bag_fingerprint.cpp)` +
   `target_link_libraries(test_bag_fingerprint ${PROJECT_NAME}_core)` in the
   existing test block (after `test_query_join`).

7. **Add `marine_survey_index/test/test_bag_fingerprint.cpp`** covering the
   cases the issue names, using `std::filesystem::temp_directory_path()`
   + a per-test unique subdirectory (created/removed in the fixture, mirroring
   the temp-file pattern in `test_schema.cpp`). As shipped, and amended for the
   two reviews:
   - **mtime accuracy**: fingerprint a temp file and assert `fp.mtime_ns`
     equals `::stat`'s own reading of the same file **exactly** — not a ±2 s
     window, which would also accept a seconds-versus-nanoseconds scaling slip
     (settled at plan review; see Open Questions).
   - **in-place rewrite at identical size** (the actual stale-data scenario):
     write a file, force a distinct mtime with the two-argument
     `last_write_time` setter rather than sleeping, overwrite its content with
     different bytes of the *same length*, re-fingerprint, and assert
     `size_bytes` is unchanged but the fingerprint no longer matches.
   - **ledger round-trip at the decision, not at the SQL.** `ledgerState()`
     stays in the executable (Approach step 1's amendment), so the round-trip
     is exercised through `fingerprintMatches()` — same size / different mtime,
     different size / same mtime, and an unindexed-looking `(0, 0)` row. No
     temp sqlite DB and no schema dependency in this test.
   - **untrustworthy-fingerprint policy**, one test per route, each asserting
     the bag cannot read as unchanged: no timestamp anywhere (asserting the
     **second** run also re-indexes, since a sentinel would round-trip through
     the ledger), a partial walk at an unreadable subdirectory, an unlistable
     mode-0111 bag directory, a non-regular single path, an unrepresentable
     year-2500 mtime, and the legacy `mtime_ns = 0` migration row. The
     permission-dependent ones skip under root, and the year-2500 one skips if
     the filesystem clamps or refuses the timestamp — a test must not assert
     filesystem behaviour it cannot guarantee.

     **As shipped, this grew past the outline above** (19 cases in
     `test_bag_fingerprint`, the last added at round 4: a missing bag URI must
     read as missing, not as undeterminable): a symlinked subdirectory, an
     unresolvable symlink
     (a loop — ELOOP, which root is not exempt from), the flag truth table on a
     constructed fingerprint, the routes that must *not* be penalised (dangling
     symlink, FIFO, symlink to a regular file), the problem-string/flags
     agreement invariant, and an unreadable member of a listable directory.
     Every permission-based route in `test_bag_fingerprint` has a
     root-observable counterpart guarding the same branch — see the round-2
     amendment in step 4. `test_indexer_exit_status`'s scan-walk routes are
     accounted for separately, in step 8.
   - **the exit-status contract**, in a separate `test_indexer_exit_status`
     that runs the built `survey_index_bag` binary, because the contract lives
     in `main()` where no library call reaches it (which is how the
     unopenable-bag hole survived a full review round): an unopenable bag, a
     real empty rosbag2 bag with a symlinked subdirectory (exit 3), a dropped
     `--scan` subtree, and a usage error.

     **As shipped this grew, like `test_bag_fingerprint` above: 20 cases.**
     The additions are the scan-walk continuation invariants (rounds 2-4), the
     ledger-key and ledger-migration cases (rounds 3-4), the argument-parsing
     usage errors and the lock-wait opt-in (round 4) — each listed where it was
     added, in steps 4 and 8.

8. **Added at pre-push review round 4** — four fixes the review reproduced or
   disproved, all inside the contract this PR authors rather than new scope:

   - **An unknown flag, or a flag with no value, is a usage error (2).** The
     argument loop assumed every `--`-prefixed token took a value, so
     `--verbose BAG` swallowed the bag and exited **0**, and a trailing
     `--scan` (i.e. `--scan $ROOT` with `ROOT` unset and unquoted) walked no
     tree and exited **0** — while the quoted spelling of the same slip already
     exited 1. Pre-existing parsing, fixed here on the standing that this PR is
     what promises exit 0 means every nominated bag is in the index. Flags now
     live in one table with whether each takes a value, and the list is
     validated before any value is read.
   - **A pre-fix ledger row keyed through a symlink is migrated, not
     orphaned.** Round 3's resolved key (`bags.path`) made such a row
     unmatchable: the lookup missed, a second row was inserted, and the stale
     row's passes double-reported that bag through the undeduplicated join
     permanently. The indexer now reconciles the ledger once before looking up
     any bag — re-key in place where the resolved key is free, delete the stale
     duplicate (CASCADE takes its passes) where it is taken, leave and warn
     about a row that cannot be resolved at all, and **refuse to index** if the
     reconciliation itself fails. Verified a no-op on this host (0 of 177 rows
     re-key); salmon and gabby are unverified, which is who it is for. A key
     that cannot be resolved now also fails its *bag* rather than falling back
     to the unresolved path, which had re-created the double insert on a
     transient EIO/ESTALE.
   - **The lock wait moved to the call site.** `openIndexDb` is also the
     explorer GUI's open (not a read-only one, as `schema.cpp` claimed), and
     every GUI call runs on the Qt thread, so `PRAGMA busy_timeout` is now a
     parameter defaulting to `0` — pre-#375 behaviour for every other consumer
     — with the indexer opting into 10 s. WAL was weighed and rejected
     (unsafe on network filesystems).
   - **Two of the three scan-walk guards became root-observable**, because
     `ci_local.sh` runs as root and is this package's only merge verification:
     the enumerability probe via descriptor exhaustion (`ulimit -n 256` over a
     400-deep chain — no mode bits, and lowering a soft limit is always
     permitted), and the mid-index handler via a per-bag-selective write
     failure, which is also the only shape that can defend the `ROLLBACK` (a
     single-bag run cannot: SQLite rolls back at `sqlite3_close` regardless, so
     only a *subsequent* bag's `BEGIN` observes it). The mode-000 `meta_ec`
     guard has no root-observable route and is recorded as the known exception.
     The permission-based siblings are kept for the real failures they
     reproduce.

## Files to Change

| File | Change |
|------|--------|
| `marine_survey_index/include/marine_survey_index/bag_fingerprint.hpp` | New. Declares `BagFingerprint` (with `mtime_valid` **and `scan_complete`**), `bagFingerprint()` (with its optional `problem` out-param), `fingerprintMatches()`. `ledgerState()` stays in the executable — plan-review finding 2. No `fingerprintStoredMtime()`: dropped at pre-push review round 1 as an identity over the struct's own invariant. |
| `marine_survey_index/src/bag_fingerprint.cpp` | New. Moves + fixes the implementations (one range-checked `::stat()` for size and mtime, explicit `mtime_valid`/`scan_complete`, no sentinel, silent library + reason reported to the caller). |
| `marine_survey_index/src/survey_index_bag_main.cpp` | Remove the moved struct/function; include the new header; `ledgerState()` delegates to `fingerprintMatches()`; qualify call sites; write path binds `fp.mtime_ns`; warn per bag on an untrustworthy fingerprint, count not-fully-readable and failed bags in the run summary, and exit non-zero on either. |
| `marine_survey_index/CMakeLists.txt` | Add `bag_fingerprint.cpp` to the core lib; add the new gtest target; refresh the stale core-lib contents comment at lines 29-31 — plan-review finding 3. |
| `marine_survey_index/test/test_bag_fingerprint.cpp` | New. The test cases above, with a `SetUp`/`TearDown` fixture for the temp tree modelled on `test_query_join.cpp` — a failing `ASSERT_*` returns early and would otherwise leak it. |
| `marine_survey_index/test/test_indexer_exit_status.cpp` | New (round 2). Runs the built `survey_index_bag` to police the exit-status contract, which lives in `main()`. |
| `marine_survey_index/README.md` | The incremental-skip sentence, the exit-status table, and the `## Testing` enumeration. |
| `docs/survey_index_schema.md` | Document the new unreadable-mtime rule in the "Incremental re-runs" contract — plan-review finding 3. Round 3 also added the network-mount attribute-cache case to "what size + mtime cannot see", and the `bags.path` column comment now says symlinks are resolved. |
| `marine_survey_index/package.xml` | Added at round 3. Declare the rosbag2 storage plugins: they are loaded by name at run time, so `rosbag2_cpp` does not pull one in, and without them the indexer opens no bag under a clean-room `rosdep install` — the environment `ci_local.sh` uses — which is this package's only merge verification, since the repo's one hosted workflow builds and tests neither it nor anything depending on it (corrected at round 4; the plan previously said the repo had no hosted workflow at all). |
| `marine_survey_index/src/schema.cpp` | Added at round 3. `PRAGMA busy_timeout` on open, so a GUI holding the index DB during the one-time re-index does not turn indexer writes into failed bags. |
| `marine_survey_index/test/test_schema.cpp` | Added at round 3. Asserts the open waits for a lock — at round 4, that it waits **only when asked**, since the default now serves a GUI thread. |
| `marine_survey_index/include/marine_survey_index/schema.hpp` | Added at round 4. `openIndexDb` gains `busy_timeout_ms = 0`; the docstring states that opening writes (it runs the DDL) and that the handle may block. |
| `marine_survey_index/src/survey_index_bag_main.cpp` (round 4) | Argument validation against a flag table; `ledgerKey()` as one function, failing the bag when it cannot resolve; `reconcileLedgerKeys()` before any lookup; the 10 s lock wait opted into at this call site; nominations deduplicated by resolved key; the scan-problem count in the run summary; `ledgerState()` throwing on an unexpected `sqlite3_step` result; `disable_recursion_pending()` on the undeterminable-type branch. |
| `marine_survey_index/test/test_indexer_exit_status.cpp` (round 4) | The argument-parsing usage errors, the two ledger-migration cases, the lock-wait opt-in, the descriptor-exhaustion walk route, and the per-bag-selective failure that defends the `ROLLBACK`. |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Test what breaks | The new tests target the exact regressions named in the issue (mtime accuracy, same-size rewrite, ledger round-trip) plus the unreadable-mtime and incomplete-walk policies this plan adds — not framework glue. Each guard is mutation-tested: removing the condition it defends fails at least one test. Round 3 found this claim was not yet true (the CLI's exit-1 scan conjunct broke no test, and neither did `!fp.authoritative()`), and round 4 found the *reason* recorded for one of the two remaining exceptions was itself wrong. As it now stands, one guard has no behavioural test and one skips as root: **`!fp.authoritative()`** is provably biconditional with the problem string it is checked against, so no behavioural test can separate them; and the **mode-000 `meta_ec` walk guard** has no root-observable route (round 4 could not construct one either) and is the known, recorded exception. The `ROLLBACK` in the mid-index handler is no longer among them — round 4 defends it with a per-bag-selective write failure, the only shape a second bag's `BEGIN` can observe, which is also root-observable. Of the three scan-walk guards, two now run as root (the enumerability probe via descriptor exhaustion, the undeterminable-type branch via ELOOP); their permission-based siblings are kept for the routes they reproduce. |
| A change includes its consequences | PR description will state the one-time full re-index of all 177 existing (`mtime_ns=0`) rows on the dev host, per the operator's note — this is expected derived-cache-rebuild cost, not a regression, and no data migration is needed. |
| Human control and transparency | An untrustworthy fingerprint — no readable timestamp, or an incomplete walk — now fails the unchanged test **on its validity flags** (`mtime_valid` / `scan_complete`), checked before any comparison, instead of silently contributing nothing to a max. The "guaranteed-mismatching sentinel" the first draft of this plan proposed was **rejected at plan review and never shipped**: the ledger persists whatever the fingerprint carries, so an in-band sentinel compares equal to itself on the next run and restores the skip. Loudness is the caller's job: the CLI warns per bag, counts them in the run summary, and exits non-zero. |
| Only what's needed | No ledger schema change, no new DB column, no speculative generalization beyond moving the two functions needed for testability. |
| Improve incrementally | Single PR, single package, bounded diff. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| None of 0001–0013 (workspace or project) | No | Offline-indexer bug fix; no vehicle control, hardware interface, store schema, or transport change. |
| ADR-0018 (local-first CI verification) | **Yes** (recorded at round 4; the table previously claimed no ADR was triggered while the plan leaned on it) | The merge gate for this PR is a full-scope `ci_local.sh` attestation, which is not a preference here but the only verification that ever runs this package's tests — the repo's one hosted workflow builds and tests neither it nor any dependent. `ci_local.sh` runs as root, which is why the root-observability of each new guard is tracked in the Principles Self-Check above rather than assumed. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `mtime_ns` computation becomes non-zero and correct | Existing on-disk indexes (177 rows, `mtime_ns=0`) will compare unequal on next run and each bag re-indexes once | Yes — called out in PR description per the operator's note; no code change needed, the index is a derived cache and self-heals on the next `survey_index_bag` run |
| `bagFingerprint()`/`fingerprintMatches()` move to the core lib (`ledgerState()` does not — Approach step 1's amendment) | `survey_index_bag_main.cpp`'s includes and any other in-package caller | Yes — the only caller is `main()`, updated in step 5; no caller outside this package exists yet (checked across the workspace's checked-out repos) |
| New test binary in CMakeLists | `docs/survey_index_schema.md` build/test instructions | N/A — that doc doesn't enumerate test binaries by name |
| `bags.path` becomes the **symlink-resolved** path (round 3), changing a persisted column's meaning | (a) existing rows keyed through a symlink, which no future run could match — the indexer now re-keys them, or deletes the duplicate when the resolved key is taken, or refuses to run; (b) `docs/survey_index_schema.md`, which said no migration was needed; (c) consumers matching `bags.path` by exact string — `cube_bathymetry/src/batch_regen_main.cpp:421` (already degrades conservatively) and `marine_perception_tools/src/sidescan_viewer_window.cpp:4058` (compares uncanonicalised dialog input for bag identity) | Yes for (a) and (b), added at round 4 — the migration and its refusal path, plus the doc's consumer note. (c) in `marine_perception_tools` is a follow-up on the consumer, out of scope here |
| `openIndexDb` gains a caller-chosen `busy_timeout_ms` (round 4) | The GUI consumer's behaviour: the default is `0`, i.e. exactly what every consumer had before this PR, so no out-of-repo call site changes meaning; the indexer opts into 10 s at its own call site | Yes — and documented in the schema doc's concurrency ground rule |
| An unreadable mtime now forces a re-index | `docs/survey_index_schema.md`'s "Incremental re-runs" contract | **Yes — amended after plan review (finding 3).** This is a *new* rule, not the code catching up to an existing description: the doc's contract says a bag whose `path`, `size_bytes` and `mtime_ns` all match is skipped, and says nothing about a timestamp that cannot be read. Widened at pre-push review round 1 (see Documentation & Instruction Impact). |
| A partial walk now forces a re-index too | The schema doc, the package README, and the walk itself (`skip_permission_denied` had to go) | **Yes — amended after pre-push review round 1 (must-fix 1).** The affected bag re-indexes on *every* run until the cause is fixed. That is deliberate and is the safe direction — the alternative, which is what shipped before this round, is skipping a changed bag permanently — but it needs the loud signal below to be actionable. |
| `survey_index_bag` now exits non-zero when a bag could not be fingerprinted or failed mid-index | Nothing automated — no script, launch file or test in this workspace consumes this binary's exit status (checked); the README documents the new behaviour | **Yes — amended after pre-push review round 1 (suggestion).** A bag re-indexing every run forever, or missing from the index after a mid-index failure, previously reported success. |
| The fingerprint carries a validity flag rather than an in-band value | The write path, which persists `mtime_ns` on both UPDATE and INSERT | **Yes — amended after plan review (finding 1).** This is the consequence the original plan missed: any sentinel the fingerprint carries gets stored and then compares equal to itself on the next run, restoring the silent skip. Validity is therefore checked before the comparison, and the persisted value is never load-bearing. |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR), **amended after plan review
  (finding 3)**:
  - `docs/survey_index_schema.md`, "Incremental re-runs" —
    the unreadable-mtime rule is genuinely **new**. The existing sentence says
    a bag whose `path`, `size_bytes` and `mtime_ns` all match its ledger row is
    skipped, and is silent on a timestamp that cannot be read; that case now
    forces a re-index and warns. **Widened at pre-push review round 1** from
    one sentence to the column's units and epoch, the incomplete-walk case, the
    one-time re-index of every pre-fix `mtime_ns = 0` row, and the residual
    holes size + mtime cannot see (mtime-preserving rewrites, coarse timestamp
    granularity, hardlink double-counting) — the doc self-declares as the
    stable cross-stage contract, so these belong in it rather than in the PR
    body (must-fix 3).
  - `marine_survey_index/README.md` — the incremental-skip sentence and the
    `## Testing` enumeration, both stale once the policy and the test set
    changed (must-fix 4 and 5).
  - `marine_survey_index/CMakeLists.txt:29-31` — the comment enumerating the
    core library's contents goes stale when `bag_fingerprint.cpp` joins it.
  - The `BagFingerprint` doc-comment was **not** carried across as-is (this
    plan's original claim): pre-push review round 1 had it rewritten into
    house style with the epoch rationale and the struct invariant, and round 2
    corrected its account of what clears `scan_complete`.
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
(`marine_survey_index`). **Eleven files touched (four new)** as landed: the new
header and source, `survey_index_bag_main.cpp`, `schema.cpp`, the two new tests
(`test_bag_fingerprint.cpp`, `test_indexer_exit_status.cpp`),
`test_schema.cpp`, `CMakeLists.txt`, `package.xml`,
`docs/survey_index_schema.md`, and the package README. (Estimated as eight and
three before the three review rounds; `package.xml`, `schema.cpp` and
`test_schema.cpp` are round-3 additions, and the count of new files was wrong
as written.)
