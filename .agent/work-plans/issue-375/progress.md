---
issue: 375
---

# Issue #375 — marine_survey_index: bags.mtime_ns is always 0, so the incremental-skip ledger compares size only and stale bags are never re-indexed

## Issue Review
**Status**: complete
**When**: 2026-09-10 10:10 -04:00
**By**: Claude Code Agent (Claude Sonnet 5)

**Issue**: #375
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Actions
- [ ] Recommendation: the root cause — `std::filesystem::last_write_time()` returning a `file_time_type` whose epoch (libstdc++'s `file_clock`, 2174-01-01) is not the Unix epoch, so a raw `time_since_epoch()` read is silently wrong-signed for any real file — is a general C++17/`<filesystem>` pitfall, not specific to this package. `grep -rl last_write_time` across this repo found only this one call site, so no other latent instance exists here today, but the gotcha is worth a `.agent/knowledge/` (or `ros2_development_patterns.md`) candidate note so a future `fs::last_write_time()` use elsewhere in the workspace converts through `file_clock::to_sys`/`::stat` instead of re-deriving the same bug. Propose only — per the consequences map, an instruction-update candidate needs the operator's approval before landing, and it's out of scope for this bug-fix PR.

### Notes (not part of the review-issue schema; kept for plan-task context)

#### Scope Assessment
**Well-scoped?** Yes. The issue pins the exact root cause (`fs::last_write_time`'s
`file_time_type` is measured from libstdc++'s `file_clock` epoch, 2174-01-01, not
Unix epoch), the exact fix shape (convert through the clock — `file_clock::to_sys`
(C++20) or `::stat`/`statx` at C++17 — plus a non-zero/non-swallowable sentinel and
an explicit "unreadable mtime" policy), and three concrete unit tests that are
stated to fail today. Verified against
`marine_survey_index/src/survey_index_bag_main.cpp`: `struct BagFingerprint`
(line 154), `fingerprint()` (line 160), the `std::max(fp.mtime_ns, ...)`
accumulation (line 174), and the `ledgerState()` unchanged-test at line ~292 all
match the issue's quotes and line numbers exactly. This is a single-file,
single-package fix with a clear, bounded test plan — no split needed.

**Right repo?** Yes. `marine_survey_index` lives in `unh_marine_autonomy`
(project repo, domain code — the survey indexer's incremental-skip correctness).
No workspace-infra content involved.

**Dependencies**: None blocking. The issue itself correctly identifies and scopes
the one soft dependency: existing indexes have `mtime_ns = 0` for all 177 rows on
the dev host, and once the fix lands every bag will re-index once (a one-time,
expected cost, not a bug) — the issue explicitly flags this as "worth stating in
the PR." No open issue needs to land first; #259 (stage 1, closed) is the origin
of the code being fixed but doesn't block this.

#### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Test what breaks | OK | The three proposed tests target the actual regressions (mtime accuracy, same-size-different-content re-fingerprint, ledger round-trip skip/re-index) rather than framework glue. Matches `docs/PRINCIPLES.md`-adjacent "Test what breaks" guidance well. |
| A change includes its consequences | OK | Issue explicitly scopes the one-time full re-index consequence of existing `mtime_ns=0` rows and says to state it in the PR. No stale docs/tests left dangling — `marine_survey_index/test/` has no `fingerprint()` coverage today and the issue adds it. |
| Human control and transparency | Watch | The fix needs an explicit decision on what an unreadable mtime means (issue already flags this: "a bag whose mtime cannot be read should probably fail the unchanged test rather than pass it"). Silently contributing nothing to a max is exactly how the original bug stayed invisible — the implementer should make the unreadable-mtime failure mode loud (log or a distinguishable sentinel), not just directionally safe. |
| Only what's needed | OK | Fix is scoped to the accumulator and its sentinel/policy; no speculative expansion (e.g., no proposal to change the ledger schema or add new columns). |
| Improve incrementally | OK | Single PR, single package, reviewable. |
| Safety First / Hardware Agnosticism / Simulation-First (project PRINCIPLES.md) | N/A | This is an offline indexer bug (derived-cache correctness), not vehicle control, hardware interface, or simulation-validated behavior. |

#### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| 0008 — ROS 2 conventions | Marginal | Touches C++ source in a ROS 2 package, but the fix is a bugfix within the existing C++17 standard already set in `marine_survey_index/CMakeLists.txt` (`CMAKE_CXX_STANDARD 17`). The issue's own fix options (stat/statx path, or C++20 `to_sys` if the standard were bumped) already account for this — no standard bump is implied or needed. |
| Others (0001–0010, 0013) | No | No new agent instructions, enforcement rule, Make target, or `progress.md`-writing skill involved. |

#### Consequences

- `marine_survey_index/test/` gains `fingerprint()` coverage (currently absent) —
  in scope, already called out by the issue.
- Existing on-disk indexes (177 rows on dev host, `mtime_ns=0`) will one-time
  full-reindex after the fix — in scope, already called out by the issue as
  something to state in the PR, not something to pre-migrate.
- Optional/deferred: a `.agent/knowledge/` note on the libstdc++ `file_clock`
  epoch pitfall (see Recommendation above) — proposed as a candidate for
  operator approval, not required for this PR.

#### Recommendations

- Propose (not apply) a `.agent/knowledge/` candidate note on the
  `std::filesystem::last_write_time()` / `file_clock` epoch pitfall, since it's a
  general C++17 `<filesystem>` gotcha that could recur wherever the workspace
  reads file mtimes in the future — repo-wide grep today shows no other call
  site, so this is prevention, not an existing second instance.
- When implementing, make the "unreadable mtime" failure mode observable (e.g.
  a log line or a distinguishable error path) rather than only correct — the
  original bug's danger was precisely that an unreadable/misread mtime failed
  silently into a value that still passed the unchanged test.

---
**Authored-By**: `Claude Code Agent`
**Model**: `Claude Sonnet 5`

## Plan Authored
**Status**: complete
**When**: 2026-09-10 10:18 -04:00
**By**: Claude Code Agent (Claude Sonnet 5)

**Plan**: `.agent/work-plans/issue-375/plan.md` at `fb8bff2`
**Branch**: feature/issue-375 at `fb8bff2`
**Phases**: single

### Open questions
- [ ] In-place-rewrite test timing: default is no artificial sleep (assert distinct `mtime_ns` across rewrite); fall back to forcing an explicit timestamp (e.g. `utimensat`) only if that proves flaky in CI — verify locally during implementation.

## Plan Review
**Status**: complete
**When**: 2026-09-10 10:22 -04:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-375/plan.md` at `fb8bff2`
**PR**: PR-less (`--issue` mode, branch `feature/issue-375`)
**Verdict**: changes-requested

### Findings
- [ ] (must-fix) The `INT64_MIN` unreadable-mtime sentinel is written back into `bags.mtime_ns` (`survey_index_bag_main.cpp:651-667`), so on the next run `ledgerState()`'s `stored == fp.mtime_ns` is `INT64_MIN == INT64_MIN` → unchanged → skip; the loud policy defeats itself on run 2. Plan must state whether `ledgerState()` returns changed whenever the fingerprint is a sentinel, or the write path refuses to persist one — and test that, not the weaker "does not match a (0,0) row" assertion — `plan.md` Approach step 4 / test step 7 bullet 4
- [ ] (must-fix) `ledgerState()` cannot move alone: it calls `prepareOrThrow()`/`SqliteError`, which live in main.cpp's anonymous namespace (73-321) and are still needed by main's re-index path (638-703). Plan lists only three symbols moving and is silent on the helpers; also, leaving the anonymous namespace for `namespace marine_survey_index` makes step 5's "call sites are unchanged" false (they become `marine_survey_index::fingerprint(...)`, as every other core-lib call in main.cpp already is) — `plan.md` Approach steps 1 and 5
- [ ] (must-fix) The unreadable-mtime policy is a NEW observable contract that `docs/survey_index_schema.md:126` ("Incremental re-runs", the stated cross-stage contract) does not cover, so "Documentation & Instruction Impact: None — this PR makes the implementation match the existing description" is right for the mtime fix and wrong for the new policy; one sentence there, this PR. `CMakeLists.txt:29-31`'s comment enumerating the core lib's contents also goes stale — `plan.md` Documentation & Instruction Impact
- [ ] (suggestion) Settle the Open Question instead of deferring: force the timestamp (`std::filesystem::last_write_time(path, t)` as a *setter* is C++17 and converts the epoch correctly inside libstdc++, so it is safe even in a test policing this bug; or `utimensat`). Same line count as "assert they differ", removes an if-it-flakes-in-CI branch, and lets the accuracy test assert an exact value rather than the plan's ±2 s window — which would not catch a future seconds-vs-nanoseconds scaling slip — `plan.md` Open Questions
- [ ] (suggestion) Take size and mtime from one `::stat` (`st.st_size` is already in the struct): halves syscalls over the recursive walk and makes "unreadable" one condition instead of two — today's code accumulates a file's size even when its mtime read fails — `plan.md` Approach step 2
- [ ] (suggestion) Record the `::stat` trade in the plan: it is the right call here (C++17 is fixed, `file_clock::to_sys` is C++20, and `marine_bathymetry_store/src/tile_io.cpp:721` is in-repo precedent). Consequences to note: `st_mtim` is POSIX.1-2008, not the macOS spelling (`st_mtimespec`) — fine for a Linux/jazzy package but newly non-portable where `<filesystem>` was not; no precision loss either way (both nanosecond-resolution, both follow symlinks, matching the walk's `is_regular_file`); a later C++20 bump leaves `::stat` correct, so no follow-up debt — `plan.md` Approach step 2
- [ ] (suggestion) stderr is the right channel — `main()` already reports every error there, so a different channel would be the inconsistency; the every-run repetition is correct loudness under automation (one line per affected bag). Be deliberate that this only reads as loud once finding 1 is fixed; unfixed it is a warning printed next to a silent skip — `plan.md` Approach step 4
- [ ] (suggestion) Mirror `test_query_join.cpp`'s fixture (SetUp/TearDown, `:memory:`) for the temp-directory lifecycle, not `test_schema.cpp` as cited — no test in this package touches the filesystem today and `test_schema.cpp` uses `:memory:`. Cleanup belongs in `TearDown()`: a failing `ASSERT_*` returns early and would leak the temp tree — `plan.md` Approach step 7

---
**Authored-By**: `Claude Code Agent`
**Model**: `Claude Opus`


## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-09-10 10:43 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-375 at `3bb5a65`
**Mode**: pre-push
**Depth**: Deep (reason: 816 changed lines across 8 files)
**Must-fix**: 6 | **Suggestions**: 12
**Round**: 1 | **Ship**: continue — the fix is correct in its core, but two reachable correctness holes remain (a partial directory walk still reports `mtime_valid == true`, and the ns arithmetic overflows), both reproduced locally

Verified independently, not taken on trust: clean rebuild with `-Wall -Wextra` (zero warnings), 133 tests / 0 failures / 18 skipped, the 5 new tests pass, and both rejected implementations were re-mutated and re-run — the pre-fix raw `file_time_type` read fails 3 tests (all `mtime_ns == 0`) and the in-band `INT64_MIN` sentinel fails the unreadable-timestamp test on both the round-trip and the second-run skip. cpplint / uncrustify / lint_cmake / copyright all clean.

### Findings
- [x] (must-fix) a partially-walked bag still reports `mtime_valid == true`, so a stable partial fingerprint skips a changed bag forever — reproduced: an unreadable subdirectory yields identical `size/mtime/valid=1` across a content change inside it — `marine_survey_index/src/bag_fingerprint.cpp:65-86`
- [x] (must-fix) `tv_sec * kNsPerS` is signed-overflow UB for mtimes past ~2262 and the wrapped negative value is marked valid — reproduced with `touch -d 2500-01-01` — `marine_survey_index/src/bag_fingerprint.cpp:47-48`
- [x] (must-fix) the one-time mass re-index of every existing `mtime_ns = 0` row, and the epoch/units of the column, are documented only in the not-yet-written PR body — both belong in the doc that self-declares as the stable cross-stage contract — `docs/survey_index_schema.md:44,125-133`
- [x] (must-fix) the package README's incremental-skip sentence still implies an unreadable-mtime bag can be skipped — `marine_survey_index/README.md:33-34`
- [x] (must-fix) the README `## Testing` paragraph enumerates covered areas and omits the new bag-fingerprint / incremental-skip test — `marine_survey_index/README.md:66-69`
- [x] (must-fix) the plan still credits the rejected "guaranteed-mismatching sentinel" as the shipped design — a future reader would reimplement the bug plan review #1 killed — `.agent/work-plans/issue-375/plan.md:178`
- [x] (suggestion) `::getpid()` used with no `<unistd.h>`; compiles only transitively through gtest — `marine_survey_index/test/test_bag_fingerprint.cpp:52`
- [x] (suggestion) the comments justify persisting `0` by a range claim the code disproves (`touch -d @0` fingerprints as `mtime=0 valid=1`); only the validity gate makes it safe — `marine_survey_index/src/bag_fingerprint.cpp:101-103`, `include/marine_survey_index/bag_fingerprint.hpp:79-83`
- [x] (suggestion) move the `std::cerr` warning from the exported library to the CLI call site — the function already returns the fact as `mtime_valid`, and `marine_perception_tools` links this library into a Qt GUI where stderr is invisible — `marine_survey_index/src/bag_fingerprint.cpp:87-92`
- [x] (suggestion) no unreadable-timestamp bucket in the run summary and the exit code stays 0 even after a per-bag failure — a bag re-indexing every run forever has no durable signal — `marine_survey_index/src/survey_index_bag_main.cpp:684-686`
- [x] (suggestion) the single-path branch applies no regular-file guard (a FIFO fingerprints as `size=0`, changing every run) and `is_directory`'s `ec` is discarded — `marine_survey_index/src/bag_fingerprint.cpp:72,84-85`
- [x] (suggestion) `fingerprintStoredMtime()` is provably an identity given the struct's own invariant; drop it or document the invariant it guards — `include/marine_survey_index/bag_fingerprint.hpp:78-83`
- [x] (suggestion) missing tests: the legacy-row migration case (`fingerprintMatches(fp, fp.size_bytes, 0)`), an assertion that the warning is actually emitted, and the production-reachable unreadable route (a `chmod 0111` bag dir, which `scanForBags` does accept — verified) rather than an empty dir, which `scanForBags` never nominates — `marine_survey_index/test/test_bag_fingerprint.cpp:157-175`
- [x] (suggestion) record the `::stat` portability trade-off (`st_mtim` is POSIX.1-2008, not macOS `st_mtimespec`) — plan-review finding 6, still unaddressed anywhere in plan or code — `marine_survey_index/src/bag_fingerprint.cpp:22`
- [x] (suggestion) document the residual holes size+mtime cannot see: an mtime-preserving rewrite (`rsync --times`, `cp -p`, `tar -p`), coarse-granularity filesystems, and hardlink double-counting — `docs/survey_index_schema.md:126-133`
- [x] (suggestion) remaining plan-sync leftovers: test-strategy bullets 1/3/4 still describe the retracted design, three "both functions move" phrasings, and the Files row omits `fingerprintStoredMtime()` — `.agent/work-plans/issue-375/plan.md:150-159,165`
- [x] (suggestion) new public header diverges from the package's `@brief/@param/@return` house style, and `fingerprint()` is a very generic exported symbol name (`bagFingerprint()` reads better) — `include/marine_survey_index/bag_fingerprint.hpp:60-83`
- [x] (suggestion) `## Notes` parses as a top-level ADR-0013 entry, orphaning the Issue Review subsections from `progress_read.py` and the triage-reviews integrator; demote to `###` — `.agent/work-plans/issue-375/progress.md:19`

### Follow-ups to file (out of scope here, by operator scope decision)
- [ ] the identical epoch bug is live in a second repo: `marine_perception_tools/src/session_index_io.cpp:166-171` computes `BagIdentity::mtime_ns` the pre-fix way, so every `.ssvc` session cache is size-only validated and a same-size bag rewrite serves a stale session index in the survey explorer. Its header at `session_index_io.hpp:25` claims parity with the very column this PR changes. Verified by reading both files. Needs its own issue in `rolker/marine_perception_tools`.
- [ ] `marine_survey_index` sets no sqlite `busy_timeout` / `BEGIN IMMEDIATE`, and `ledgerState()`'s SELECT is outside the transaction; the one-time mass re-index widens the concurrent-writer window considerably. Pre-existing.

## Implementation
**Status**: complete
**When**: 2026-09-10 10:56 -04:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-375 at `f03a803`
**Addressed**: `## Local Review (Pre-Push)` (2026-09-10 10:43 -04:00, verdict changes-requested, round 1, at `3bb5a65`) — all 6 must-fixes and all 12 suggestions
**Commits**: `5cfc685`, `fc8f7e9`, `c80cd8a`, `7d510c6`, `ea990da`, `6509d89`, `f03a803`

### Actions
- [x] (must-fix) a partially-walked bag no longer reports as authoritative: `BagFingerprint` carries a second trust flag `scan_complete`, and `fingerprintMatches()` requires `mtime_valid` **and** `scan_complete` before comparing anything. `skip_permission_denied` is gone (the walk is still non-throwing, per #259 — an unreadable entry is now *recorded* rather than hidden), `is_regular_file`'s `error_code` is honoured, a failed `::stat` clears the flag, `is_directory`'s `ec` is checked, and a walk that stops early names the entry it stopped at rather than the bag root — `marine_survey_index/src/bag_fingerprint.cpp`, `include/marine_survey_index/bag_fingerprint.hpp` (`5cfc685`)
- [x] (must-fix) the ns conversion is range-checked before the multiply: `tv_sec` against the largest/smallest values whose ns conversion fits in `int64_t`, `tv_nsec` into [0, 1e9). An out-of-range timestamp is treated exactly like an unreadable one — no wrapped value reaches the ledger — `marine_survey_index/src/bag_fingerprint.cpp` (`fc8f7e9`)
- [x] (must-fix) the schema doc now carries the column's units and epoch (UNIX ns UTC from `st_mtim`, `0` = unknown), the one-time re-index of every pre-fix `mtime_ns = 0` row (177 on the dev host) as expected derived-cache cost, and the incomplete-walk case alongside the no-timestamp case — `docs/survey_index_schema.md` (`c80cd8a`)
- [x] (must-fix) the README's incremental-skip paragraph states that a bag the indexer could not fully read is never skipped, and how the run says so — `marine_survey_index/README.md` (`7d510c6`)
- [x] (must-fix) the README `## Testing` enumeration lists the bag-fingerprint / incremental-skip test and what it covers — `marine_survey_index/README.md` (`7d510c6`)
- [x] (must-fix) the plan no longer credits the rejected sentinel: the Principles Self-Check records that it was killed at plan review and never shipped, and that validity flags checked before any comparison do the work — `.agent/work-plans/issue-375/plan.md` (`ea990da`)
- [x] (suggestion) `<unistd.h>` (and `<fcntl.h>` for the new `utimensat` test) included explicitly — `marine_survey_index/test/test_bag_fingerprint.cpp` (`5cfc685`, `fc8f7e9`)
- [x] (suggestion) the comments no longer justify persisting `0` by a range claim the code disproves — `touch -d @0` really does fingerprint as mtime 0 with a valid reading, so only the validity gate makes it safe; the assertion stays as the struct invariant the write path relies on — `marine_survey_index/src/bag_fingerprint.cpp`, `test/test_bag_fingerprint.cpp` (`5cfc685`, `f03a803`)
- [x] (suggestion) the `std::cerr` warning moved out of the exported library to the CLI call site: `bagFingerprint()` is silent and reports the reason through an optional `problem` out-param, so the Qt GUI in `marine_perception_tools` that links this library is not writing to an invisible stderr — `marine_survey_index/src/bag_fingerprint.cpp`, `src/survey_index_bag_main.cpp` (`5cfc685`)
- [x] (suggestion) the run summary gained a not-fully-readable bucket and a failed-bag bucket, and the run exits non-zero on either — a bag re-indexing every run forever, or missing after a mid-index failure, previously reported success — `marine_survey_index/src/survey_index_bag_main.cpp` (`5cfc685`)
- [x] (suggestion) the single-path branch guards on `is_regular_file` (a FIFO used to fingerprint as size 0 with a moving timestamp) and `is_directory`'s `ec` is no longer discarded — an undeterminable type clears `scan_complete` — `marine_survey_index/src/bag_fingerprint.cpp` (`5cfc685`)
- [x] (suggestion) `fingerprintStoredMtime()` dropped; the invariant it guarded (`mtime_ns == 0` while `!mtime_valid`) is stated on the struct and the write path binds `fp.mtime_ns` directly — `include/marine_survey_index/bag_fingerprint.hpp`, `src/survey_index_bag_main.cpp` (`5cfc685`)
- [x] (suggestion) tests added: the legacy `mtime_ns = 0` migration row (and that it settles after one re-index), the partial-walk route at a mode-0000 **subdirectory** (the reviewer's reproduction — identical size and mtime across a content change behind it), the production-reachable mode-0111 bag directory `scanForBags` does nominate, a non-regular single path, an unrepresentable year-2500 mtime, and — in place of asserting a stderr warning, now that the library is silent — that the `problem` string is non-empty and names the offending path, and empty for a readable bag. The permission-dependent tests skip under root; the year-2500 one skips if the filesystem clamps or refuses the timestamp — `marine_survey_index/test/test_bag_fingerprint.cpp` (`5cfc685`, `fc8f7e9`)
- [x] (suggestion) the `st_mtim` POSIX.1-2008 vs macOS `st_mtimespec` trade-off is recorded in the new header and in the plan — `include/marine_survey_index/bag_fingerprint.hpp`, `.agent/work-plans/issue-375/plan.md` (`5cfc685`, `ea990da`)
- [x] (suggestion) the schema doc documents what size + mtime cannot see: an mtime-preserving rewrite (`cp -p`, `rsync --times`, `tar -p`, a backup restore) at an identical byte count, a rewrite inside one coarse timestamp tick, and hardlink double-counting (a spurious re-index — the safe direction), with the remedy — `docs/survey_index_schema.md` (`c80cd8a`)
- [x] (suggestion) plan-sync leftovers cleared: test-strategy bullets rewritten to what shipped (exact `::stat` equality rather than a ±2 s window; the round-trip exercised through `fingerprintMatches()` rather than a temp sqlite `ledgerState()`; one test per untrustworthy route rather than an empty-directory stand-in), both "both functions move" phrasings corrected (`ledgerState()` stays in the executable), the `fingerprintStoredMtime()` row removed as the helper is gone, and the file count corrected to seven — `.agent/work-plans/issue-375/plan.md` (`ea990da`)
- [x] (suggestion) the new header follows the package's `@brief/@param/@return` house style (as in `footprint.hpp`), and `fingerprint()` is renamed `bagFingerprint()` — no caller outside this package exists yet, checked across the workspace's checked-out repos — `include/marine_survey_index/bag_fingerprint.hpp` (`5cfc685`)
- [x] (suggestion) `## Notes` demoted to `###` (its subsections to `####`), so the Issue Review entry keeps its subsections; verified with `progress_read.py --type "Issue Review"` — `.agent/work-plans/issue-375/progress.md` (`6509d89`)

### Verification
- `colcon build --packages-select marine_survey_index` clean, and a forced full recompile of all three touched translation units with `-Wall -Wextra` produced **zero** warnings.
- `colcon test --packages-select marine_survey_index` → **139 tests, 0 failures, 18 skipped** (the 18 skips are pre-existing, unchanged from the review's run; none of the six new tests skipped on this host — the year-2500 mtime and the FIFO both took effect). `test_bag_fingerprint` is now 11 cases, up from 5. cpplint / uncrustify / lint_cmake / copyright all clean (they run inside the package's test suite).
- Each intermediate commit was built and tested before being committed, so `5cfc685` stands on its own without the overflow guard (138 tests, 0 failures at that point).
- `survey_index_bag --help` still runs and exits 0.
- No pre-commit hooks: this project repo carries no `.pre-commit-config.yaml` and no installed hooks (only git's samples). Nothing was bypassed — `--no-verify` was never used. The ament linters that would gate this code run inside `colcon test`, above.
- The stale `build/*/ament_cmake_python/` directories the review flagged (`marine_autonomy`, `marine_interfaces`) did **not** block anything this round; no build tree was cleared.

### Notes for the re-review
- **One behaviour change worth a deliberate look**: `survey_index_bag` now exits **1** when any bag could not be fingerprinted authoritatively or failed mid-index, where it previously always exited 0. Nothing automated in this workspace consumes this binary's exit status (checked across scripts, launch files and tests), and it is what the review's suggestion asked for, but it is a CLI contract change and the README now documents it.
- **The affected-bag cost of must-fix 1**: a bag with a permanently unreadable member now re-indexes on **every** run rather than being skipped once. That is the safe direction and it is loud (per-bag warning, summary bucket, non-zero exit) — but it is a real cost, recorded in the plan's Consequences table.
- Both operator scope boundaries were honoured: nothing about the general libstdc++ `file_clock` pitfall was written into workspace guidance (it stays split out to `rolker/ros2_agent_workspace#623`), and `marine_perception_tools/src/session_index_io.cpp` was not touched — the follow-up bullets under the review entry are left unchecked for the host to file.
- One unchecked action remains in the older `## Issue Review` entry (the `.agent/knowledge/` note candidate). It is the item the operator split out to `rolker/ros2_agent_workspace#623`; it is outside this pass's source review entry, so its box was deliberately left as-is rather than edited from here.


## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-09-10 11:08 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-375 at `c8fc1b4`
**Mode**: pre-push
**Depth**: Deep (reason: 1231 changed lines across 9 files; new installed public header + CLI contract change)
**Must-fix**: 3 | **Suggestions**: 11
**Round**: 2 | **Ship**: continue — must-fix count fell 6 → 3, but one is a genuine correctness hole of the same class the PR exists to close (a symlinked subdirectory presents as fully authoritative, cross-confirmed by both adversarial lenses and reproduced here), and the gate added for round-1 must-fix 1 is provably untested

Round-1 closure verified independently, not taken on trust: forced recompile of all three touched translation units with `-Wall -Wextra -Wpedantic` produced zero warnings; 139 tests / 0 errors / 0 failures / 18 skipped reproduced; `test_bag_fingerprint` runs 11 cases, all passing, none skipped on this host. The ns range guard's arithmetic was checked by hand at both bounds (worst cases `9223372035999999999` and `-9223372036000000000`, both in range). Round-1 must-fixes 2-6 are genuinely closed. Must-fix 1 is **partially** closed: the mechanism it named (an unreadable subdirectory) is fixed, a sibling route is not, and the `scan_complete` gate it added has no test — proved by mutation, deleting the `scan_complete` conjunct from `fingerprintMatches()` breaks zero tests while deleting the `mtime_valid` conjunct breaks one. No caller of `fingerprint()` or `fingerprintStoredMtime()` survives anywhere in the workspace; neither changed doc makes a cross-store parity claim, so nothing here is made inaccurate by marine_perception_tools#53 remaining unfixed. Three independent searches agree that nothing in the workspace consumes this binary's exit status today.

### Findings
- [x] (must-fix) a symlinked subdirectory inside a bag is neither descended into nor flagged (`directory_entry::is_regular_file()` returns false with a clear `error_code`), so the bag fingerprints `scan_complete = true` with arbitrary content invisible and stable across changes to it — the #375 failure class one level up, the one `scan_complete` exists to close; reproduced (10→20 bytes and new content behind the symlink, byte-identical fingerprint, `fingerprintMatches` accepts it); the header's and schema doc's stated meaning of `scan_complete` is false as written — `marine_survey_index/src/bag_fingerprint.cpp:110-133`, `include/marine_survey_index/bag_fingerprint.hpp:62-67`
- [x] (must-fix) a bag that cannot be opened at all `continue`s without incrementing `n_bags_failed`, so it lands in no summary bucket and the run exits **0** — verified by running the binary against an unparseable `metadata.yaml`: `done: 0 indexed, 0 unchanged skipped, 0 not fully readable, 0 failed` and `EXIT=0`, which is exactly what the new contract's own comment says must not happen — `marine_survey_index/src/survey_index_bag_main.cpp:504-509`
- [x] (must-fix) the regression guard for round-1 must-fix 1 does not guard it: on this host `PartialWalkIsNotAuthoritative` yields `size=0, mtime_valid=0`, so the `mtime_valid` gate carries the assertion and the "change is invisible" checks compare 0 to 0; the reachable `mtime_valid=1, scan_complete=0` state (verified: 40 files, one unreadable subdir, `size=70` of 400) is untested, and both permission-dependent tests `GTEST_SKIP` under root — which is how hosted CI and `ci_local.sh --user root` both run, so the gate is green-by-skip in every merge-gating path; a dangling symlink is a root-observable route — `marine_survey_index/test/test_bag_fingerprint.cpp:207-243,247-265`
- [x] (suggestion) the CLI's warning, its `n_bags_unreadable` counter and the process exit status all key on `fingerprint_problem.empty()` rather than on the flags, making an empty-string convention load-bearing for the exit code across the library boundary; branch on `!fp.mtime_valid || !fp.scan_complete` and use the string only for the message — `marine_survey_index/src/survey_index_bag_main.cpp:388-396`
- [x] (suggestion) two flags is the right model (all three interesting combinations are independently reachable) but the shape invites conjuncting them wrong, as the mutation above shows; add a single `bool authoritative() const` accessor on `BagFingerprint` so no caller assembles the conjunction by hand — `include/marine_survey_index/bag_fingerprint.hpp:70-76`
- [x] (suggestion) the PR establishes an exit-status contract but documents it only as "exits non-zero" in prose, omits the mid-index-failure cause entirely, and conflates "the index DB could not be opened, nothing was done" (pre-existing `return 1`) with "indexed fine, one bag has a stray unreadable file, forever"; state the codes (0/1/2) in the README usage block and consider a distinct code for the durable-but-non-fatal case — `marine_survey_index/README.md:15-42`, `src/survey_index_bag_main.cpp:366,710`
- [x] (suggestion) non-regular entries *inside* a bag directory are silently skipped while the identical hazard on a single bag path is flagged, so the header's "every way the walk can fail to see the whole bag ... clears `scan_complete`" and the schema doc's enumeration both over-claim (verified: a FIFO inside a bag yields `scan_complete=1, problem=''`) — `marine_survey_index/src/bag_fingerprint.cpp:121-127`, `include/marine_survey_index/bag_fingerprint.hpp:82-87`, `docs/survey_index_schema.md:135-141`
- [x] (suggestion) `scanForBags` still passes `skip_permission_denied` and discards both the loop and per-entry `error_code`, so an unreadable directory under `--scan` makes whole bags vanish from the run with no warning, no counter and exit 0 — strictly worse than the case just fixed (missing from the index vs. re-indexing needlessly), and downstream `cube_bathymetry`'s dirty-tile guard only fires when the dirty set is *entirely* empty, so one dropped bag yields an authoritative-looking marker and stale store tiles; fix here or file a scoped follow-up — `marine_survey_index/src/survey_index_bag_main.cpp:154-176`
- [x] (suggestion) a broken symlink anywhere under a bag earns a permanent re-index *and* a permanent exit 1 even though it hides nothing; distinguish "definitively not a regular file" from "readability unknown" — `marine_survey_index/src/bag_fingerprint.cpp:121-124`
- [x] (suggestion) the max is taken over regular files only, so a member deleted and another added at the same total byte count with older-than-newest mtimes leaves the fingerprint identical (the bag directory's own mtime moved but is ignored); either fold walked directories' mtimes into the max or add this to the schema doc's otherwise careful "what size + mtime cannot see" list — `marine_survey_index/src/bag_fingerprint.cpp:126`, `docs/survey_index_schema.md:156-163`
- [x] (suggestion) `0 = unknown` is stated as an equivalence but is not one: a genuine epoch-zero mtime (`touch -d @0`, and some archive extractions) stores `0` with `mtime_valid` true — harmless for the decision, misleading for an operator auditing the DB — `docs/survey_index_schema.md:44-46,148`
- [x] (suggestion) two `problem` strings misdescribe the cause: an empty directory reports "no readable timestamp under ..." rather than "holds no regular files", and a failed `is_regular_file` reports "is not a regular file or a directory" because `file_ec` is short-circuited away — `marine_survey_index/src/bag_fingerprint.cpp:136,139,146`
- [x] (suggestion) no test covers a `::stat` failure on a member of a *listable* directory — the "permanently unreadable member re-indexes every run" policy the README now documents; a mode-0444 subdirectory reaches it — `marine_survey_index/test/test_bag_fingerprint.cpp`
- [x] (suggestion) the four summary counters read as a partition but are not one: a bag can increment both `n_bags_unreadable` and `n_bags_indexed`, and an unopenable bag increments none, so the counts do not sum to the nominated bag count — `marine_survey_index/src/survey_index_bag_main.cpp:703-708`
- [x] (suggestion) plan-sync leftovers from round 1's must-fix 6, all now harmless history rather than a credited rejected design: the Documentation Impact claim that the `BagFingerprint` doc-comment "moves across as-is" is false (it was rewritten into house style with the epoch and invariant rationale, itself a round-1 finding); "Seven files touched (two new)" — three are new; "the four test cases above" against eleven shipped; the README missing from the Files to Change table; and stale pre-change line-number citations presented in the present tense — `.agent/work-plans/issue-375/plan.md:276-278,304,225,217-226,113,178,224`

### Verification performed
- [x] forced recompile of `bag_fingerprint.cpp`, `survey_index_bag_main.cpp` and `test_bag_fingerprint.cpp` under `-Wall -Wextra -Wpedantic`: zero warnings
- [x] `./core_ws/test.sh marine_survey_index` → 139 tests, 0 errors, 0 failures, 18 skipped; `test_bag_fingerprint` 11/11 pass, 0 skipped on this host
- [x] mutation test of both trust-flag conjuncts in `fingerprintMatches()` (working tree restored clean afterwards)
- [x] standalone probes against the compiled library for the symlink-to-directory, FIFO, dangling-symlink, hardlink, symlink-to-file and many-files partial-walk routes
- [x] `survey_index_bag` run against an unopenable bag to observe the summary line and exit status
- [x] adversarial re-read of the suite against a reconstructed pre-fix reader: 4 of 11 tests fail, so the new tests are real regression guards

### Judgements requested by the operator
- [x] **Exit 1 on a partially successful run** — right direction, wrong granularity, and currently under-implemented (must-fix 2). Nothing in the workspace reads the status today; the named near-term consumer, `unh_echoboats_project11/scripts/build_bathy_store.sh`, runs `set -euo pipefail`, so a permanent permission wart would abort a store build. Keep non-zero; separate "index unusable" from "durable wart" and document the codes.
- [x] **Re-index every run for a permanently unreadable member** — right trade, and the loudness is proportionate for a durable condition. The problem is the trigger set, not the volume: too broad for a dangling symlink (hides nothing), too narrow for a symlinked directory (hides everything).
- [x] **Two trust flags** — the right model; all three interesting combinations are independently reachable, so one flag cannot encode it. Add an `authoritative()` accessor and a test for the second gate.
- [x] **The `problem` out-param** — the right mechanism, and keeping the library silent is correct (`marine_perception_tools` really does link the core library). The defect is at the CLI, which keys the exit status on the string instead of the flags.
- [x] **`bagFingerprint()` rename and the dropped `fingerprintStoredMtime()`** — clean; no caller or test left behind anywhere in the workspace, and the installed header surface is coherent (covered by the existing `install(DIRECTORY include/)` and `ament_export_targets`).

### Notes for the host
- This project repo has **no hosted build/test workflow** (only `ros-base-docker.yml`), so under ADR-0018 the `ci-local` attestation is the *only* merge verification available here — not a shortcut past a green Actions run. Both permission-dependent fingerprint tests skip under root, which is how both `ci_local.sh` and hosted CI run.
- Carried forward, unchanged from round 1 and still out of scope: `marine_survey_index` sets no sqlite `busy_timeout` and no WAL, and `ledgerState()`'s SELECT is outside the transaction; the one-time 177-bag re-index materially widens the concurrent-reader `SQLITE_BUSY` window. Pre-existing, amplified by this PR, not caused by it.
- `marine_perception_tools/src/session_index_io.cpp` was deliberately not re-raised (rolker/marine_perception_tools#53), and the general libstdc++ `file_clock` pitfall was deliberately not written into workspace guidance (rolker/ros2_agent_workspace#623). Both boundaries were honoured by the diff.

## Implementation
**Status**: complete
**When**: 2026-09-10 11:27 -04:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-375 at `92cadff`
**Addressed**: `## Local Review (Pre-Push)` (round 2, 2026-09-10 11:08 -04:00, at `c8fc1b4`) — 3 must-fixes and 12 suggestions (its header says 11; the list carries 12). All 15 fixed, none deferred.
**Commits**: `51d9039`, `9e1bc09`, `7e1ec95`, `e4a84c9`, `2614c99`, `92cadff`

### Actions
- [x] (must-fix) symlinked subdirectory presents as a complete walk — entries are now classified through `symlink_status()` before anything else. The line is drawn at what the walk *knows*: content it could not see (unresolvable type, a symlink whose target's existence cannot be established, a directory it will not enumerate) clears `scan_complete`; an entry definitively carrying no bag bytes does not. Symlinks are still not followed into directories — a link can close a cycle a recursive walk would never leave — so the case is reported instead — `marine_survey_index/src/bag_fingerprint.cpp:114-166`
- [x] (must-fix) an unopenable bag was counted nowhere and exited 0 — `++n_bags_failed` in the reader-open handler, so it lands in the `indexed + skipped + failed` partition like any other failure — `marine_survey_index/src/survey_index_bag_main.cpp:566-575`
- [x] (must-fix) the round-1 gate was undefended — both trust flags now have root-observable guards: `SymlinkedSubdirectoryHidesContentButIsNotTrusted` reaches `mtime_valid=1, scan_complete=0` with a 10-byte size and a real timestamp (so nothing compares 0 to 0) and reproduces the invisible content change; `IncompleteWalkIsRejectedBeforeAnythingIsCompared` asserts the flag truth table on a directly constructed fingerprint, control case included; `UnresolvableSymlinkIsNotAuthoritative` uses a symlink loop (ELOOP, which root is not exempt from). The two permission-based tests are kept and relabelled with what defends their branch under root — `marine_survey_index/test/test_bag_fingerprint.cpp`
- [x] (suggestion) exit status keyed on an empty-string convention — the warning, the counter and the status now branch on `fp.authoritative()`; the string is the message only, with a fallback so a drift cannot produce a blank warning — `marine_survey_index/src/survey_index_bag_main.cpp:444-457`
- [x] (suggestion) `BagFingerprint::authoritative()` added; `fingerprintMatches()` and the CLI both go through it, so no caller assembles the conjunction by hand — `include/marine_survey_index/bag_fingerprint.hpp:78-85`
- [x] (suggestion) exit codes stated, and the durable case separated: 0 clean, 1 index incomplete (failed/unopenable bag, un-enumerable `--scan` tree, or the DB itself), 2 usage, 3 complete-but-re-indexing-forever. Documented as a table in the README usage block and as the contract comment at the return — `marine_survey_index/README.md:28-37`, `src/survey_index_bag_main.cpp:780-794`
- [x] (suggestion) the docs' enumeration of what clears `scan_complete` over-claimed (a FIFO inside a bag did not) — both documents now state the rule the code implements, in both directions — `include/marine_survey_index/bag_fingerprint.hpp:62-68,88-104`, `docs/survey_index_schema.md:129-150`
- [x] (suggestion) `scanForBags` dropped subtrees silently — `skip_permission_denied` removed, every `error_code` inspected (iterator construction, per-entry type, `metadata.yaml` probe, increment), problems warned per line and folded into exit 1. A symlinked directory under a scan root is reported for the same reason it is not walked — `marine_survey_index/src/survey_index_bag_main.cpp:154-224`
- [x] (suggestion) a broken symlink no longer costs a permanent re-index: ENOENT/ENOTDIR from resolving a link is "definitively resolves to nothing", distinct from "readability unknown" — `marine_survey_index/src/bag_fingerprint.cpp:136-147`
- [x] (suggestion) delete-one/add-one at an identical total with older-than-newest mtimes added to the schema doc's "what size + mtime cannot see" list, with why directory mtimes are *not* folded into the maximum — `docs/survey_index_schema.md:170-181`
- [x] (suggestion) `0 = unknown` no longer stated as an equivalence: a genuine epoch-zero mtime stores 0 with the reading valid; the distinction never reaches the decision but an operator auditing the DB cannot tell them apart — `docs/survey_index_schema.md:44-47,153-162`
- [x] (suggestion) both misdescribing `problem` strings fixed: an empty bag directory now says it holds no regular files (nothing failed), and a failed `is_regular_file` reports the error instead of short-circuiting `file_ec` away into a definitive answer — `marine_survey_index/src/bag_fingerprint.cpp:196-206,211-216`
- [x] (suggestion) `UnreadableMemberOfAListableDirectoryReindexesEveryRun` covers the mode-0444 route (skips as root); the same `statFile` failure branch is reached root-observably by `UnrepresentableMtimeIsNotTrusted` — `marine_survey_index/test/test_bag_fingerprint.cpp`
- [x] (suggestion) the summary line no longer reads as a partition it never was: `indexed + skipped + failed` accounts for the nominated bags and says "of N"; "not fully readable" is reported as the cross-cutting sub-count it is — `marine_survey_index/src/survey_index_bag_main.cpp:769-779`
- [x] (suggestion) plan-sync leftovers cleared, and the plan now records what round 2 landed — `.agent/work-plans/issue-375/plan.md`

### Verification performed
- [x] `colcon build` + full `colcon test`: **155 tests, 0 errors, 0 failures, 19 skipped**. All 19 skips are `cppcheck` per-file entries (the tool is absent on this host) — **no gtest case skips**: `test_bag_fingerprint` 18/18 and `test_indexer_exit_status` 4/4 all run here (non-root).
- [x] forced recompile of every touched translation unit with `-Wall -Wextra -Wpedantic`: zero warnings.
- [x] **Mutation-tested every guard relied on** (working tree restored and re-verified green after each): dropping `scan_complete` from `authoritative()` → 3 failures, all root-independent; dropping `mtime_valid` → 2; removing the symlinked-directory branch → 1; treating an unresolvable symlink as harmless → 1; flagging definitively-non-regular entries as incomplete → 1; reverting the whole classification to `is_regular_file()` → 2 (so the new tests fail against the pre-fix reader); removing `++n_bags_failed` → 1; collapsing the exit codes back to 0/1 → 1; discarding the scan problems again → 1.
- [x] One mutation is **not** defended, deliberately and stated here rather than papered over: swapping `!fp.authoritative()` back to `!fingerprint_problem.empty()` at the CLI fails nothing, because the library never produces one without the other. That coupling removal is guarded indirectly, by the new `ProblemStringAndTrustFlagsAlwaysAgree` test pinning the invariant it used to depend on.
- [x] `test_indexer_exit_status` runs the real binary and exercises exit 1 (unopenable bag, with the summary counts), exit 3 (a real empty rosbag2 bag written by `rosbag2_cpp::Writer`, with a symlinked subdirectory: indexes fine, fingerprint untrustworthy), the dropped-`--scan`-subtree warning, and exit 2. A failure to execute the binary is a test failure, not a skip.

### Notes for the host
- **Scope boundaries honoured**: nothing was written into workspace guidance about the libstdc++ `file_clock` epoch (rolker/ros2_agent_workspace#623), and `marine_perception_tools` was not touched (rolker/marine_perception_tools#53).
- **Behaviour change worth a line in the PR body**: the indexer's success exit code for a run with a durable fingerprint wart is now **3**, not 1. Any consumer that tested `!= 0` is unaffected; one that tested `== 1` would need updating. Nothing in the workspace reads this status today (three independent searches at round 2 agreed), and the named near-term consumer `unh_echoboats_project11/scripts/build_bathy_store.sh` runs `set -euo pipefail`, so it still aborts on either — the codes exist so it *can* distinguish them.
- **New behaviour that could surprise in the field**: a symlinked directory under a `--scan` root is now warned about and makes the run exit 1, because bags beneath it are missed entirely. If any survey data layout composes trees with directory symlinks, that becomes a per-run warning until the real paths are nominated. Deliberate — silently dropping bags is the failure this PR exists to close — but it is the one change an operator would notice.
- Carried forward, still out of scope: no sqlite `busy_timeout`/WAL, and `ledgerState()`'s SELECT outside the transaction; the one-time 177-bag re-index widens the concurrent-reader `SQLITE_BUSY` window.
- Nothing pushed. Next: `review-code` re-review (round 3) in a fresh context.
