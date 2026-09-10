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

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-09-10 11:41 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-375 at `40541b2`
**Mode**: pre-push
**Depth**: Deep (reason: 2052 changed lines across 10 files; installed public header + CLI exit-status contract)
**Must-fix**: 4 | **Suggestions**: 11
**Round**: 3 | **Ship**: continue — every round-2 must-fix and all 12 suggestions are genuinely closed (re-verified by mutation, not read), but round 2's own fix introduced a correctness regression: removing `skip_permission_denied` without disabling recursion makes ONE unreadable directory abandon the rest of the `--scan` walk, so bags that used to index now silently never become nominated (reproduced: 6 of 10 bags found). Not precision — data loss. All four must-fixes are mechanical with known one-line shapes, so the fourth pass is short and targeted, not open-ended.

Round-2 closure verified independently, not taken on trust. Clean rebuild at `40541b2`: **155 tests, 0 errors, 0 failures, 19 skipped**, and the skips were read out of the JUnit XML per file — all 19 are `cppcheck` entries, `test_bag_fingerprint` 18/18 and `test_indexer_exit_status` 4/4 with **zero gtest skips** (run non-root). Correction to the round-2 fix pass's account: cppcheck is **installed** (`/usr/bin/cppcheck` 2.13.0); `ament_cppcheck` auto-skips it with "Test Skipped due to cppcheck 2.13.0 performance issues", so C++ static analysis is unrun in every merge-gating path — I ran cppcheck by hand over the changed files and it is clean on the diff (the one `identicalInnerCondition` hit is at untouched lines 656/662 and is a false positive: `drain_pending()` mutates the container).

**Ten mutations run myself** (tree restored and re-verified green after each), not read off the list: dropping `scan_complete` from `authoritative()` → **5 failures** (the round-2 finding is closed — it was 0); dropping `mtime_valid` → 3; symlinked-directory branch removed → 2; unresolvable symlink treated as harmless → 1; dangling-symlink exemption removed → 1; `++n_bags_failed` removed → 1; exit codes collapsed to 0/1 → 1; scan symlink report removed → 1. **Two mutations kill nothing**: the CLI's `!fp.authoritative()` → `!fingerprint_problem.empty()` (the one the fix pass reported honestly), and `!scan_problems.empty()` dropped from the exit-1 fold (must-fix 3, newly found).

**Judgement on the indirect guard**: sufficient here, and materially unlike the round-2 finding. The two CLI conditions are provably biconditional — every `incomplete()` call site sets both the flag and a non-empty string, and the only other string-setting path leaves `mtime_valid` false — so no behavioural test *could* distinguish them, and `ProblemStringAndTrustFlagsAlwaysAgree` pins exactly the invariant that makes them equivalent. Round 2's case was the opposite: a reachable state that behaved differently and simply had no test.

### Findings
- [x] (must-fix) one unreadable directory abandons the whole `--scan` walk, so readable bags after it in readdir order never become nominated and the "of N nominated" summary cannot reveal it — reproduced: 10 bags + one mode-000 directory yields **6 of 10**, and a one-line `it.disable_recursion_pending()` on the failed `metadata.yaml` probe restores 10/10 with the warning and the exit-1 fold intact (verified); cross-confirmed by both adversarial lenses — `marine_survey_index/src/survey_index_bag_main.cpp:194-224`
- [x] (must-fix) a `--scan` tree that could not be enumerated exits **2** (usage error), not the documented **1**, whenever the failure leaves the bag list empty, and no summary line is printed at all — a scheduler cannot tell "the survey disk is not mounted" from "I invoked it wrong"; test the scan problems before the empty-bags return — `marine_survey_index/src/survey_index_bag_main.cpp:408-414`, README exit table
- [x] (must-fix) the `!scan_problems.empty()` conjunct in the exit-1 fold is defended by **no test** — deleting it fails zero tests, because the only test of that route asserts exit 2 (which comes from `bags.empty()`, never reaching the fold); the guard does have real effect (a nominated good bag beside a symlinked subtree → exit 1 with 0 failed, 0 unreadable — verified against the built binary), so nominate a real bag in that test and assert 1 — `marine_survey_index/test/test_indexer_exit_status.cpp:186-187`, `src/survey_index_bag_main.cpp:794`
- [x] (must-fix) `package.xml` declares no rosbag2 **storage plugin**, so the new test's `rosbag2_cpp::Writer::open()` has nothing to write with under a clean-room `rosdep install`: `rosbag2_cpp` carries `rosbag2_storage_default_plugins` only as a non-transitive `test_depend`, no plugin is named anywhere in this repo's manifests, `ci_local.sh --clean-room` is `ros:jazzy-ros-core` + one `rosdep install --from-paths`, and the hosted workflow's `--packages-select` list excludes `marine_survey_index` entirely — so ci_local is the ADR-0018 gate and this test fails there; add `<test_depend>rosbag2_storage_mcap</test_depend>` (and the matching runtime depend, an already-undeclared pre-existing gap) — `marine_survey_index/package.xml:30-36`
- [x] (suggestion) `$<TARGET_FILE:survey_index_bag>` in `target_compile_definitions` creates no build-order dependency; all three existing precedents in this repo pair it with `add_dependencies` (`marine_bathymetry_store/CMakeLists.txt:192,221`, `marine_sidescan_mosaic/CMakeLists.txt:173`) and this one does not — `marine_survey_index/CMakeLists.txt:105-112`
- [x] (suggestion) a symlink *to* a bag directory is nominated as a second distinct bag (`bag_key` is `absolute().lexically_normal()`, which does not resolve symlinks, while `is_directory()` does), so every pass interval and nav point is inserted twice under two `bag_id`s — verified: a symlink to a bag under a scan root indexes cleanly at exit 0 under the symlink path; pre-existing, but this PR is the one that sets the symlink policy and it reports the harmless case while staying silent on the harmful one — `marine_survey_index/src/survey_index_bag_main.cpp:194-202,440`
- [x] (suggestion) `resolvesToNothing()` is load-bearing but untested — `directory_entry::is_directory(ec)` *sets* ENOENT for a dangling symlink, so without it one broken link under a `--scan` root would force exit 1 every run; the `meta_ec` "could not tell whether … is a bag" and the increment "stopped at" reports are likewise untested **(partly deferred: `resolvesToNothing()` and the `meta_ec` report are now tested; the increment "stopped at" report is asserted ABSENT by three tests and is no longer reachable by any construction I could build — see the round-3 Implementation entry)** — `marine_survey_index/src/survey_index_bag_main.cpp:156-159,194-224`
- [x] (suggestion) the blanket warning suffix "any bag beneath it is missing from this run" is applied to the `scan of … stopped at …` problem too, where what is missing is every bag *after* that point in the walk, not one subtree — `marine_survey_index/src/survey_index_bag_main.cpp:408-410`
- [x] (suggestion) the mid-index-failure `catch` (rollback + `++n_bags_failed`) and the `openIndexDb` exit-1 path are both named in the contract table but untested; only the `reader.open` branch is exercised — `marine_survey_index/test/test_indexer_exit_status.cpp`
- [x] (suggestion) the `popen` command string is concatenated unquoted, so a space or shell metacharacter in `$TMPDIR` or the build path mis-parses into extra argv words instead of failing meaningfully — quote the exe, the `--db` path and `args` — `marine_survey_index/test/test_indexer_exit_status.cpp:81-85`
- [x] (suggestion) the README's exit table names a `set -euo pipefail` store build as the consumer, but under `set -e` exit 3 aborts that build exactly as hard as 1, so the distinction the codes exist for does not survive its own documented consumer; show the intended handling (`rc=0; survey_index_bag … || rc=$?; case $rc in 3) warn ;; esac`) — `marine_survey_index/README.md:28-37`
- [x] (suggestion) the README's "a symlinked directory is *reported* rather than walked" over-states it: a symlink to a *bag* directory is indexed normally at exit 0 (verified), and only symlinks to intermediate non-bag directories warn — which materially narrows the field surprise the impl entry warns about, and is worth saying — `marine_survey_index/README.md:37-46`
- [x] (suggestion) the header's rationale "it is exported from the core library and linked into GUI processes (`marine_perception_tools`) where stderr is invisible" rests on a caller that does not exist: mpt links the whole core library but references neither `bagFingerprint` nor `BagFingerprint` anywhere — say "will be linked", or drop the specific attribution — `marine_survey_index/include/marine_survey_index/bag_fingerprint.hpp:96-99`
- [x] (suggestion) the schema doc's otherwise thorough "what size + mtime cannot see" list omits the one that bites on a real survey tree: on NFS/CIFS `::stat` serves the attribute cache (NFS `acregmax` 60 s default), so a bag rewritten on the server and indexed immediately after can fingerprint identically — `docs/survey_index_schema.md:170-176`
- [x] (suggestion) genuinely amplified by this fix, and now one line to settle: the one-time re-index turns a run over 177 unchanged bags from zero writes into 177 write transactions, `openIndexDb` executes DDL (so even opening takes a write lock) with no `PRAGMA busy_timeout`, and mpt's `survey_index_bridge.cpp:36` opens the same DB in its constructor — during the migration run a GUI-held lock converts indexer writes into `n_bags_failed` → exit 1; carried forward as out of scope for two rounds, restated because the remedy is now known and cheap — `marine_survey_index/src/schema.cpp:89-97`
- [x] (suggestion) two plan claims are now inaccurate: Estimated Scope says "Eight files touched (three new)" against four new files, and the Principles Self-Check asserts "Each guard was mutation-tested: removing the condition it defends fails at least one test" — falsified by must-fix 3 and by the CLI mutation the fix pass itself reported as undefended — `.agent/work-plans/issue-375/plan.md`

### Verification performed
- [x] clean rebuild + `colcon test` at `40541b2`: 155 tests, 0 errors, 0 failures, 19 skipped; per-file JUnit XML read to confirm all 19 skips are cppcheck and **zero** gtest cases skip
- [x] cppcheck run by hand over the changed files (ament auto-skips it on 2.13.0): clean on the diff; cpplint / uncrustify / lint_cmake / copyright / xmllint all pass in-suite
- [x] ten mutations applied, built, run and restored individually (list above); two kill no test, one of them newly found
- [x] the `--scan` walk-abandonment regression reproduced against the built binary (6 of 10 bags), and the one-line candidate fix verified to restore 10/10 with the warning intact
- [x] the scan-problem exit-1 path confirmed reachable and correct with a real rosbag2 bag nominated (0 failed, 0 unreadable, exit 1), and a symlink-to-bag-directory confirmed to index at exit 0
- [x] "nothing consumes this binary's exit status or the new API" re-verified independently across all seven `layers/main/*/src/` trees: only this package's own CMake, docs and work-plan prose
- [x] house-precedent check for running a built binary from an `ament_add_gtest`: three existing instances in this repo, all pairing `$<TARGET_FILE:>` with `add_dependencies`

### Judgements requested by the operator
- [x] **Exit 0/1/2/3 partition** — coherent, documented in both the README table and the return-site comment, and the three-way split earns its complexity: 1 and 3 are disjoint (1 dominates), and separating "you cannot rely on this index" from "the index is fine but one bag re-indexes forever" is exactly what a scheduler needs. Two defects, both above: the 1-vs-2 precedence is undocumented and lands on the wrong side in the worst instance of cause 1, and exit 3 collapses back into 1 under the `set -e` consumer the table names.
- [x] **Refusing to follow directory symlinks, plus reporting** — the right call. libstdc++'s `recursive_directory_iterator` has no cycle detection under `follow_directory_symlink`, so following correctly means a hand-rolled walk with a visited `(dev, ino)` set — real new complexity, and `st_dev` is not stable on network mounts. And the refusal is *narrower* than the impl entry implies: a symlink to a bag directory is indexed normally, so the plausible layout "a scan root of symlinks to bags on other volumes" already works; only intermediate symlinked directories warn. Lens B checked `~/data`: zero symlinks across 97 bags, so no current layout is affected.
- [x] **The dangling-symlink exemption** — sound, and for the right reason. The #375 hazard is *stability*: content hidden in a way that does not move the fingerprint when it changes. A dangling link's target appearing makes `::stat` succeed and adds size and mtime, so the fingerprint moves and the bag re-indexes. The adjacent worry — a link into a temporarily unmounted volume — resolves the same way: while unmounted the fingerprint is smaller than the stored row, so the bag re-indexes rather than skips, and the rosbag2 reader fails on the missing member into `n_bags_failed` → exit 1. Self-heals in the safe direction.
- [x] **Running the built binary from a unit test** — right mechanism and house style: `marine_bathymetry_store` and `marine_sidescan_mosaic` already do exactly this three times over, and this one improves on them by using `popen` (output captured into the assertion message) instead of `std::system`. Locating the exe via `$<TARGET_FILE:>` is the correct approach — absolute path, no ament-index or PATH guessing. Three gaps, all above: the missing `add_dependencies`, the unquoted command string, and the missing storage-plugin dependency that makes it fail at the merge gate. Not findings, checked: `pclose` reaps the child, `ament_add_test`'s default 60 s ctest TIMEOUT bounds a hang, and `WIFEXITED` is asserted rather than assumed.

### Notes for the host
- **Round 2's three must-fixes and all 12 suggestions are genuinely closed** — the `scan_complete` gate that broke zero tests at round 2 now breaks five. Two of round 3's four must-fixes are *new*, introduced by round 2's own fixes (the `skip_permission_denied` removal, and the new binary-running test), and one is a contract incoherence in newly written code. That is why the count went 3 → 4 rather than falling.
- **Shape of the remaining work**: four mechanical edits with known fixes — one `disable_recursion_pending()` line, one reordered return, one test that nominates a bag, one `test_depend`. No design question is open.
- Scope boundaries honoured by the diff, and not re-raised here: the general libstdc++ `file_clock` pitfall (rolker/ros2_agent_workspace#623) and the identical bug in `marine_perception_tools` (rolker/marine_perception_tools#53).
- **Static-analysis coverage gap worth knowing beyond this PR**: `ament_cppcheck` self-skips on cppcheck 2.13.0, so no C++ static analysis runs in `colcon test` on this host or in CI for any package in this repo. Not a finding against this PR — I ran it by hand and the diff is clean — but it means the 19 skips are a tooling gap, not an absent tool.

## Implementation
**Status**: complete
**When**: 2026-09-10 12:58 -04:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-375 at `4a450b6`
**Addressed**: `## Local Review (Pre-Push)` (round 3, 2026-09-10 11:41 -04:00, branch at `40541b2`) — 4 must-fix, 12 suggestions
**Commits**: `e56a3a1` `b41a850` `2e73d2e` `ef7f940` `69ebfe0` `282ec5c` `5d8b63e` `055a702` `19b6bf3` `bb7c390` `4a450b6`

**The pattern round 3 called out, and what was done about it**: two of the four
must-fixes were introduced by round 2's own fixes. So every guard added or
relied on this round was mutation-tested (seven mutations, each applied, built,
run and restored individually — list below), and every scenario the earlier
rounds cared about was re-run against the built binary afterwards, not reasoned
about.

**Beyond the reviewer's one-line shape, in one place**: the reviewer's fix for
must-fix 1 (`disable_recursion_pending()` on the failed `metadata.yaml` probe)
closes the mode-000 case. Checking what it does *not* fix found a second route
to the same data loss: a traverse-only directory (mode 0111) answers the
metadata probe **without error**, so it reaches no reporting branch, and then
cannot be listed — `increment()` fails and abandons the walk exactly as before.
Reproduced: 8 of 10 bags. So enumerability is now probed up front for that
branch too (one `directory_iterator` on a directory the walk is about to open
anyway). Both routes now nominate 10 of 10 with their warnings intact.

### Actions
- [x] (must-fix) one unreadable directory abandons the whole `--scan` walk — `src/survey_index_bag_main.cpp:203-234`, `e56a3a1`. Descent is cancelled at both points the walk learns it cannot read a directory. Two order-independent tests (whatever readdir order the filesystem gives, every bag must be reached and the walk must never report having stopped); both permission-based, so they skip as root — see the honesty note below.
- [x] (must-fix) an unenumerable `--scan` tree exits 2, not 1, and prints no summary — `src/survey_index_bag_main.cpp:429-445`, README exit table, `b41a850`. Scan problems are tested before the empty-bag return, and the run is summarised like any other (`of 0 nominated`). The README now states the 1-over-2 precedence and which runs print a summary. Tested with a missing scan root — the same condition as an unmounted mountpoint, and needs no permission bits, so it runs as root too.
- [x] (must-fix) the `!scan_problems.empty()` conjunct is defended by no test — `test/test_indexer_exit_status.cpp`, `b41a850`. The test now nominates a real bag beside the dropped subtree, so `0 failed`, `0 not fully readable`, exit 1 — the incomplete scan is the only thing that can make it non-zero. Deleting the conjunct now fails two tests (was zero). Landed in the same commit as must-fix 2 because its old assertion of exit 2 is exactly what that fix changes.
- [x] (must-fix) `package.xml` names no rosbag2 storage plugin — `package.xml:30-42`, `ef7f940`. `<exec_depend>` **and** `<test_depend>` on `rosbag2_storage_mcap`, plus `<exec_depend>rosbag2_storage_sqlite3</exec_depend>` for the .db3 bags still in `~/data/logs`. Both keys verified to resolve (`ros-jazzy-rosbag2-storage-mcap`, `ros-jazzy-rosbag2-storage-sqlite3`). The runtime half was the larger gap and was pre-existing.
- [x] (suggestion) `$<TARGET_FILE:>` with no `add_dependencies` — `CMakeLists.txt:113-116`, `69ebfe0`.
- [x] (suggestion) a symlink to a bag directory is indexed as a second distinct bag — `src/survey_index_bag_main.cpp:483-495`, `5d8b63e`. The ledger key resolves through `weakly_canonical` (falling back to the plain absolute path), so the second nomination finds the first bag's row and skips it. Tested; fails with the old key. The `bags.path` column comment says so now.
- [x] (suggestion) `resolvesToNothing()` and the scan reports are untested — `2e73d2e`, partly deferred (see below). Three new tests: a dangling symlink under a scan root is a clean run at exit 0 (deleting `resolvesToNothing`'s body fails it), an ELOOP entry produces the "could not determine the type" report root-observably, and the `meta_ec` report is asserted by the mode-000 walk test.
- [x] (suggestion) the blanket "any bag beneath it" suffix is wrong for the "stopped at" problem — `282ec5c`. Each problem now carries its own consequence.
- [x] (suggestion) the mid-index-failure handler and the `openIndexDb` exit-1 path are untested — `2e73d2e`, `bb7c390`. `openIndexDb`: exit 1 with **no** summary, because nothing was done. Mid-index: re-indexing a changed bag into a read-only index DB fails at the first write with the transaction open — `1 failed (of 1 nominated)`, exit 1, and dropping `++n_bags_failed` fails it. Permission-based, skips as root.
- [x] (suggestion) the `popen` command string is unquoted — `test/test_indexer_exit_status.cpp:102-129`, `e56a3a1`. Exe, `--db` path and every interpolated path are single-quoted.
- [x] (suggestion) exit 3 does not survive the `set -e` consumer the README names — `README.md`, `19b6bf3`. The table no longer claims `set -euo pipefail` as the consumer, and shows the `rc=0; … || rc=$?; case $rc in` handling that keeps the distinction.
- [x] (suggestion) the README over-states the symlink refusal — `README.md`, `19b6bf3`. A symlink to a *bag* directory is nominated and indexed normally (and, since this round, under the bag's resolved path); only intermediate symlinked directories warn.
- [x] (suggestion) the header's rationale names a caller that does not exist — `include/marine_survey_index/bag_fingerprint.hpp:96-99`, `19b6bf3`. "will be called from them".
- [x] (suggestion) the schema doc omits the network-mount attribute cache — `docs/survey_index_schema.md`, `19b6bf3`.
- [x] (suggestion) no `PRAGMA busy_timeout`, so a GUI-held lock converts writes into failed bags — `src/schema.cpp:89-98`, `055a702`. 10 s, with a test that fails if the pragma is dropped.
- [x] (suggestion) two inaccurate plan claims — `.agent/work-plans/issue-375/plan.md`, `4a450b6`. Eleven files (four new) as landed, the mutation claim restated with its two named exceptions, and the three round-3 additions listed in Files to Change.

### Verification performed
- [x] clean rebuild (`build/` + `install/` for the package removed) and full `colcon test`: **164 tests, 0 errors, 0 failures, 19 skipped**; the skips read out of the per-file JUnit XML are **all 19 cppcheck** — zero gtest cases skipped (run non-root). `test_indexer_exit_status` 12/12, `test_schema` 4/4, `test_bag_fingerprint` 18/18.
- [x] cppcheck run by hand over the four changed C++ files (`ament_cppcheck` self-skips on 2.13.0): the only hit is the known `identicalInnerCondition` false positive at the untouched `drain_pending` site (now line 716; `drain_pending()` mutates the container). The two `syntaxError` reports are cppcheck failing to parse gtest's `TEST_F` macro, not the diff.
- [x] **seven mutations**, each applied, built, run and restored individually: `disable_recursion_pending()` on the failed metadata probe → 1 failure; the enumerability probe → 1; `!scan_problems.empty()` in the exit-1 fold → **2** (was 0 at round 3); the scan-problem branch of the empty-bag return → 1; `resolvesToNothing()` forced false → 1; the canonical ledger key reverted to `absolute().lexically_normal()` → 1; `++n_bags_failed` in the mid-index handler → 1 (now 2 tests total cover that counter).
- [x] every earlier round's scenario re-run against the built binary after all changes: the partial walk with a locked directory (**10 of 10** nominated, both mode 000 and mode 0111, warning intact — was 6-8 of 10); a symlinked intermediate subdirectory beside a good bag (reported, exit 1); an unenumerable scan root and a missing scan root (exit **1** with a summary, was 2 with none); a dangling symlink beside a good bag (silent, exit 0); a symlink to a bag (indexed once).
- [x] real-tree run: `--scan ~/data/logs/sim`, 51 bags nominated, 50 indexed, 1 failed (`2026-06-05T05.07.10` — a truncated recording whose metadata will not parse; a genuine on-disk defect, and exactly what the exit status exists to report), exit 1. Second run skips all 50 unchanged, so the fingerprint round-trips over real bags, and the stored keys are plain absolute paths.

### Honest notes on what is *not* defended
- **The permission-based tests skip as root**, and `ci_local.sh` runs its container `--user root` — so the two walk-abandonment tests and the mid-index-failure test do not run in the ADR-0018 merge gate. This is the pre-existing house pattern (`test_bag_fingerprint` already skips four tests as root) and it has a root-observable counterpart for each shared guard: the exit-1 scan fold is covered by the symlinked-subtree and ELOOP tests, and the `n_bags_failed` counter by the unopenable-bag test. What has **no** root-observable equivalent is the walk-abandonment invariant itself: only a failed `opendir` abandons the walk, and root cannot be denied one on a local filesystem. I could not construct a root-visible route (ELOOP resolves to the symlink branches; an fd-exhaustion route through `popen` would be flaky by nature).
- **The `!fp.authoritative()` CLI condition** remains behaviourally indistinguishable from `!fingerprint_problem.empty()`, as round 3 judged: the two are provably biconditional, `ProblemStringAndTrustFlagsAlwaysAgree` pins the invariant that makes them so, and no behavioural test could separate them. Unchanged, deliberately.
- **The `ROLLBACK` statement** in the mid-index handler is exercised but not *observed*: the read-only-DB route that reaches the handler cannot commit anything either way, so a test cannot tell a rollback from its absence. Observing it needs a failure induced part way through a transaction the same run can then read back.
- **The "scan of … stopped at …" report** is now asserted *absent* by three tests and I could not construct a case that still reaches it — both known routes are cancelled before `increment()`. It stays as the last-resort guard against a TOCTOU (a directory readable when probed, unreadable a moment later) and an untested line; deleting it would fail nothing, but it is the difference between a silently truncated walk and a reported one.

### Next step
Lifecycle: **Implementation** → **review-code** (re-review, round 4)

    .agent/scripts/dispatch_subagent.sh --mode in-process --issue 375 --skill review-code

Nothing was pushed — the host performs pushes. Round 4 should look hardest at
the two places this pass went beyond a one-line fix: the up-front enumerability
probe (a second `opendir` per non-bag directory, and whether the branch it sits
in is the right one) and the canonicalised ledger key (which changes what
`bags.path` means for any bag whose recorded path ran through a symlink).

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-09-10 13:19 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-375 at `0df2f04`
**Mode**: pre-push
**Depth**: Deep (reason: 2556 changed lines across 13 files; installed public header + CLI exit-status contract + a persisted-column semantics change)
**Must-fix**: 4 | **Suggestions**: 16
**Round**: 4 | **Ship**: recommended — **every change this pass made is correct**: I re-verified the three the host flagged (enumerability probe, canonical ledger key, storage-plugin deps) by measurement and execution, and found no defect in any of them, so the round-2/round-3 pattern of a fix introducing a regression did NOT repeat. All four must-fixes are mechanical with named shapes and no design question is open. One is genuine reproduced data loss and is worth the fifth pass on its own: a valueless `--scan`, or any unknown flag, silently swallows a bag or a whole tree and exits **0** — pre-existing arg-parsing, but this PR is the one that authors the contract promising 0 means "every nominated bag is in the index", exactly the standing on which round 3's pre-existing symlink defect was fixed. The other three are a migration consequence, a false test-coverage claim, and a one-line comment.

**Independent verification, not taken on trust.** Clean rebuild of the package at `0df2f04` (build + install trees removed): **164 tests, 0 errors, 0 failures, 19 skipped**, and the skips were read out of the per-file JUnit XML — all 19 are `cppcheck` entries, **zero gtest cases skipped** (run non-root); `test_indexer_exit_status` 12/12, `test_bag_fingerprint` 18/18, `test_schema` 4/4. cppcheck run by hand (`ament_cppcheck` self-skips on 2.13.0): clean on the diff, the only hit being the known `identicalInnerCondition` false positive at untouched line 716 plus a style hit in an untouched header. `ci_local.sh -n` confirms its full scope is 18 packages **including** `marine_survey_index`, and `ci_local.sh:428` is `docker run --rm --user root`; the repo's one hosted workflow (`.github/workflows/ros-base-docker.yml`) lists this package in neither its build nor its test set and nothing depends on it, so the root-run local attestation is the entire ADR-0018 gate. (The plan's phrasing "this repo has no hosted build/test workflow" is false; the true and more useful statement is that the workflow does not reach this package.)

**Mutations I ran myself.** The two new walk guards were mutation-tested in a standalone harness that reproduces `scanForBags` line for line, which isolates the guard rather than the build: dropping `disable_recursion_pending()` on the failed metadata probe takes the mode-000 tree from **4 of 4 bags to 2 of 4 with the walk abandoned**; dropping the enumerability probe takes the mode-0111 tree from **4 of 4 to 1 of 4, abandoned**. Both guards are load-bearing. Against the built binary I confirmed the `!scan_problems.empty()` conjunct is now the sole cause of its exit (a real indexed bag beside a symlinked subtree → `1 indexed, 0 unchanged skipped, 0 failed (of 1 nominated); 0 not fully readable`, exit **1** — nothing else in the fold can produce that), and that a missing scan root exits **1 with a summary** while no bags exits **2**. A fresh adversarial pass independently mutation-tested the `ROLLBACK` statement: deleting it leaves all 164 tests green.

### The three changes the host asked me to scrutinise hardest
- **The up-front enumerability probe — right branch, negligible cost, and it closes a third route nobody had named.** Measured on the real trees: 672 extra `opendir`s over the CIFS survey tree (1225 directories, 551 bags), **~+50 ms on a ~950 ms walk (+5%)**, stable across three runs; 48 extra on `~/data/logs`, below noise. Zero false positives across those 672 real directories — 551 bags found with the probe on and off. It cannot become a new failure route: it opens the same directory the walk is about to open, is scope-bound and destroyed before `increment()`, and where it *does* fail it converts a whole-walk abandonment into a reported, contained subtree skip. Demonstrated for **fd exhaustion**, a route neither round 3 nor the fix pass identified: `ulimit -n 256` over a 400-deep chain gives one `could not enumerate` warning, no `stopped at`, and all 4 bags nominated with the probe, versus `STOPPED AT` and 2 of 4 without it (verified against the built binary). The only correction is the comment: "Same cost either way" is false — it is one extra `opendir` per non-bag directory, and that is what buys not abandoning the walk.
- **The canonical ledger key — a no-op on the live index, and the host's re-index worry does not materialise.** Verified against `~/data/world/survey_index.db` after today's prefix rewrite: **all 177 rows re-key to exactly the same string**, because no component of `/mnt/nadata/map2026asv/logs/gabby/logs/...` is a symlink and `weakly_canonical` succeeded for every one of them. The key is also stable whether the autofs share is mounted or not — an unmountable indirect share reports ENOENT, which `weakly_canonical` treats lexically and returns the identical path. So **nothing re-indexes because of canonicalisation**; all 177 rows do re-index once, for the pre-existing `mtime_ns = 0` reason the schema doc already documents (confirmed: 177 of 177 rows hold `mtime_ns = 0`). What neither the review nor the fix pass had is the *other* direction, and it is must-fix 2 below.
- **The two storage-plugin dependencies — correct, and they pull no weight into the core library.** Both keys resolve (`rosdep resolve` → `ros-jazzy-rosbag2-storage-mcap` / `ros-jazzy-rosbag2-storage-sqlite3`, both installed). `exec_depend` is the right placement because the plugins are loaded by name at run time (a pre-existing gap, correctly identified as the larger half), and the mcap `test_depend` covers the test writer's default format. Nothing reaches the core library: `bag_fingerprint.cpp` adds no rosbag2 include, no `find_package` was added, and `ament_export_dependencies` is unchanged, so downstream CMake consumers see no new requirement. Independently confirmed by the systemic-lens pass.

### Findings
- [x] (must-fix) a valueless `--scan`, and any unknown `--flag`, silently lose data and report success: `--scan` with no value records no problem, prints no warning and exits **0** with the tree never walked, and the `++i` that treats every `--`-prefixed token as taking a value makes `--verbose <bag>` swallow the bag — reproduced both ways against the built binary (`of 1 nominated`, exit 0), and reachable from an ordinary slip (`--scan $ROOT` with `ROOT` unset; the *quoted* form correctly exits 1, so the safe and unsafe spellings differ only by quoting). Cross-confirmed by both adversarial lenses and by me, independently. Reject a valueless `--scan` and an unrecognised flag with exit 2 — `marine_survey_index/src/survey_index_bag_main.cpp:418-433`
- [ ] (must-fix) the canonicalised ledger key **orphans** rather than migrates any pre-fix row recorded through a symlinked path: the lookup misses, a second `bags` row is inserted, and the stale row's `passes`/`nav_track` persist and double-report that bag through `query.cpp`'s undeduplicated join — forever, since no future run can ever match the old key. Reproduced end to end (two rows, one bag, exit 0, no warning). The schema doc's "One-time re-index at the #375 fix … needs no migration" is false for that subset. This dev host is verified unaffected (0 of 177 keys change); salmon and gabby are not verified. Floor: one sentence naming the "delete `survey_index.db` once" remedy. Preferred: a three-line backfill that re-keys a matching unresolved row instead of inserting beside it — `src/survey_index_bag_main.cpp:483-494`, `docs/survey_index_schema.md`
- [ ] (must-fix) the walk-continuation invariant — the round-3 regression that lost 4 of 10 bags — executes in **no** merge-gating path, and the plan asserts the opposite in four places. Two of the three root-skipped tests can be made root-observable cheaply, demonstrated: the enumerability-probe guard via `ulimit -n 256` over a 400-deep chain (verified against the built binary; order- and depth-independent assertions, and `ulimit` lowering is always permitted, so it is not flaky), and the mid-index handler via breaking the schema under an open transaction instead of a read-only file (verified by the systemic lens). The mode-000 `meta_ec` guard has no root-observable route — I could not construct one either, and that residual is acceptable **recorded as the known exception**, which is what the plan must say instead of "each shares its guard with a root-observable sibling" (`plan.md` Principles Self-Check ~282, Approach step 4 ~186, test outline ~253, and the `package.xml` Files-to-Change row) — `test/test_indexer_exit_status.cpp:271,299,406`, `plan.md`
- [ ] (must-fix) `schema.cpp:98` states as fact that the index "is opened **read-only** by the explorer GUI" — it is not: `marine_perception_tools/src/survey_index_bridge.cpp:36` calls this same read-write `openIndexDb`, which runs the DDL. Verified, and cross-confirmed by the governance and systemic passes. The truth is a stronger argument for the pragma (two write-capable handles), so this is a one-line correction that improves the rationale rather than weakening it — `src/schema.cpp:98-100`
- [ ] (suggestion) the `weakly_canonical` fallback mints a *second* key for the same bag instead of failing the bag, so a transient canonicalisation error (EIO/ESTALE on the `soft` CIFS mount this tree actually lives on) re-creates by accident the double-insert `5d8b63e` set out to eliminate; treating `key_ec` as a bag-level failure keeps "one bag is one row" unconditional — `src/survey_index_bag_main.cpp:490-494`
- [ ] (suggestion) `busy_timeout` only *delays* the failure under this DB's `journal_mode = delete` (measured: the indexer waited 10.03 s against a held read transaction and then reported `database is locked` → 1 failed → exit 1); `PRAGMA journal_mode = WAL` removes it and is persistent in the file, so one call covers both processes — **but** WAL is unsafe on network filesystems and this DB sits beside stores that may live on the NAS, so weigh that before adopting it — `src/schema.cpp:98-106`
- [ ] (suggestion) `openIndexDb` is also the GUI's open, and every `bridge_->` call site in `sidescan_viewer_window.cpp` runs on the Qt GUI thread, so a query arriving during an indexer commit now blocks the event loop for up to 10 s instead of raising the error the GUI already handles; the timeout is a per-consumer policy and belongs at the call site — `src/schema.cpp:98-106`
- [ ] (suggestion) a run that exits 1 solely because of a scan problem prints an all-clear summary (`0 failed … 0 not fully readable`) and then exits 1, so anyone triaging from the summary line sees a clean run with an inexplicable code; the zero-bags path already appends its cause — add `; N scan problem(s)` to the normal one — `src/survey_index_bag_main.cpp:829-833`
- [ ] (suggestion) `ledgerState` discards `sqlite3_step`'s return, so a `SQLITE_BUSY` surviving the new 10 s timeout, or `SQLITE_CORRUPT`, reads as "this bag is not in the ledger" and takes the INSERT path for a bag that already has a row; the `path UNIQUE` constraint contains it loudly today, but only incidentally — throw on a result that is neither `SQLITE_ROW` nor `SQLITE_DONE`, matching `stepDoneOrThrow` — `src/survey_index_bag_main.cpp:317-333`
- [ ] (suggestion) nominated bags are never deduplicated, so the same bag named twice reports `1 bag(s) indexed, 1 unchanged skipped` — an operator reads "one bag was already up to date" about a bag this run wrote seconds earlier, and `ABagReachedThroughASymlinkIsNotASecondBag` asserts that exact string, locking the behaviour in; a `sort`+`unique` on the resolved keys costs nothing — `src/survey_index_bag_main.cpp:416-433`
- [ ] (suggestion) the undeterminable-entry-type branch omits the `disable_recursion_pending()` the other two problem branches take, and the omission is the one their comments warn about; harmless where `readdir` returns a real `d_type` (verified on ext4 and on the production CIFS mount), live on a `DT_UNKNOWN` backend — free to add — `src/survey_index_bag_main.cpp:197-201`
- [ ] (suggestion) the probe's comment "Same cost either way: the walk is about to open this directory anyway" is false — the walk opens it *again* on `increment()`; say what it costs (one extra `opendir` per non-bag directory; measured +5% of walk time on the CIFS survey tree) and what that buys — `src/survey_index_bag_main.cpp:227-234`
- [ ] (suggestion) the attribute-cache hazard is documented against the wrong filesystem and overstated ~60×: the real survey mount is **CIFS** with `actimeo=1`, not NFS with `acregmax` 60 s (verified in `/proc/mounts`) — quote the mount actually in use — `docs/survey_index_schema.md`, "What size + mtime cannot see"
- [ ] (suggestion) a nonexistent bag URI — the commonest operator error — is reported as "could not determine what … is … treating this bag as changed, so it re-indexes every run" and counted as *not fully readable*, which sends the operator looking for a permission problem; the exit code is right, the message is not — `src/bag_fingerprint.cpp:161-165`
- [ ] (suggestion) the mid-index test does not defend the `ROLLBACK` (independently mutation-verified: deleting the statement leaves 164 tests green), and the honesty note's stated reason is the wrong one — SQLite rolls back at `sqlite3_close`, so only a *subsequent bag's* `BEGIN` in the same run can observe it, which needs a per-bag-selective write failure rather than a read-only file; correct the note and, if the guard is to be defended, use that shape — `test/test_indexer_exit_status.cpp:398-436`
- [ ] (suggestion) `bags.path` is now one-sided across the repo boundary: `marine_perception_tools/src/sidescan_viewer_window.cpp:4058` compares `p.bag_path == current_bag_uri_` for bag *identity* against uncanonicalised `QFileDialog` input, so a bag opened through a symlinked path now compares unequal and re-opens. `cube_bathymetry/src/batch_regen_main.cpp:421` also matches `bags.path` exactly but already names "a symlinked mount" as a known miss and degrades conservatively (marker suppressed → full regen), so nothing fails silently there. The schema doc should say consumers matching exactly must canonicalise too; the mpt half is a follow-up on the consumer — `docs/survey_index_schema.md`
- [ ] (suggestion) one permanently truncated bag in `~/data/logs/sim` makes the documented consumer contract a permanent exit 1 (reproduced: 58 indexed, 1 failed of 59, exit 1, and it will never repair), and the README's own snippet routes that to `exit "$rc"` — the predictable operator response is `|| true`, which discards the whole 0/1/2/3 distinction; an acknowledge or exclude path is what makes the contract survive contact with the tree — `README.md` exit table
- [ ] (suggestion) plan drift: the canonical ledger key (`5d8b63e`) appears in no Approach step, no Files-to-Change row and no Consequences row though it changes a persisted column's meaning; the ADR table says "None of 0001–0013" while **ADR-0018 is triggered** and the plan leans on it; and the exit-status test set is described as four cases where twelve landed (the plan already annotates exactly this growth for `test_bag_fingerprint`) — `.agent/work-plans/issue-375/plan.md`
- [ ] (suggestion) `PRAGMA busy_timeout = 10000` is documented nowhere: the schema doc self-declares as the cross-stage contract and consumers share `openIndexDb`, so it belongs there (opening executes DDL and therefore takes a write lock; an open now waits up to 10 s; exceeding it is still a failed bag, not a retry), and `openIndexDb`'s docstring should say it may block. The README exit table also omits that `--help` and a bare invocation exit **2** (verified) — `docs/survey_index_schema.md`, `include/marine_survey_index/schema.hpp`, `README.md`
- [ ] (suggestion) the README's "both hosted CI and `ci_local.sh` run as root" is true but implies hosted CI runs these tests; it does not build or test this package at all, which is *why* `ci_local` is the sole gate — and the current sentence covers only the trust flags, not the scan-walk guards — `README.md:125-129`
- [ ] (suggestion) two `.agents/README.md` Common Pitfalls candidates for the operator (proposal, never auto-applied): the runtime-loaded rosbag2 storage plugins (`exec_depend` because plugins load by name — without them the indexer opens no bag under a clean-room `rosdep install`), and the `survey_index.db` cross-repo coupling to `marine_perception_tools`' bridge, for which the guide already carries the same shape of note for `BathyDem`

### Verification performed
- [x] clean rebuild (package `build/` + `install/` removed) and full `colcon test` at `0df2f04`: 164 tests, 0 errors, 0 failures, 19 skipped; per-file JUnit XML read to confirm all 19 skips are cppcheck and **zero** gtest cases skip
- [x] cppcheck run by hand over the four changed C++ sources plus the new header: clean on the diff (the one `identicalInnerCondition` hit is at untouched line 716, a false positive because `drain_pending()` mutates the container); cpplint / uncrustify / lint_cmake / copyright / xmllint all pass in-suite
- [x] both new walk guards mutation-tested in a line-for-line standalone harness (mode 000: 4→2 bags + abandoned; mode 0111: 4→1 + abandoned) and the fd-exhaustion route measured with and without the probe against the built binary
- [x] probe cost measured on both real trees: CIFS survey tree 672 probes / ~+50 ms of ~950 ms (three runs), `~/data/logs` 48 probes / no measurable cost; identical bag counts probe on and off, so zero false positives over 672 real directories
- [x] all 177 live ledger keys re-computed under both the old and the new key formula: **0 differ, 0 canonicalisation failures**; key stability confirmed for an unmounted autofs share and a nonexistent tail; `mtime_ns = 0` confirmed on 177 of 177 rows
- [x] the symlink-keyed migration hazard reproduced end to end (two `bags` rows for one bag, exit 0, no warning)
- [x] both argument-parsing silent-loss routes reproduced against the built binary (unknown flag swallows a positional bag → exit 0; trailing `--scan` → exit 0; quoted `--scan ""` → exit 1)
- [x] the exit-status fold re-walked against the binary for every reachable combination: scan problem beside a good bag → 1 with `0 failed, 0 not fully readable`; missing scan root → 1 with a summary; no bags → 2; `--help` → 2; over-PATH_MAX subtree → reported, walk survives, 4 of 4 nominated
- [x] both rosdep keys resolved and confirmed installed; core-library purity confirmed unchanged (no rosbag2 include, no new `find_package`, `ament_export_dependencies` untouched)
- [x] `ci_local.sh -n` dry run (full scope, 18 packages, includes this one) and `ci_local.sh:428` `--user root` confirmed; the hosted workflow's build and test package lists confirmed to exclude this package with nothing depending on it
- [x] cross-repo consumers of `bags.path` read in source: `cube_bathymetry/src/batch_regen_main.cpp:421` (exact match, already conservative on a symlinked mount) and `marine_perception_tools/src/sidescan_viewer_window.cpp:4058` (exact match against uncanonicalised input — a real consequence)

### Judgements the host asked for
- [x] **Is the "stopped at" report dead code?** No — keep it. Both routes the fix pass names are cancelled before `increment()`, and so is fd exhaustion (the probe pre-empts it), but a TOCTOU (readable when probed, unreadable a moment later) and a mid-directory `readdir` failure (EIO, a stale network handle) still reach it, and neither is constructible in a test. The fix pass was right that it could not build a case and right to keep the line; "unreachable by any construction" is the overstatement, not the decision.
- [x] **Is root-skipping acceptable as the only ADR-0018 verification?** Not as it stands — and not because the house pattern is wrong (it is genuinely pre-existing: `marine_bathymetry_store/test/test_tile_io.cpp:1104,1184,1251` and `marine_sidescan_mosaic/test/test_tier2_processed_dem.cpp:643`), but because this package has **zero** hosted coverage, so the skip is total rather than partial, and because two of the three skipped guards turn out to be root-observable cheaply (both constructions demonstrated above). The residual mode-000 guard is acceptable; what is not acceptable is a plan that says every permission-based route has a root-observable sibling when four passages assert it and it is false. That is must-fix 3, and it is the same class of finding as round 2's undefended guard: the claim, not the code, is what would let the gap through again.
- [x] **Did the round-2/round-3 pattern repeat?** No. I looked specifically for what this pass *broke* rather than what it fixed, and found nothing: the probe is in the right branch, costs 5% of a walk on the real tree, has zero false positives over 672 real directories, and strictly improves three failure routes; the canonical key is a verified no-op on every live row and stable across mount state; the dependency additions reach nothing they should not. Every must-fix this round is either pre-existing (1), a migration consequence with the code path itself correct (2), or a claim rather than code (3, 4).

### Notes for the host
- **Correctness vs precision, plainly**: must-fix 1 is **correctness** — reproduced silent data loss, three independent readers agreeing, and worth the fifth pass on its own. Must-fix 2 is **correctness** for any host with a symlink-reached ledger row, with a one-sentence documentation floor and a three-line real fix; this host is verified unaffected. Must-fix 3 is **verification** — a real merge-gate hole plus four false claims about it. Must-fix 4 is **accuracy** — one line. Nothing here is polish, and nothing here is a design question.
- **Shape of the remaining work**: two small edits in `main()` (reject a valueless `--scan` and an unknown flag; re-key or document the orphaned row), two test changes with demonstrated constructions, one comment line, and the plan corrections. The fifth pass should re-run the targeted checks, not a full round.
- Scope boundaries honoured and not re-raised: the general libstdc++ `file_clock` pitfall (rolker/ros2_agent_workspace#623) and the identical bug in `marine_perception_tools` (rolker/marine_perception_tools#53).
- **Two follow-ups worth filing, out of scope here**: `marine_perception_tools`' raw `bags.path` identity comparison (`sidescan_viewer_window.cpp:4058`) now that the writer canonicalises, and the fact that 10 of this repo's 18 packages — this one included — sit outside the hosted workflow's build and test lists.
- `git notes --ref=ci-local` carries no record on `0df2f04` yet; the ADR-0018 attestation is the host's step after the fixes land.

### Next step
Lifecycle: **Local Review** → **address-findings** (round 4) → **review-code** (round 5, targeted)

    .agent/scripts/dispatch_subagent.sh --mode in-process --issue 375 --skill address-findings

Nothing was pushed. Ship is **recommended** once the four must-fixes land: they are mechanical, their shapes are named above, and no further independent read is needed to settle any of them.
