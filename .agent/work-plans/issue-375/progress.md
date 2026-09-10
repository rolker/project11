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
