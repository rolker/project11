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

## Notes (not part of the review-issue schema; kept for plan-task context)

### Scope Assessment
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

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Test what breaks | OK | The three proposed tests target the actual regressions (mtime accuracy, same-size-different-content re-fingerprint, ledger round-trip skip/re-index) rather than framework glue. Matches `docs/PRINCIPLES.md`-adjacent "Test what breaks" guidance well. |
| A change includes its consequences | OK | Issue explicitly scopes the one-time full re-index consequence of existing `mtime_ns=0` rows and says to state it in the PR. No stale docs/tests left dangling — `marine_survey_index/test/` has no `fingerprint()` coverage today and the issue adds it. |
| Human control and transparency | Watch | The fix needs an explicit decision on what an unreadable mtime means (issue already flags this: "a bag whose mtime cannot be read should probably fail the unchanged test rather than pass it"). Silently contributing nothing to a max is exactly how the original bug stayed invisible — the implementer should make the unreadable-mtime failure mode loud (log or a distinguishable sentinel), not just directionally safe. |
| Only what's needed | OK | Fix is scoped to the accumulator and its sentinel/policy; no speculative expansion (e.g., no proposal to change the ledger schema or add new columns). |
| Improve incrementally | OK | Single PR, single package, reviewable. |
| Safety First / Hardware Agnosticism / Simulation-First (project PRINCIPLES.md) | N/A | This is an offline indexer bug (derived-cache correctness), not vehicle control, hardware interface, or simulation-validated behavior. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| 0008 — ROS 2 conventions | Marginal | Touches C++ source in a ROS 2 package, but the fix is a bugfix within the existing C++17 standard already set in `marine_survey_index/CMakeLists.txt` (`CMAKE_CXX_STANDARD 17`). The issue's own fix options (stat/statx path, or C++20 `to_sys` if the standard were bumped) already account for this — no standard bump is implied or needed. |
| Others (0001–0010, 0013) | No | No new agent instructions, enforcement rule, Make target, or `progress.md`-writing skill involved. |

### Consequences

- `marine_survey_index/test/` gains `fingerprint()` coverage (currently absent) —
  in scope, already called out by the issue.
- Existing on-disk indexes (177 rows on dev host, `mtime_ns=0`) will one-time
  full-reindex after the fix — in scope, already called out by the issue as
  something to state in the PR, not something to pre-migrate.
- Optional/deferred: a `.agent/knowledge/` note on the libstdc++ `file_clock`
  epoch pitfall (see Recommendation above) — proposed as a candidate for
  operator approval, not required for this PR.

### Recommendations

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
