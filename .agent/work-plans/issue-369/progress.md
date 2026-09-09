---
issue: 369
---

# Issue #369 — Survey tiles are written at one fixed level regardless of depth — CUBE's capture radius adapts, the grid does not

## Issue Review
**Status**: complete
**When**: 2026-09-09 11:56 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #369
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: needs-more-detail

### Actions
- [ ] Pin the forward-only vs. retroactive-reprocess scope split before plan-task: design question 6 ("Existing stores") makes retroactive reprocessing of Shoals/Massabesic contingent on #366 (the import ledger), which is still **OPEN**. Recommend scoping this PR to new imports only and filing retroactive reprocessing as a follow-up gated on #366, unless #366 lands first.
- [ ] Pin the live-vs-offline scope (design question 4) toward the option the issue itself names as best-fitting: `draft` stays fixed-level at write time (matches the existing D8 draft/processed asymmetry — draft is disposable/regenerable, processed is authoritative) and only `processed`/`import_bag` gets depth-adaptive level selection. Adapting the live node raises retiling machinery that isn't otherwise in scope and would make this a much larger change.
- [ ] Record the resulting decision as a `uma-ADR-0010` D9 amendment (the issue itself proposes this under "Where the work lands") — D9 currently states plainly "born at fine native levels," which a depth-adaptive `processed` layer contradicts. Capture-decisions principle: this is a store-contract change, not just an implementation detail.
- [ ] Verify, not just note, the capacity-radius/lattice-spacing question in the issue's "Worth verifying" paragraph: at a 0.9 m lattice the farthest a sounding can sit from its nearest node is 0.64 m, which exceeds the 0.5 m capture floor that applies below 10 m depth. Confirmed by reading `capture_distance_scale` (0.05, `parameters.h:205`) and its use in `node.cpp:154` — the formula in the issue is accurate. Whether shallow soundings are actually dropped depends on the node-selection loop, which the issue explicitly says it did not trace; this should be resolved with a test before or during implementation, not left as a residual note.
- [ ] State the seam behaviour explicitly (design question 3) rather than assuming it. `uma-ADR-0013` D8 already establishes that `shallowestReliable()` scans all layers and levels regardless of what's drawn, so a depth-adaptive `processed`/`draft` layer is safe by the same argument #331 already used for `reference`'s native-wins pyramid — but that argument should be written down for this layer too, not inherited silently.
- [ ] If `processed` becomes a mixed-level, region-disjoint layer (the same shape #331 built for `reference`), it inherits `uma-ADR-0013` D2 (producer-side error nesting) and D3 (a declared coverage manifest) obligations — the writers (`import_bag`, the depth-pyramid builder) need to emit both, following the precedent #331 already set.
- [ ] Design question 5 (bounds) needs a concrete floor, ceiling, and a storage estimate in the plan — "worth naming" in the issue is not sufficient; an unbounded fine level in shallow water is exactly the kind of open-ended default this workspace's principles flag as needing a stated decision, not a default left implicit.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | Issue lays out its reasoning and open questions plainly; no hidden behavior proposed. |
| Enforcement over documentation | Watch | The eventual level-choice policy needs a validated bound (question 5) and a documented seam rule (question 3), not just prose — flag as plan-task scope. |
| Capture decisions, not just implementations | Action needed | The issue itself expects to amend `uma-ADR-0010` D9; that amendment should land as part of this work, not be assumed. |
| A change includes its consequences | Action needed | Existing-store reprocessing (question 6) is explicitly entangled with #366 (open); scope must be pinned so the PR's consequences are bounded and don't silently wait on an unrelated open issue. |
| Only what's needed | OK | Motivated by a measured mismatch (0.9 m cells vs. ~8 mm–12 cm sensor footprint across the surveyed depth range), not speculative. |
| Improve incrementally | Watch | As written (6 open design questions, two writers, a possible ADR amendment, a possible retroactive reprocess) this is large for one PR; the issue's own "draft fixed / processed adapts" and "new imports only" options are the way to keep it incremental. |
| Test what breaks | Action needed | The 0.64 m vs. 0.5 m capture-floor question is a concrete correctness risk (possible silently-uncaptured shallow soundings) and needs a regression test, not just a documentation note. |
| Workspace vs. project separation | OK | Correctly filed in `unh_marine_autonomy` (project repo); this is domain content (bathymetry store policy), not workspace infra. |
| Workspace improvements cascade to projects | N/A | Not applicable — project-domain change. |
| Primary framework first, portability where free | N/A | Not applicable. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| `uma-ADR-0010` (Geospatial World Model) | Yes | D9 ("LOD is a per-layer process") currently states `draft`/`processed` are "born at fine native levels" — a depth-adaptive level would amend this directly, as the issue itself anticipates. |
| `uma-ADR-0013` (Bounded LOD Navigation) | Yes | D2 (producer-side error nesting) and D3 (declared coverage manifest) apply if `processed` becomes mixed-level/region-disjoint, same as #331 did for `reference`. D8 (safety queries never consult an LOD level) already covers the safety argument the issue asks about in question 3 — it should be cited explicitly rather than re-derived. |
| ADR-0001 (Adopt ADRs) | Yes | A store-contract decision of this size needs its rationale captured, per the issue's own "Where the work lands" section. |

### Consequences

- Depends on #366 (import ledger, OPEN) if retroactive reprocessing of existing stores (Shoals, Massabesic) is included in scope — recommend excluding it from this PR and tracking separately.
- Touches both `cube_bathymetry` (writer: `import_bag`, live node) and `unh_marine_autonomy` (store-contract policy, `uma-ADR-0010` D9) — cross-repo consequence, both should land together or be sequenced explicitly.
- If `processed` becomes mixed-level, `docs/sonar_ecosystem.md` and any store-contract documentation referencing "one native level per survey layer" need updating in the same PR.

### Recommendations

- Scope this PR to: (1) the level-choice policy for `processed` only, driven by depth per the issue's own preferred proxy, decided per-tile-or-region per a two-pass `import_bag` scheme; (2) leave `draft` fixed-level, matching the existing D8 draft/processed asymmetry; (3) new imports only, with retroactive reprocessing filed as an explicit follow-up gated on #366.
- Carry the `uma-ADR-0010` D9 amendment in the same PR as the implementation (matches how #331 landed its D9 mixed-level amendment for `reference`) rather than deferring documentation.
- Add a regression test for the capture-floor/lattice-spacing interaction (0.05×depth vs. 0.5 m floor vs. cell diagonal) before relying on the depth-adaptive level to close the gap the issue describes.

---

## Plan Authored
**Status**: complete
**When**: 2026-09-09 12:06 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-369/plan.md` at `38b720e`
**Branch**: feature/issue-369 at `38b720e`
**Phases**: single

### Open questions
- [ ] File the companion `cube_bathymetry` issue (wiring `import_bag` to call the new depth-level policy per grid/tile) before implementation starts, so the two PRs can be explicitly sequenced/cross-linked — no `cube_bathymetry` worktree exists in this dispatch.
- [ ] Confirm the deliberate level-8 coarsest-floor choice against any deeper-water campaigns on the near-term roadmap.
- [ ] Confirm `depthAdaptiveLevel` belongs in `marine_bathymetry_store` (this plan's choice) rather than `marine_autonomy`'s `gggs` module.

---
**Authored-By**: `Claude Code Agent`
**Model**: `Claude Sonnet`

---

## Plan Review
**Status**: complete
**When**: 2026-09-09 12:09 -04:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-369/plan.md` at `38b720e`
**PR**: PR-less (`--issue` mode, dispatched fresh-context sub-agent)
**Verdict**: changes-requested

### Findings
- [ ] (must-fix) The storage estimate is wrong: `gggs::Level::fromCellSize` returns the level whose cells are **≤** the requested size (`level.h:64-70`, `test_gggs.cpp:269-270`), so a 0.5 m capture radius maps to level 11 (0.453 m), and level 10 (0.906 m) is not reached until the capture radius is ≥ 0.906 m — depth ≥ ~18.1 m. The claim "water ≥ 10 m still lands at level 10 as today" and "not a uniform 4× over the whole survey" are both false for the motivating 1–15 m Shoals box — `plan.md` step 2, "Storage estimate" bullet
- [ ] (must-fix) Consequence of the above: the policy is not depth-adaptive anywhere the platforms survey. Level transitions occur only at ~18.1 m (11→10), ~36.2 m (10→9) and ~72.5 m (9→8); across 0–18 m it returns a constant level 11. As written this PR decides a **new fixed level** (10 → 11) for the survey layer, not a depth-adaptive one. Either decouple the cell size from the capture radius (e.g. cell = k·capture with k < 1) so transitions fall inside the operating range, or state the fixed-level-in-practice outcome plainly in the ADR amendment — `plan.md` step 1 and step 2
- [ ] (must-fix) The D9 amendment as drafted would state that `processed` tiles "are now written at a level chosen by `depthAdaptiveLevel`" while nothing in either repo calls it. That is an as-built claim for behavior that does not exist (Documentation Accuracy). Phrase it as the decided policy with the writer wiring pending, naming the companion `cube_bathymetry` issue — `plan.md` step 4, first bullet
- [ ] (must-fix) The deferral to `cube_bathymetry` is described as "wiring" but is a re-architecture: `import_bag_main.cpp:1129-1136` builds one `GeoMapSheet(float cell_size)` for the whole run and pins `accumulator_config.cell_size_m` to `geo_map_sheet.nominalCellSizeMeters()` — CUBE's estimation grid and the store level are the *same* resolution, one per sheet. Per-tile or per-region levels require multiple sheets or a reworked accumulator. Say so in the plan, and file the companion issue **before** implementation (it is currently an Open Question, not a step) so the sequencing is real — `plan.md` Consequences table, row 2; Open Questions, item 1
- [ ] (suggestion) The issue's design question 2 (unit of the decision — per tile / per region / two-pass) is left unanswered, yet `depthAdaptiveLevel(double depth_m)` presumes a scalar depth per decision unit. Record the intended unit in the ADR amendment, or note that the signature may change when the caller lands — `plan.md` step 1
- [ ] (suggestion) The step-3 "general invariant" (`cell·√2/2 < capture radius`) is near-tautological given `fromCellSize`'s ≤ contract — it tests the GGGS API, not the policy. Keep it as a regression guard, but the load-bearing assertions are the explicit expected levels at the transition depths (~18.1 / ~36.2 / ~72.5 m) and at the clamps — `plan.md` step 3
- [ ] (suggestion) `finest_level = 11` is unreachable with the default constants (a 0.5 m floor can never demand finer than level 11), so the clamp only binds if the mirrored `cube_bathymetry` constants change. Worth noting so the boundary test does not claim to exercise a live bound — `plan.md` step 1, step 3
- [ ] (suggestion) The Issue Review named `docs/sonar_ecosystem.md` as a possible stale doc; the plan drops it silently. I checked it — line 96 asserts no single native level for `processed`, so it is not stale, but the Documentation & Instruction Impact section should record "checked, not stale" rather than omit it — `plan.md` Documentation & Instruction Impact

### Verified as accurate
The plan's citations all check out: level cell sizes (8 ≈ 3.62 m, 9 ≈ 1.81 m, 10 ≈ 0.906 m, 11 ≈ 0.453 m); the level-8 crossover at ~72.5 m ("~73 m"); the `s102/run.cpp:184-185` `fromCellSize` precedent; `buildDepthOverviewPyramid`'s coverage of `draft`/`processed`/`reference` (`overview_pyramid.hpp:45-51`); `BathymetryStore`'s level-agnostic contract (`bathymetry_store.hpp:84-85`); `capture_distance_scale = 0.05` (`parameters.h:205`) and its use (`node.cpp:154`); `uma-ADR-0010` D9's "born at fine native levels" and its D8 safety argument.

### Evaluation

| Dimension | Verdict | Notes |
|---|---|---|
| Scope | Needs work | Right size for one PR, but it changes no on-disk behavior and its value rests entirely on a companion PR that is larger than the plan implies |
| Issue alignment | Needs work | Design question 2 unanswered; the depth-adaptivity the issue asks for does not materialize over the motivating depth range under this policy |
| File targeting | Good | Header/source/CMake/test/README/ADR is the complete set for this repo |
| Consequences | Needs work | Cross-repo consequence named but its true size understated; `sonar_ecosystem.md` dropped without a verdict |
| Documentation & instruction impact | Good | Section present, non-silent, candidates framed as proposals |
| Principle alignment | Needs work | "Capture decisions" is served, but the amendment would enshrine an incorrect storage number and an as-built claim for absent behavior |
| ADR compliance | Concern | D9 amendment wording and numbers both need correcting before they land |
| ROS conventions | Good | Pure library function; no topics, parameters, QoS or lifecycle surface |

### Summary
The plan is well-researched and every source citation in it holds up, but its central quantitative claim does not: because `fromCellSize` snaps to the level at or finer than the requested cell size, the proposed policy returns level 11 for all water shallower than ~18 m — a uniform 4× tile-count increase across the whole Shoals survey and no adaptivity within it. That number and the resulting behavior are slated to be written into `uma-ADR-0010` D9, so they must be corrected before implementation, not after. Not ready for implementation as written.

### Recommended Actions
- [ ] Recompute the level/depth mapping against `fromCellSize`'s actual ≤ contract and fix step 2's storage estimate and crossover claims
- [ ] Decide explicitly whether to make the policy adaptive within 0–18 m (decouple cell size from the capture radius) or to record it as a fixed level 10 → 11 change with adaptivity only in deeper water — this is an operator-visible decision, not an implementation detail
- [ ] Reword the D9 amendment so it states the decided policy with the writer pending, never as-built behavior
- [ ] File the companion `cube_bathymetry` issue before implementation begins, describing the `GeoMapSheet` single-resolution constraint honestly
- [ ] Add "checked, not stale" for `docs/sonar_ecosystem.md`; retarget the tests onto the transition depths and clamps
