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

---

## Implementation
**Status**: complete
**When**: 2026-09-09 12:44 -04:00
**By**: Claude Opus

**Plan**: `.agent/work-plans/issue-369/plan.md` (revised at `0c58c2e`)
**Branch**: feature/issue-369 at `3e90e9a` — **not pushed**, no PR (host publishes)
**Commits**:
- `0c58c2e` plan: revise #369 after plan review — drop the 0.5 m floor, clamp at level 14
- `855d9a2` Add depthAdaptiveLevel: choose a store tile's GGGS level from its depth
- `ed0e3d6` ADR-0010: amend D9 — processed is depth-adaptive and mixed-level
- `3e90e9a` docs: document depthAdaptiveLevel in the store README

### What landed

**Plan revision (`0c58c2e`)** — all four must-fix findings and all four
suggestions from `## Plan Review` addressed, plus the two operator decisions:

| Plan Review finding | Resolution |
|---|---|
| (must-fix) storage estimate wrong — `fromCellSize` is at-or-finer | Ladder and storage recomputed from `level.h:64-70` + `core.h:70,82`; every step stated as 4×, the level-14 clamp as **256×** |
| (must-fix) policy not adaptive in the surveyed range | The 0.5 m floor is **dropped** (operator decision); three transitions now fall inside 1–15 m (9.06 / 4.53 / 2.26 m) |
| (must-fix) D9 amendment would be an as-built claim | Reworded as a decided policy with the writer **pending**, naming cube_bathymetry#143 |
| (must-fix) cube_bathymetry deferral described as "wiring" | Described as the re-architecture it is, citing `import_bag_main.cpp:1030` (one `GeoMapSheet` per run) and `:1130-1133` (store cell size pinned to it) |
| (suggestion) design question 2 unanswered | Answered: one level per store tile, from the **shallowest** depth in it; noted the signature may change when the writer lands |
| (suggestion) `cell·√2/2 < capture radius` is near-tautological | Demoted to a regression guard; the load-bearing tests are the transition depths and both clamps |
| (suggestion) `finest_level = 11` unreachable | Moot — the fine clamp is now 14 and **both clamps genuinely bind**; the tests exercise live bounds |
| (suggestion) `docs/sonar_ecosystem.md` dropped silently | Recorded as **checked, not stale** (its "Store — bathy" row already says "multi-level (D3/D4)") |

**Code (`855d9a2`)** — `marine_bathymetry_store::depthAdaptiveLevel(depth_m,
policy)`: requested cell size `capture_distance_scale · |depth|` (0.05, no
floor) → `gggs::Level::fromCellSize` → clamped to `[8, 14]`. Ladder:
≥72.47 m → 8, ≥36.24 → 9, ≥18.12 → 10, ≥9.06 → 11, ≥4.53 → 12, ≥2.26 → 13,
below → 14 (0.057 m). Fails loud (`std::invalid_argument`) on a non-finite
depth, an inverted or out-of-range clamp, and a non-positive
`capture_distance_scale`; zero depth returns the fine clamp, guarded *before*
`fromCellSize` (whose `log2(0)` → `int` cast is UB). Eight GTest cases:
transition depths (derived from `Level(L).cellSize()`, not transcribed),
literal expectations, both clamps, monotonicity over a 0.05–200 m sweep, sign
symmetry, a custom policy, the throwing cases, and the 9 m regression guard.

**ADR (`ed0e3d6`)** — `uma-ADR-0010` D9 amended in the document's existing
dated-amendment style, plus an "In flight" entry. **README (`3e90e9a`)** — new
"Depth-adaptive level selection" section and the new test target listed.

### Test results (verbatim)

Built with `./core_ws/build.sh marine_interfaces marine_vertical_datum
marine_autonomy marine_tiled_raster_store marine_bathymetry_store` (the
worktree's `core_ws/install` was empty, so the four dependencies were built
first). `stderr output` on `marine_autonomy`/`marine_bathymetry_store` is
pre-existing `-Wunused-result` from GDAL `RasterIO` calls in the S-102 test
fixtures, untouched by this work.

`./core_ws/test.sh marine_bathymetry_store`, final run after every commit:

```
Starting >>> marine_bathymetry_store
[Processing: marine_bathymetry_store]
[Processing: marine_bathymetry_store]
Finished <<< marine_bathymetry_store [1min 7s]

Summary: 1 package finished [1min 8s]
Summary: 355 tests, 0 errors, 0 failures, 42 skipped
```

Per-suite, from the xunit results: `test_depth_adaptive_level` 8/8 pass;
`copyright` 42/42, `cpplint` 42/42, `uncrustify` 42/42, `lint_cmake` 1/1,
`xmllint` 1/1 — all pass with the three new files included. The 42 skips are
`cppcheck`, pre-existing and unrelated.

One test failed on the first run and was fixed, not silenced: the 9 m
regression guard asserted `EXPECT_LT(level, 10)` when "finer than level 10"
means a **higher** GGGS level number. Corrected to `EXPECT_GT` (with a comment
saying why), and the behaviour it checks — the policy returns level 12 at 9 m —
was correct all along.

### Deferred / not done

- **The writer.** Nothing calls `depthAdaptiveLevel`; this PR changes no on-disk
  behaviour. `import_bag` needs
  [cube_bathymetry#143](https://github.com/rolker/cube_bathymetry/issues/143)
  (one `GeoMapSheet` per run ties CUBE's estimation grid to the store tiling).
  Per operator scope, `draft` and the live node stay fixed-level, and the policy
  applies to new imports only — no retroactive reprocess (gated on #366).
- **The knowledge-doc note** flagging that `capture_distance_scale` is duplicated
  across two repos with no automated link stayed a proposal, as the plan framed
  it (agent-instruction changes are operator-decided).

### Actions

- [ ] **Operator decision — coarse clamp 8 or 10?** Kept at 8 as pinned, but the
  rationale that came with it ("a floor one level coarser than today's 10, so
  `processed` is never coarser than today") does not hold: level 8 is *two*
  levels coarser than 10, and the ladder already returns level 9 between ~36 m
  and ~72 m, before the clamp binds. So level 8 **bounds** the coarse end at
  ~3.6 m cells; it does not guarantee "never coarser than today". That is
  acceptable in practice — both operator platforms survey far shallower — but if
  the guarantee is what was wanted, the clamp is `10`, a one-constant change in
  `DepthAdaptiveLevelPolicy` plus the ladder text in the ADR and README. The
  plan, the ADR amendment and the header all state the corrected rationale
  rather than the original claim.
- [ ] Open question carried from the plan: whether `depthAdaptiveLevel` belongs
  in `marine_bathymetry_store` (chosen, following the `s102/run.cpp` precedent)
  or in `marine_autonomy`'s `gggs` module. The Plan Review raised no concern
  about the choice.

---
**Authored-By**: `Claude Code Agent`
**Model**: `Claude Opus`

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-09-09 13:15 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-369 at `e6a76b7`
**Mode**: pre-push (base `origin/jazzy`, no PR)
**Depth**: Deep (reason: 1196 changed lines across 8 files + an ADR amendment)
**Must-fix**: 4 | **Suggestions**: 9
**Round**: 1 | **Ship**: continue — two of the must-fixes are substantive: the amendment's safety argument does not hold against `query.cpp`, and a mixed-level `processed` silently disables the D8 draft anti-clobber

Specialists: Static Analysis (ament_cpplint + ament_uncrustify + cppcheck on the
three new files: clean; `colcon test marine_bathymetry_store` re-run
independently: 355 tests, 0 failures, 42 skipped, `test_depth_adaptive_level`
8/8), Governance, Plan Drift, Claude Adversarial Lens A + Lens B. Copilot and
local-model specialists off (not requested). No `.agents/review-context.yaml` in
this repo — review used `.agents/README.md` only.

The code itself is clean and the arithmetic is right: every cell size, tile
extent, transition depth and 4^(L-10) multiplier in the header, README and ADR
tables was recomputed and holds, `fromCellSize`'s at-or-finer contract is read
correctly, and every cross-repo citation was verified on disk
(`parameters.h:205` = 0.05, `node.cpp:154`, `import_bag_main.cpp:1030`/`:1131-1133`,
`s102/run.cpp:185`) as were issues #366, #369 and cube_bathymetry#143. Plan
adherence is exact — the six planned files, no scope creep, every planned test
present. The findings are concentrated in the ADR amendment's claims about a
mixed-level `processed`, which three store mechanisms do not currently support.

Operator-pinned decisions (0.05 with no floor, fine clamp 14, coarse clamp 8,
offline/`processed` only, new imports only, writer deferred to
cube_bathymetry#143) were treated as settled and are not findings.

### Findings
- [x] (must-fix) The amendment's "why a mixed-level `processed` is safe" paragraph inverts ADR-0013 D8: `shallowestReliable` re-resolves **one** cell per level from the query cell's centre, so a level-10/11 costmap query over level-13/14 tiles point-samples 1 of 64-256 native cells and can miss the shoal those levels exist to resolve. D8 requires reading the finest data *for the region* — it is the obligation this change first makes binding, not a guarantee already met — `docs/decisions/0010-geospatial-world-model.md` (safety paragraph) vs `marine_bathymetry_store/src/query.cpp:112-126`, `:38-44`
- [x] (must-fix) A mixed-level `processed` silently disables the ADR-0010 D8 cross-layer anti-clobber: `clearOverlappedDraft` keys on the processed tile's `GridIndex`, which carries its level, so a level-12 processed tile never matches a level-10 draft grid and clears zero cells with no diagnostic. `bathymetry_store.hpp:238-240` documents the very assumption ("draft and processed both come from CUBE at the store level") that "`draft` stays fixed-level" invalidates; uncleared draft blunders still win in `shallowestReliable` — `marine_bathymetry_store/src/bathymetry_store.cpp:131-136`
- [x] (must-fix) The zero guard is applied to the double but `fromCellSize` receives `static_cast<float>(...)`, so a positive double that underflows in float reaches exactly the UB path the guard and header claim to prevent; built and run, `depthAdaptiveLevel(1e-44)` returns level **8** — the coarsest end, the inversion of the documented shallow-to-finest behaviour. Finite depths above ~6.8e39 overflow to +inf the same way. Validate the narrowed float — `marine_bathymetry_store/src/depth_adaptive_level.cpp:82-90`
- [x] (must-fix) The storage bound is wrong at the low end: "bounded by 1× (all water ≥ 18.1 m) and 256×" contradicts the same paragraph nine lines later — the ladder returns level 9 above ~36 m and level 8 above ~72 m, i.e. 1/4× and 1/16×. True bound is 1/16×-256×; same sentence in the plan — `docs/decisions/0010-geospatial-world-model.md:482-483`, `.agent/work-plans/issue-369/plan.md:200`
- [x] (suggestion) "never has cells coarser than the capture radius" is stated unconditionally but the fine clamp falsifies it below ~1.13 m depth (0.057 m cells vs a 0.05·d radius) — the test quietly starts its invariant loop at the clamp; state the exception in the comment, header, README and ADR — `marine_bathymetry_store/src/depth_adaptive_level.cpp:86-88`
- [x] (suggestion) The decision unit is circular as specified: the chosen level *defines* the tile extent (54.4 m at 14 vs 869.7 m at 10), so "the shallowest depth in that tile" is not determinable before the level is known. Name the fixpoint / separate decision region alongside the "may change when the writer lands" caveat — `marine_bathymetry_store/include/marine_bathymetry_store/depth_adaptive_level.hpp:68-73`
- [x] (suggestion) "the pyramid needs no change" holds only under an unstated writer obligation: native-wins suppresses a derived parent **whole tile**, and unlike `reference`'s disjoint S-102 footprints, depth bands in one contiguous survey can share a parent index — the shallow band's fold is then dropped at that level and coarser. Say the writer must not emit two native levels over the same ground — `marine_bathymetry_store/src/overview_pyramid.cpp:296-300`
- [x] (suggestion) No recovery contract for the throw at the destined call site: NaN is reachable in the intended use (an all-no-data tile's "shallowest depth"), and `import_bag_main.cpp` has no top-level catch around the tile loop, so an uncaught `invalid_argument` would end a multi-hour import. One sentence on expected caller behaviour (abort vs skip tile) — `marine_bathymetry_store/include/marine_bathymetry_store/depth_adaptive_level.hpp:131-135`
- [x] (suggestion) (cross-confirmed by both adversarial lenses) `kMaxGggsLevel = 20` duplicates `gggs::levels.size() - 1` with nothing tying them together — derive it, or `static_assert`, so a GGGS table change is a compile error rather than a silently over-restrictive clamp — `marine_bathymetry_store/src/depth_adaptive_level.cpp:42`
- [x] (suggestion) `RegressionAgainstFixedLevelTen` starts its loop exactly on a level boundary that `TransitionDepths` deliberately refuses to assert on (rounding accident); start at `transitionDepth(finest_level) * 1.001` — `marine_bathymetry_store/test/test_depth_adaptive_level.cpp:191`
- [x] (suggestion) No test pins the default policy constants themselves (0.05 / coarsest 8 / finest 14) — the operator-pinned values the ADR and README publish. The transition and clamp tests derive their expectations from the same struct, so changing a default passes the whole suite while contradicting the published ladder — `marine_bathymetry_store/test/test_depth_adaptive_level.cpp`
- [x] (suggestion) The amendment's bolded lead sentence reads as as-built ("`processed` **is** a depth-adaptive, mixed-level layer") and is only qualified four paragraphs later; "is to be" matches the pending-writer framing the rest of the amendment is careful about — `docs/decisions/0010-geospatial-world-model.md:449-451`
- [x] (suggestion) Pre-existing, outside the diff but now load-bearing in a second place: `fromCellSize`'s `@return` line says "smallest cells that are >= cell_size", contradicting its own brief (at-or-finer, which is the true contract this policy depends on) — `marine_autonomy/include/marine_autonomy/gggs/level.h:58-63`

### Governance

| Principle | Verdict | Notes |
|---|---|---|
| Capture decisions, not just implementations | Watch | The D9 amendment lands with the change, but three of its claims overstate what the store supports today (must-fix 1, 2, suggestion on the pyramid) |
| A change includes its consequences | Concern | The cross-repo writer consequence is filed and sized honestly (cube_bathymetry#143, verified OPEN), but two in-repo consequences of a mixed-level `processed` — the sub-sampled safety query and the disabled draft anti-clobber — are neither named nor filed |
| Documentation accuracy | Concern | Every citation verified and correct; the storage lower bound (must-fix 4) is not |
| Enforcement over documentation | Pass | Clamps, edge cases and the ladder are a function plus 8 tests, not prose (see the suggestion on pinning the defaults) |
| Test what breaks | Pass | Transitions, both clamps, monotonicity, sign symmetry, fail-loud cases and the #369 regression guard; 8/8 verified independently |
| Only what's needed / Improve incrementally | Pass | One header/source pair, one test file, CMake, README, ADR — no scope creep from the plan |

| ADR | Triggered | Compliant | Notes |
|---|---|---|---|
| `uma-ADR-0010` D9 | Yes | Partly | Amended as required; wording and the storage bound need the corrections above |
| `uma-ADR-0013` D2/D3 | Yes | Partly | Generic coverage-manifest machinery is real, but whole-tile native-wins suppression makes "needs no change" conditional on a writer obligation |
| `uma-ADR-0013` D8 | Yes | No | Cited as a guarantee already met; `query.cpp`'s per-level point sample does not meet it once `processed` is finer than the query level |
| ADR-0008 (ROS 2 conventions) | Yes | Yes | License header matches the package, `install(DIRECTORY include/)` covers the new header, library source and gtest target registered correctly |

Consequence check: ADR-0010 D9 amended (done); `marine_bathymetry_store/README.md`
updated (done); `docs/sonar_ecosystem.md` verified not stale (line 96 already says
"multi-level (D3/D4)") — correctly recorded rather than silently dropped;
`.agents/README.md` has no verified-parameter table in this repo, so no entry is
owed. Instruction-update **candidate** (proposal only, operator decides): a
`.agent/knowledge/` note that `capture_distance_scale` is duplicated across two
repos with no automated link — as the plan framed it.

### Plan Adherence

Exact. All six planned files changed and nothing else; the policy struct,
edge-case decisions, ADR bullets and test list all match the revised plan, and
the implementation adds two validations (out-of-range clamp, non-positive scale)
beyond it. One sync item: the plan repeats the wrong storage lower bound at
`plan.md:200` and should be corrected with the ADR.

### Next step

Verdict is changes-requested, so the next phase is `address-findings` on this
entry, then a re-dispatch of `review-code`. Must-fixes 1 and 2 are the ones to
settle first, and both are operator-visible: they may be answered by rewording
the amendment plus filing follow-ups (a region-aware `shallowestReliable`, and a
level-aware `clearOverlappedDraft`) rather than by code in this PR — but they
should not be left unstated in the record. Not pushed; no PR.

---
**Authored-By**: `Claude Code Agent`
**Model**: `Claude Opus`

## Implementation
**Status**: complete
**When**: 2026-09-09 13:57 -04:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-369 at `791a740`
**Addressed**: `## Local Review (Pre-Push)` (2026-09-09 13:15 -04:00, verdict
changes-requested, branch at `e6a76b7`) — all 4 must-fixes and all 9 suggestions
**Commits**: `fd5788d`, `803958f`, `753940a`, `b047a39`, `ca7b9c2`, `437b34e`,
`de6d253`, `f3b2996`, `791a740`

Both operator-pinned must-fixes (1 and 2) were fixed **in code here**, as
directed, not by rewording the amendment. Each has regression tests that were
**run against the unfixed code first and observed to fail**, so they are proven
to bite; no existing test was weakened or skipped. All 13 findings are actioned —
none deferred, none declined.

### Actions

- [x] (must-fix) `shallowestReliable` reads the finest data for the REGION —
  `marine_bathymetry_store/src/query.cpp`. Levels at or coarser than the query
  cell still point-resolve (one such cell contains the whole query cell). A
  finer level now enumerates **every** native cell the query cell covers and
  keeps the shoalest reliable value. `reliableSamples` got the same treatment —
  the finding named only `shallowestReliable`, but that function's contract is
  "the caller costs each sample and takes the most hazardous", so a centre
  point-sample would have hidden the same hazard. `bestSource` is deliberately
  left point-resolving (it is the best-available display lookup) and now says so
  in its docs. Cost stays bounded by data: grids with no tile in the layer are
  skipped without touching a cell. Verified failing first: 4 of the 5 new tests
  (`ShallowestReliableReadsEveryCoveredFineCell` -30 vs -0.4,
  `ShallowestReliableCoversAFourLevelStep` -25 vs -0.2,
  `ShallowestReliableDoesNotReadBeyondTheQueryCell`,
  `ReliableSamplesReturnsEveryCoveredFineCell` 1 sample vs 15). The fifth
  (`ShallowestReliableStillPointResolvesCoarserLevels`) is a no-change guard and
  passes both ways, by design.
- [x] (must-fix) `clearOverlappedDraft` is level-aware —
  `marine_bathymetry_store/src/bathymetry_store.cpp`. It now walks every GGGS
  level the `draft` layer holds. Draft at or finer than the processed tile: the
  draft cell lies inside exactly one processed cell (levels nest exactly), which
  decides it; the gated-drop-hole rule is unchanged. Draft **coarser**: cleared
  only where this tile fully supersedes it (the cell lies entirely inside the
  tile and every processed cell under it has data) — otherwise kept, because
  clearing would discard draft data over ground this tile does not speak for and
  a wrongly-cleared draft cell is a lost hazard, and **counted** in the new
  `DraftClearResult::coarse_draft_cells_retained`. No level combination is a
  quiet no-op now; an invalid processed `GridIndex` throws rather than returning
  an empty result. Both overloads deduplicate `tiles_touched` (a coarse draft
  tile can now be reached by several processed tiles). The stale contract comment
  at `bathymetry_store.hpp:238-240` is replaced by the level-aware contract.
  Verified failing first: both cross-level tests cleared **0** cells against the
  previous code.
- [x] (must-fix) Float-narrowing hole in the zero guard —
  `marine_bathymetry_store/src/depth_adaptive_level.cpp`. The guard now tests the
  narrowed `float` that `fromCellSize` actually receives, at both ends:
  underflow (a positive double that is `0.0f`, e.g. 1e-44) returns `finest_level`
  and overflow to `+inf` returns `coarsest_level` — each the shoal-biased end for
  that input, and both short-circuiting the undefined `log2(0)`/`log2(inf)` cast.
  Verified failing first: `depthAdaptiveLevel(1e-44)` returned level **8**.
- [x] (must-fix) Storage bound corrected to **1/16×–256×** in both places —
  `docs/decisions/0010-geospatial-world-model.md` and
  `.agent/work-plans/issue-369/plan.md`. The plan's table also gained its two
  missing coarse rows (level 9 = 1/4×, level 8 = 1/16×).
- [x] (suggestion) The capture-radius claim's clamp exception (below ~1.13 m the
  fine clamp holds cells coarser than the radius) is now stated in the comment,
  the header, the README and — via the ladder table's clamp rows — the ADR.
- [x] (suggestion) The decision unit's circularity is named, with what the writer
  must do about it (a level-independent decision region, or a fixpoint
  iteration) — `depth_adaptive_level.hpp`, README.
- [x] (suggestion) The pyramid's "needs no change" is now stated with its writer
  obligation (whole-tile native-wins suppression means no two native levels over
  one parent index) — in the ADR and at the suppression site,
  `marine_bathymetry_store/src/overview_pyramid.cpp:296`.
- [x] (suggestion) Recovery contract for the throw: a decision unit with no data
  (shallowest depth = NaN) should be **skipped** by the caller, not allowed to end
  a multi-hour import; configuration throws are fatal at startup —
  `depth_adaptive_level.hpp`, README.
- [x] (suggestion, cross-confirmed) `kMaxGggsLevel` derives from the GGGS level
  table (`std::tuple_size` + two `static_assert`s) instead of duplicating 20 — a
  table change is now a compile error.
- [x] (suggestion) `RegressionAgainstFixedLevelTen` starts at
  `transitionDepth(finest_level) * 1.001`, off the boundary `TransitionDepths`
  deliberately declines to assert on.
- [x] (suggestion) `DefaultPolicyIsThePublishedOne` pins 0.05 / coarsest 8 /
  finest 14 directly, so changing a default can no longer pass a suite whose
  other expectations derive from the same struct.
- [x] (suggestion) The amendment's bolded lead reads "is *to be*", with a
  parenthetical separating what landed (policy + store support) from what has
  not (the writer).
- [x] (suggestion) `fromCellSize`'s `@return` corrected to at-or-finer —
  `marine_autonomy/include/marine_autonomy/gggs/level.h:58-63`.

### Found while fixing (not in the review)

- **Boundary-alignment bug in the new region walk.** Writing the cross-level
  draft tests surfaced it: a box corner that coincides exactly with a cell
  boundary — which it always does here, since GGGS levels nest exactly — can land
  an ulp on the wrong side when the two levels' spans are computed by different
  routes, starting a walk one cell short and reading no-data outside the box.
  `insetForIteration` now insets **both** corners, not just the maximum. Caught
  by `ClearOverlappedDraftClearsCoarserDraftOnlyWhereFullySuperseded` failing
  against my own first implementation.
- **`src/cell_geometry.hpp`** (new, internal, not installed) holds the extent and
  inset helpers, so the query walk and the draft clear share one copy of that
  arithmetic rather than drifting apart.

### Build and test (run from the worktree root, verbatim)

`./core_ws/build.sh marine_bathymetry_store` — `Finished <<< marine_bathymetry_store`.

`./core_ws/test.sh marine_bathymetry_store`:

```
Summary: 369 tests, 0 errors, 0 failures, 43 skipped
```

(was 355/42 before this pass: +14 gtests — 5 query, 3 store, 2 policy, and 4 more
lint/cppcheck entries from the new header; the 43rd "skip" is cppcheck's per-file
entry for `cell_geometry.hpp`.) Every linter clean: copyright, cppcheck, cpplint,
lint_cmake, uncrustify, xmllint all `failures=0`.

`./core_ws/test.sh marine_autonomy` (the `level.h` doc change):

```
Summary: 516 tests, 0 errors, 0 failures, 59 skipped
```

`./core_ws/test.sh bathymetry_layer marine_tiled_raster_store marine_sidescan_mosaic`
(the store's in-repo consumers, after a full `./core_ws/build.sh`):

```
Summary: 892 tests, 0 errors, 9 failures, 101 skipped
```

All 9 failures are **pre-existing lint failures in `marine_sidescan_mosaic`**
(2 cpplint line-length in `src/sidescan_mosaic_bag.cpp`, 5 uncrustify diffs, and
the two aggregate entries) — untouched by this branch, which changes no file in
that package. `bathymetry_layer` (the safety consumer of `shallowestReliable`) and
`marine_tiled_raster_store` are green.

### Deferred / not done

Nothing from the review. Still deliberately out of scope, per the operator's pins:
the writer that will call `depthAdaptiveLevel` (cube_bathymetry#143), retroactive
reprocessing of existing level-10 stores (gated on #366), and the proposed
`.agent/knowledge/` note about `capture_distance_scale` being duplicated across
two repos with no automated link — a proposal for the operator, not an action
taken here.

Not pushed; no PR opened.

### Next step

Lifecycle: **Implementation** → **review-code** (re-review the fixes cold).

---
**Authored-By**: `Claude Code Agent`
**Model**: `Claude Opus`
