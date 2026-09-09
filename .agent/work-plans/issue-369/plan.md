# Plan: Survey tiles are written at one fixed level regardless of depth

## Issue

https://github.com/rolker/unh_marine_autonomy/issues/369

## Context

`~/data/world/depths/processed` and `reference` write every tile at one fixed
GGGS level. `chart` already varies resolution with source compilation scale.
CUBE's own capture radius (`cube_bathymetry/include/cube_bathymetry/parameters.h`,
`capture_distance_scale = 0.05`, used in `node.cpp:154` as
`max(0.05 * |depth|, 0.5 m)`) is depth-adaptive; the lattice it writes onto is
not — level 10 (~0.9 m cells) covers the whole 1–15 m+ Isles of Shoals depth
range.

**Operator-pinned scope** (2026-09-09 run-issue checkpoint 1, not re-opened
here):

1. **Driver = depth**, using CUBE's own `max(0.05·|depth|, 0.5 m)` proxy.
2. **Writer = offline `import_bag` / `processed` only.** `draft` and the live
   node stay fixed-level. No retiling in this PR.
3. **New imports only.** No reprocessing of the existing level-10 Shoals or
   Massabesic `processed` stores; that is a follow-up gated on
   [#366](https://github.com/rolker/unh_marine_autonomy/issues/366) (import
   ledger).

**Store-side machinery already exists and needs no new work.** `BathymetryStore`
is explicitly multi-level and level-agnostic today (`bathymetry_store.hpp`:
"tiles at heterogeneous GGGS levels coexist within a layer",
`importTiles`/`set` accept any level, the constructor's `gggs_level` is only
the `cellIndex(lat,lon)` convenience default). `buildDepthOverviewPyramid`
(`marine_bathymetry_store/overview_pyramid.hpp`, landed via
[#331](https://github.com/rolker/unh_marine_autonomy/issues/331)) already
covers **`draft`, `processed` AND `reference`** with mixed-level, native-wins
folding, `uma-ADR-0013` D1/D2/D3 per-tile geometric error, and a coverage
manifest — all generic over layer directory, not `reference`-specific. So the
D3/D2 "emit per-tile error + coverage manifest" obligation from the Issue
Review is **already discharged** for `processed` the moment `processed` tiles
exist at more than one level; nothing new needs writing there.

What is genuinely missing is narrower than the issue's six design questions
suggested: **a depth → GGGS-level policy function**, callable at import time.
There is a direct precedent for exactly this shape:
`marine_bathymetry_store/src/s102/run.cpp:184-185` already selects
`import_options.level = gggs::Level::fromCellSize(record.resolution_m).level()`
per S-102 dataset resolution. This plan adds the depth-driven analogue.

**Where the writer wiring lands is out of scope for this repo/worktree.**
`import_bag` (`cube_bathymetry/src/import_bag_main.cpp:1130-1132`) currently
computes one `accumulator_config.cell_size_m` for an entire run and passes it
to a single-level `GeoMapSheet`/`BathymetryStore::fromCellSize`. Making
`import_bag` choose a level per grid/tile from the CUBE-estimated depth is a
`cube_bathymetry` change (`sensors_ws`, a different repo — no worktree for it
exists here). This PR ships the store-contract policy function and its ADR
amendment in `unh_marine_autonomy`; wiring `import_bag` to call it is filed as
an explicit companion issue (see Consequences) so the two land as a sequenced
pair, per the Issue Review's cross-repo consequence note.

## Approach

1. **Add `marine_bathymetry_store::depthAdaptiveLevel(depth_m)`.** New
   `include/marine_bathymetry_store/depth_adaptive_level.hpp` +
   `src/depth_adaptive_level.cpp`, added to the library sources
   (`CMakeLists.txt` alongside `overview_pyramid.cpp`). Pure function:

   ```cpp
   struct DepthAdaptiveLevelPolicy {
     float capture_distance_scale = 0.05f;  // mirrors cube_bathymetry::Parameters
     float capture_distance_floor_m = 0.5f; // mirrors cube_bathymetry::Parameters
     uint8_t finest_level = 11;             // ceiling — see step 2
     uint8_t coarsest_level = 8;            // floor — see step 2
   };
   gggs::Level depthAdaptiveLevel(
     double depth_m, const DepthAdaptiveLevelPolicy & policy = {});
   ```

   Body: `cell_size = max(policy.capture_distance_scale * |depth_m|,
   policy.capture_distance_floor_m)`, then
   `gggs::Level::fromCellSize(cell_size)`, clamped into
   `[coarsest_level, finest_level]`. Choosing `cell_size` **equal to** the
   capture radius (not merely proportional to it) is what makes the
   diagonal-vs-floor question in step 3 provable in general, not just
   spot-checked: for any depth, the resulting tile's diagonal is
   `cell_size · √2/2 ≈ 0.707 · capture_radius(depth) < capture_radius(depth)`,
   so a node can never be farther from its nearest cell than its own capture
   radius, by construction.

   Doc comment cites `cube_bathymetry/include/cube_bathymetry/parameters.h`'s
   `capture_distance_scale`/floor as the source of truth the defaults mirror,
   and flags that a change to one must be reviewed against the other (no
   automated link between the two repos' constants — this is a `.agent/`
   knowledge-file candidate, tracked in Documentation & Instruction Impact).

2. **Derive concrete floor/ceiling, not placeholders.** Using
   `gggs::Level(N).cellSize()` (nominal cell size halves per level; level 10
   ≈ 0.906 m per the issue's own "~0.9 m" figure):

   | Level | Nominal cell size |
   |---|---|
   | 8 | ~3.63 m |
   | 9 | ~1.81 m |
   | 10 | ~0.906 m (today's fixed level) |
   | 11 | ~0.453 m |

   - **Ceiling (finest) = level 11.** The capture-radius floor
     (`max(0.05·d, 0.5)` bottoms out at exactly 0.5 m for any `d ≤ 10 m`), so
     `depthAdaptiveLevel` never asks for cells finer than
     `fromCellSize(0.5)` = level 11, regardless of how shallow the water gets.
     This is a **derived** bound, not a chosen one — it falls straight out of
     reusing CUBE's own floor, and it is what keeps a 0.3 m puddle from
     demanding an unbounded tile count (closes design question 5's "unbounded
     fine level" risk without a separate arbitrary cap).
   - **Floor (coarsest) = level 8.** Below 10 m the CUBE formula would keep
     coarsening (e.g. 3.63 m cells at ~73 m depth), but Isles of Shoals tops
     out at 15 m+ and Massabesic is shallower still — the operator platforms
     have not surveyed water where the unclamped formula would reach level 8
     on its own. Level 8 is chosen as a deliberate floor one level coarser
     than today's fixed level 10, i.e. `processed` is guaranteed to be **no
     coarser than today**, only ever finer or equal — a strict resolution
     improvement, never a regression. (`chart`'s own coastal-scale levels
     bottom out around level 6-8 per `uma-ADR-0010` D7, so level 8 also keeps
     `processed` from crossing into chart's coarse territory.)
   - **Storage estimate.** Level 11 cells are `(0.906/0.453)² = 4×` the tile
     density of level 10 per unit area covered at the finest end. Only water
     shallower than ~10 m reaches level 11 (by construction, step 1); water
     ≥ 10 m still lands at level 10 as today (`0.05·10 = 0.5` — the formula's
     crossover). For the Shoals survey box (mixed 1–15 m+), this bounds the
     worst case at a 4× tile-count increase over today's uniform level 10,
     concentrated in the shallow fraction of the footprint — not a uniform
     4× over the whole survey. Record this estimate in the ADR amendment
     (step 5) so it is not re-derived later.

3. **Regression test — resolve the 0.64 m vs 0.5 m question with a proof, not
   a spot check.** New `marine_bathymetry_store/test/test_depth_adaptive_level.cpp`
   (registered in `CMakeLists.txt` next to `test_depth_overview`):
   - Parametrized/looped over a depth range (e.g. 0.1 m to 20 m in fine
     steps, plus the exact boundary depths for each level transition) asserting
     `depthAdaptiveLevel(depth).cellSize() * std::sqrt(2.0) / 2.0 <
     std::max(0.05 * std::abs(depth), 0.5)` — i.e. the general invariant from
     step 1, not just the specific 0.9 m/0.64 m/0.5 m numbers the issue
     flagged. This is the test the Issue Review and operator scope both
     require ("RESOLVED WITH A TEST, not left as a note").
   - A regression case at the exact old failure point (level 10, depth ≈ 9 m)
     showing the **old** fixed-level-10 behavior would have failed this
     invariant (`0.9 * sqrt(2)/2 = 0.636 > 0.5`) while the new policy does not
     (asserts the new function picks level 11 there).
   - Boundary tests at the floor/ceiling clamps (very shallow → level 11 cap,
     depth ≥ the level-8 crossover → level 8 floor, not coarser).
   - A monotonicity test: `depthAdaptiveLevel` is non-increasing in level
     (non-decreasing in cell size) as `|depth|` increases, since a level
     regression at some intermediate depth would silently reintroduce the
     original bug at that depth band.

4. **Amend `uma-ADR-0010` D9.** `docs/decisions/0010-geospatial-world-model.md`
   D9's `draft`/`processed` bullet currently reads "born at fine native
   levels" — flatly contradicted by a depth-adaptive `processed`. Add a dated
   amendment (matching the D7 clipping-withdrawal amendment's format already
   in this document) stating:
   - `processed` (not `draft`) tiles are now written at a level chosen by
     `marine_bathymetry_store::depthAdaptiveLevel` from the CUBE-estimated
     depth at import time (`import_bag`, wired in the companion
     `cube_bathymetry` PR — cite its issue number once filed).
   - `draft` stays fixed-level (unchanged) — explicit, not silently implied,
     since a future reader must not assume both layers moved together.
   - The floor/ceiling/storage numbers from step 2.
   - **Seam behaviour, stated explicitly rather than inherited**: cites
     `uma-ADR-0013` D8 by name — `shallowestReliable()` and any least-depth
     query already scan every layer and level regardless of what a
     depth-adaptive `processed` looks like at any single level, so a native
     level boundary inside `processed` carries no safety exposure, exactly
     the argument D9's `reference` bullet already makes for
     `reference`'s native-wins pyramid. Note the one asymmetry worth naming:
     `reference`'s mixed levels come from *disjoint* source regions (S-102
     footprints), while `processed`'s mixed levels come from *depth bands
     within one contiguous survey* — a display consumer sees more frequent
     level transitions across a single pass than `reference` ever produces,
     which is a display/UX note for `camp`, not a store-contract change.
   - Cross-reference this issue (#369) and the enabling #331 machinery.

5. **`marine_bathymetry_store/README.md`**: document
   `depthAdaptiveLevel` alongside the existing `fromCellSize`/S-102
   level-selection description (verify current README content before editing
   — do not assume its structure).

## Files to Change

| File | Change |
|------|--------|
| `marine_bathymetry_store/include/marine_bathymetry_store/depth_adaptive_level.hpp` | New — policy struct + pure function |
| `marine_bathymetry_store/src/depth_adaptive_level.cpp` | New — implementation |
| `marine_bathymetry_store/CMakeLists.txt` | Add source file + new gtest target |
| `marine_bathymetry_store/test/test_depth_adaptive_level.cpp` | New — invariant, regression, boundary, monotonicity tests |
| `marine_bathymetry_store/README.md` | Document the new function; verify structure first |
| `docs/decisions/0010-geospatial-world-model.md` | D9 amendment (dated, in the existing amendment style) |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Capture decisions, not just implementations | D9 amendment lands in this PR, not deferred |
| Test what breaks | Step 3 proves the capture-floor/lattice-diagonal invariant generally, not just at the flagged depth |
| Only what's needed | Store-side mixed-level machinery (D2/D3) is reused, not rebuilt — #331 already generalized it |
| Improve incrementally | Scoped to the policy function + ADR text; `import_bag` wiring is an explicit, separate, sequenced PR |
| A change includes its consequences | Cross-repo consequence (cube_bathymetry wiring) is named and will be filed as a tracked issue, not left implicit |
| Enforcement over documentation | The invariant is a compile-time-checked function + a test, not a policy written down and hoped for |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| `uma-ADR-0010` D9 | Yes | Amended in this PR (step 4) |
| `uma-ADR-0013` D2/D3 | Yes, but already satisfied | `buildDepthOverviewPyramid` already emits per-tile geometric error + coverage manifest generically for any layer directory (#331); no new code needed once `processed` has mixed-level tiles |
| `uma-ADR-0013` D8 | Yes | Stated explicitly in the D9 amendment (step 4), citing D8 by name rather than re-deriving the safety argument |
| ADR-0001 (Adopt ADRs) | Yes | This PR is itself the ADR-amendment vehicle |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `processed` becomes mixed-level | `uma-ADR-0010` D9 | Yes (step 4) |
| Depth-adaptive policy exists in `marine_bathymetry_store` | `cube_bathymetry` `import_bag` must call it | **No — companion issue**, filed in `cube_bathymetry` (sensors_ws repo, no worktree here), referencing this issue and PR. It is the load-bearing follow-up: this PR alone changes no on-disk behavior, since nothing calls `depthAdaptiveLevel` yet. |
| Existing level-10 Shoals/Massabesic `processed` stores stay untouched | Retroactive reprocess | No — tracked separately, gated on #366 (per operator scope decision 3) |
| `capture_distance_scale`/floor duplicated as defaults in two repos | A knowledge-doc note flagging the duplication so a future `cube_bathymetry` parameter change doesn't silently desync | Proposed only — see Documentation & Instruction Impact |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): `docs/decisions/0010-geospatial-world-model.md`
  D9 (amendment, step 4); `marine_bathymetry_store/README.md` (new function,
  step 5).
- **Agent-instruction candidates** (proposals only — operator decides): the
  `capture_distance_scale`/`capture_distance_floor_m` defaults in
  `depth_adaptive_level.hpp` intentionally mirror constants owned by a
  different repo (`cube_bathymetry::Parameters`) with no automated link
  between them — worth a one-line note in
  `.agent/knowledge/` or `.agents/README.md`'s verified-parameter table
  flagging that a future change to CUBE's `capture_distance_scale` should
  prompt a check of this function's defaults. Not applied automatically.

## Open Questions

- [ ] Companion `cube_bathymetry` issue (wiring `import_bag` to call
  `depthAdaptiveLevel` per grid/tile) needs to be filed — this plan names it
  but does not create it, since no `cube_bathymetry` worktree exists in this
  dispatch. Recommend filing immediately after this plan is reviewed, before
  implementation starts, so the PRs can be explicitly sequenced/cross-linked.
- [ ] Confirm the level-8 floor choice (step 2) against any deeper water the
  operator platforms plan to survey before this ships — it is a deliberate,
  named choice ("no coarser than today"), not a computed one, and should be
  revisited if a deep-water campaign is on the near-term roadmap.
- [ ] Whether `depthAdaptiveLevel` should live in `marine_bathymetry_store`
  (this plan's choice, following the `s102/run.cpp` precedent and the
  existing `cube_bathymetry → marine_bathymetry_store` dependency direction)
  versus `marine_autonomy`'s `gggs` module — flagged for review-plan; this
  plan prefers `marine_bathymetry_store` because the policy is a bathymetry
  store-contract decision (ADR-0010 D9), not a generic GGGS utility.

## Estimated Scope

Single PR, in `unh_marine_autonomy` only. Small: one new header/source pair,
one new test file, an ADR amendment, and a README update. The companion
`cube_bathymetry` wiring is a separate, sequenced PR (tracked as an open
question above), consistent with the operator's "writer = offline
`import_bag`/`processed` only, new imports only" scope.
