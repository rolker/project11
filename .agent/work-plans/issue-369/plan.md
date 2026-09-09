# Plan: Survey tiles are written at one fixed level regardless of depth

## Issue

https://github.com/rolker/unh_marine_autonomy/issues/369

**Revision history**: authored 2026-09-09 (`38b720e`); revised 2026-09-09 after
the `## Plan Review` **changes-requested** verdict and the operator's decisions
at the run-issue plan checkpoint (see *Operator-pinned scope* below). The
revision recomputes the depth→level mapping and the storage estimate against
`gggs::Level::fromCellSize`'s actual at-or-finer contract, drops the 0.5 m cell
floor, clamps the fine end at level 14, rewords the ADR amendment as a decided
policy with the writer pending, and retargets the tests.

## Context

`~/data/world/depths/processed` and `reference` write every tile at one fixed
GGGS level. `chart` already varies resolution with source compilation scale.
CUBE's own capture radius (`cube_bathymetry/include/cube_bathymetry/parameters.h:205`,
`capture_distance_scale = 0.05`, used in `node.cpp:154` as
`max(0.05 * |depth|, 0.5 m)`) is depth-adaptive; the lattice it writes onto is
not — level 10 (~0.906 m cells) covers the whole 1–15 m+ Isles of Shoals depth
range.

**Operator-pinned scope** (2026-09-09 run-issue checkpoints 1 and 2; not
re-opened here):

1. **Driver = depth.** The requested cell size is `0.05 * |depth_m|` — CUBE's
   `capture_distance_scale` applied to depth. **The 0.5 m floor is dropped.**
   Rationale, in the operator's framing: CUBE's 0.5 m floor is a minimum
   *acceptance distance*, not a statement about achievable resolution, and
   inheriting it as a *resolution* floor flattens the policy to a single level
   across the entire surveyed range (the defect the Plan Review found in the
   first draft).
2. **Fine clamp at level 14** (~0.057 m cells).
3. **Writer = offline `import_bag` / `processed` only.** `draft` and the live
   node stay fixed-level. No retiling in this PR.
4. **New imports only.** No reprocessing of the existing level-10 Shoals or
   Massabesic `processed` stores; that is a follow-up gated on
   [#366](https://github.com/rolker/unh_marine_autonomy/issues/366) (import
   ledger).
5. **Coarse clamp stays at level 8** — the number the first draft chose. Its
   *rationale* has been corrected below (§ "The coarse clamp"): it does **not**
   mean "never coarser than today", and the first draft's claim that it did was
   arithmetically wrong. Flagged for the operator at the implementation review
   checkpoint.

**Store-side machinery already exists and needs no new work.** `BathymetryStore`
is explicitly multi-level and level-agnostic today (`bathymetry_store.hpp:84-85`:
"tiles at heterogeneous GGGS levels coexist within a layer",
`importTiles`/`set` accept any level, the constructor's `gggs_level` is only
the `cellIndex(lat,lon)` convenience default). `buildDepthOverviewPyramid`
(`marine_bathymetry_store/overview_pyramid.hpp:45-51`, landed via
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

**The writer is a separate, larger change in another repo.** It is filed as
[cube_bathymetry#143](https://github.com/rolker/cube_bathymetry/issues/143) —
"import_bag writes one resolution per run — depth-adaptive store levels
(uma#369) need the estimation grid decoupled from the store tiling". It is
**not "wiring"**: `import_bag_main.cpp:1030` constructs a single
`cube::GeoMapSheet geo_map_sheet(resolution, iho_order)` for the entire run, and
`:1130-1133` pins `accumulator_config.cell_size_m` to
`geo_map_sheet.nominalCellSizeMeters()` — CUBE's *estimation* grid and the
*store* tiling are the same resolution, one per run, by construction. Making
the store level vary with depth therefore requires either several sheets per
run or an accumulator that decouples estimation resolution from store cell
size. That is a re-architecture of the import pipeline, and this plan states so
rather than implying a one-line call site.

Consequently **this PR changes no on-disk behaviour by itself.** It ships the
store-contract policy (a pure function plus its tests) and the ADR amendment
that records the decision; cube_bathymetry#143 is the load-bearing sequel.

## Approach

### 1. Add `marine_bathymetry_store::depthAdaptiveLevel(depth_m)`

New `include/marine_bathymetry_store/depth_adaptive_level.hpp` +
`src/depth_adaptive_level.cpp`, added to the library sources
(`CMakeLists.txt` alongside `overview_pyramid.cpp`). Pure function, no I/O:

```cpp
struct DepthAdaptiveLevelPolicy {
  double capture_distance_scale = 0.05;  // mirrors cube_bathymetry::Parameters
  uint8_t finest_level = 14;             // operator-pinned clamp (~0.057 m)
  uint8_t coarsest_level = 8;            // ~3.62 m; see "The coarse clamp"
};
gggs::Level depthAdaptiveLevel(
  double depth_m, const DepthAdaptiveLevelPolicy & policy = {});
```

Body: `cell_size = policy.capture_distance_scale * |depth_m|`, then
`gggs::Level::fromCellSize(cell_size)`, clamped into
`[coarsest_level, finest_level]`.

Edge cases, decided rather than left to `fromCellSize`'s float path:

- **Non-finite depth** (NaN/inf) → `std::invalid_argument`. A silently clamped
  NaN would write a whole survey at the fine clamp; failing loud is the
  workspace's standard.
- **`depth_m == 0`** (or any depth whose requested cell size is ≤ 0) →
  `finest_level`. `fromCellSize(0)` would evaluate `log2(inf)` and cast an
  infinite double to `int` (UB), so this is guarded before the call, not after.
- **Negative depth** is accepted and treated by magnitude (`|depth_m|`), matching
  `node.cpp:154`'s `std::abs`, since the sign convention differs between the
  CUBE node and store consumers.
- **`finest_level < coarsest_level`** in a caller-supplied policy →
  `std::invalid_argument` (an inverted clamp otherwise silently returns the
  wrong end).

The doc comment cites `cube_bathymetry::Parameters::capture_distance_scale` as
the source of truth the default mirrors, and states that no automated link
exists between the two repos' constants (a `.agent/`-knowledge candidate — see
Documentation & Instruction Impact).

**Decision unit (issue design question 2), answered rather than deferred.** The
scalar signature means *one depth decides one level for one decision unit*. The
decided unit is a **store tile**, and the contract is that the caller passes the
**shallowest** `|depth|` over that unit: the shallowest sounding is the one with
the tightest capture radius, so sizing the lattice from it is what keeps every
sounding in the unit within its own capture distance of a node. This is stated
in the header and in the ADR amendment. **The signature may change when
cube_bathymetry#143 lands** — an accumulator that decouples estimation from
store tiling may want a depth *range* per unit rather than a scalar — and the
ADR amendment says so, so a future reader does not read the scalar form as
settled.

### 2. The level ladder, computed against `fromCellSize`'s real contract

`gggs::Level::fromCellSize(c)` returns the level whose cells are **at or finer
than** `c` (`level.h:64-70`: `raw = ceil(log2(level_0_grid_size / (c * 960)))`;
`ceil` rounds toward the finer level). The first draft assumed the opposite,
which is what made its mapping and storage estimate wrong.

Nominal cell size is `level_0_grid_size / (2^L * 960)` with
`level_0_grid_size = 2π·a·8/360` (`core.h:70,82`):

| Level | Cell size | Grid (tile) extent | Applies when |
|---|---|---|---|
| 8 (coarse clamp) | 3.624 m | 3478.7 m | depth ≥ 72.47 m |
| 9 | 1.812 m | 1739.4 m | 36.24 m ≤ depth < 72.47 m |
| 10 (today's fixed level) | 0.906 m | 869.7 m | 18.12 m ≤ depth < 36.24 m |
| 11 | 0.453 m | 434.8 m | 9.06 m ≤ depth < 18.12 m |
| 12 | 0.227 m | 217.4 m | 4.53 m ≤ depth < 9.06 m |
| 13 | 0.113 m | 108.7 m | 2.26 m ≤ depth < 4.53 m |
| 14 (fine clamp) | 0.057 m | 54.4 m | depth < 2.26 m |

Each boundary is `Level(L).cellSize() / 0.05` exactly; the tests derive them
that way rather than hard-coding the rounded figures above.

With the 0.5 m floor dropped, the policy is genuinely adaptive **inside** the
1–15 m Isles of Shoals range: it crosses three transitions there (9.06 m,
4.53 m, 2.26 m) instead of returning one constant level, which was the Plan
Review's central objection.

**The coarse clamp.** Level 8 is kept as the operator pinned it, but its
first-draft rationale — "a floor one level coarser than today's level 10, so
`processed` is guaranteed no coarser than today" — is **wrong twice**: level 8
is *two* levels coarser than 10, and the clamp is not what governs the coarse
end anyway. Under the ladder above the policy already returns level 9 (1.81 m,
coarser than today) for 36–72 m water, before the clamp ever binds. So the
honest statement is: **level 8 bounds the coarse end at ~3.6 m cells for water
deeper than ~72 m**, and the policy *can* write coarser than today's level 10 in
water deeper than ~36 m — deeper than either operator platform surveys today
(Shoals tops out around 15 m+; Massabesic is shallower), which is why this is
acceptable rather than a regression in practice. If "never coarser than today"
is the actual requirement, the coarse clamp must be **10**, not 8; that is a
one-constant change and is called out for the operator at the implementation
review checkpoint rather than decided here.

**Storage estimate, stated honestly.** Level steps are a **4× cell-count (and
tile-count) increase per step for the same ground area**, so relative to today's
uniform level 10 the multiplier is `4^(L-10)`:

| Depth band | Level | Cells/tiles per unit area vs today |
|---|---|---|
| ≥ 72.47 m | 8 (clamp) | 1/16× |
| 36.24–72.47 m | 9 | 1/4× |
| 18.12–36.24 m | 10 | 1× (unchanged) |
| 9.06–18.12 m | 11 | 4× |
| 4.53–9.06 m | 12 | 16× |
| 2.26–4.53 m | 13 | 64× |
| < 2.26 m | 14 (clamp) | **256×** |

The level-14 clamp is four steps finer than today, i.e. 256× — that must be
said plainly, not softened. The whole-survey figure depends on the survey's
depth-area histogram, which has **not** been computed here (no such histogram
was derived for this plan, and inventing one would be fabrication); it is
bounded below by 1/16× (all water ≥ 72.5 m, at the level-8 coarse clamp) and
above by 256× (all water < 2.26 m).
For a Shoals-like box that is mostly 9–18 m with shallow fringes, the bulk lands
at 4× with small fractions at 16–256×.

Two secondary costs, both real and both consequences of the tile extent column
above rather than of the cell count:

- **Partial-tile overhead.** Tiles are dense 960×960 rasters regardless of how
  much of the tile the survey actually touches. A level-14 tile spans 54.4 m,
  so a single survey line crossing shallow water produces many mostly-empty
  tiles, each costing a full (compressed) raster.
- **Resident-tile pressure in the importer.** `import_bag`'s
  `accumulator_config.max_resident_tiles` bounds how many tiles are held at
  once; a 16–256× tile count at the same footprint changes that budget's
  meaning. This belongs to cube_bathymetry#143 and is named there.

These numbers are recorded in the ADR amendment (step 4) so they are not
re-derived — or re-guessed — later.

### 3. Tests — target the transitions and the clamps

New `marine_bathymetry_store/test/test_depth_adaptive_level.cpp` (registered in
`CMakeLists.txt` next to `test_depth_overview`). The first draft's headline
assertion (`cell·√2/2 < capture radius`) is dropped as the load-bearing test:
given `fromCellSize`'s at-or-finer contract it is near-tautological and tests
the GGGS API rather than this policy. What the suite asserts instead:

- **Transition depths.** For every level `L` in 9…14, the boundary
  `d_L = Level(L).cellSize() / 0.05` derived from the API: `d_L` (and a hair
  above) returns `L`, a hair below returns `L+1`. This is the mapping table
  above, pinned.
- **A few literal expectations** so the suite is not purely self-referential —
  e.g. 12 m → level 11, 6 m → level 12, 3 m → level 13, 25 m → level 10.
- **Both clamps.** 100 m and 1000 m → level 8 (not 7 or coarser); 1.0 m, 0.2 m
  and 0.0 m → level 14 (not 15 or finer). Unlike the first draft's unreachable
  `finest_level = 11`, **both clamps genuinely bind** under the pinned
  constants, so these exercise live bounds.
- **Custom-policy coverage.** A policy with a different
  `capture_distance_scale` and a different clamp pair, to prove the constants
  are parameters and not hard-coded; and an inverted clamp
  (`finest < coarsest`) throwing `std::invalid_argument`.
- **Monotonicity.** Over a dense depth sweep (0.05 m → 200 m), the returned
  level is non-increasing as `|depth|` increases — a level regression inside a
  band would silently reintroduce the original bug at that depth.
- **Sign symmetry.** `depthAdaptiveLevel(-d) == depthAdaptiveLevel(d)`.
- **Non-finite depth** (NaN, ±inf) throws `std::invalid_argument`.
- **Regression marker for the motivating defect.** At depth 9 m the old fixed
  level 10 gives 0.906 m cells against a 0.45 m capture radius; assert the
  policy returns a level strictly finer than 10 there, and that its cell size is
  ≤ the capture radius `0.05·d`, for a depth sweep across the surveyed range.
  (Kept as a guard, not as the proof of the policy.)

### 4. Amend `uma-ADR-0010` D9

`docs/decisions/0010-geospatial-world-model.md` D9's `draft`/`processed` bullet
currently reads "born at fine native levels" — a single fine native level per
layer, which a depth-adaptive `processed` contradicts. Add a dated amendment in
the same style as the D7 clipping-withdrawal amendment already in this document,
stating:

- **The decided policy, with the writer explicitly pending.** `processed` tiles
  are *to be* written at a level chosen by
  `marine_bathymetry_store::depthAdaptiveLevel` from the CUBE-estimated depth of
  the tile; the function and its contract land with #369, and **no writer calls
  it yet** — `import_bag` is tracked by
  [cube_bathymetry#143](https://github.com/rolker/cube_bathymetry/issues/143),
  whose single-`GeoMapSheet`-per-run constraint is described there. The
  amendment must never read as as-built behaviour (Documentation Accuracy).
- **`draft` stays fixed-level**, explicitly — a future reader must not assume
  both layers moved together.
- **The ladder and the storage multipliers** from step 2, including the 256×
  figure at the level-14 clamp and the note that the coarse clamp permits
  coarser-than-today tiles below ~36 m.
- **The decision unit**: one level per store tile, chosen from the shallowest
  depth in the tile; the scalar signature may change when the writer lands.
- **Seam behaviour, stated rather than inherited**: cite `uma-ADR-0013` D8 by
  name — `shallowestReliable()` and any least-depth query scan every layer and
  level regardless of what is drawn, so a native level boundary inside
  `processed` carries no safety exposure, the same argument D9 already makes for
  `reference`'s native-wins pyramid. One asymmetry is worth naming:
  `reference`'s mixed levels come from *disjoint* source regions (S-102
  footprints), while `processed`'s come from *depth bands within one contiguous
  survey* — a display consumer sees far more frequent level transitions across a
  single pass than `reference` ever produces. That is a display/UX note for
  `camp`, not a store-contract change.
- Cross-reference #369 and the enabling #331 machinery.

### 5. `marine_bathymetry_store/README.md`

Document `depthAdaptiveLevel` alongside the existing level-selection material
(the S-102 `fromCellSize` path), including the ladder, the "shallowest depth in
the tile" contract, and the fact that no writer calls it yet. Verify the
README's current structure before editing rather than assuming it.

## Files to Change

| File | Change |
|------|--------|
| `marine_bathymetry_store/include/marine_bathymetry_store/depth_adaptive_level.hpp` | New — policy struct + pure function |
| `marine_bathymetry_store/src/depth_adaptive_level.cpp` | New — implementation |
| `marine_bathymetry_store/CMakeLists.txt` | Add source file + new gtest target |
| `marine_bathymetry_store/test/test_depth_adaptive_level.cpp` | New — transitions, clamps, custom policy, monotonicity, edge cases |
| `marine_bathymetry_store/README.md` | Document the new function; verify structure first |
| `docs/decisions/0010-geospatial-world-model.md` | D9 amendment (dated, in the existing amendment style) |

**Added after review round 1** (the pre-push `review-code` pass found that two
store mechanisms silently assumed `processed` was single-level, so the
amendment's safety argument did not hold; the operator directed that both be
fixed in code here rather than deferred):

| File | Change |
|------|--------|
| `marine_bathymetry_store/src/query.cpp` | `shallowestReliable` / `reliableSamples` read **every** native cell a query cell covers at a finer level, not one sample from its centre (uma-ADR-0013 D8) |
| `marine_bathymetry_store/include/marine_bathymetry_store/query.hpp` | Region-aware contract; `bestSource` documented as the point/display query |
| `marine_bathymetry_store/src/cell_geometry.hpp` | New — internal (not installed) cell/grid extent + iterator-inset helpers, shared by the query walk and the draft clear |
| `marine_bathymetry_store/src/bathymetry_store.cpp` | `clearOverlappedDraft` walks every level `draft` holds; coarse draft cells clear only under full supersession and are otherwise counted |
| `marine_bathymetry_store/include/marine_bathymetry_store/bathymetry_store.hpp` | Level-aware contract; new `DraftClearResult::coarse_draft_cells_retained` |
| `marine_bathymetry_store/test/test_query.cpp` | Five region-coverage tests (four verified to fail against the previous walk) |
| `marine_bathymetry_store/test/test_store.cpp` | Three cross-level draft-clear tests (two verified to fail against the previous clear) |
| `marine_bathymetry_store/src/overview_pyramid.cpp` | Comment: whole-tile native-wins suppression is a constraint on a mixed-level writer |
| `marine_autonomy/include/marine_autonomy/gggs/level.h` | Doc fix: `fromCellSize` returns at-or-finer (its `@return` said the opposite) |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Capture decisions, not just implementations | D9 amendment lands in this PR, phrased as a decision with a pending writer — not as as-built behaviour |
| Test what breaks | Tests pin the transition depths and both clamps (the policy), not a GGGS-API tautology |
| Only what's needed | Store-side mixed-level machinery (D2/D3) is reused, not rebuilt — #331 already generalized it |
| Improve incrementally | Scoped to the policy function + ADR text; the `import_bag` re-architecture is cube_bathymetry#143 |
| A change includes its consequences | The cross-repo consequence is filed, sized honestly (re-architecture, not wiring), and cross-linked |
| Enforcement over documentation | The clamp, the edge cases and the ladder are a function plus tests, not prose |
| Documentation accuracy | Every number here is derived from `level.h`/`core.h`; the storage figure is bounded, and the un-computed depth histogram is declared un-computed |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| `uma-ADR-0010` D9 | Yes | Amended in this PR (step 4), as a decided policy with the writer pending |
| `uma-ADR-0013` D2/D3 | Yes, but already satisfied | `buildDepthOverviewPyramid` already emits per-tile geometric error + coverage manifest generically for any layer directory (#331); no new code needed once `processed` has mixed-level tiles |
| `uma-ADR-0013` D8 | Yes | **Revised after review round 1.** Citing D8 was not enough: the query point-sampled one cell per level, and the D8 anti-clobber (`clearOverlappedDraft`) was level-keyed and cleared nothing across levels. Both are fixed in this PR, and the amendment now records them as what a mixed-level `processed` required |
| ADR-0001 (Adopt ADRs) | Yes | This PR is itself the ADR-amendment vehicle |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `processed` becomes mixed-level | `uma-ADR-0010` D9 | Yes (step 4) |
| `processed` becomes mixed-level | `shallowestReliable` / `reliableSamples` must read the finest data for the **region** (D8) | Yes — added in round 1 (was not in the original plan) |
| `processed` becomes mixed-level | `clearOverlappedDraft` must clear across levels | Yes — added in round 1 (was not in the original plan) |
| `processed` becomes mixed-level | The overview pyramid's whole-tile native-wins suppression becomes a writer obligation (no two native levels over one parent) | Recorded in the ADR and at the suppression site; enforcement belongs to cube_bathymetry#143 |
| A depth-adaptive policy exists in `marine_bathymetry_store` | `cube_bathymetry`'s `import_bag` must be re-architected to call it — one `GeoMapSheet` per run today (`import_bag_main.cpp:1030`, `:1130-1133`) ties the estimation grid to the store tiling | **No — filed as [cube_bathymetry#143](https://github.com/rolker/cube_bathymetry/issues/143)**. Load-bearing: this PR alone changes no on-disk behaviour |
| Tile counts rise 4×–256× in shallow water | `import_bag`'s `max_resident_tiles` budget; store disk planning | Named here; belongs to cube_bathymetry#143 |
| Existing level-10 Shoals/Massabesic `processed` stores stay untouched | Retroactive reprocess | No — tracked separately, gated on #366 (operator scope decision 4) |
| `capture_distance_scale` duplicated as a default in two repos | A knowledge-doc note so a future `cube_bathymetry` parameter change doesn't silently desync | Proposed only — see Documentation & Instruction Impact |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): `docs/decisions/0010-geospatial-world-model.md`
  D9 (amendment, step 4); `marine_bathymetry_store/README.md` (new function,
  step 5).
- **`docs/sonar_ecosystem.md` — checked, not stale.** The Issue Review flagged it
  as a candidate ("one native level per survey layer"). Its "Store — bathy" row
  already describes the store as "multi-level (D3/D4)" and makes no
  single-native-level claim, so nothing there contradicts a depth-adaptive
  `processed`. No edit needed; recorded here so the check is visible rather than
  a silent omission.
- **Agent-instruction candidates** (proposals only — operator decides): the
  `capture_distance_scale` default in `depth_adaptive_level.hpp` intentionally
  mirrors a constant owned by a different repo (`cube_bathymetry::Parameters`)
  with no automated link between them — worth a one-line note in
  `.agent/knowledge/` or `.agents/README.md`'s verified-parameter table so a
  future change to CUBE's `capture_distance_scale` prompts a check of this
  function's default. Not applied automatically.

## Open Questions

- [x] Companion `cube_bathymetry` issue — **filed**:
  [cube_bathymetry#143](https://github.com/rolker/cube_bathymetry/issues/143).
- [x] Design question 2 (unit of the decision) — **answered**: one level per
  store tile, from the shallowest depth in the tile; signature may change when
  the writer lands (step 1, recorded in the ADR amendment).
- [ ] **Coarse clamp = 8 or 10?** Kept at 8 per the operator's pin, but the
  "never coarser than today" rationale that accompanied it does not hold (see
  § "The coarse clamp"). If that guarantee is wanted, the clamp is 10. One
  constant; surfaced at the implementation review checkpoint.
- [ ] Whether `depthAdaptiveLevel` should live in `marine_bathymetry_store`
  (this plan's choice, following the `s102/run.cpp` precedent and the existing
  `cube_bathymetry → marine_bathymetry_store` dependency direction) versus
  `marine_autonomy`'s `gggs` module. This plan prefers
  `marine_bathymetry_store` because the policy is a bathymetry store-contract
  decision (ADR-0010 D9), not a generic GGGS utility. Not raised as a concern by
  the Plan Review.

## Estimated Scope

Single PR, in `unh_marine_autonomy` only. Originally small: one new
header/source pair, one new test file, an ADR amendment, and a README update.
Review round 1 added the two store-side consequences above (the region-aware
safety query and the level-aware draft clear) with their tests — roughly a
doubling, and the reason the amendment's safety claim is now true rather than
asserted. The `cube_bathymetry`
writer is a separate and substantially larger change
(cube_bathymetry#143), consistent with the operator's "writer = offline
`import_bag`/`processed` only, new imports only" scope.
