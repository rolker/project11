# bathymetry_layer

A **Nav2 Costmap 2D** plugin that turns clearance from the bathymetric store
([`marine_bathymetry_store`](../marine_bathymetry_store)) into occupancy cost, so
the planner routes around shoals.

This is the **D1** (prior-only, static) deliverable of issue
[#164](https://github.com/rolker/unh_marine_autonomy/issues/164): the layer reads
the store's persisted layers (`processed/` + `draft/` and the read-only `reference/`
prior — ADR-0010 D8 split the pre-D8 `survey/` into processed/draft) from disk —
transparently, through the store's best-source query overlay, so this layer needs
no per-layer knowledge — and converts depth to cost. It does **not** yet
re-read the store live as the boat surveys — that is the D2 follow-on (see
[Follow-on work](#follow-on-work)).

## How it works

1. On each `updateBounds`, the layer caches the water-surface ellipsoidal height
   from the `map_tide` frame's z-origin (a TF lookup at `TimePointZero`, mirroring
   `s57_layer`'s tide lookup), and (re)loads the store tiles intersecting the
   buffered costmap window (`loadWindow` / `evictOutside`, the #205 windowed-tile
   API) so memory stays bounded under a global costmap.
2. On `updateCosts`, each cell is projected to WGS84 lat/lon (via the `earth` TF
   frame) and resolved to a GGGS cell. **Clearance** is

   ```text
   clearance = z(map_tide) − seafloor_ellipsoidal_height
   ```

   Both terms are WGS84 ellipsoidal heights (up-positive): a seafloor 5 m below
   the ellipsoid has height `-5.0`, so a *shallower* (more hazardous) seafloor has
   a *larger* (less-negative) height and therefore a *smaller* clearance.
3. The cost comes from the **worst-case clearance** — the point-estimate
   clearance minus one standard deviation of vertical uncertainty
   (`clearance − σ`, the shallowest the seafloor plausibly is) — through a
   confidence-gated ramp (same shape as `s57_layer::get_cost_from_grid`,
   ADR-0010 D7):
   - `worst_case_clearance >= maximum_caution_depth` → `FREE_SPACE`
   - between `minimum_depth` and `maximum_caution_depth` → linearly scaled cost
     (shallower = higher cost); this ramp is **trust-independent**
   - `worst_case_clearance < minimum_depth` → keepout **only for trusted data**
     (`σ ≤ confidence_gate`) → `LETHAL_OBSTACLE`; a high-σ (untrusted) cell is
     instead capped at `MAX_NON_OBSTACLE` (top of the caution band) — costed
     go-slow, never hard-forbidden on uncertainty alone

   So keepout is reserved for trusted data; chart-grade (CATZOC) σ that reads
   shallow is costed as caution, not wholesale keepout. A cell whose only data
   carries **σ = ∞ / NaN** (genuinely *unknown* quality) has no usable magnitude
   of uncertainty and stays conservatively `LETHAL_OBSTACLE` (the "data but no
   reliable sample" path below), the same as an over-uncertain cell was treated
   before this rework.

## No-data policy (safety)

Per cell the layer uses a **two-query** pattern (ADR-0002 §D7; review finding M1):

1. `reliableSamples(store, cell, ∞)` — every usable sample covering the cell.
2. If that set is **empty**, `hasAnyData(store, cell)` — quality-blind: "is there
   *any* data here?" — separates the two causes. If `hasAnyData` is `false` → the
   cell is **truly unsurveyed** → by default the
   layer **leaves the master cost untouched** (NO_INFORMATION) so another prior
   (e.g. `s57_layer`) can fill it in. The layer never *writes* NO_INFORMATION.
   Setting **`unsurveyed_is_lethal: true`** flips this: a no-data cell is written
   **`LETHAL_OBSTACLE`** instead. This is for a closed-basin water body whose prior
   covers the whole navigable interior, so the only no-data cells are land — the
   layer then marks the shoreline lethal without a separate land-mask. It is a
   blanket rule (every no-data cell, not just "land") and, via max-cost combine,
   overrides other priors on those cells, so enable it per deployment. It stays
   behind the tide gate (below): no lethal-land is written before a valid tide.
3. If `hasAnyData` is `true` with an empty sample set → the cell is **surveyed but
   unusable** (its only data carries a NaN σ) → conservative **`LETHAL_OBSTACLE`**.
4. Otherwise the sample set is non-empty. `reliableSamples` returns **every** sample
   **without a finite-σ reject-filter** (∞ keeps all finite-σ samples eligible; only
   NaN-σ samples are dropped). Each sample is costed by the worst-case-clearance / confidence-gate
   model above (ADR-0010 D7), and the cell takes the **MAX (most-hazardous) cost
   over all reliable samples** — so a shallower but untrusted sample cannot mask a
   co-located trusted keepout. A high-σ cell is **costed as caution**, not
   rejected. Only a cell whose only data has **σ = ∞ / NaN** (unknown quality) is
   written **`LETHAL_OBSTACLE`** (conservative) — bucketed with no-data, since no
   usable magnitude of uncertainty is known. (The pre-#248
   per-cell staleness gate was **retired** — ADR-0002 Amendment A2.4: the bathy
   store holds a surveyed *static* bottom, not a live sensor feed, so per-cell age
   is not a meaningful costmap hazard.)

### Both queries are region-aware, and what that costs (uma#369)

`processed` is depth-adaptive (uma#369): the store writes level 12-14 tiles where
the water is shallow, so one costmap query cell can cover 16-256 native store
cells and a 0.2-0.5 m rock can sit in any of them. `reliableSamples` **and**
`hasAnyData` therefore read every covered native cell (`uma-ADR-0013` D8) rather
than the one under the query cell's centre. `bestSource` must never be used on
this path — it is a point lookup, and while it was the gate here (before uma#369
round 2) a single no-data native cell under the centre suppressed the
region-aware query entirely and dropped the rock.

The cost of that correctness lands on the **live costmap thread**, and the
fan-out is a config parameter, not a future concern: the query level comes from
`BathymetryStore::fromCellSize(resolution_)`, so a 2 m or 4 m global costmap over
today's uniform level-10 `processed` already fans out 4x or 16x per cell, and a
1 m global over level-14 tiles would be 256 covered cells per costmap cell —
about 2.56 M cell visits (each a map find and a `push_back`) for a single
100x100 tile. `generateTile` checks its time budget only *between* tiles, so one
tile is uninterruptible once started. Point-sampling is the defect this fixed, so
the remedy is a bound or an interruption point;
[#371](https://github.com/rolker/unh_marine_autonomy/issues/371) tracks bounding
or measuring it. `hasAnyData` short-circuits at the first cell holding data, so
the existence gate costs one cell over surveyed ground; only a genuinely empty
region pays its full walk.

### The query cell is smaller than the costmap cell (uma#369, round 3)

`fromCellSize(resolution)` returns the coarsest level whose cells are *at or
finer than* the costmap resolution, so the GGGS query cell is always smaller than
the costmap cell it costs: 0.60 m² of a 1.00 m² cell at 1 m resolution and 43.5
degrees north, and as little as 18% of the cell at a resolution just under a
level boundary. The layer therefore evaluates every query cell the costmap cell's
ground overlaps and keeps the **most hazardous** verdict, rather than the one
under the cell's centre — otherwise 40-82% of every costmap cell went unqueried
no matter how fine the store was.

Two asymmetries are deliberate. The lat/lon box of a map-frame-aligned cell
over-covers under a rotated map, which can only add a hazard. And an *unsurveyed*
query cell inside the costmap cell makes it lethal under `unsurveyed_is_lethal`,
which is stricter than the rule inside a query cell, where partial no-data still
counts as surveyed: within a cell the gaps are routine holes in one fused
surface, across cells they are whole cells of the basin's prior with nothing in
them.

**What the closed-basin flag now costs in channel width.** Because a costmap
cell goes lethal when *any* overlapping query cell is unsurveyed, and a query
cell counts even where it overlaps the cell by a sliver, the lethal boundary
advances past the true unsurveyed edge by one costmap cell plus one query cell:
**1.90 m in latitude and 1.66 m in longitude** at 1 m resolution and 43.5
degrees north, against roughly 0.5 m under the old centre test. That is about
1.2 to 1.4 m more per side, so a surveyed gap between two unsurveyed shoals
loses ~2.5 m of usable width, and nothing is logged when it does. The direction
is the shoal-safe one and the flag exists for basins whose prior fills the
interior, but the magnitude is an operational input, not a rounding error: on a
lake where the boat threads a 4 m gap, this is the difference between transiting
it and refusing it.

### Store residency, the other half of the lever (uma#369, round 3)

A store tile is a 960x960 pair of `double` bands — about **14.7 MB at every
level**; only the ground it covers shrinks. That is ~19 MB per km² of survey at
level 10, ~1.2 GB at level 13 and ~4.9 GB at level 14. `refreshWindow` loads the
whole buffered costmap window with no tile or byte cap, synchronously, and it
runs before and outside the per-cycle budget that guards rendering. A 1 km global
costmap over a shallow level-13/14 store is therefore a multi-gigabyte
synchronous load on the costmap thread: an OOM kill, or a stall the planner sees
as a costmap that is not current. Reachable once the depth-adaptive writer lands
(cube_bathymetry#143); bounding it is an eviction-policy design, tracked in
[#376](https://github.com/rolker/unh_marine_autonomy/issues/376).

The critical distinction: a surveyed-but-noisy cell is a **navigable-with-caution
obstacle**, costed by its worst-case clearance — not an unsurveyed cell (treating
it as unsurveyed would be a safety regression), and no longer wholesale keepout
(pre-#276 that would have made CATZOC-grade chart regions all-LETHAL).

## Parameters

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `enabled` | bool | `true` | Enable the layer. |
| `store_path` | string | `""` | Path to the on-disk store directory. Empty = contributes no cost. A leading `~`/`~/` is expanded to `$HOME`, so one portable value (`~/data/world/depths`) resolves on both the boat (`field` user) and a dev/sim host. |
| `minimum_depth` | double | `1.0` | Clearance (m) below which a cell is `LETHAL_OBSTACLE`. |
| `maximum_caution_depth` | double | `2.5` | Clearance (m) at/above which a cell is `FREE_SPACE`; between `minimum_depth` and this the cost ramps. |
| `confidence_gate` | double | `0.5` | 1-sigma vertical-uncertainty **trust threshold** (m). A sample with `σ ≤ confidence_gate` is *trusted* and may drive a cell to `LETHAL_OBSTACLE` when its worst-case clearance (`clearance − σ`) is below `minimum_depth`. A sample with larger σ is *costed as caution* (never LETHAL on uncertainty alone); only `σ = ∞ / NaN` (unknown quality) stays conservatively LETHAL. **Renamed from `max_uncertainty` (#276) — see the migration note below; this is NOT the old reject-filter.** |
| `unsurveyed_is_lethal` | bool | `false` | When `true`, a truly-unsurveyed (no-data) cell is `LETHAL_OBSTACLE` instead of left untouched. Blanket rule — suits a closed basin whose prior fills the whole interior (no-data = land). Still gated by the tide (no cost before a valid `map_tide`). |
| `map_tide_frame` | string | `map_tide` | Frame whose z-origin is the current water-surface ellipsoidal height. Prefix per platform (`<tf_prefix>/map_tide`). |
| `map_frame` | string | `map` | Ellipsoid-referenced world frame (REP-105 `map`, z=0 at the WGS84 ellipsoid). The water-surface height is read as `map_tide_frame`'s z in this frame. **Must** be set and distinct from both `map_tide_frame` and the costmap's own global frame — otherwise the tide lookup self-references and reads 0, flooding the survey LETHAL (#220). Prefix per platform (`<tf_prefix>/map`). |
| `buffer_fraction` | double | `0.05` | Fractional margin around the costmap window for `loadWindow` / `evictOutside`. |
| `tide_invalidate_threshold` | double | `0.1` | Re-render the cached cost tiles only when the water surface moves more than this (m) from the value they were rendered against. Above realistic sea-surface-estimate jitter (~±0.02 m) so noise doesn't constantly re-render and defeat the cache, while still tracking a real (e.g. reservoir) level change. |
| `update_timeout` | double | `0.5` | Per-cycle wall-clock budget (s) for tile (re)rendering. The window fills over a few cycles (robot-first) instead of blocking the costmap thread. |
| `ready_radius` | double | `200.0` | Radius (m) around the vehicle whose tiles must be rendered before the layer reports `current_`. Decouples readiness from the full window so a large global (which can take minutes to render fully) doesn't stall the planner: the robot-centred core is ready in seconds, the rest fills outward in the background, and the local costmap covers the immediate surroundings meanwhile. Scale-independent — enlarging the global for longer transits doesn't change time-to-ready. |

### Migration: `max_uncertainty` → `confidence_gate` (#276)

`max_uncertainty` was **renamed to `confidence_gate` and its meaning changed** —
this is a **semantic** change, not just a rename:

- **Before (`max_uncertainty`, reject-filter):** any cell whose σ exceeded the
  value was treated as unreliable and written `LETHAL_OBSTACLE`. CATZOC-grade
  chart σ (0.5 m – several m) entering the store would therefore render chart-only
  regions **wholesale keepout**, or force a global gate relaxation that also
  weakened the filter for noisy draft data.
- **After (`confidence_gate`, trust-threshold):** the value is the σ **at/below
  which a sample is trusted enough to justify keepout**. Cost is driven by the
  worst-case clearance (`clearance − σ`); a cell with σ *above* the gate is costed
  as **caution** (never LETHAL on uncertainty alone), and only trusted
  (`σ ≤ confidence_gate`) worst-case-shallow cells become `LETHAL_OBSTACLE`.
  `σ = ∞ / NaN` (unknown quality) stays conservatively LETHAL.

**Action:** rename the key in every `nav2_params` and remove `max_uncertainty`.
The default (`0.5`) is unchanged, so a straight rename preserves survey-grade
behavior, but re-read the meaning before tuning — raising it now widens the
*trusted-keepout* band, it does **not** relax a reject-filter. A config that still
sets `max_uncertainty:` is **ignored** and logs a one-shot deprecation `WARN` at
layer init (the key falls back to the `confidence_gate` default rather than
silently carrying the old value).

## Layer coexistence

`bathymetry_layer` is designed to compose with `s57_layer` (or any other prior)
in a layered Nav2 costmap:

- **Max-cost combine.** The layer only ever *raises* a cell's cost (or fills a
  `NO_INFORMATION` cell); it never lowers an existing higher cost. So when both
  `s57_layer` and `bathymetry_layer` write the same master grid, the higher cost
  per cell wins, regardless of plugin order.
- **Unsurveyed cells are skipped.** Where `bathymetry_layer` has no store data it
  leaves the master cost untouched, so `s57_layer` (the broad chart prior) is the
  sole contributor there. Conversely, where the store has data but the chart does
  not, `bathymetry_layer` is the sole contributor. **Exception:** with
  `unsurveyed_is_lethal: true` this layer writes `LETHAL_OBSTACLE` on no-data
  cells, which (max-cost) overrides `s57_layer` there — so use that mode only when
  this layer should own the "no data = land" verdict for the whole window.
- **Where both have data**, the store's measured clearance typically supersedes
  the chart's coarser depth for accurate shoal detection — and because the combine
  is max-cost, the more conservative (higher-cost) opinion always survives.
- **Recommended plugin order** in `nav2_params`: `chart_layer` (S57 prior) first,
  `bathymetry_layer` (MBES/contour store refinement) second. Order does not change
  the final costmap (max-cost is order-independent); listing bathy second is the
  conventional "refinement on top of chart" reading.

## nav2_params registration (cross-repo follow-on)

Bizzy's (and the echoboats') `nav2_params` live in **separate platform repos** —
`unh_echoboats_project11/bizzyboat_project11` and
`seafloor_echoboat_project11/echoboat_project11`, both under `platforms_ws`, not in
this (`unh_marine_autonomy`) repo. Wiring the plugin into those configs is a
cross-repo change and is therefore tracked as a **follow-on issue**, not landed
here (see [Follow-on work](#follow-on-work)). The registration snippet to add to
the `local_costmap` and `global_costmap` plugin lists:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["chart_layer", "bathymetry_layer", "inflation_layer"]
      bathymetry_layer:
        plugin: "bathymetry_layer::BathymetryLayer"
        enabled: True
        store_path: "/path/to/bathy_store"   # override per platform
        minimum_depth: 1.0
        maximum_caution_depth: 2.5
        confidence_gate: 0.5          # σ trust threshold (was max_uncertainty, #276)
        unsurveyed_is_lethal: False   # True for a closed basin (no-data = land)
        map_tide_frame: <tf_prefix>/map_tide
        buffer_fraction: 0.05
```

Add the same block under `global_costmap` (the layer is usable on both — its
windowed tile residency keeps memory bounded even on a large global costmap).

## Follow-on work

- **D2 — live survey tile reload.** After [#189](https://github.com/rolker/unh_marine_autonomy/issues/189)
  (atomic tile writes) merges, add tile-change detection on the `survey/` layer so
  the costmap refines where the boat has surveyed. File as a new issue:
  "Depends on #164 (D1) and #189 (atomic writes)".
- **nav2_params registration.** Add the snippet above to the bizzy / echoboat
  `nav2_params` in `platforms_ws` (cross-repo — own issue on the platform repo).
- **Sim acceptance run.** costmap-reflects-prior + shoals-LETHAL + planner-routes-
  around validation. Gated on [#163](https://github.com/rolker/unh_marine_autonomy/issues/163)
  A2 (contour importer) and the sim MBES / harness work; tracked against those
  issues, not this one.
