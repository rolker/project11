# marine_bathymetry_store

Persistent multi-source bathymetric data store built on the GGGS spatial index.
This package is **Phase 1 (the store core)** of the larger effort tracked by
[unh_marine_autonomy#86](https://github.com/rolker/unh_marine_autonomy/issues/86)
and designed in
[ADR-0002](../docs/decisions/0002-bathymetric-data-store.md).

## What this phase provides

- An in-memory, GGGS-tiled store with **priority source layers**.
- **Queries** — best-available, shallowest-reliable, and the region form of
  best-available.
- **Per-tile GeoTIFF persistence** (incremental save / reload), including the
  atomic wholesale swap of the `chart/` layer.
- A **GeoTIFF importer** (`geotiff_import.hpp`) and its `import_geotiff` CLI, for
  loading a depth/uncertainty raster into a layer's fused surface.

It deliberately does **not** include the CUBE / bathy-BAG importers,
ROS services/topics, a costmap plugin, or robot↔operator distribution — those
are later phases of #86. The store core is a plain C++ library with no ROS-node
or consumer-specific logic.

## Concepts

### Source layers (`bathy_cell.hpp`)

`SourceLayer` is an ordered enum in descending query priority — `Processed = 0`
(the authoritative off-boat `import_bag` re-run), `Draft = 1` (live on-boat CUBE),
`Reference = 2` (a prior surface imported before the survey: a chart-derived
contour prior, an external processed grid), `Chart = 3` (official navigation
products, S57 exports). The numeric value is the priority rank, and
`source_layers_by_priority` is the array every query and I/O path iterates.

The pre-#248 `chart`/`draft`/`processed` taxonomy first collapsed to
`survey` + `reference` (ADR-0002 Amendment A2.1). ADR-0010 **D8** then re-split
`survey` back into **`Processed`** and **`Draft`** (Amendment A3): the live and
offline CUBE paths produce different surfaces, so fusing them let a gappy live pass
clobber an authoritative re-run under a multi-day campaign loop. `Chart` was
reintroduced by #275 as a *distinct* layer for ADR-0010 D3/D7 — not the pre-#248
`chart`, and not a generalization of `reference`: it holds official products
regenerated wholesale from S57, at the lowest priority under D4's placeholder
ordering (`processed > draft > reference > chart`).

A cell's source is **implied by which layer holds it**: the store keeps one tile
map per layer, so source is the map, not a per-cell field. Priority is a
**non-destructive query-time overlay** — live `draft` ingest never clobbers the
read-only `reference` prior (ADR-0002 §D3), and never outranks a `processed`
re-run cell. **Anti-clobber (D8):** a `processed` import clears overlapped `draft`
cells **cell-wise** (only where it has data, so the re-run's gated-drop holes leave
draft intact) and **across GGGS levels** — `processed` is depth-adaptive and
mixed-level (uma#369) while `draft` stays fixed-level, so the clear walks every
level `draft` holds rather than keying on the processed tile's own level. A
`draft` cell coarser than the processed tile clears only where that tile fully
supersedes it; the ones it only partly covers are kept (the shoal-safe direction)
and reported in `DraftClearResult::coarse_draft_cells_retained`.

#### Write gates

Two layers are read-only on a normal store, each with a construction flag the
importer opts into (`BathymetryStore(level, reference_writable,
chart_staging_writable)`, mirrored by `fromCellSize`):

| Layer | Flag | Writable how |
|---|---|---|
| `Processed` | — | always writable (offline `import_bag` re-run) |
| `Draft` | — | always writable (live CUBE ingest) |
| `Reference` | `reference_writable=true` | cell-wise `set()` / `importTiles()` on an importer store |
| `Chart` | `chart_staging_writable=true` | **staging only** — build the layer in a staging store, `save()` it, then swap it in with `replaceChartLayer` |

Both `set()` and `importTiles()` throw `std::logic_error` when the gate is shut.
The gates block *writes*, not *reads*: `load()` populates `Chart` on any store, so
a `chart/` layer on disk participates in `bestSource` / `shallowestReliable` —
which feed navigation. Until the cost-model rework
([#276](https://github.com/rolker/unh_marine_autonomy/issues/276)) lands there is
**no mechanical block** on that, so the standing precondition is that no deployed
store carries a `chart/` layer (tracked in #276, not here — see the note in
`query.cpp:93`).

### Per-cell record (`BathyCell`)

`depth` (ellipsoidal height, WGS84, metres — up-positive, so a *shallower*
seafloor is a *greater* value) and `uncertainty` (1-σ, metres), both `double` — a
single 2-band `Float64` value tile. The pre-#248 per-cell `timestamp` and
`source_index` were dropped (ADR-0002 Amendment A2.2): the store is a regenerable
cache over raw bags, the per-cell time band's only reader (the costmap staleness
gate) was retired, and per-cell source provenance is a constant for a single
platform (coarse provenance moved to the store-level `registry.json`). A
fully-allocated 960×960 tile is ≈14 MB; tiles are allocated lazily, only when first
written.

### Queries (`query.hpp`)

- `bestSource(store, cell)` — the highest-priority layer with data. Walks
  `source_layers_by_priority` in order (`Processed` → `Draft` → `Reference` →
  `Chart`) and returns the first layer holding a value for the cell. Where a
  layer is finer than the query cell this **point-resolves** the native cell at
  the query cell's centre: one representative value, for display and
  best-available lookups — not a safety query.
- `shallowestReliable(store, cell, max_uncertainty)` — the shallowest (greatest
  ellipsoidal height) value among layers whose uncertainty is within tolerance.
  For navigation-safety use. Reads the finest data **for the region**
  (uma-ADR-0013 D8): a level finer than the query cell covers it with many native
  cells (16 one level finer, 256 four levels finer — a level-14 depth-adaptive
  `processed` tile under a level-10 query, uma#369), and every one of them is
  read, shoalest reliable value winning.
- `reliableSamples(store, cell, max_uncertainty)` — every passing sample rather
  than the shoalest pick, for callers that cost each and take the most hazardous
  (ADR-0010 §D7). Region-aware in the same way, so one query cell over a fine
  `processed` tile can return one sample per covered native cell.
- `forEachCellBestSource(store, min, max, visitor)` — the region form, over a
  geographic box.

Every query returns `std::optional`: **`std::nullopt` means *unknown*** — no
(reliable) layer covers the cell. A safety-conscious consumer must treat unknown
as not-safe, never as deep water (ADR-0002 §D7).

### Persistence (`tile_io.hpp`)

Each dirty tile is written as a single 2-band `Float64` GeoTIFF (depth,
uncertainty) at `<dir>/<layer>/<level>_<row>_<col>.tif`, WGS84-georeferenced from
its GGGS grid corners (the pre-#248 `_time.tif` / `_source.tif` companions were
dropped — ADR-0002 Amendment A2.2). `save()` writes only dirty tiles
(incremental); `load()` rebuilds the store, recovering each tile's `GridIndex` from
the file geotransform and rejecting tiles written at a different GGGS level.
Coarse store-level provenance (`StoreMetadata{platform, sensor, survey, date}`) is
persisted once at the store root as `registry.json` (`registry.hpp`). The layer
subdirectory names are `processed/`, `draft/`, `reference/`, `chart/`
(`layerDirName`). A legacy on-disk `survey/` layer **auto-migrates** to `processed/`
on `load()`/`loadWindow()` (single atomic rename; a store holding both `survey/` and
`processed/` is refused — ADR-0010 D8, Amendment A3).

#### Wholesale chart regeneration (`replaceChartLayer`)

`replaceChartLayer(staged_chart_dir, store_dir)` swaps a freshly built chart layer
into a store — ADR-0010 D7's *wholesale regeneration, never a merge*. It always
targets `chart/` and takes no `SourceLayer` argument, so no caller can
wholesale-replace `processed/`, `draft/`, or `reference/`. The producer builds the layer in a
staging store (`chart_staging_writable=true`), `save()`s it, and hands the
resulting `<staged>/chart` directory to this call.

The `import_geotiff` CLI produces and commits a staged chart layer with a
two-phase `--stage` / `--commit` grammar (#289). `--stage` builds the
write-gated chart layer into a staging directory (constructing the store with
`chart_staging_writable=true`); one `--commit` performs the atomic
`replaceChartLayer` swap into the live store:

```bash
# Build the chart layer: one or more --stage calls into the SAME (fresh) dir.
import_geotiff --stage /path/to/staged chart cellA.tif --level 5
import_geotiff --stage /path/to/staged chart cellB.tif --level 7
# Atomically swap the staged chart layer into the live store.
import_geotiff --commit /path/to/staged /path/to/store
```

- **Offline/maintenance only.** `--commit` swaps the whole layer and must run
  only while navigation is **not** consuming the store (ADR-0010 D7 nav-down
  precondition). This CLI does **not** enforce a nav-liveness check — that
  enforced check is the cron updater's job
  ([rolker/s57_tools#28](https://github.com/rolker/s57_tools/issues/28)), which
  is where D7 places the updater obligation.
- **Same filesystem.** `<staged_dir>` and `<store_dir>` must be on the same
  filesystem: `replaceChartLayer`'s atomic `rename(2)` rejects a cross-device
  staged dir before touching the live layer.
- **Fresh dir per cycle.** `--stage` accumulates at *grid-tile* granularity:
  re-staging a GeoTIFF that lands on an already-staged GGGS grid overwrites that
  grid's tile wholesale, but tiles for grids you do **not** re-stage this cycle
  persist untouched (it never deletes). That per-grid persistence is the
  stale-carry risk — a reused non-empty dir keeps prior-cycle tiles for any grid
  the new cycle happens not to cover, and `--commit` then swaps them into the
  live store as if current. Point repeated `--stage` calls at one dir to
  accumulate cells *within* a regeneration cycle, but start each new cycle from a
  fresh/empty staged dir (the CLI warns when the staged dir is non-empty).
- **Import ≠ costmap-active.** Importing chart data into the store does **not**
  activate it in the costmap; no deployed store may carry a `chart/` layer until
  the cost-model rework
  ([#276](https://github.com/rolker/unh_marine_autonomy/issues/276)) lands.

`draft`/`processed`/`reference` imports keep the single-step form
`import_geotiff <store_dir> <layer> <geotiff>`; `chart` is stage/commit only (a
chart layer is never written into a live store incrementally).

**Tidal-datum-referenced sources**
([#315](https://github.com/rolker/unh_marine_autonomy/issues/315)): a grid
referenced to MLLW imports with
`--source-datum mllw --geoid <grid> --vdatum-dir <dir>` — each pixel's
vertical offset is the MLLW ellipsoidal height *at that point* from the
VDatum grids (the same `world/datum/` grids the `enc_updater` provisions),
never a hand-computed constant. Pixels default to positive-down depths below
the datum (`h_ellip = mllw_z − depth`); pass `--source-up` for up-positive
datum-relative heights. Mutually exclusive with
`--depth-scale`/`--depth-offset`; a point outside the grids' coverage is a
hard error (a partially converted surface never imports); the source datum
is recorded in `registry.json` provenance (`datum` field).

Sequence — everything that can refuse does so **before** the live layer is
touched, so a rejected regeneration leaves the old layer standing (D7):

1. **Validate.** The store dir must be a directory. The staged dir must be a real
   directory (not a symlink), must not alias the live `chart/` or its
   `.chart_backup/` (an equivalence check that cannot be resolved is itself a
   refusal — it fails closed), must contain no symlinked *top-level* entries, must
   hold ≥ 1 `.tif`, and must sit on the same filesystem as the store (a
   cross-device `rename(2)` returns `EXDEV` and cannot commit atomically).
2. **Crash recovery.** A leftover `.chart_backup/` marks an interrupted prior run.
   If `chart/` is absent the backup holds the only copy of the old layer, so it is
   **restored**; otherwise it is stale and **dropped** — tolerantly: the removal
   uses the `error_code` overload, and a backup that still survives (a persistent
   `EACCES`/`EROFS`/`EBUSY`) is renamed aside to `.chart_backup.stale.<n>/` with a
   warning, so a failed cleanup can never wedge later regenerations. Only a backup
   that can be neither removed nor moved refuses the run.
3. **Swap.** An existing `chart/` is renamed to `.chart_backup/`, then
   `staged → chart/` is the single commit point (atomic on one filesystem). If the
   commit fails, the backup is renamed back (best-effort, via `error_code`, so a
   double fault never masks the original error) and the original exception
   propagates.
4. **Cleanup.** On success the backup is removed with the `error_code` overload —
   a committed swap is never reported to the caller as a failure. A cleanup failure
   only warns on `std::cerr`; the leftover is cleared by step 2 of the next run.

Caller contract: run only while no consumer holds the store open (D7's nav-down
precondition). Aside dirs (`.chart_backup.stale.<n>/`) are not layer dirs, so
`load()` ignores them; clear them by hand.

#### Depth overview pyramids (`build_depth_overviews`, uma-ADR-0010 D9 / uma-ADR-0011 / uma-ADR-0013)

`build_depth_overviews` is an **offline batch** builder that generates a coarse
overview pyramid for a depth layer, so survey bathymetry participates in
world-zoom **display**. (Display only — no query path reads the sidecar; see the
uma-ADR-0013 D8 note below.) It folds a layer's native
tiles into coarser parents, level by level, into a flat `overviews/` sidecar next
to them (`<layer>/overviews/<level>_<row>_<col>.tif` — the level rides in the
filename, exactly as in the native layer):

```bash
# Rebuild the sidecar for a store's processed layer, every coarser level down to
# the apex. The layer's native levels are discovered — nothing to declare.
ros2 run marine_bathymetry_store build_depth_overviews /path/to/store/processed
# The mixed-level reference layer, stopping at level 6, with a dry run first.
ros2 run marine_bathymetry_store build_depth_overviews /path/to/store/reference \
  --min-level 6 --dry-run
```

- **Layer scope (D9).** `draft/`, `processed/` **and `reference/`** get a
  generated pyramid. `chart/` does not — it inherits the ENC scale ladder, a
  cartographer-curated shoal-biased native pyramid. Point the builder at one of
  the first three, not at `chart/`.
- **Mixed-level, native-wins.** The layer's native levels are **discovered**;
  there is no `--fine-level` to declare (it was deleted in
  [#331](https://github.com/rolker/unh_marine_autonomy/issues/331), not kept as
  an assertion). The build folds from the finest native level toward the apex,
  writing a derived tile **only where no native tile exists** at that
  `(level, index)`. Nothing compiled is overwritten or merged into. A level whose
  every parent is already native writes zero derived tiles and is still fully
  covered — that is the ordinary case for a mixed-level layer, not a failure.

  **The display inverts this rule, and both halves matter.** On disk native
  always wins. On screen a consumer composites every level ≤ its selection with
  finer over coarser, so a *derived* level 7 folded from 3.6 m harbour data draws
  *over* a *native* level 6 compiled at 14.5 m wherever both exist. That is
  intended and ECDIS-consistent (uma-ADR-0013 D3's corollary).
  (`marine_sidescan_mosaic`'s `build_sidescan_overviews` keeps its own
  `--fine-level`: that store is genuinely single-level, so the flag still asserts
  something true there. The divergence is deliberate.)
- **Shallowest-preserving fold (D9).** Each coarse cell carries its shoalest
  reliable child's whole `{depth, uncertainty}` pair — the **maximum** ellipsoidal
  height (the cell most hazardous to navigation), never a mean, and the σ always
  travels with the depth it describes. A coarse corridor query then plans around
  the rock rather than averaging it away.
- **Never upsamples.** Only coarser levels are built — `--min-level` must be
  strictly below the layer's finest **discovered** native level; the native tiles
  are the finest data.
- **Coverage manifest + geometric error (uma-ADR-0013 D1/D2/D3).** Each run
  writes `overviews/coverage.json` declaring which `(level, index)` zones the
  sidecar holds and each derived tile's geometric error in metres
  (`max(level GSD, max child ε)` — a saturated conservative bound). It is staged
  inside `overviews.tmp/`, so it rides the swap and is crash-consistent with the
  tiles it describes. The layer's **native** coverage is *not* written here: this
  builder does not own those tiles, so a file it wrote would go stale on the next
  import with nothing able to detect it.
- **Safety is untouched (uma-ADR-0013 D8).** No query path reads the sidecar or
  the manifest. `shallowestReliable()` keeps scanning native tiles to the finest
  available level for a region, so native-wins is a display-and-storage decision
  with no operational consequence.
- **Wholesale + crash-safe.** Each run rebuilds the whole sidecar (derived,
  regenerable — safe to re-run after every ingest) into a staging `overviews.tmp/`
  and swaps it in only once complete, so an interrupted or failing run never
  displaces a complete sidecar with a partial one. The swap prefers
  `renameat2(RENAME_EXCHANGE)` so `overviews/` is never momentarily absent for a
  concurrent reader (CAMP treats a missing `overviews/` as "no overviews"), and
  falls back to rename-aside via `overviews.old/` where the filesystem cannot
  exchange directory entries.
  A malformed/unreconstructable tile name is skipped loudly and **refuses the
  swap** (exit 4) — at **any** level, since under discovery that name's coverage
  would be missing from every level built beneath it. `overviews.tmp/` doubles as
  the per-layer run lock.
- **Loader-transparent.** `load()` / `loadWindow()` skip `overviews/` (and the
  `overviews.tmp/`/`overviews.old/` swap transients) silently; the store recovers
  each tile's level from its own filename, so the sidecar is not loaded as fine
  data (uma-ADR-0011 Consequences). `coverage.json` is not a `.tif`, so the
  flat-layout loaders already skip it silently too.

### Depth-adaptive level selection (`depth_adaptive_level.hpp`, uma-ADR-0010 D9 / uma#369)

`depthAdaptiveLevel(depth_m)` chooses the GGGS level a `processed` store tile
should be written at, from the depth that tile covers. It is the depth-driven
analogue of the level selection `s102_import` already does per dataset
resolution (`src/s102/run.cpp` maps `record.resolution_m` through
`gggs::Level::fromCellSize`).

The requested cell size is `capture_distance_scale · |depth|` (default 0.05,
mirroring `cube_bathymetry::Parameters::capture_distance_scale` — **no automated
link between the two repos' constants**), mapped through `fromCellSize` (which
returns the level at or finer than the request) and clamped to `[8, 14]`:

| Level | Cell size | Tile extent | Applies when |
|---|---|---|---|
| 8 (coarse clamp) | 3.624 m | 3478.7 m | depth ≥ 72.47 m |
| 9 | 1.812 m | 1739.4 m | 36.24–72.47 m |
| 10 | 0.906 m | 869.7 m | 18.12–36.24 m |
| 11 | 0.453 m | 434.8 m | 9.06–18.12 m |
| 12 | 0.227 m | 217.4 m | 4.53–9.06 m |
| 13 | 0.113 m | 108.7 m | 2.26–4.53 m |
| 14 (fine clamp) | 0.057 m | 54.4 m | depth < 2.26 m |

CUBE's 0.5 m capture *floor* is deliberately not carried over — it is a minimum
acceptance distance, not a resolution floor, and treating it as one pins
everything shallower than ~18 m to level 11. Level 10 is what every `processed`
tile holds today, so each step finer is **4× the cells and tiles over the same
ground**: 4× at 11, 16× at 12, 64× at 13, **256×** at the level-14 clamp.

- **Decision unit**: one level per store tile, sized from the **shallowest**
  depth in the tile (that sounding has the tightest capture radius). This is
  circular as stated — the level chosen *defines* the tile extent — so the writer
  (cube_bathymetry#143) must break the circle with a level-independent decision
  region or a fixpoint iteration. The scalar signature may change when it lands.
- **The clamps are the exception** to "cells are never coarser than the capture
  radius": below ~1.13 m of water the request is finer than a level-14 cell and
  the fine clamp holds the lattice there.
- **Fails loud**: a non-finite depth, an inverted or out-of-range clamp, and a
  non-positive `capture_distance_scale` all throw `std::invalid_argument`. A
  caller whose decision unit has no data (shallowest depth = NaN) should skip
  that unit rather than let the throw end a multi-hour import. A zero depth —
  and any request that underflows or overflows the float `fromCellSize` takes —
  returns a clamp rather than reaching its undefined `log2(0)`/`log2(inf)` path:
  the fine clamp at the underflow end, the coarse clamp at the overflow end. The
  overflow end is set by the **grid** size: `fromCellSize` multiplies the cell
  size by 960 in float before the `log2`, so the break-down point is a cell size
  above ~3.5e35 — **|depth| above ~7.1e36** at the default scale, three decades
  below `FLT_MAX` itself. Physically unreachable; the guard is on the value that
  actually overflows.
- **Nothing calls it yet.** `import_bag` builds one `cube::GeoMapSheet` per run
  and pins the store cell size to it, so CUBE's estimation grid and the store
  tiling are one resolution by construction; decoupling them is
  [cube_bathymetry#143](https://github.com/rolker/cube_bathymetry/issues/143).
  Existing level-10 stores are untouched — the policy applies to new imports
  only.

## Build & test

This package lives in the `unh_marine_autonomy` repo and builds in `core_ws`:

```bash
colcon build --symlink-install --packages-select marine_bathymetry_store
colcon test --packages-select marine_bathymetry_store
```

Tests (`test_store`, `test_query`, `test_tile_io`, `test_geotiff_import`,
`test_depth_overview`, `test_depth_adaptive_level`) are headless GTest and cover
priority precedence, no-data
handling, the height-aware shallowest-reliable semantics, persistence round-trip
(depth + uncertainty), incremental (dirty-only) save, level-mismatch rejection,
the GeoTIFF importer, the depth overview-pyramid builder (shallowest-preserving
fold, {depth, σ} pair coherence, no-upsample invariant, malformed-filename
skip→swap-refusal, and the loader's silent `overviews/` skip), the
depth-adaptive level policy (every level transition depth, both clamps,
monotonicity in depth, and the fail-loud edge cases), and the coarse
`StoreMetadata` round-trip.

The chart suite additionally covers the write gates (`set` / `importTiles` on a
normal store, and both the constructor and `fromCellSize` staging opt-ins), the
full `Processed > Draft > Reference > Chart` best-source walk-down, and every
`replaceChartLayer` path: round-trip through a swap, wholesale supersession of
stale tiles, each refusal (missing / empty / symlinked / aliased staged dir,
non-directory store), stale-backup recovery from a crashed run, restoration of the
old layer when the commit rename fails, restoration of an orphaned backup, and a
failed post-commit cleanup — including the *second* swap that must still succeed
over the backup left behind. The permission-dependent cases `GTEST_SKIP` when the
suite runs as root, where directory-permission denial is bypassed.

## S-102 importer (`s102_import`, #278)

Fetches NOAA S-102 gridded bathymetry (depth + 1σ uncertainty, HDF5, MLLW,
ed3.0.0) for a geographic area and imports it into a store layer:

```bash
ros2 run marine_bathymetry_store s102_import \
  --area -70.75,42.90,-70.55,43.05 \
  --store /path/to/scratch_store \
  --datum vdatum --geoid <geoid.tif> --vdatum-grids <gtx_dir>
```

`--cache` defaults to the canonical S-102 import cache
`~/data/world/s100/s102` (ADR-0010 D3 as amended by #288; the `s100/` sibling
holds S-100-family products alongside `charts/` and `datum/` under
`~/data/world/`). The default is tilde-expanded via `$HOME`; pass `--cache
<dir>` to override it (an explicit value is used verbatim — no `~`/env-var
expansion). Pre-world cache locations such as `~/data/stores/s102_cache` are
**superseded**; moving an existing cache into `~/data/world/s100/s102` on a
host is an operator step, not this tool's job.

> **Never generated by default on field hosts.** `s102_import` is
> **operator-run only**: nothing in the updater, provisioner, or deploy path
> invokes it, so S-102 is never fetched or generated on field hosts
> (pandy/gabby) without an operator running this command. Giving `--cache` a
> default path is safe **only** because this tool never runs unattended — do
> not wire `s102_import` (or `s100/` cache creation) into any automated step.

> **SCRATCH STORES ONLY until uma#276 lands** (ADR-0010 D7 precondition): the
> current `bathymetry_layer` treats high-uncertainty cells as reject → LETHAL
> under `unsurveyed_is_lethal`, so chart-grade σ (CATZOC/S-102) under a live
> costmap would render charted regions wholesale keepout. Never point
> `--store` at a store a live costmap reads until the worst-case-clearance
> cost model (uma#276) is deployed.

Pipeline (all stages fail loud; see `src/s102/`):

1. **Discover** — finds the newest `Navigation_Tile_Scheme_*.gpkg` catalog in
   the `noaa-s102-pds` bucket (or takes `--catalog <url|path>`) and queries it
   for tiles intersecting `--area`. Downloads key off catalog URLs only.
2. **Fetch** — SHA256-verified downloads into `--cache` (`tiles/` +
   `tiles.json` registry; the catalog is cached too). Idempotent: unchanged
   issuances are cache hits, and payloads are re-hashed on every access.
   **Fetch before deploying** — with a warm cache every later stage works
   `--offline` (ADR-0010 boat-as-offline-tooling).
3. **Convert** — warps each tile to geographic WGS84 (the store rejects
   projected rasters) and applies the per-cell MLLW→ellipsoid shift
   (`height = mllw_z(lat,lon) − depth`, up-positive, NaN nodata) via
   `marine_vertical_datum` (`--datum vdatum`) or a fixed offset
   (`--datum constant:<m>`, scratch use). Non-MLLW tiles (e.g. Great Lakes
   LWD) are refused, never silently shifted.
4. **Import** — through `importGeoTiff` at the GGGS level matching each
   tile's native resolution (4m/16m from the catalog). Whole-pipeline
   idempotency via `<store>/s102_imported.json` (`tile_id → issuance`);
   re-runs are no-ops until a tile is reissued (`--force` overrides).

## Dependencies

- `marine_autonomy` — the GGGS spatial index (`gggs::Level`/`GridIndex`/`CellIndex`).
- `geographic_msgs` — `GeoPoint` for the region query API.
- GDAL — GeoTIFF persistence; S102/GPKG drivers + warping for `s102_import`.
- `marine_vertical_datum` (#274) — per-cell MLLW→ellipsoid for `s102_import`.
- OpenSSL (`libssl-dev`) — SHA256 tile verification for `s102_import`.
