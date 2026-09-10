# Survey Index Schema (`survey_index.db`)

The SQLite sidecar written by `marine_survey_index`'s `survey_index_bag` and
read by `survey_index_query` — and, in later stages of
[#258](https://github.com/rolker/unh_marine_autonomy/issues/258), by the
explorer's overview pane and interval loader. This document is the **stable
cross-stage contract**; consumers should build against it, not against the
implementation.

Design decisions behind it were made at the
[#259](https://github.com/rolker/unh_marine_autonomy/issues/259) plan
checkpoint (2026-07-13); see `.agent/work-plans/issue-259/plan.md`.

## Ground rules

- **Regenerable sidecar.** The bags are the data of record; the index is a
  derived cache. Deleting `survey_index.db` and re-running the indexer always
  reproduces it. There are no migrations — an incompatible schema bumps
  `schema_version` and the open fails with a regenerate hint.
- **Index = "where did the sensor look."** Pass intervals are computed from
  ping geometry (nav + sonar extents), independent of what any store
  accepted. Pings rejected by CUBE or absent from store coverage still index.
- **Tile keys live in the stores' key space.** Tiles are GGGS grids `(level,
  tile_row, tile_col)` — the same key space as the tiled stores' per-tile
  GeoTIFFs (e.g. bathy `10_17788_13902.tif` ⇔ `level=10, tile_row=17788,
  tile_col=13902`). The **default indexing level is L14 (~54 m tiles) for both
  sensors** — a target-inspection neighbourhood, deliberately finer than the
  stores' native tiling (bathy L10 ≈ 870 m, sidescan L13 ≈ 108 m). Finer keys
  roll **up** to store tiles through the GGGS parent hierarchy, so consumers
  joining against store tiling aggregate L14 rows under their L10/L13 parents.
  Mixed levels coexist in one DB; queries carry the level in the key.

## Tables (schema version 2)

```sql
CREATE TABLE schema_version (
  version INTEGER NOT NULL           -- always exactly one row
);

CREATE TABLE bags (
  id            INTEGER PRIMARY KEY,
  path          TEXT    NOT NULL UNIQUE,  -- absolute, lexically normalized
  size_bytes    INTEGER NOT NULL,         -- fingerprint: total regular-file bytes
  mtime_ns      INTEGER NOT NULL,         -- fingerprint: newest mtime under the bag
  indexed_at_ns INTEGER NOT NULL          -- wall clock when (re-)indexed
);

CREATE TABLE passes (
  id          INTEGER PRIMARY KEY,
  bag_id      INTEGER NOT NULL REFERENCES bags(id) ON DELETE CASCADE,
  level       INTEGER NOT NULL,      -- GGGS quadtree level of the tile key
  tile_row    INTEGER NOT NULL,      -- GGGS grid row (from south)
  tile_col    INTEGER NOT NULL,      -- GGGS grid column (from west)
  sensor_type TEXT    NOT NULL,      -- see vocabulary below
  topic       TEXT    NOT NULL,      -- the bag topic the pings came from
  t_start_ns  INTEGER NOT NULL,      -- first ping stamp in the pass (UNIX ns)
  t_end_ns    INTEGER NOT NULL,      -- last ping stamp in the pass (UNIX ns)
  ping_count  INTEGER NOT NULL       -- pings that touched this tile in the pass
);
CREATE INDEX passes_tile ON passes(level, tile_row, tile_col);
CREATE INDEX passes_bag  ON passes(bag_id);

CREATE TABLE nav_track (              -- added in v2 (#265)
  id         INTEGER PRIMARY KEY,
  bag_id     INTEGER NOT NULL REFERENCES bags(id) ON DELETE CASCADE,
  t_ns       INTEGER NOT NULL,        -- ping header stamp (UNIX ns)
  latitude   REAL    NOT NULL,        -- WGS-84 degrees
  longitude  REAL    NOT NULL         -- WGS-84 degrees
);
CREATE INDEX nav_track_bag ON nav_track(bag_id, t_ns);
CREATE INDEX nav_track_geo ON nav_track(latitude, longitude);
```

## `nav_track` semantics (v2)

A **decimated nav track** per bag for the explorer's overview map (#258):
drawing the survey track — and, from consecutive time-ordered points, travel
direction — without opening any bag.

- **Provenance: sensor ground origins, not a vehicle frame.** Each point is a
  posed ping's *sensor ground origin* (the same `earth`→sensor resolution the
  pass footprints use), interleaved across all indexed sonar topics and then
  distance-decimated as one stream. It is **not** a `base_link` track; at the
  default stride the per-sensor offsets are negligible for map display, but
  consumers must not treat the points as a single-antenna nav solution.
- **Distance-based decimation.** A point is kept iff it is the first posed
  ping of the bag or ≥ the stride (default **10 m**, indexer `--nav-stride-m`)
  from the *last kept* point. Spatially uniform by construction: a
  station-keeping boat adds no points; a fast transit stays fully sampled.
  The gate runs in bag stream order and rows are stored time-ordered; when
  ping stamps arrive slightly out of order, an occasional consecutive-by-time
  gap below the stride is possible — treat the stride as a density target,
  not a per-pair guarantee.
- **Same lifecycle as passes.** Written in the bag's atomic transaction;
  deleted and rewritten when a changed bag is re-indexed; cascades away with
  the bag row.

**Accessors** (`marine_survey_index/query.hpp`): `queryNavTrack(db, bag_id)` —
one bag's track ordered by time; `queryNavTrackInBox(db, lat_min, lon_min,
lat_max, lon_max)` — all points in a box ordered by bag id then time (segment
into per-bag polylines at bag-id changes). Boxes crossing the antimeridian
throw, matching `tilesForBoundingBox`.

## `sensor_type` vocabulary

**Extends** the ADR-0005 D3 `sensor_class` vocabulary (it does not redefine
it): `mbes-bathy` is used verbatim; sidescan is stored **channel-split** as
`sidescan-port` / `sidescan-stbd` (D3 `sidescan` + channel suffix) so one
channel can be pulled without re-indexing. Consumers filtering by the plain
D3 class `sidescan` must match both channel values (the query CLI's
`--sensor sidescan` does `LIKE 'sidescan%'`). New sensors add new values —
additive, no schema change.

## Semantics

- **A "pass"** is a maximal run of pings from one `(tile, sensor_type,
  topic)` whose inter-ping gaps are all ≤ the merge gap (default 5 s,
  indexer `--merge-gap`). Two survey lines crossing the same tile minutes
  apart are two passes — that per-pass identity is load-bearing for the
  explorer's single-pass sidescan display (#258 stages 3/5).
- **Footprints are conservative.** MBES: sensor position ± outermost good
  detections' across-track extent. Sidescan: sensor position + max slant
  range to the ensonified side (slant bounds ground range — no bottom model
  at index time). A tile listed may be *near* the swath edge; the drill-down
  stages do the exact math.
- **Incremental re-runs.** A bag whose `path`, `size_bytes`, and `mtime_ns`
  all match its ledger row is skipped; a changed bag has its passes deleted
  and re-indexed atomically (single transaction per bag). **A bag whose
  timestamp cannot be read at all** — every `stat` beneath it failed, or it
  holds no regular files — never satisfies that test: it is treated as changed,
  re-indexed, and reported on stderr, so an unreadable mtime cannot present as
  an up-to-date bag. `mtime_ns` is then stored as `0`, which is not
  load-bearing; the re-index decision is made before it is read.
