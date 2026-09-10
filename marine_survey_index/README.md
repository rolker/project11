# marine_survey_index

Offline **survey index + query CLI**: maps a location to the raw bag data that
ensonified it. Stage 1 of the survey-data-exploration umbrella
([#258](https://github.com/rolker/unh_marine_autonomy/issues/258); this
package is [#259](https://github.com/rolker/unh_marine_autonomy/issues/259)).

The motivating question: *"someone reported a possible target at this lat/lon
— which bags, which time windows, which sonar saw that spot?"* The stores are
averaged products; target-level review needs the raw data behind a location,
and finding it by hand means opening bags one by one. The index answers in
seconds.

## Tools

### `survey_index_bag` — the indexer

```bash
ros2 run marine_survey_index survey_index_bag <bag_uri ...> [--scan DIR] \
    [--db survey_index.db] \
    [--mbes-topic /bizzy/sensors/m3/detections] \
    [--port-topic ...sonar_image_port] [--stbd-topic ...sonar_image_starboard] \
    [--mbes-level 14] [--sidescan-level 14] [--level N] \
    [--merge-gap 5.0] [--nav-stride-m 10.0] \
    [--earth-frame earth] [--sound-speed 1500]
```

Single interleaved chronological pass per bag (the bounded-TF-window pattern
from cube#63 / the sidescan importer): georeferences every MBES
`SonarDetections` and sidescan `RawSonarImage` ping, computes its conservative
ground-footprint bounding box, and records per-GGGS-tile **pass intervals** in
a SQLite sidecar. Indexing is from **ping geometry, not store acceptance** —
pings CUBE rejected still index. Unchanged already-indexed bags are skipped
(size+mtime ledger); changed bags are re-indexed atomically. A bag the indexer
could not fully read — no readable timestamp anywhere beneath it, or an
incomplete walk (an unreadable subdirectory, an entry of undeterminable type,
an unrepresentable timestamp) — is never skipped: it is treated as changed,
warned about on stderr, counted in the run summary, and makes the run exit
non-zero, because a partial reading is stable and would otherwise skip a
changed bag indefinitely. A **decimated nav track** (one point per ≥
`--nav-stride-m` metres, default 10) is recorded per bag so the explorer map
can draw the survey track from the index alone.

The default level is **L14 (~54 m tiles)** for both sensors — a
target-inspection neighbourhood, finer than the stores' native tiling (bathy
L10 ≈ 870 m ⇔ `10_<row>_<col>.tif`; sidescan L13 ≈ 108 m); L14 keys roll up
to store tiles via the GGGS parent hierarchy. Measured: a 2.5 GB Massabesic
sonar bag (632k pings) indexes in ~12 s.

### `survey_index_query` — the answer

```bash
ros2 run marine_survey_index survey_index_query \
    (--point LAT LON [--radius M] | --box LATMIN LONMIN LATMAX LONMAX) \
    [--db survey_index.db] [--level N] [--sensor TYPE] [--json]
```

Prints matching pass intervals grouped by bag, ordered by time — bag path,
UTC interval, duration, ping count, sensor, tile, topic. `--sensor` accepts
`mbes-bathy`, `sidescan-port`, `sidescan-stbd`, or `sidescan` (both
channels). `--json` for downstream tools (the #258 explorer consumes the same
index directly).

## Schema

`survey_index.db` is a **regenerable sidecar** — bags remain the data of
record. The schema is the cross-stage contract for #258 stages 2–5:
see [`docs/survey_index_schema.md`](../docs/survey_index_schema.md).

## Testing

Bag-I/O-free unit tests cover the DB-open contract, the interval
merge/split logic, footprint→tile enumeration (boundary straddling), the
query tile-join (sensor filters, level separation), the nav-track
decimation gate and accessors, and the bag fingerprint the incremental
skip decides on (mtime accuracy against `::stat`, a same-size in-place
rewrite, the ledger round-trip, and every route by which the fingerprint
must refuse to call a bag unchanged — unreadable timestamp, partial walk,
unrepresentable mtime, legacy `mtime_ns = 0` row):

```bash
colcon test --packages-select marine_survey_index
```
