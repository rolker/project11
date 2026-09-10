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

**Exit status** (a scheduler or a store build is the consumer, so the codes
distinguish the causes):

| Code | Meaning |
|------|---------|
| `0` | every nominated bag is in the index, and every fingerprint is trustworthy |
| `1` | the index is **incomplete**: a bag failed mid-index or could not be opened, a `--scan` tree could not be fully enumerated (bags may be missing outright), or the index DB itself could not be opened (nothing was done) |
| `2` | usage error — bad flag value, or nothing nominated by a well-formed command line |
| `3` | the index is complete, but at least one bag cannot be fingerprinted authoritatively and so **re-indexes on every run** until the cause is fixed |

`1` dominates `3`, and it also dominates `2`: a `--scan` tree that could not be
enumerated exits `1` even when it leaves nothing nominated at all — "the survey
disk is not mounted" is an incomplete index, not a mistyped command line, and a
scheduler has to be able to tell them apart. Every run that got as far as
nominating bags prints a `done:` summary line, including that one (`of 0
nominated`); a run that exits `1` because the index DB itself could not be
opened prints no summary, because nothing was done.

The distinction only survives a consumer that reads the code. Under `set -e`
an exit `3` aborts a store build exactly as hard as a `1`, which throws away
the difference between "the index is unusable" and "the index is fine, one bag
re-indexes every run" — so take the status yourself:

```bash
rc=0; survey_index_bag --scan ~/data/logs --db "$db" || rc=$?
case $rc in
  0) ;;
  3) echo "warning: a bag re-indexes every run; index is complete" >&2 ;;
  *) echo "error: survey index incomplete (rc=$rc)" >&2; exit "$rc" ;;
esac
```

Single interleaved chronological pass per bag (the bounded-TF-window pattern
from cube#63 / the sidescan importer): georeferences every MBES
`SonarDetections` and sidescan `RawSonarImage` ping, computes its conservative
ground-footprint bounding box, and records per-GGGS-tile **pass intervals** in
a SQLite sidecar. Indexing is from **ping geometry, not store acceptance** —
pings CUBE rejected still index. Unchanged already-indexed bags are skipped
(size+mtime ledger); changed bags are re-indexed atomically. A bag the indexer
could not fully read — no readable timestamp anywhere beneath it, or an
incomplete walk (an unreadable subdirectory, a *symlinked* subdirectory, an
entry of undeterminable type, an unrepresentable timestamp) — is never skipped:
it is treated as changed, warned about on stderr, counted in the run summary,
and makes the run exit non-zero, because a partial reading is stable and would
otherwise skip a changed bag indefinitely. Symlinks are deliberately not
followed *into* directories, by the fingerprint or by `--scan` (a link can
close a cycle a recursive walk would never leave). This is narrower than it
sounds: a symlink to a **bag** directory is nominated and indexed normally
(under the bag's resolved path, so it is not a second bag), so a scan root made
of links to bags on other volumes works. Only a symlink to an *intermediate*
directory is reported rather than walked — bags beneath one are missed, so name
the real path, or those bags, on the command line instead.
An entry that definitively holds no bag bytes (a FIFO, socket or device node,
or a symlink that resolves to nothing) is skipped without penalty: it hides
nothing. A **decimated nav track** (one point per ≥
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
symlinked subdirectory, unresolvable symlink, unrepresentable mtime, legacy
`mtime_ns = 0` row — plus the routes it must *not* penalise, a dangling
symlink and a FIFO).

The permission-based routes cannot run as root (root ignores the mode bits),
and both hosted CI and `ci_local.sh` run as root — so each trust flag is
guarded by at least one route that needs no permission trick (a symlinked
subdirectory reaches `mtime_valid && !scan_complete` directly). Keep it that
way: a guard that skips in every merge-gating path defends nothing.

`test_indexer_exit_status` runs the built `survey_index_bag` binary, because
the exit-status contract above lives in `main()` where no library call reaches
it: an unopenable bag, an indexed-but-untrustworthy bag, a `--scan` subtree
that was dropped, and a usage error.

```bash
colcon test --packages-select marine_survey_index
```
