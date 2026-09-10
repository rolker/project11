# Sonar Data Ecosystem — Big-Picture Map

A single living map of how sonar data flows from sensor to operator, and which
umbrella issues + ADRs own each stage. This is a **tracker, not a spec** — the
authoritative design lives in the linked ADRs and umbrella issues. Keep it
current; when an umbrella closes or a frontier shifts, update the relevant row.

> Status legend: ✅ done · 🔨 in progress · 📋 designed, not built · ⚠️ blocked/degraded

_Last verified: 2026-08-20 (world-model reframe under ADR-0010; store-bathy row,
ADR spine, and frontier refreshed against merge state);
"Where to direct efforts" frontier updated 2026-08-20 ([#311](https://github.com/rolker/unh_marine_autonomy/issues/311))._

## The two arcs

Sonar data drives **two peer arcs** that share the GGGS store and the operator's
map:

```
ARC 1 — COVERAGE  ("where did we survey / what's the depth & backscatter")
  ACQUIRE ─→ ESTIMATE ─→ STORE ─→ ┬─→ LIVE TRANSPORT → CAMP / web viewer
  (sensor)   (live grid)  (tiled) │
                                  ├─→ COSTMAP → Nav2
                                  ├─→ RENDER (display)
                                  └─→ REPROCESS → processed layer

ARC 2 — TARGETS   ("what did we find")
  EXPLORE ─→ MARK ─→ CONTACT ─→ DISTRIBUTE ─→ DISPLAY (CAMP / web)
  (rqt views over   (draw-box   (contact_manager
   sidescan & MBES)  → TargetAnnotation)  CRUD / curate / confirm)
```

Everything is GGGS-tiled (Colin Ware's Global Geographic Grid System); the
**store is the hub** for Arc 1, and both arcs converge on the operator's map.
Arc 2 is the **search mission's payload** — for Lake Massabesic (submerged-object
search), finding and marking targets *is* the point; Arc 1 tells you where you've
looked.

## The world model (the STORE hub)

The collection of geospatial stores is the vehicle's **world model**
([ADR-0010](decisions/0010-geospatial-world-model.md), Accepted 2026-08-20): its
persistent knowledge of the environment, organized as **raster stores (fields) +
feature stores (features) + the provenance registry** ([ADR-0005](decisions/0005-multi-platform-provenance-registry.md)),
all sharing the GGGS spatial index, the WGS84-ellipsoidal datum invariant
(ADR-0010 D5), and a regenerable-from-source philosophy (every store is a cache
derivable from bags, the ENC corpus, or reference inputs). Adoption is
**umbrella-level only** — no package renames; `marine_bathymetry_store`,
`marine_tiled_raster_store`, etc. keep their mechanism names.

The adopted on-disk root is **`~/data/world/`**, organized by theme × provenance
(ADR-0010 D3):

```
~/data/world/
├── depths/     chart | reference | draft | processed   (marine_bathymetry_store)
├── imagery/    sidescan (two-tier, ADR-0006) + MBES backscatter (ADR-0007)
├── features/   contacts (ADR-0004); chart features stay thin until S-100 (D11)
├── charts/     the ENC corpus + edition registry (cron-managed updater, D7)
├── s100/       S-100 family; S-102 import cache at s100/s102/ (#288)
└── datum/      geoid/ + vdatum/ grids + user/ override polygons (#288)
```

For the **depth** theme the four provenance layers replace ADR-0002 A2's
`survey`/`reference` pair, walked `processed > draft > reference > chart` for the
best-source query (ADR-0010 D4/D8) — the survey-first half (`processed > draft`)
is settled, but the `reference > chart` tail is the **documented placeholder**:
ADR-0010 D4 defers the real reference-vs-chart arbitration:

| Layer | What | Lifecycle |
|-------|------|-----------|
| `chart` | Official navigation products (S57 → S-100) | Regenerated wholesale from the corpus on edition change (D7); write-gated to that path |
| `reference` | Third-party priors (contour models, external grids/BAGs) | One-shot bespoke imports, σ per source; construction-time read-only gate |
| `draft` | Live on-boat CUBE output | Streaming append; known-gappy; disposable, regenerable from bags (D8) |
| `processed` | Off-boat deterministic `import_bag` re-run | Authoritative; distributed back; clears overlapped draft cell-wise **and level-aware** (D8, [#369]) |

**Adopted target, materialized on the dev host.** `~/data/world/` is the
canonical root: this repo's path literals (launch defaults, doc examples,
`~/data/world/survey_index.db`) were repointed and the dev host's store
content migrated under it in **uma#310** (clean cut, no compat symlink —
stale readers fail loudly). Still pending from #310's follow-ups: the
`unh_echoboats_project11` nav2 `store_path`/`draft_dir` config repoints
(separate PR in that repo) and the salmon/gabby store-root moves,
sequenced with a field-rebuild day.

## Arc 1 — Coverage

| Stage | What | Owning umbrella(s) | ADR | Status |
|-------|------|--------------------|-----|--------|
| **Acquire — MBES** | M3 multibeam driver (`kongsberg_em_bridge`) | — | — | ✅ functional; ⚠️ open data-quality bugs: gyro health [echoboats#337], time-sync [echoboats#338] (mru_transform SBG-primary [echoboats#339] ✅ closed) |
| **Acquire — sidescan** | Garmin GCV driver + platform integration | [#185](https://github.com/rolker/unh_marine_autonomy/issues/185) | — | 🔨 protocol cracked; driver/platform in flight |
| **Acquire — other** | DeltaT sonar driver | [#111](https://github.com/rolker/unh_marine_autonomy/issues/111) | — | 📋 |
| **Estimate — bathy** | CUBE draft tiles (live), backscatter co-estimation, slope correction | — | 0007 | ✅ mature ([cube#54], [cube#70] closed); slope-correction merged but dormant (needs [cube#59]) |
| **Estimate — sidescan** | `marine_sidescan_mosaic`: live mosaic (L13, uint16) + offline Tier-1/Tier-2 chain | [#171](https://github.com/rolker/unh_marine_autonomy/issues/171) (I2), [#185](https://github.com/rolker/unh_marine_autonomy/issues/185) | — | ✅ built: live mosaic node + Tier-1 `.sst1` archive + Tier-2 `flat`/`processed` builders + overview pyramid ([#188]); placement is DEM-orthorectified against the bathy store in the `processed` build ([#297], `--bathy-store`), flat-bottom in the live `draft` path by design (ADR-0006 D6/D9) and in the cheap `flat` Tier-2 builder by scope |
| **Store — core** | Generic band/dtype tiled-GeoTIFF store core | [#172](https://github.com/rolker/unh_marine_autonomy/issues/172) | 0002 | ✅ closed (`marine_tiled_raster_store`) |
| **Store — bathy** | World-model **depths** theme: four provenance layers (`chart`/`reference`/`draft`/`processed`), best-source priority query, multi-level (D3/D4) | [#86](https://github.com/rolker/unh_marine_autonomy/issues/86) | 0002 / 0010 | ✅ readiness arc complete; **D8** draft/processed split + cell-wise `clearOverlappedDraft` anti-clobber ([#313] + [cube#134]) ✅; **chart** layer + wholesale regeneration ([#280], exporter [s57#29], CLI [#291]) ✅; **D9** depths overview pyramid (shallowest-preserving `overviews/` sidecar, `build_depth_overviews`) ([#188] via [#320]) ✅, generalised to **mixed-level layers** with a native-wins rule and a per-tile geometric-error coverage manifest (uma-ADR-0013 D2/D3) so `reference` gets coverage below its coarsest native level ([#331]) 🔨; **D9** `processed` itself is now depth-adaptive and therefore mixed-level (cell = 0.05 x depth, clamped to levels 8-14), which made the safety query region-aware and the draft clear level-aware ([#369]) 🔨; [#151] levels ✅; [#189]/[#256] atomic tile write 📋 (gates live chart regen, D7) |
| **Store — backscatter** | Two-tier backscatter store (GeoCoder, draft/processed) | [#180](https://github.com/rolker/unh_marine_autonomy/issues/180) | 0006 / 0007 | 🔨 **M3/MBES half ✅ done**: [cube#80] offline producer merged → `marine_mbes_backscatter_store`; **sidescan half 🔨 in flight**: durable `processed` build with best-source compositing + registry, DEM orthorectification and a `projection.json` provenance sidecar ([#297]); GeoCoder radiometry (beam pattern, slope, EGN) still 📋 |
| **Store — provenance** | Cross-store multi-platform source-id + registry | [#179](https://github.com/rolker/unh_marine_autonomy/issues/179) | 0005 | 📋 deferred (multi-platform later tier) |
| **Live transport** | `SonarVisualizationTile` + anti-entropy tile-sync | [#230](https://github.com/rolker/unh_marine_autonomy/issues/230) (= #86-Phase-6 / #171-I3) | 0008 | ✅ **end-to-end**: #230 transport + [cube#78] producer + [camp#121] consumer (live cache) all merged; a **second independent consumer** (`marine_web_view`'s `coverage_renderer`, [#345](https://github.com/rolker/unh_marine_autonomy/issues/345)) reconciles the same catalog/request/tile triple in Python, which is the first cross-language exercise of the protocol |
| **Costmap** | Bathy → Nav2 costmap layer | [#127](https://github.com/rolker/unh_marine_autonomy/issues/127) | — | ✅ **usable** — tide-frame fix [uma#220] + latch-hardening [uma#223] merged (PR#222). Midpoint+uncertainty cost-model rework ✅ **landed** ([#290](https://github.com/rolker/unh_marine_autonomy/pull/290)) — an ADR-0010 D7 precondition for chart ingestion (see the frontier), not just an enhancement |
| **Render — CAMP** | Unified band-select + colormap raster render + live cache | [#175](https://github.com/rolker/unh_marine_autonomy/issues/175) (I4), [camp#121], [camp#108], [camp#63] | 0001 / 0008 | ✅ file-store display + GPU raster ([camp#90]), band-select ([camp#108], PR [camp#124] merged), and live cache ([camp#121], PR [camp#139] merged) all landed |
| **Render — web** | Browser SA viewer (contacts + bathy + sidescan) | [#166](https://github.com/rolker/unh_marine_autonomy/issues/166) | 0008 | 🔨 `marine_web_view` is live: `state_renderer` publishes position + track and a pinned-scale bathymetry/hillshade basemap ([#341](https://github.com/rolker/unh_marine_autonomy/issues/341)), and `coverage_renderer` renders live MBES coverage from the ADR-0008 transport to slippy PNGs in S3 ([#345](https://github.com/rolker/unh_marine_autonomy/issues/345)) — the **second** end-to-end ADR-0008 consumer after CAMP. Contacts and sidescan on the web map still 📋 |
| **Reprocess** | Offline M3 bag → store tiles; PINGMapper offline sidescan EGN | [#171](https://github.com/rolker/unh_marine_autonomy/issues/171) (C1) | — | ✅ M3 import landed ([cube#63] closed); 📋 sidescan offline pipeline to validate; ⚠️ draft→**processed** promotion workflow thin |
| **Survey index / query** | Location → raw bag data that saw it: `marine_survey_index` per-tile pass intervals (ping geometry, not store acceptance) + query CLI | [#258](https://github.com/rolker/unh_marine_autonomy/issues/258) / [#259](https://github.com/rolker/unh_marine_autonomy/issues/259) | — | ✅ stage 1 (indexer + CLI); explorer stages 2–5 📋 (schema contract: [survey_index_schema.md](survey_index_schema.md)) |
| **Metering** | Per-topic priority on the `udp_bridge` rate-limiter | [udp_bridge#19](https://github.com/rolker/udp_bridge/issues/19) | 0008 (D9) | 📋 |

[cube#15]: https://github.com/rolker/cube_bathymetry/issues/15
[cube#44]: https://github.com/rolker/cube_bathymetry/issues/44
[cube#54]: https://github.com/rolker/cube_bathymetry/issues/54
[cube#59]: https://github.com/rolker/cube_bathymetry/issues/59
[cube#63]: https://github.com/rolker/cube_bathymetry/issues/63
[cube#70]: https://github.com/rolker/cube_bathymetry/issues/70
[cube#78]: https://github.com/rolker/cube_bathymetry/issues/78
[cube#80]: https://github.com/rolker/cube_bathymetry/issues/80
[cube#81]: https://github.com/rolker/cube_bathymetry/issues/81
[camp#63]: https://github.com/rolker/camp/issues/63
[camp#90]: https://github.com/rolker/camp/issues/90
[camp#108]: https://github.com/rolker/camp/issues/108
[camp#121]: https://github.com/rolker/camp/issues/121
[camp#124]: https://github.com/rolker/camp/pull/124
[camp#139]: https://github.com/rolker/camp/pull/139
[rqt#58]: https://github.com/rolker/rqt_operator_tools/issues/58
[uma#220]: https://github.com/rolker/unh_marine_autonomy/issues/220
[uma#223]: https://github.com/rolker/unh_marine_autonomy/issues/223
[echoboats#337]: https://github.com/rolker/unh_echoboats_project11/issues/337
[echoboats#338]: https://github.com/rolker/unh_echoboats_project11/issues/338
[echoboats#339]: https://github.com/rolker/unh_echoboats_project11/issues/339
[#151]: https://github.com/rolker/unh_marine_autonomy/issues/151
[#163]: https://github.com/rolker/unh_marine_autonomy/issues/163
[#171]: https://github.com/rolker/unh_marine_autonomy/issues/171
[#185]: https://github.com/rolker/unh_marine_autonomy/issues/185
[#188]: https://github.com/rolker/unh_marine_autonomy/issues/188
[#189]: https://github.com/rolker/unh_marine_autonomy/issues/189
[#230]: https://github.com/rolker/unh_marine_autonomy/issues/230
[#256]: https://github.com/rolker/unh_marine_autonomy/issues/256
[#280]: https://github.com/rolker/unh_marine_autonomy/pull/280
[#291]: https://github.com/rolker/unh_marine_autonomy/pull/291
[#297]: https://github.com/rolker/unh_marine_autonomy/issues/297
[#313]: https://github.com/rolker/unh_marine_autonomy/pull/313
[#320]: https://github.com/rolker/unh_marine_autonomy/pull/320
[#331]: https://github.com/rolker/unh_marine_autonomy/issues/331
[#369]: https://github.com/rolker/unh_marine_autonomy/issues/369
[cube#134]: https://github.com/rolker/cube_bathymetry/pull/134
[s57#29]: https://github.com/rolker/s57_tools/pull/29

## Arc 2 — Targets (find, mark, curate)

The human-in-the-loop search arc: an operator explores sonar data in live `rqt`
views, marks a target, and that mark becomes a curated **Contact** distributed to
the map. Contacts ride a **separate** transport (SQLite + `marine_control` state
topic), intentionally *not* the Arc-1 raster tile-sync.

| Stage | What | Owning issue(s) | ADR | Status |
|-------|------|-----------------|-----|--------|
| **Explore** | `rqt` sonar review tools: waterfall over sidescan **and now MBES**, echogram, target views | [rqt#53] (echogram), [rqt#40] (MBES backscatter extractor), [rqt#50]/[rqt#69]/[rqt#73]/[rqt#82] (waterfall) | — | ✅ waterfall + echogram modernized ([rqt#53] done: PR#52/#55/#64/#80 merged); MBES backscatter extractor [rqt#40] 📋 |
| **Mark** | draw-a-box target marking in the live view → `TargetAnnotation` | [rqt#59] | — | 📋 (gated on the `TargetAnnotation` msg); **near-term subset [rqt#86] ✅ done** (PR#89) — waterfall box-drag publishes a `Contact` to the operator bag. Spawned the lightweight `marine_contacts` builder pkg ([uma#243]), bag recording ([echoboats#348]), configurable topic ([rqt#90]) |
| **Mark → Contact** | live-view marking publishes into `contact_manager` | [rqt#81] | 0004 | 📋 — the load-bearing link between the arcs |
| **Contact** | unified `Contact` CRUD store + curate/confirm + distribution | [#157](https://github.com/rolker/unh_marine_autonomy/issues/157) / [#167](https://github.com/rolker/unh_marine_autonomy/issues/167) (+#156) | 0004 | 🔨 v1 core in flight |
| **Device control** | bridgeable settings for the review tools / CA tuning | [#168](https://github.com/rolker/unh_marine_autonomy/issues/168) | 0003 | 🔨 |
| **Display** | contacts on the CAMP + web map (shared with Arc 1 render) | [#166](https://github.com/rolker/unh_marine_autonomy/issues/166) | — | 📋 |

[rqt#40]: https://github.com/rolker/rqt_operator_tools/issues/40
[rqt#50]: https://github.com/rolker/rqt_operator_tools/issues/50
[rqt#53]: https://github.com/rolker/rqt_operator_tools/issues/53
[rqt#59]: https://github.com/rolker/rqt_operator_tools/issues/59
[rqt#69]: https://github.com/rolker/rqt_operator_tools/issues/69
[rqt#73]: https://github.com/rolker/rqt_operator_tools/issues/73
[rqt#81]: https://github.com/rolker/rqt_operator_tools/issues/81
[rqt#82]: https://github.com/rolker/rqt_operator_tools/issues/82
[rqt#86]: https://github.com/rolker/rqt_operator_tools/issues/86
[rqt#90]: https://github.com/rolker/rqt_operator_tools/issues/90
[uma#243]: https://github.com/rolker/unh_marine_autonomy/issues/243
[echoboats#348]: https://github.com/rolker/unh_echoboats_project11/issues/348

> **Tracking gap (follow-up):** unlike Arc 1 (which has #86 / #171 / ADR-0008),
> the explore→mark→contact→display arc has **no unifying umbrella** tying the
> `rqt` marking tools ([rqt#59]/[rqt#81]) to the `contact_manager` backend
> (#157/#167) to display (#166). **Candidate follow-up: file a "target detection
> arc" umbrella** so this arc is as legible as the coverage arc. Not yet filed —
> noted here pending a decision (see #232).

## ADR spine

| ADR | Title | Stage |
|-----|-------|-------|
| [0001](decisions/0001-shared-scalar-colormap.md) | Shared scalar colormap | Render |
| [0002](decisions/0002-bathymetric-data-store.md) | Bathymetric data store (GGGS) | Store |
| [0003](decisions/0003-bridgeable-device-control.md) | Bridgeable device control | Arc 2 (tools/tuning) |
| [0004](decisions/0004-unified-perception-contact.md) | Unified perception contact | Arc 2 (Targets) |
| [0005](decisions/0005-multi-platform-provenance-registry.md) | Multi-platform provenance/registry | Store |
| [0006](decisions/0006-multi-platform-backscatter-store.md) | Multi-platform backscatter store | Store |
| [0007](decisions/0007-mbes-backscatter-store.md) | MBES backscatter store | Estimate/Store |
| [0008](decisions/0008-live-sonar-coverage-transport-and-render.md) | Live sonar coverage transport & render | Transport/Render |
| [0009](decisions/0009-sonar-info-message.md) | `SonarInfo` per-sensor acoustic metadata | Acquire/Estimate |
| [0010](decisions/0010-geospatial-world-model.md) | Geospatial world model (taxonomy, datum invariant, per-layer processes) | Store (umbrella) |
| [0011](decisions/0011-overview-pyramid.md) | Overview pyramids (sidecar layout, fold-policy contract) | Store |

## Where to direct efforts

### Near-term: last few days of Massabesic — three operator tools ✅ COMPLETE

Mission-driven priority (Roland, 2026-06-27): the three tools worth having before
the final surveying days. **All three landed (as of 2026-06-29).** Coverage
(Tools 1–2) is **band-selectable** — the operator views it as **depth /
uncertainty / backscatter** through one CAMP band-picker + colormap (ADR-0008 D6).

1. **Display existing coverage, especially multibeam — ✅ DONE.** M3 bathy in the
   GGGS store ([cube#63] closed) + CAMP's GGGS raster layer ([camp#90]) + offline
   backscatter producer ([cube#80], `marine_mbes_backscatter_store`) + the
   band picker (depth/uncertainty/backscatter + colormap, [camp#108], PR [camp#124]
   **merged**).
2. **Display new coverage as it's collected — ✅ DONE.** [#230] transport closed +
   [cube#78] producer merged (3-band live tile, sim-verified on Massabesic M3) +
   the CAMP consumer / disk-backed live cache ([camp#121], PR [camp#139] **merged**)
   — the last bottleneck, now closed.
3. **Mark a target on the live sidescan view → save a contact to the operator bag —
   ✅ DONE.** [rqt#86] (PR#89 **merged**): waterfall box-drag publishes a
   `marine_interfaces/Contact` (`ORIGIN_HUMAN`) on `/operator/sonar_waterfall/contacts`,
   recorded by the operator bag ([echoboats#348]); topic is a configurable plugin
   setting ([rqt#90]). The Qt-free Contact builder got a lightweight home
   (`marine_contacts`, [uma#243]). Sidesteps the unsettled `TargetAnnotation`
   ([rqt#59]) and the `contact_manager` backend (#157/#167).

**Producer symmetry:** both backscatter producers are merged — existing coverage
from the offline import ([cube#80], durable store layer), live coverage from
[cube#78] (display tile). **Both surface the co-estimated value UNCORRECTED**; the
nadir-stripe angle-correction ([cube#81]) is now **closed** (PR cube#84 merged, an
empirical angular-response/ARA approach) and corrects both once enabled.

**Next frontier (2026-07-13): survey data exploration —
[#258](https://github.com/rolker/unh_marine_autonomy/issues/258).** The campaign
is over and the question changed from "display coverage live" to "review what we
collected" (target candidates to re-survey / ROV-dive on Massabesic, and the same
tooling for the late-August Isles of Shoals data). The stores are averaged
products; target work needs the **raw, un-averaged data behind a location**.
#258 is the umbrella: a tile-indexed explorer — survey index + "which bags saw
this spot" query CLI ([#259](https://github.com/rolker/unh_marine_autonomy/issues/259),
stage 1) → stores-as-overview pane → multi-bag single-pass drill-down (raw
soundings + sidescan with shadows preserved) → per-tile CUBE re-runs with custom
parameters → surface texturing / sidescan drape. CAMP deliberately stays the
realtime monitoring/planning tool; exploration lives in the offline explorer
(`marine_perception_tools`, growing out of `sidescan_target_viewer`).

**World-model arc (2026-08-20): decision adopted, migration pending.** The
Isles-of-Shoals return to ENC-covered water drove [ADR-0010](decisions/0010-geospatial-world-model.md)
(Accepted): the stores are now framed as one **world model** rooted at
`~/data/world/`, with the depth theme's four provenance layers, per-cell
best-source in the store (not costmap override hacks), and a fully
GNSS-ellipsoidal runtime (no `chart_datum` frame). The load-bearing pieces are
merged — D6 datum library, D3/D7 chart layer + regeneration, D8 draft/processed
split, D9 depths pyramid, D10 `s57_layer` split. **Remaining frontier is
operational, not design:** (1) the store-root migration's cross-host tail
(uma#310 landed the dev host + this repo's literals; the echoboats config PR
and the salmon/gabby moves ride the field-rebuild day); (2) the
chart-updater's cron-cycle operational validation (the
updater shipped in s57_tools#28 / [PR#33](https://github.com/rolker/s57_tools/pull/33); the nav-liveness-gated regeneration loop
still needs a real cron cycle exercised); (3) field datum-grid provisioning
(uma#288 — geoid/VDatum download + user-polygon materialization).

The other legibility gaps remain: **Arc 2's `contact_manager` link** — the
load-bearing `mark → contact_manager` hop ([rqt#81]) plus the CRUD/curate store
(#157/#167, in flight — where a reviewed mark becomes a curated re-survey/ROV
target) and a unifying target-arc umbrella (see the tracking-gap note above) —
and the **sidescan track** ([#171]/[#185]: live mosaic + offline EGN). See
"Longer-term / supporting" below.

### Longer-term / supporting

- **Costmap cost-model rework** (worst-case clearance = clearance − σ, `confidence_gate`;
  midpoint-depth + per-band uncertainty) — ✅ **landed** ([#290](https://github.com/rolker/unh_marine_autonomy/pull/290), ADR-0010 D7 precondition for
  chart ingestion; replaces the old `max_uncertainty` gate, now deprecation-warned
  and ignored). The tide-frame blocker ([uma#220]/[uma#223]) was already fixed, so
  the layer was usable before this; the rework generalizes shore-keepoff / unknown-lake
  behavior and unblocks CATZOC-grade chart σ entering the store.
- **Land acquisition data-quality bugs** ([echoboats#337] gyro health, [echoboats#338]
  time-sync) — corrupt every downstream product in *both* arcs; cheap, foundational.
- The **sidescan track** ([#171]/[#185]) and the **draft→processed + coverage-QC
  loop** (the gap between "collected data" and "finished survey") — [cube#80] is
  a first step, producing an authoritative backscatter layer off-boat.

## Related

- [sonar_reference.md](sonar_reference.md) — durable hardware/protocol facts
  (M3 identity & interfaces, backscatter characteristics, QINSy real-time
  route rationale) and data-of-record locations
- [interfaces.md](interfaces.md) — ROS message/topic/service definitions
- [data_flows.md](data_flows.md) — system-level data flow diagrams
- [decisions/](decisions/) — Architecture Decision Records
