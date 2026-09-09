# ADR-0010: The Geospatial World Model — Taxonomy, Datum Invariant, and Per-Layer Processes

## Status

Accepted (2026-08-20). Proposed 2026-07-24; tracked by
[rolker/unh_marine_autonomy#272](https://github.com/rolker/unh_marine_autonomy/issues/272)
(closed on completion of the core implementation).

**"Accepted" means the decision is adopted, not that every consequence has
shipped.** The load-bearing implementation is merged (citations below); named
operational follow-ons remain open and are listed after them.

Implementing PRs (all merged; host-verified 2026-08-20):

| Decision | Merged as |
|---|---|
| D6 — vertical-datum library (`marine_vertical_datum`, ROS-free, core_ws) | [uma#279](https://github.com/rolker/unh_marine_autonomy/pull/279) |
| D3 / D7 — `chart` layer + wholesale corpus regeneration | [uma#280](https://github.com/rolker/unh_marine_autonomy/pull/280) |
| D7 — `s57_to_geotiff` exporter | [s57_tools#29](https://github.com/rolker/s57_tools/pull/29) |
| D7 — chart operator CLI | [uma#291](https://github.com/rolker/unh_marine_autonomy/pull/291) |
| D7 — cost-model rework (worst-case clearance / `confidence_gate`, precondition) | [uma#290](https://github.com/rolker/unh_marine_autonomy/pull/290) |
| D8 — `draft`/`processed` quality axis + cell-wise `clearOverlappedDraft` | [uma#313](https://github.com/rolker/unh_marine_autonomy/pull/313) + [cube_bathymetry#134](https://github.com/rolker/cube_bathymetry/pull/134) |
| D9 — depths overview pyramid (shallowest-preserving `overviews/` sidecar) | [uma#320](https://github.com/rolker/unh_marine_autonomy/pull/320) |
| D10 — `s57_layer` depth/obstacle split | [s57_tools#31](https://github.com/rolker/s57_tools/pull/31) (issue [s57_tools#30](https://github.com/rolker/s57_tools/issues/30)) |

**In flight** (not yet merged — kept out of the table above, which records only
merged, host-verified work):

- D9 — `reference` native-wins pyramid (mixed-level generalisation; amends the
  `reference` line below) —
  [uma#331](https://github.com/rolker/unh_marine_autonomy/issues/331)
- D9 — depth-adaptive `processed` levels (the policy function and the amendment
  below; the writer is a separate repo) —
  [uma#369](https://github.com/rolker/unh_marine_autonomy/issues/369) +
  [cube_bathymetry#143](https://github.com/rolker/cube_bathymetry/issues/143)

Open follow-ons (adopted target, not yet fully materialized):

- **Store-root migration** off `~/data/stores/` onto `~/data/world/` —
  [uma#310](https://github.com/rolker/unh_marine_autonomy/issues/310)
  (**unstarted**). Today `~/data/world/` exists only as a dev-host eval store
  ([uma#314](https://github.com/rolker/unh_marine_autonomy/issues/314): 13 ENC cells).
- **Chart-updater operational validation** — the corpus updater shipped
  ([s57_tools#28](https://github.com/rolker/s57_tools/issues/28), [PR#33](https://github.com/rolker/s57_tools/pull/33) merged);
  its nav-liveness-gated cron regeneration cycle still needs a real cron cycle
  exercised end-to-end (D7).
- **Field datum-grid provisioning** — geoid/VDatum download + user-polygon
  materialization into `world/datum/`
  ([uma#288](https://github.com/rolker/unh_marine_autonomy/issues/288); grids
  placed manually until it lands, per the D3 amendment).

Cross-cutting per the ADR-0001 convention: spans `marine_bathymetry_store`,
`marine_tiled_raster_store`, `marine_autonomy` (GGGS), `s57_tools`,
`mru_transform`, `cube_bathymetry`, `camp`, and the contact stores (ADR-0004).

**Amends [ADR-0002](0002-bathymetric-data-store.md):** re-introduces a `chart`
source layer (D3, D7) and restores the draft/processed quality axis that
Amendment A2 ([#248](https://github.com/rolker/unh_marine_autonomy/issues/248))
collapsed (D8) — both on new evidence recorded below, not a re-litigation of
A2's reasoning. ADR-0002 and ADR-0005 receive header pointers to this ADR in
the same PR.

**Builds on (unchanged):** ADR-0004 (unified Contact), ADR-0005 (provenance
registry — dormant per its #248 amendment, retained as the multi-platform
contract), ADR-0006/0007 (sibling backscatter stores), ADR-0008 (live tile
transport).

## Context

The August 2026 Isles of Shoals survey returns operations to ENC-covered
coastal waters after a season on an uncharted lake. Bringing charts back
exposed three structural facts, and the Massabesic season surfaced a fourth:

1. **Cross-layer override is impossible in the costmap.** `bathymetry_layer`
   max-combines into the master grid (it can only raise cost); ordering it
   after `s57_layer` therefore cannot let surveyed-deeper data relax charted
   shoal cost, and switching to overwrite would clear charted rocks and wrecks
   that ride in on the same `elevation` band
   ([echoboats#276](https://github.com/rolker/unh_echoboats_project11/issues/276)'s
   footgun). Per-cell best-source selection must happen **in the store**, before
   cost is computed. ADR-0002 D7 anticipated exactly this: "Eventually S57
   depths flow *through* the store rather than directly to the costmap."

2. **Chart data has a lifecycle unlike anything the store holds.** ENC editions
   are authoritative supersessions: a hazard removed in a new edition must
   vanish. The import path handles repeat imports badly in both directions
   (verified in `geotiff_import.cpp` / `bathymetry_store.cpp`): the
   lowest-uncertainty-wins merge resolves only *within-import* pixel
   contention into a local tile set, which `importTiles` then
   `insert_or_assign`s **whole-tile** into the layer — so a re-import
   silently replaces every prior same-layer cell in the tiles it touches,
   while tiles outside the new import's footprint linger stale forever
   (no delete-by-source). Charts also arrive with their own
   level-of-detail structure (the harbor/approach/coastal scale ladder) and
   their own quality metadata (CATZOC), neither of which the current layer
   model expresses.

3. **The day-to-day survey loop degrades a single fused survey layer.** The
   live CUBE node loses pings in bursts under subscriber-queue backpressure
   (observed as ping-gap striping in live tiles), while the offline
   `import_bag` re-run processes every bagged ping deterministically
   (~0.5 % residual nav/TF-gated drops). CUBE estimates are not
   ping-count-invariant, so the live surface is both gappier and noisier than
   the re-run. Under A2's single last-write-wins `survey` layer, the next
   day's live pass overwrites the previous night's authoritative re-run
   wherever swaths overlap — **continuing to survey makes the store worse**.
   Massabesic never exercised this loop (single coverage, processed once);
   a multi-day campaign lives in it.

4. **A TF frame is the wrong container for a chart datum.** `chart_datum` is a
   single z-offset, but the MLLW–ellipsoid separation is a spatially varying
   surface — decimeters of error across a coastal survey area, plus a runtime
   dependency (grids + node on the boat) for what is fundamentally static
   per-location data.

Meanwhile the store system has grown beyond bathymetry: backscatter imagery
(ADR-0006/0007), contacts (ADR-0004), live tile sync (ADR-0008), and now chart
depths, overhead clearances, and chart features. This ADR names the resulting
system, fixes its taxonomy and invariants, and records the per-layer processes.

## Decision

### D1 — The world model: one umbrella, not one database

The collection of geospatial stores is the vehicle's **world model**: its
persistent knowledge of the environment, organized as **raster stores (fields)
+ feature stores (features) + the provenance registry (ADR-0005)**, sharing the
GGGS spatial index, the ellipsoidal datum invariant (D5), and the
regenerable-from-source philosophy (every store is a cache derivable from
source material — bags, the ENC corpus, reference inputs).

Adoption is **umbrella-level only**: the on-disk root becomes `~/data/world/`,
documentation (including `docs/sonar_ecosystem.md`) reframes around the world
model, and consumers keep depending on the individual stores. **No package
renames** — `marine_bathymetry_store`, `marine_tiled_raster_store`, etc. remain;
their names describe mechanisms, not the concept.

### D2 — Fields are rasters; features stay vector; rasterization happens late

- **Fields** — continuous scalar surfaces (depth, uncertainty, backscatter,
  and eventually overhead ceilings) — live in GGGS-tiled raster stores.
- **Features** — discrete geometries with attributes and lifecycles
  (restricted areas, caution zones, floating hazards, wrecks-as-objects,
  contacts) — stay **vector**, rasterized only at the consumer (a costmap
  layer, a display renderer), never at ingest.

Late rasterization preserves feature identity, which is operationally
load-bearing: an operator authorized to work inside a restricted area waives
*that feature*; baked-in cost cells cannot be waived. The contacts pipeline
(ADR-0004) is the existing feature-side precedent.

### D3 — Taxonomy: theme × provenance

```
~/data/world/
├── depths/       chart | reference | draft | processed     (bathymetry store)
├── imagery/      sidescan store (two-tier, ADR-0006) +
│                 MBES backscatter store (CUBE-coupled, ADR-0007)
├── features/     contacts (ADR-0004); chart features thin — see D11
├── charts/       the ENC corpus itself + edition registry (cron-managed, D7)
├── s100/         S-100 family products; S-102 import cache at s100/s102/
│                 (populated by the operator-run s102_import CLI, not the
│                 cron updater; amended 2026-08-20, #288)
└── datum/        vertical-datum support data: geoid/ + vdatum/ grids
                  (updater-managed target — provisioning is a queued
                  s57_tools follow-on, currently manual) + user/ override
                  polygons materialized from git
                  (amended 2026-08-20, #288 — see amendment below)
```

Provenance layers for the depth theme, replacing ADR-0002 A2's
`survey`/`reference` pair:

| Layer | What | Lifecycle process |
|---|---|---|
| `chart` | Official navigation products (S57 → S-100 later) | Regenerated wholesale from the corpus on edition change (D7) |
| `reference` | Third-party priors (contour models, external grids/BAGs) | One-shot bespoke imports, σ per source |
| `draft` | Live on-boat CUBE output | Streaming append; live view via the ADR-0008 display transport (store-level sync remains ADR-0002 D6, deferred); evictable, disposable (D8) |
| `processed` | Off-boat deterministic re-run (`import_bag`) | Authoritative product, distributed back; clears overlapped draft (D8) |

The **imagery theme keeps its own tiering**: the sidescan and MBES backscatter
stores are deliberate siblings with cross-store arbitration at the query layer
(ADR-0006 D8); their layer names are not force-renamed to match the depth
theme.

**Write gates**: both prior layers are closed to normal ingest — `reference`
keeps A2's construction-time read-only-prior gate (explicit importer opt-in,
mechanism unchanged), and `chart` is writable **only** by the D7 regeneration
path. Live and replay ingest write `draft`/`processed` exclusively.

**Migration of existing stores**: the NH GRANIT prior is already in
`reference/` (correctly, post-A2). The existing fused `survey/` layer carries
no per-cell live-vs-re-run provenance (A2 dropped it), so there is nothing to
split — it is **re-classified wholesale to `processed/`**, which is accurate:
the current Massabesic stores were regenerated via `import_bag` (the
authoritative path). `draft/` starts empty. Beyond that, path changes are
config migration.

#### D3 amendment (2026-08-20, #288) — `datum/` and `s100/` siblings

**`datum/` is support data, not a store** — geoid and VDatum regional grids
plus user override polygons, consumed across themes (S-102 depth imports, the
D6 datum library and its CAMP/operator use, `chart_datum_node`). Because no
single store owns it and it is neither a feature set nor a registry (D1), it
sits as a **top-level sibling** rather than inside `charts/` or `depths/`.
Grids (`datum/geoid/`, `datum/vdatum/`) are **intended to be updater-managed**
like the ENC corpus (D7), but the updater does not yet provision them: that
download step (projsync geoid + VDatum bundle) is a queued `s57_tools`
follow-on. Until it lands, grids are placed manually.

**`datum/user/` is the one git-authored exception** to the
regenerable-from-source posture: override polygons (e.g.
`massabesic_datum_polygons.yaml`) are safety-relevant and stay PR-reviewed in
their project repo, which remains the source of truth; a deploy step
**materializes** a copy into `world/datum/user/` for discovery (the deploy
step is a queued follow-on in `bizzyboat_project11` — #288 plan item 5 — not
yet implemented). The invariant holds with git as the source — the copy is
regenerable, never hand-edited in place, and never updater-authored.

**`s100/`** hosts S-100 family products as their own top-level sibling
(decided at the #288 plan checkpoint): the family will span both raster
(S-102) and feature (S-101) products, so it does not belong under `charts/`.
The S-102 import cache's canonical location is `~/data/world/s100/s102/` —
superseding both the `~/data/world/charts/s102` example previously documented
in the `marine_bathymetry_store` README (#278; `s102_import` itself has no
default `--cache` path) and ad-hoc pre-world locations in use on existing
hosts (e.g. `~/data/stores/s102_cache`).

### D4 — Layers encode process; σ encodes trust

A layer is defined by *how data enters and is maintained* (the table in D3),
**not** by a fixed trust rank. Trust is the per-cell uncertainty σ, the
universal quality measure every source fills (survey σ from CUBE; chart σ from
CATZOC, D7; reference σ from each import recipe; unknown → σ = ∞).

Query semantics (refining ADR-0002 D3):

- **Shallowest-reliable (safety)**: unchanged — examines every layer and level,
  selects on depth + uncertainty only, ignores layer identity (ADR-0002 D7 /
  ADR-0005 D5 carve-out). This is the mode driving costmap cost; the costmap
  uses best-source only as a data-existence check, so nothing below affects
  navigation behavior.
- **Best-source (default)**: `processed > draft` is decided here — it is the
  D8 anti-clobber semantics and importers build against it. **Arbitration
  within the prior class (`reference` vs `chart`) is explicitly deferred** to
  the first consumer that needs a fused best-estimate answer (a CAMP fused
  depth view, QC tooling, voyage-planner display — none exist yet, and some
  candidate consumers want per-layer or per-consumer answers anyway). The
  policy is query-time only — no on-disk footprint, changeable without
  touching stored data. Until decided, implementations use a fixed
  `processed > draft > reference > chart` walk as a documented placeholder.
  Candidate policies recorded so the decision is not re-derived: (a) fixed
  ordering (simple, predictable; wrong when a low-σ reference BAG should beat
  a CATZOC-C chart); (b) lowest-σ wins among priors (honest; less
  predictable); (c) per-consumer choice (likely where this lands — QC,
  chart-comparison, and display plausibly each want different answers).

### D5 — Ellipsoidal invariant: datum conversion at import; `map_tide` is the only runtime vertical reference

All stored heights are WGS84-ellipsoidal (ADR-0002 D4, now made operational
end-to-end). Datum conversion happens **once, at import time, wherever import
runs** — per-cell VDatum separation for ENC (more accurate than any single
offset), constant offset for lakes, native ellipsoidal for our own surveys.

At runtime the entire vertical world is GNSS-ellipsoidal: clearance =
`z(map_tide)` − stored height, with `map_tide` self-measured by
`sea_surface_estimator`. No tide tables, no gauge feeds, no datum grids in the
navigation loop. **The `chart_datum` TF frame exits the autonomy stack** once
`s57_layer` stops computing depth (D10); it was also spatially wrong as a
frame (Context §4). Overhead clearances, charted relative to high water, are
conservative when compared raw and are handled per D10.

### D6 — Vertical-datum library: ROS-free, in core_ws

The datum machinery is extracted from `mru_transform` into a small **ROS-free
library in core_ws** (placement constraint: `s57_tools` (core), CAMP (ui), and
`mru_transform` (platforms) must all be able to depend on it):

- the PROJ/VDatum grid query (geoid + regional `.gtx` → MLLW/MHHW relative to
  ellipsoid at a lat/lon), currently inline in `chart_datum_node`;
- the existing ROS-free resolution chain (`datum_config`: lake-param →
  override polygons → VDatum → fallback polygons → absent), moved wholesale —
  the precedence chain *is* the lake story and importers need it too.

Consumers: the S57 exporter (per-cell, offline), CAMP (display-time
chart-datum readouts, operator-side), and `chart_datum_node` reduced to a thin
transitional wrapper. VDatum grids live wherever imports run — including the
boat as **offline tooling** (D7) — but never in the runtime stack.
[mru_transform#8](https://github.com/rolker/mru_transform/issues/8) (TF-frame
datum hierarchy design) is updated/closed against this direction;
mru_transform#7 (VDatum service) was delivered as `chart_datum_node` and is
subsumed.

### D7 — The chart layer is regenerated from the corpus, never merged

The `chart` layer is a **derived cache over the ENC corpus** (`world/charts/`),
maintained by an updater (cron-friendly; folds
[s57_tools#5](https://github.com/rolker/s57_tools/issues/5)) that downloads
ENC updates and **rebuilds the chart layer wholesale** on change — build into a
temporary layer directory, atomic swap. Never cell-wise merged: editions are
supersessions, and regeneration is what makes removal, re-arbitration, and
the clipping rule below trivial. Other layers are untouched.

Export rules (`s57_to_geotiff`, new tool in `s57_tools`, feeding the existing
`import_geotiff`):

- **Depth sources**: DEPARE/DRGARE polygons → band midpoint depth +
  half-band σ floor; SOUNDG points. (Structurally the NH GRANIT recipe,
  generalized.)
- **CATZOC → σ** (M_QUAL, currently ignored in `marine_charts`): per the ZOC
  table — A1 ≈ 0.5 m + 1 %d, A2/B ≈ 1.0 m + 2 %d, C ≈ 2.0 m + 5 %d, D/U →
  large but **finite** σ (never keepout-grade; see the cost-model work this
  feeds). **Finite-σ is a hard contract, not a nicety:** `bathymetry_layer`
  buckets a *non-finite* σ (σ = ∞) with no-data as **unknown quality** (the
  σ = ∞ ↔ unknown-quality mapping is D4) → conservative LETHAL (the consumer
  policy, ADR-0002 §D7), so exporting D/U as σ = ∞ would render
  exactly the CATZOC D/U cells this rule protects as keepout-grade — the opposite
  of the intent. σ = ∞ stays reserved for *genuinely unknown* quality (D4); the
  exporter MUST emit a large **finite** σ for D/U (e.g. a ZOC-D floor) so those
  cells cost as caution (go-slow), never keepout. Verify against
  `bathymetry_layer::evaluateCell`'s finite-σ branch before the first D/U export.
- **Datum**: per-cell chart-datum → ellipsoid via the D6 library.
- **Scale → GGGS level**: each ENC cell exports at the level matching its
  compilation scale (≈0.5 mm-at-scale resolvable ground distance, mapped to
  the nearest GGGS level). The store is already multi-level (ADR-0002 D2/#151).
- ~~**Largest scale governs**: each chart's raster footprint is clipped by all
  finer-scale charts' footprints, so coarse cells are never generated where
  finer coverage exists — standard chart practice, and it prevents the
  shallowest-reliable walk from letting a coarse band's midpoint shadow a
  confident harbor-chart depth.~~

  **Amended 2026-08-22
  ([#337](https://github.com/rolker/unh_marine_autonomy/issues/337),
  [s57_tools#49](https://github.com/rolker/s57_tools/issues/49)):
  footprint-clipping is withdrawn. Every chart is exported entire, at its own
  level.** Clipping by a finer cell's *declared* `M_COVR` deleted the coarser
  levels the rest of this family of documents runs on:
  [ADR-0013](0013-bounded-lod-navigation.md) D5 makes *"every level always has a
  renderable answer, by upsampling from a resident ancestor"* an invariant, and
  D3's corollary fills coverage gaps from coarser levels and marks them (S-52
  3.1.7 overscale). Both need the coarse material to still exist.

  It also falsified **D9 below, in this same ADR**. D9 exempts `chart` from
  pyramid generation because "the scale ladder is a cartographer-curated
  pyramid" — true of the ENC corpus, and false of what we stored, because
  footprint-clipping turns a nested ladder into a region-disjoint patchwork with
  no ancestors. This clause was what broke that premise; removing it makes D9
  correct as written.

  Measured over the Isles of Shoals survey box: charted coverage **34.6% →
  99.2%** (level 6 alone, 19.2% → 99.1%), recovering 64.6% of the survey area
  from charts already on disk. The corpus also exports 13 of 13 cells instead of
  12 — one approach cell was being clipped to nothing, which had looked like a
  conversion failure.

  **The safety rationale is preserved, not discarded** — it is a *precedence*
  concern, and precedence does not require deletion. The display draws finer
  over coarser (`camp#194` composites every level at or below the selection and
  discards no-data pixels, so finer covers coarser where it exists and coarse
  shows through the gaps). `shallowestReliable` takes the shallowest reliable
  value across all layers and levels, so a coarse band's midpoint can only win
  where it is **shallower** — conservative — and its large CATZOC σ routes it to
  go-slow rather than keepout under the #276 cost model. Scale precedence now
  lives in the consumers, where a coarse value can be *ranked* without being
  *destroyed*.

**Chart ingestion is gated on the cost-model rework** — a precondition, not a
parallel track, and satisfied by
[uma#276](https://github.com/rolker/unh_marine_autonomy/issues/276) (the tracking
issue), which landed as
[PR uma#290](https://github.com/rolker/unh_marine_autonomy/pull/290) — the
identifier the implementing-PR table and `sonar_ecosystem.md` frontier cite.
`bathymetry_layer`'s former `max_uncertainty` gate treated over-uncertain cells
as not-reliable → LETHAL, so CATZOC-grade σ entering the store would have
rendered chart-only regions wholesale keepout (or forced a global gate
relaxation that also weakened it for noisy draft data). #276 replaced it with
the worst-case-clearance / `confidence_gate` cost model (design settled
2026-06-25: high-σ ⇒ go-slow via worst-case clearance = clearance − σ, keepout
only on data trusted at or below the gate; a config still setting
`max_uncertainty` gets a one-shot deprecation warning and the value is ignored).

First cut regenerates **only while navigation is down** — an *enforced*
precondition, not an assumption: the updater checks a navigation-liveness
signal (lock/heartbeat) and refuses to swap while nav is active, cron or not.
Live regeneration under a running consumer is deferred until atomic tile
writes ([uma#189](https://github.com/rolker/unh_marine_autonomy/issues/189)/[#256](https://github.com/rolker/unh_marine_autonomy/issues/256))
and a store-change invalidation in `bathymetry_layer` exist. Two robustness
rules for the updater: (a) the edition registry is written **inside** the
staged layer directory so the atomic swap is the single commit point — a
half-failed run leaves the old layer + old registry fully intact, never a
current-registry/stale-layer split; (b) the updater surfaces chart-layer
age/health (last successful regeneration, last download attempt) so repeated
download failures age the layer loudly, not silently. Downloaded ENC data is
validated before the swap — catalog checksums where NOAA provides them, plus
a sanity check on the exported rasters (nonzero cell counts, plausible depth
range) — so a corrupt or truncated download can never swap in.

### D8 — Quality axis restored: `draft` (live) vs `processed` (offline re-run)

Amends ADR-0002 A2.1. The live and offline CUBE paths produce **different
surfaces** (Context §3), so they are different quality classes:

- **`draft`** — live on-boat CUBE: immediate, feeds today's costmap and the
  operator coverage view via ADR-0008 sync; known-gappy; disposable and
  regenerable from bags.
- **`processed`** — the deterministic off-boat re-run: authoritative,
  distributed back to boat and operator stores.
- Best-source priority `processed > draft` (D4), so a fresh live pass adds
  data where the re-run has none but never degrades re-run cells.
- **Regeneration clears overlapped draft, cell-wise**: when a new processed
  import lands, draft cells are removed **only where the processed import has
  data** (not the whole import footprint). Draft cells in the re-run's small
  gated-drop holes survive — harmless under `processed > draft` and strictly
  more data than clearing by footprint — while stale gap-striping never
  accumulates under the authoritative surface.

A2's collapse was correct for its context (single platform, single coverage,
processed once). ADR-0002's **A1** supersession note (#221) already recorded
the discipline this amendment follows: an axis removed as speculative must be
re-introduced when a workflow actually needs one (it named the
revisit-and-compare workflow and an epoch/versioning axis; the day-to-day
campaign loop is the workflow that arrived, and the quality axis is the
minimal form it needs — neither note pre-authorized D8, but the reversal
pattern is the recorded, intended one).

The same live-vs-re-run degradation applies in principle to the **MBES
backscatter store** (ADR-0007, CUBE-coupled, likewise collapsed to a single
layer by #248). It is **accepted there for now**: backscatter is a
display/QC product, not a navigation-safety input, so last-write-wins
striping costs image quality, not safety. Revisit ADR-0007 (with a header
pointer) if/when backscatter becomes a decision input or the striping
measurably hurts mosaic QC. Follow-ups, not
gating this ADR: quantify the live-vs-re-run delta (A/B diff of the two stores
for one Massabesic day), and characterize/mitigate live-node backpressure in
`cube_bathymetry` (shrinks the gap; cannot close it — determinism and
full-data replay remain offline properties).

### D9 — LOD is a per-layer process

- **`chart`**: LOD arrives built-in — the scale ladder is a
  cartographer-curated pyramid, and chart generalization is shoal-biased, so
  coarse levels are inherently conservative. No pyramid generation.
- **`draft`/`processed`**: born at fine native levels; overview levels are
  generated ([uma#188](https://github.com/rolker/unh_marine_autonomy/issues/188))
  with **shallowest-preserving aggregation** — never a mean; the rock must
  survive the downsample.

  **Amended 2026-09-09
  ([#369](https://github.com/rolker/unh_marine_autonomy/issues/369)):
  `processed` is *to be* a depth-adaptive, mixed-level layer — one level per
  store tile, chosen from the depth that tile covers. `draft` is unchanged and
  stays fixed-level.** (The policy and the store-side support are decided and
  landed; the writer is not — see "the writer is pending" below.) "Born at fine native levels" read as *one* fine level per
  layer, and every `processed` tile written to date is level 10 (~0.906 m
  cells) regardless of whether it holds 2 m or 15 m of water. CUBE's own capture
  radius already follows depth — `capture_distance_scale · |depth|`
  (`cube_bathymetry/.../parameters.h`, `node.cpp:154`) — so the lattice was the
  only part of the chain that did not.

  *The policy.* `marine_bathymetry_store::depthAdaptiveLevel` (this repo, #369)
  requests a cell size of `0.05 · |depth|` — **no floor** — and maps it through
  `gggs::Level::fromCellSize` (the level at or finer than the request), clamped
  to `[8, 14]`:

  | Level | Cell size | Applies when |
  |---|---|---|
  | 8 (coarse clamp) | 3.624 m | depth ≥ 72.47 m |
  | 9 | 1.812 m | 36.24–72.47 m |
  | 10 (every tile today) | 0.906 m | 18.12–36.24 m |
  | 11 | 0.453 m | 9.06–18.12 m |
  | 12 | 0.227 m | 4.53–9.06 m |
  | 13 | 0.113 m | 2.26–4.53 m |
  | 14 (fine clamp) | 0.057 m | depth < 2.26 m |

  CUBE's 0.5 m term is deliberately **not** carried over: it is a minimum
  *acceptance distance*, not a claim about achievable resolution, and treating
  it as a resolution floor pins every depth shallower than ~18 m to level 11 —
  a new fixed level across the whole surveyed range rather than an adaptive one.

  *Storage, stated plainly.* Each level step is a **4× cell- and tile-count
  increase over the same ground**, so against today's uniform level 10 the
  multiplier is `4^(L-10)`: 4× at level 11, 16× at 12, 64× at 13 and **256× at
  the level-14 clamp**. The whole-survey figure depends on the survey's
  depth-area histogram, which has not been computed; it is bounded by 1/16×
  (all water ≥ 72.5 m — the coarse clamp is level 8, two steps *coarser* than
  today's 10, so levels 9 and 8 are 1/4× and 1/16×) and 256× (all water
  < 2.26 m). Tiles are dense 960×960 rasters whether or not the survey fills
  them, and a level-14 tile spans only 54.4 m,
  so shallow water also pays a partial-tile overhead — and `import_bag`'s
  `max_resident_tiles` budget means something different at 16–256× the tile
  count. The **coarse clamp is a bound, not a floor at today's resolution**:
  between ~36 m and ~72 m the ladder returns level 9, coarser than today's 10.
  That is deeper than either operator platform surveys, which is why it is
  acceptable rather than a regression; if "never coarser than today" is ever
  required, the coarse clamp becomes 10.

  *The decision unit.* One level per **store tile**, sized from the
  **shallowest** depth in that tile — the shallowest sounding carries the
  tightest capture radius, so sizing from it keeps every sounding in the tile
  within its own capture distance of a node. `depthAdaptiveLevel`'s scalar
  signature encodes exactly that, and **may change** (to a depth range, say)
  when the writer lands.

  *The writer is pending — this is a decided policy, not as-built behaviour.*
  Nothing writes depth-adaptive `processed` tiles yet, and no existing store is
  affected. `import_bag` constructs one `cube::GeoMapSheet` for an entire run
  (`import_bag_main.cpp:1030`) and pins the store cell size to that sheet
  (`:1130-1133`), so CUBE's estimation grid and the store tiling are the same
  resolution by construction; decoupling them is a re-architecture of the import
  pipeline, tracked as
  [cube_bathymetry#143](https://github.com/rolker/cube_bathymetry/issues/143).
  Retroactive reprocessing of the existing level-10 Isles of Shoals and
  Massabesic stores is out of scope and gated on
  [#366](https://github.com/rolker/unh_marine_autonomy/issues/366) (import
  ledger) — this policy applies to new imports only.

  *Why a mixed-level `processed` is safe — two store changes it required.*
  [ADR-0013](0013-bounded-lod-navigation.md) D8: safety queries never consult an
  LOD level, and read the finest available data **for a region**. That is the
  obligation this amendment makes binding, not a guarantee it inherited: two
  store mechanisms assumed `processed` was single-level and had to change with
  it (both landed with the policy, in #369).

  1. **The safety query now reads the region, not a point — including its
     existence gate.** `shallowestReliable()` and `reliableSamples()` re-resolved
     *one* cell per level from the query cell's centre. Under a level-10/11
     costmap query, a level-13/14 `processed` tile covers that cell with 64-256
     native cells, so the walk read one of them and a 0.2-0.5 m rock — precisely
     what those levels exist to resolve — could fall between samples. Both
     queries now enumerate **every** native cell a query cell covers at any finer
     level and keep the shoalest reliable value.

     A *third* query had to change with them, and it is the one that decides
     whether the other two run at all. `bathymetry_layer::evaluateCell`
     implements the ADR-0002 D7 two-query no-data policy, and its first query —
     "is there ANY data here?" — was `bestSource()`, a point lookup. On
     `nullopt` it returned early, so the region-aware query never ran: one
     no-data native cell under the query cell's **centre** (a gated-drop hole, a
     between-lines gap, an absent 54.4 m tile beside a present one) declared the
     whole costmap cell unsurveyed and dropped a rock in any of the other 255.
     The existence gate is now `hasAnyData()`, region-aware over the same ground
     as the depth queries and quality-blind as D7 requires, and `evaluateCell`
     runs `reliableSamples()` **first** — the region-aware query is never gated
     behind a point query. `bestSource()` itself stays a point lookup by design:
     it is the best-available *display* query. What was wrong before #369 round 2
     was not `bestSource`'s contract but the safety path's use of it, and its
     `@warning` now says so.
  2. **The cross-layer anti-clobber (D8, below) is now level-aware.**
     `clearOverlappedDraft` keyed on the processed tile's `GridIndex`, which
     carries its level, so a finer `processed` tile matched no fixed-level
     `draft` tile, cleared nothing and said nothing — superseded draft blunders
     would have kept winning `shallowestReliable`. The clear now walks every
     level `draft` holds; a `draft` cell coarser than the processed tile clears
     only where that tile fully supersedes it, and the partly-covered ones are
     kept and **counted**, never silently skipped.

  *The consequence of reading the region: per-query fan-out on the live costmap
  thread.* Correctness here is not free, and the cost is a **config parameter**,
  not a future condition. `bathymetry_layer` derives its query level from
  `BathymetryStore::fromCellSize(resolution_)`, so the gap between the query
  level and the store's native level is set by the costmap's resolution against
  whatever the store holds: a 2 m global costmap over today's *uniform level-10*
  `processed` already fans out 4 covered cells per costmap cell, a 4 m global
  16 — both reachable on the boat now, without waiting for the depth-adaptive
  writer. A 1 m global over level-14 tiles would be 256, about 2.56 M cell visits
  (each a map find and a `push_back`) inside one 100x100 rendered tile, and
  `generateTile` checks its time budget only *between* tiles, so a tile is
  uninterruptible once started. This has not been measured, and no bound exists.
  Narrowing the query back toward a point sample is not the remedy — that is the
  defect just fixed — so the remedy is a measurement and then a bound or an
  interruption point, tracked as
  [#371](https://github.com/rolker/unh_marine_autonomy/issues/371). The existence
  gate is the cheap half: `hasAnyData()` stops at the first cell holding data, so
  over surveyed ground it costs one cell and only a genuinely empty region pays
  the full walk.

  With those in place a native level boundary *inside* `processed` carries no
  operational risk — the same argument the `reference` bullet below makes for
  its native-wins pyramid, and the same one that exempts `chart` above. The
  D2/D3 obligations that come with a mixed-level layer are discharged
  generically: `buildDepthOverviewPyramid`
  ([#331](https://github.com/rolker/unh_marine_autonomy/issues/331))
  emits per-tile geometric error and a coverage manifest for `draft`,
  `processed` and `reference` alike, so it needs no change when `processed`
  starts holding more than one level. It does, however, interact with native-wins
  in a way that is a **known coarse-tier consequence, not a dischargeable writer
  obligation**. Native-wins suppresses a derived parent as a *whole tile*, and
  unlike `reference`'s disjoint S-102 footprints, depth bands within one
  contiguous survey share parent indices; where a shallow and a deep band write
  native tiles under one parent, the shallow band's fold is dropped at that level
  and every level coarser.

  An earlier draft of this amendment stated that as an obligation on the writer —
  "never emit two native levels over the same ground". That is **unsatisfiable**,
  and recording it as discharged would have been false: the ladder *guarantees*
  mixed native levels under one parent. A 217 m level-12 tile that is deep on one
  half and shallow on the other yields native level 12 beside native 13/14 under
  the same parent index; the only writer that could honour the obligation is one
  that is not depth-adaptive across a tile boundary, which is the whole policy.

  Safety is unaffected — it is held by the region-aware native query above, which
  never consults an LOD level (ADR-0013 D8). What is affected is the **coarse
  display tier**: at levels above the native boundary, the shallow band's fold
  may be missing where the deep band's native tile wins the parent. Record it as
  a display consequence for `camp` and a design input for the coarse-tier work,
  alongside the level-transition asymmetry below — not as something
  cube_bathymetry#143 can fix.

  *One asymmetry, for the display consumers.* `reference`'s mixed levels come
  from **disjoint** source regions (S-102 footprints); `processed`'s come from
  **depth bands within one contiguous survey**. A display consumer therefore
  sees far more frequent level transitions across a single pass than `reference`
  ever produces. That is a display/UX consideration for `camp`, not a
  store-contract change.
- **`reference`**: overview levels are generated, **native-wins**
  ([#331](https://github.com/rolker/unh_marine_autonomy/issues/331)). Fold
  upward from the layer's finest native level toward the apex, writing a derived
  tile only at a `(level, index)` pair that has **no native tile**. Nothing
  compiled is ever overwritten or merged into.

  *Why this replaced "as imported".* That line was defensible when `reference`
  held one hand-placed grid imported once. `s102_import` made it false: it maps
  each S-102 dataset's native resolution through `gggs::Level::fromCellSize()`,
  so one import populates several levels over disjoint ground — and below each
  region's coarsest native level the layer renders **nothing**. At Isles of
  Shoals that is the whole prior disappearing below level 6, and the NH GRANIT
  prior disappearing below level 10.

  *Why native-wins is safe, not merely conservative.* [ADR-0013](0013-bounded-lod-navigation.md)
  D8: safety queries never consult an LOD level. `shallowestReliable()` reads to
  the finest available data for a region regardless of what is drawn, so a
  compiled coarse cell being less shoal-biased than a hypothetical merge carries
  no operational risk. This is the same argument that exempts `chart` above.

  *Native-wins is a **storage** rule; the display **inverts** it — read the two
  together.* On disk, native always wins: nothing compiled is overwritten. On
  screen, a consumer composites every level ≤ its selection with **finer over
  coarser**, so a derived level 7 folded from 3.6 m harbour data draws *over* a
  native level 6 compiled at 14.5 m wherever both exist. That inversion is
  intended and ECDIS-consistent
  ([ADR-0013](0013-bounded-lod-navigation.md) D3's corollary: fill from a more
  general purpose, draw at one scale, flag overscale). Reading either half alone
  predicts the wrong picture: the storage rule alone suggests the compiled tile
  is what you see at level 7, and the display rule alone suggests compiled data
  gets overwritten. Neither is true.

  *Residual cost, worth documenting.* A future region holding compiled coarse
  data **and** a finer survey that found something shallower will show the
  compiled value at coarse zoom. That is ECDIS behaviour, the safety path is
  unaffected, and the eventual answer is a coverage-aware "finer data exists
  here" indication rather than a silent merge.
- **Never upsample**: coarse data is never rasterized finer than its source;
  consumers wanting finer fall through to coarser levels (the level-aware
  query already does this).

This fills the "refinement policy — staged" gap ADR-0002 D2/#151 left open.

### D10 — Consumers: the costmap and the voyage planner; `s57_layer` keeps only non-depth semantics

- **Costmap**: `bathymetry_layer` is the single depth authority, windowing
  finest-available around the vehicle. `s57_layer` gains a mode suppressing
  its depth ramp, retaining land, `restricted`, `overhead`, `caution`,
  `unsurveyed`, and point hazards — a late-rasterizing feature consumer (D2).
  Its `chart_datum`/tide machinery becomes dead code in this mode (D5).
- **Voyage planner** (future consumer, no data-model change): plans
  corridors at coastal/approach levels — cheap and conservative by D9 — and
  drops to harbor level at route endpoints. The level-aware walk exists in
  the query today; the **optional target-resolution bound is specified in
  ADR-0002 D2 but not yet implemented** — a small query-API addition the
  planner will need, not new store machinery. Our own survey data joins
  coarse-level planning only once D9 pyramids exist.
- **Overhead ceilings**: deferred. The end-state is an ellipsoidal-ceiling
  field (charted clearance converted at import; runtime check symmetric with
  the seafloor). Until then `s57_layer`'s raw high-water-referenced
  comparison is conservative and sufficient.

### D11 — Chart features stay thin until S-100

No chart-feature ingestion into a store we own: **the ENC corpus is the
feature store** (`marine_charts` already queries it), and the world model
tracks only corpus membership + editions (D7's registry). Revisit at the
S-100 transition: S-101 replaces the feature model wholesale, and S-102
(gridded depth + uncertainty) maps natively onto the D3 raster convention —
the thin choice leaves nothing to unwind.

### D12 — What the world model is not

Not a general GIS. No arbitrary-CRS support (WGS84/ellipsoidal only), no
styling/symbology, no external spatial database, no ambition to replace GDAL
or QGIS for analysis. The value is the opinionated subset: GGGS-tiled,
σ-everywhere, ellipsoidal, regenerable, queryable by Nav2 and CAMP. Additions
that don't serve a navigation, survey, or operator-display consumer are out of
scope.

## Consequences

- **Positive**: per-cell best-source across surveyed and charted depth with no
  costmap-layer override hacks; chart updates flow automatically and
  supersede cleanly; the store stops degrading under the daily survey loop;
  the boat's runtime is fully GNSS-ellipsoidal (no datum grids, no
  `chart_datum` frame); one documented mental model ("world") over a system
  that had outgrown "the bathy store".
- **Costs**: a new library (D6), a new exporter + updater (D7), the layer
  re-split touching store, importers, and `cube_bathymetry` write paths (D8),
  the `s57_layer` split (D10), and config migration (`~/data/stores` →
  `~/data/world`: nav2 `store_path`s and launch files on the boat, CAMP /
  operator-station store paths, and dev-tooling defaults keyed to the old
  root such as the survey index at `~/data/stores/survey_index.db`). Each
  lands as its own issue/PR.
- **Supersessions / housekeeping**: ADR-0002 header pointer (same PR);
  [uma#163](https://github.com/rolker/unh_marine_autonomy/issues/163)
  (chart-source layer) closed or repurposed toward D7;
  [s57_tools#23](https://github.com/rolker/s57_tools/issues/23) (clean-room
  ENC→costmap design) answered by this ADR;
  [echoboats#276](https://github.com/rolker/unh_echoboats_project11/issues/276)'s
  dangling "#14" reference re-pointed at the D10 split work;
  [s57_tools#26](https://github.com/rolker/s57_tools/issues/26) mooted for the
  costmap chain by D5/D10; mru_transform#8 updated/closed per D6;
  `docs/sonar_ecosystem.md` reframed (follow-on doc task).
- **Sequencing risk**: the long pole for August is D6 + D7 (library +
  exporter + regeneration semantics). The D10 split can precede D7 (run
  `s57_layer` obstacles-only from day one, retiring `chart_datum` early) at
  the cost of no charted-depth costs until the exporter lands; or
  echoboats#276's interim (re-enable `s57_layer` whole) runs first, keeping
  `chart_datum` temporarily. That choice is an implementation decision,
  recorded in the issues, not fixed here.

## References

- Umbrella / prior ADRs: [ADR-0002](0002-bathymetric-data-store.md) (+ A2/#248),
  [ADR-0004](0004-unified-perception-contact.md),
  [ADR-0005](0005-multi-platform-provenance-registry.md),
  [ADR-0006](0006-multi-platform-backscatter-store.md),
  [ADR-0007](0007-mbes-backscatter-store.md),
  [ADR-0008](0008-live-sonar-coverage-transport-and-render.md)
- Issues: [uma#86](https://github.com/rolker/unh_marine_autonomy/issues/86),
  [uma#151](https://github.com/rolker/unh_marine_autonomy/issues/151),
  [uma#163](https://github.com/rolker/unh_marine_autonomy/issues/163),
  [uma#188](https://github.com/rolker/unh_marine_autonomy/issues/188),
  [uma#189](https://github.com/rolker/unh_marine_autonomy/issues/189)/[#256](https://github.com/rolker/unh_marine_autonomy/issues/256),
  [echoboats#276](https://github.com/rolker/unh_echoboats_project11/issues/276),
  [s57_tools#5](https://github.com/rolker/s57_tools/issues/5)/[#23](https://github.com/rolker/s57_tools/issues/23)/[#25](https://github.com/rolker/s57_tools/issues/25)/[#26](https://github.com/rolker/s57_tools/issues/26),
  [mru_transform#7](https://github.com/rolker/mru_transform/issues/7)/[#8](https://github.com/rolker/mru_transform/issues/8)
- Field evidence: live-tile ping-gap striping (operator observation,
  Massabesic 2026-06/07); costmap max-combine (`bathymetry_layer.cpp`
  `updateCosts`); merge-only import (`geotiff_import.cpp`); CATZOC unhandled
  (`marine_charts` `s57_dataset.cpp`).
