# ADR-0013: Bounded LOD Navigation for Large Geospatial Data

## Status

Proposed (2026-08-21). Tracked by
[rolker/unh_marine_autonomy#329](https://github.com/rolker/unh_marine_autonomy/issues/329).

Amended 2026-08-22 while still *Proposed*, so this is an edit rather than a
superseding ADR (the ADR-0001 immutability rule binds once **accepted**): D3
gained the manifest-trust rules — authority scope, the coverage-vs-content
staleness split, the permitted error direction, and the partial-read contract —
which the original stated no position on. Raised during the review of
[#331](https://github.com/rolker/unh_marine_autonomy/issues/331), the first
consumer of D3, and tracked by
[#334](https://github.com/rolker/unh_marine_autonomy/issues/334).

Establishes the consumer-side model that [ADR-0010](0010-geospatial-world-model.md)
(world model, D7 chart ladder, D9 per-layer LOD) and
[ADR-0011](0011-overview-pyramid.md) (overview sidecar) produce data for, and
adds the two store-contract items neither records: per-tile geometric error and
an explicit coverage declaration. Governs the display-side decisions currently
made independently in
[camp-ADR-0013](https://github.com/rolker/camp/blob/jazzy/docs/decisions/0013-lod-level-selection-demand-driven-load.md)
(LOD level selection and demand-driven load),
[camp-ADR-0010](https://github.com/rolker/camp/blob/jazzy/docs/decisions/0010-bounded-eviction-overview-pyramid.md)
(bounded live-tile eviction), and
[camp-ADR-0011](https://github.com/rolker/camp/blob/jazzy/docs/decisions/0011-viewport-clip-render-convention.md)
(viewport-clip render convention), which become implementations of this model
rather than independent answers.

**Citation convention (normative for this family of documents).** This repo and
`camp` both number ADRs sequentially and have collided on adjacent topics:
`camp-ADR-0010`/`uma-ADR-0010` and `camp-ADR-0011`/`uma-ADR-0011` are different
decisions, and this document is `uma-ADR-0013` while `camp-ADR-0013` is the LOD
selection ADR it most directly governs. **Every cross-repo ADR reference must be
repo-qualified in the `camp-ADR-00NN` / `uma-ADR-00NN` form.** A bare "ADR-0013"
is ambiguous by construction. Within a single repo's documents, relative links
to sibling ADRs remain unqualified.

The workspace repo (`ros2_agent_workspace`) is a third ADR namespace and
collides with this one too — its `ADR-0012` is the cross-reference-addendum
policy while `uma-ADR-0012` is curvature-preserving speed regulation. It has no
short prefix here; **cite a workspace ADR by full link**, as
[ADR-0002](0002-bathymetric-data-store.md) already does. Naming it in prose
("the workspace's ADR-0012") is not sufficient qualification under this rule.

## Context

The displays must navigate datasets far larger than memory: a 3.6 GB sidescan
store, a global ENC chart corpus, and survey grids that grow every field day.
The governing intent has always been the same, and it is a pixel-budget
argument: a display has a fixed number of pixels, so it should render only at
the resolution those pixels can resolve, load only the data needed to render
that, and anticipate viewpoint changes so navigation stays smooth. Rendering
transiently at a coarser resolution while finer data loads is expected and
acceptable; blocking the view is not.

That intent was never recorded. Each piece built since implements a
locally-correct fragment, and because no document states the shared model, each
new consumer re-derives its own:

- **Three unrelated level-selection metrics.** camp's `selectLodLevel` uses
  ground metres-per-pixel against `gggs::Level::fromCellSize`;
  `MapTiles::paint` uses `levelOfDetailFromTransform` against two magic
  constants; `SonarLiveCacheLayer` has no level selection at all and separates
  detail by draw order.
- **Four unrelated eviction policies.** camp `MapTiles` caps by tile count at
  `max(256, 4 × visible)`; `SonarLiveCacheLayer` budgets bytes with
  farthest-from-viewport ordering and an LRU fallback;
  `cube_bathymetry::GeoMapSheet` caps by count with lossless CUBE reload;
  `marine_bathymetry_store::evictOutside` evicts by spatial window.
- **Two half-solutions in the same directory.** `GggsTileLayer` has view-scale
  level selection, demand-driven load, and a progressive coarse backdrop — and
  no eviction whatsoever. `SonarLiveCacheLayer` has a byte budget,
  view-distance eviction, fold-to-parent so evicted areas degrade rather than
  blank, a protected apex level, and hysteresis-guarded reload — and no level
  selection.
- **No anticipation anywhere.** Both viewport-driven paths gate on "moved since
  last kick" in order to *suppress* redundant work, which is the opposite
  posture from prefetching.
- **The hierarchy is used as a naming scheme, not a structure.**
  `marine_autonomy/gggs/` is a correct quadtree with `parent()`/`children()`
  index math that resolves through geographic centres so it stays valid across
  the polar scale-factor bands, round-trip pinned by tests. Every consumer then
  flattens it: `GggsTileLayer` holds `std::vector<std::unique_ptr<GggsTile>>`
  and linear-scans all tiles at all levels once per frame; the store layers use
  `std::map<GridIndex, …>`, ordered by level-then-row-then-col rather than
  clustered spatially. Nothing descends the tree.

This last point has a root cause worth recording, because it explains the drift
rather than assigning blame. GGGS derives from Ware, Mayer, Johnson, Jakobsson
& Ferrini (2020), *A global geographic grid system for visualizing bathymetry*,
which states plainly: *"our purpose is not primarily to support
multi-resolution rendering, although that might be a side benefit."* GGGS is a
data-organisation hierarchy that is shaped like a rendering LOD ladder. It was
reasonably mistaken for one, and so the rendering half was never built.

Two symptoms have now reached the field. `camp#194` — a region-disjoint native
ladder (the ENC chart store, `uma-ADR-0010` D7) renders only one region band per
zoom, because `selected_level_` was an equality filter over a flat tile list.
`camp#195` — multi-level residency is unbounded, on a layer that has no
within-level eviction either, which is the same shape that OOM'd the salmon
operator station in `SonarLiveCacheLayer` (`camp#153`, the reason
`camp-ADR-0010` exists).

The exposure is widening now, not hypothetically. `s102_import` maps each S-102
dataset's native resolution through `gggs::Level::fromCellSize()` and imports at
the matching level — correct behaviour, and the only behaviour S-102 permits,
since Edition 3.0.0 (December 2024) *deleted* its multi-resolution gridding
annexes. The consequence is that `reference/` is becoming a heterogeneous-level,
region-disjoint layer, and `uma-ADR-0010` D9 generates no overview pyramid for
`reference` by design. The Appledore 1 m MLLW grid (#314) lands on top of that,
in the layer the operator station will be panning during the survey.

## Decision

### D1 — Screen-space error is the selection metric, and it is the only one

Every tile carries a **geometric error in metres**: the error introduced if
that tile is rendered and its children are not. Selection compares that error,
projected to pixels, against a single threshold `τ` (pixels). Refine when the
projected error exceeds `τ`; render when it does not.

The projection operator is the **only** thing that differs between views:

- **Perspective (3D):** `SSE = ε · H_px / (d · 2·tan(fov_y / 2))`
- **Orthographic (2D top-down):** `SSE = ε / ground_metres_per_pixel`

where, for the tile being tested:

| Symbol | Meaning | Units |
|---|---|---|
| `ε` | the tile's geometric error (D2) | metres |
| `SSE` | screen-space error, compared against `τ` | pixels |
| `H_px` | viewport height | pixels |
| `d` | distance from the eye to the tile's bounding volume | metres |
| `fov_y` | vertical field of view | radians |
| `ground_metres_per_pixel` | ground sample distance at the tile's latitude | metres/pixel |

In the orthographic case the distance term vanishes entirely. This is not a
convenience: it is what makes one stored error and one user-facing knob serve
both a chart view and a 3D scene. Cesium implements exactly this split, and
Ulrich's chunked-LOD `K·δ/D` and Lindstrom & Pascucci's `λ·ε/d` are the same
quantity under different names.

`τ` is the single user-facing quality knob and belongs in the UI. For
calibration: Nanite runs 0.5–1 px, Lindstrom & Pascucci's demos 2–4 px, Cesium
defaults to 16 px for streamed web tilesets. A displayed, adjustable `τ` is also
the cheapest field lever on a marginal laptop.

**Consequence for camp:** `selectLodLevel` already computes
`mercator_mpp × metersPerUnit(lat)` — that *is* the orthographic denominator.
What is missing is the numerator. camp has the projection operator and no error
metric, and substitutes "which level has cells about one pixel wide," which is
correct for a nested pyramid and wrong for a native ladder.

### D2 — Error nesting is a producer obligation, not a renderer's problem

Top-down refinement is only correct if errors are **saturated**: a tile's error
is at least the maximum of its descendants' errors,

```
ε_i = ε̂_i                                  (leaf)
ε_i = max(ε̂_i, max over children ε_j)      (interior)
```

together with a bounding volume that provably contains every descendant's
content. Without this, a descendant's error can project from a point nearer the
eye than its parent's and refinement silently produces the wrong cut. This is a
property of the data, so the **writers** must bake it: `overview_builder`, the
depth pyramid builder, `s102_import`, and `s57_to_geotiff` each record a
per-tile geometric error when they emit a tile.

Nesting is not optional and not enforceable at read time without reading
everything. Producers that cannot compute a meaningful error must record a
conservative upper bound rather than omit the field.

### D3 — Coverage is declared, not discovered

A layer declares the set of `(level, index)` zones it actually holds — a
**coverage manifest** — at mixed levels, in one object. Consumers read the
manifest; they do not infer coverage from a directory scan.

This is the decision that makes a region-disjoint ladder and a nested pyramid
*the same model*. A conventional pyramid is the degenerate case where the
coverage set is ancestrally closed. The ENC chart ladder, a mixed-level
`reference/`, and a `draft/` layer with a generated sidecar are all just
different coverage sets over the same GGGS.

The shape is IVOA's Multi-Order Coverage map: cells at mixed orders in one set,
with cheap set operations between coverages. Astronomy adopted it for exactly
this problem — surveys of wildly differing footprint and depth over one
browsable hierarchy — and pairs it with HiPS so that the tile pyramid and the
coverage declaration are two separate artifacts. When a wire format is needed,
OGC 2D TMS 2.0 `TileMatrixSetLimits` is the normative expression (*"If a
tileMatrix identifier is not mentioned, it should be interpreted as a tileMatrix
that is not available"*), and Cesium's `layer.json` `available` array is the
widely-implemented equivalent that additionally allows several rectangles per
level.

**A manifest is authoritative only for the artifact that wrote it, and only
about *which zones*, never about *what is in them*.** A derived manifest
describes the derived tiles its producer owns. It is not a statement about the
layer as a whole, and a consumer must never read one as though it were.

Two failure modes follow, and they are not the same:

- **Coverage staleness** — the manifest names a different set of zones than the
  artifact now holds.
- **Content staleness** — the zones are right but the data behind them was
  derived from source tiles that have since changed.

A producer that writes its manifest *into the same atomic publish as the tiles
it describes* eliminates the first for its own artifact: the two cannot
disagree, because they land together or not at all. It does nothing about the
second. Regeneration is operator-invoked — nothing re-runs a pyramid builder
after an import — so a derived manifest can be arbitrarily stale in content
while remaining perfectly accurate about coverage, and nothing in the document
reveals it. **Producers must therefore record enough provenance to make
disagreement with their source detectable** (a generation time and a source
tile count at minimum). Until a manifest carries that, a consumer cannot
distinguish "current" from "long superseded" and must not present derived data
as though it were freshly sourced.

**The permitted error direction is under-reporting.** A consumer may conclude
from a manifest *at least this much data is here*; it may never conclude *and
nothing else is*. Absence of a zone is not evidence of absence of data —
it may equally be a manifest that predates the data. Over-reporting is the
dangerous direction and must be prevented where it is cheap to prevent: at
write time, in the same publish as the tiles.

**A partial read is not a successful read.** A reader that recovers only some of
a manifest — a truncated document, a run it had to skip, a declared extent it
had to refuse — has learned a *narrower* coverage than the artifact claims, and
returning that as a complete answer converts a corrupt file into a confident
lie about what a layer holds. Complete, partial, and absent must be
distinguishable at the API boundary, and **partial takes the same path as
absent**: fall back to the directory scan, which is always correct if slower.
The scan is the floor this decision rests on; a manifest is an optimisation
over it, never a replacement for it.

None of this reaches a safety answer. Per D8 no query path consults a manifest
or an overview level, so the worst outcome of a stale, partial, or absent
manifest is a display that is coarser, blanker, or more out-of-date than it
needed to be — never a wrong depth.

**Corollary — coverage gaps are filled from coarser levels, and the fill is
marked.** Where the selected level has no coverage, a coarser level's data may
be drawn in its place. This matches how ECDIS has behaved for decades (S-52
clause 3.1.7: fill the uncovered part of the display from a more general
navigational purpose, draw everything at one scale, and flag anything shown at
two or more times its compilation scale as overscale). Our displays owe the same
honesty: a coarse fill under a fine selection is not the same product, and the
UI must be able to say so.

### D4 — Residency is budgeted, and pressure relaxes quality rather than thrashing

Each layer has an explicit residency budget. Under pressure the system **raises
`τ`** — degrading the quality target — instead of evicting more aggressively or
failing. This is a closed loop: memory pressure feeds back into the selection
metric, so the display degrades visibly and predictably rather than churning.

Eviction is a hybrid of view-state and history: tiles selected in the current
frame are protected regardless of recency, and everything else is LRU. LRU alone
retains a useless working set after a viewpoint jump; distance-only ordering
discards history along a path being traversed. The sentinel-node list structure
makes "cannot evict what this frame selected" a structural property rather than
a checked condition, at O(1) per operation.

Budget units are per-layer. Byte budgets are required where payload size varies
by orders of magnitude; a **count** budget is legitimate and cheaper to reason
about for fixed-size tiles, and GGGS tiles are a fixed 960×960 — so camp's
`MapTiles` and `cube`'s `GeoMapSheet` are not wrong to count. What is not
acceptable is having no budget at all.

The working set is `{tiles selected this frame} ∪ {tiles in the prefetch
horizon}`. The budget must exceed the first set or the system thrashes by
construction; headroom above it is what funds the second.

### D5 — The frame never blocks, and the picture never gets worse

Two guarantees, both defined in terms of what was rendered last frame:

- **Descending (zoom in):** a subtree's finer selection is abandoned in favour
  of an already-resident ancestor only when some selected descendant is not yet
  renderable **and** none of them rendered last frame. The second condition
  matters: once fine detail is on screen, a transient load stall must not snap
  the view back to coarse.
- **Ascending (zoom out):** still-resident finer tiles keep drawing until the
  coarser selection's visible set has finished loading.

Every level must always have a renderable answer, by upsampling from a resident
ancestor if necessary. "There is always a valid, if blurry, picture" is an
invariant, not an optimisation.

Refinement work is **interruptible and priority-ordered** against a per-frame
budget: running out of time degrades quality, never drops the frame.

camp already implements a field-verified fragment of the ascending guarantee
(the backdrop retention in `itemsIntersecting`, verified 2026-07-31) without the
second condition on the descending side. That fragment is correct and should be
generalised, not replaced.

### D6 — Anticipation is required behaviour, but predictors are earned by measurement

Navigation should be smooth because the data is usually already there. The
ordering rule is cheap and should be adopted regardless: prioritise by
`(1 − dot(tile_direction, view_direction)) × distance`, so nearer and more
screen-central tiles load first, in tiers (urgent / current view / speculative).
Requests for tiles that have left the view are cancelled, and a cancelled tile
is marked as needing reload so that aggressive cancellation is safe by
construction.

A **speculative** prefetch horizon is sized as camera speed × p95 load latency,
widened by heading uncertainty. But it must be justified by measurement first:
the geometry-clipmaps authors declined to prefetch at all on the grounds that
their update granularity made latency irrelevant, and an interruptible,
priority-ordered loader captures most of the smoothness for a fraction of the
complexity. **Measure tile load latency before building a predictor.** Prefetch
depth beyond the measured horizon wastes bandwidth and, worse, evicts tiles that
are still needed.

For a top-down map the dominant predictor is pan direction plus zoom-to-centre;
a one-tile ring around the viewport plus the children of the centre tiles at the
next level captures most of the available benefit.

### D7 — One selection core; renderers are thin adapters

The coherence mechanism is **shared code, not a shared document**. A
renderer-agnostic library owns tree traversal, the screen-space-error test with
a pluggable projection, frustum culling, the request scheduler, and the
budgeted cache. It has no Qt, no OpenGL, no Ogre, and no ROS. Given a coverage
manifest, a view state, and a budget, it answers: draw these, load these next,
evict these.

Renderers implement a narrow prepare/upload/free interface and put pixels on the
screen. A document alone would drift again — nothing forces an agent working in
one repo to read another repo's ADRs — whereas a linked library enforces the
model at compile time.

The core's shape deliberately mirrors `cesium-native`'s (geometric error in
metres, a view-state abstraction with an orthographic constructor, a
byte-budgeted cache with current-frame protection), so that adopting it later
replaces the core while leaving the store contract and the renderers intact.
See "Alternatives considered".

### D8 — Safety queries never consult an LOD level

An LOD hierarchy answers "what should I draw," not "what is the least depth
here." Truncated traversal of an additive hierarchy yields a uniform-density
*thinning*, never a per-cell extremum — correct for display, wrong for extrema.
Any shoal-finding, least-depth, or clearance query must read to the finest
available level for the region, and `shallowestReliable()` must continue to
scan all layers and all levels.

The one thing that makes a coarse tile safe to *consult* is a
shallowest-preserving fold, which `uma-ADR-0010` D9 already mandates for depth
(never mean). That decision now has an explicit rationale: it is what allows a
coarse depth level to be conservative rather than merely representative. The
imagery MEAN fold of [ADR-0011](0011-overview-pyramid.md) §4 carries no such
guarantee and must never back a safety query.

## Consequences

- **Store writers gain two additive outputs**: a per-tile geometric error and a
  coverage manifest. Both are derivable from what is already on disk. **No
  migration is required** — a layer without them is readable, and a consumer
  falls back to today's level-as-resolution behaviour until they appear.
- **Manifest writers owe provenance; manifest readers owe a partial-read
  signal.** D3's trust rules are not satisfied by the first implementation
  ([#331](https://github.com/rolker/unh_marine_autonomy/issues/331)), which
  publishes the derived manifest atomically with its tiles — closing coverage
  staleness for that artifact — but records no generation time or source tile
  count, and returns a manifest object from a document whose runs it had to
  skip or truncate. Both gaps are tracked by
  [#334](https://github.com/rolker/unh_marine_autonomy/issues/334); until they
  close, the scan fallback is the only fully correct read, and no consumer
  should be written that cannot fall back to it.
- **camp converges its two half-solutions.** `GggsTileLayer` gains
  `SonarLiveCacheLayer`'s budget and eviction; `SonarLiveCacheLayer` gains
  level selection. `camp-ADR-0013` and `camp-ADR-0010` become implementations
  of D1–D5 and need cross-reference addendums (permitted under the workspace's
  [ADR-0012](https://github.com/rolker/ros2_agent_workspace/blob/main/docs/decisions/0012-permit-cross-reference-addendums-in-adrs.md)
  without superseding), not rewrites.
- **camp's flat tile vector becomes a level-bucketed spatial index.** The
  per-frame cost stops scaling with store size and starts scaling with what is
  visible. `camp#195` is re-scoped from "bound multi-level residency" to D4 in
  full — viewport-scoped retention with a budget, independent of level count,
  since the unbounded-pan axis pre-dates multi-level compositing and already
  applies to a single large layer.
- **`cube_bathymetry` is a producer here, not a consumer.** Its job is to emit
  D1/D2/D3 metadata when it writes tiles. Its own `GeoMapSheet` eviction is a
  *compute* working set with lossless CUBE reload — a different problem from a
  display working set — and its ADR-0001 stands unchanged.
- **`reference/` needs coverage-aware rendering or a relaxation of
  `uma-ADR-0010` D9.** This is the near-term field exposure (#314, #316) and is
  not solved by this ADR alone.
- **`rqt` is out of scope.** Its sonar and bathymetry widgets render single
  messages rather than tiled stores. The genuinely shared piece,
  `marine_colormap`, is unaffected.
- **The 3D renderer is deferred and unconstrained by this ADR.** D1's projection
  split is what keeps both paths open. Note that rviz2's vendored Ogre is built
  with `OGRE_THREAD_SUPPORT 0` (its own paging degrades to blocking loads on the
  GUI thread) and is float32 throughout (geodetic coordinates jitter —
  ros2/rviz#1012), so any rviz2 path needs a floating-origin scheme and its own
  loader. A GeoZui4D-descended viewer starts from a better position: `bagViewer`
  already implements the perspective projection operator of D1 as
  `lod = floor(log2(ncols / sizeInPixels))`, with frustum culling and level
  selection as one computation.
- **GGGS is described as WorldCRS84Quad-compatible, not as a DGGS.** It is a
  congruent, strictly nested lat/lon quadtree with a 2×1 root — the OGC-
  registered `WorldCRS84Quad` tile matrix set, and the same convention as
  Cesium's `GeographicTilingScheme`. It is not equal-area, so it sits outside
  OGC Topic 21's Equal-Area Earth DGGS package. Claiming DGGS conformance would
  be a stretch; claiming TMS compatibility is accurate and buys tooling interop.

## Alternatives considered

**Leave the model implicit and per-repo.** Rejected: this is the status quo that
produced three selection metrics, four eviction policies, and two symptoms in
the field. Each repo's ADRs are locally coherent and collectively silent.

**Adopt `cesium-native` now as the shared core.** Deferred, not rejected. It
implements D1 (including an orthographic `ViewState`), D4 (`maximumCachedBytes`
with per-frame amortised unloading and a memory-adjusted error target), D5
(both substitution rules), and D6, behind a five-method renderer interface, with
a public `TilesetContentLoader` that a GGGS store could implement without
forking. osgEarth deleted its own 3D Tiles implementation and adopted it, which
is the strongest available signal. The costs are real: roughly thirty vcpkg
dependencies with no selection-only build, C++20 against a C++17 distribution,
no API stability guarantee pre-1.0, and an `ament_vendor` wrapper to write and
maintain. **This ADR is deliberately shaped so that adopting it later replaces
the core and preserves the store contract and renderers.** Nothing here should
be built in a way that forecloses it.

**Adopt OGC 3D Tiles as the store format.** Rejected for our data. There is no
gridded-coverage or heightfield content type in 1.0 or 1.1; a bathymetric grid
can only be meshed into glTF, losing row/column structure and per-cell nodata
semantics. The tell is that Cesium's own marine product, Cesium World
Bathymetry, ships as quantized-mesh — a Cesium community format with no OGC
document number — rather than as 3D Tiles. There is also no GDAL driver. What is
worth borrowing is the *model*: error in metres as the only selection knob, and
the separation of tile availability from content availability.

**Use COG internal overviews to carry the levels.** Structurally impossible for
a region-disjoint ladder. OGC 21-026 Requirement 6 states that reduced-
resolution subfiles carry no georeferencing of their own and share the
full-resolution origin, with resolution *derived* from the pixel-count ratio.
Extent is defined identical by construction; there is nowhere to record a
different footprint. This independently validates
[ADR-0011](0011-overview-pyramid.md)'s rejection of `BuildOverviews`, for a
second reason beyond the per-file and per-band ones recorded there.

**Wait for a hydrographic standard to specify this.** Rejected as a plan, though
worth tracking. S-102 deleted its multi-resolution annexes in Edition 3.0.0;
S-101 Edition 2.0.0 abolished usage bands for display in favour of per-dataset
scale ranges but defers the normative selection algorithm to S-98 Annex C
Appendix C-5, marked *in development*. Anything we build now is ahead of the
standard rather than behind it. Note also that S-100 Part 8 already specifies a
Morton-ordered, sparse, hole-tolerant quadtree as DCF 6 *variable cell size*,
described as *"particularly useful for hydrographic data"* — and no product
specification uses it. If a standards hook is ever wanted for a native
multi-resolution grid, that slot is unclaimed.

## References

- Ware, Mayer, Johnson, Jakobsson & Ferrini (2020), *A global geographic grid
  system for visualizing bathymetry*, Geosci. Instrum. Method. Data Syst. 9(2),
  375–384. [doi:10.5194/gi-9-375-2020](https://doi.org/10.5194/gi-9-375-2020) —
  the GGGS foundation, including its explicit disclaimer of rendering LOD as a
  goal.
- Lindstrom & Pascucci (2002), *Terrain Simplification Simplified*, IEEE TVCG
  8(3) — error saturation and the nested bounding-volume condition (D2).
- Ulrich (2002), *Rendering Massive Terrains using Chunked Level of Detail
  Control*, SIGGRAPH course notes — the portable `K·δ/D` form of D1.
- Losasso & Hoppe (2004), *Geometry Clipmaps*, ACM TOG 23(3) — bounded per-frame
  update work, and the argument against premature prefetching (D6).
- OGC 17-083r4, *Two Dimensional Tile Matrix Set and Tile Set Metadata* 2.0 —
  `TileMatrixSetLimits`, the normative wire form of D3.
- IVOA *Multi-Order Coverage map* 2.0 (2022) — mixed-order coverage sets (D3).
- OGC 21-026, *Cloud Optimized GeoTIFF* 1.0, Requirement 6 — why COG cannot
  carry a region-disjoint ladder.
- IHO S-52 Edition 6.1.1 clause 3.1.7, and S-57 Appendix B.1 clause 2.2 — the
  band-select, gap-fill, mark-overscale rule behind D3's corollary.
- IHO S-100 Edition 5.2.0 Part 8 §8-5.2.5 — DCF 6 variable cell size.
- Calder & Ladner (2022), *A Variable Resolution Grid Extension for BAG Files*
  v1.2.1 — the two-level nested-pyramid model, and its backward-compatibility
  design.
