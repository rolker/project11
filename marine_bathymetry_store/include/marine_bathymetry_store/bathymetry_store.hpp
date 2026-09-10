// Copyright 2026 Center for Coastal and Ocean Mapping & NOAA-UNH Joint
// Hydrographic Center, University of New Hampshire
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef MARINE_BATHYMETRY_STORE__BATHYMETRY_STORE_HPP_
#define MARINE_BATHYMETRY_STORE__BATHYMETRY_STORE_HPP_

#include <array>
#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "geographic_msgs/msg/geo_point.hpp"
#include "marine_autonomy/gggs.h"
#include "marine_bathymetry_store/bathy_cell.hpp"
#include "marine_bathymetry_store/bathymetry_tile.hpp"

namespace marine_bathymetry_store
{

class BathymetryStore;
struct StoreMetadata;
// Persistence free functions (defined in tile_io.cpp). Forward-declared here so
// the store can friend them: `load` / `loadWindow` must reach getOrCreateTile on
// any layer — including the read-only `Reference` prior — which the public API
// otherwise forbids.  `evictOutside` must reach the non-const layerMap() to erase
// tiles without const_cast.
std::size_t save(
  BathymetryStore & store, const std::string & dir, const StoreMetadata * metadata);
std::size_t load(
  BathymetryStore & store, const std::string & dir, StoreMetadata * metadata);
std::size_t loadWindow(
  BathymetryStore & store, const std::string & dir,
  const geographic_msgs::msg::GeoPoint & min_pt,
  const geographic_msgs::msg::GeoPoint & max_pt,
  StoreMetadata * metadata);
std::size_t evictOutside(
  BathymetryStore & store,
  const geographic_msgs::msg::GeoPoint & min_pt,
  const geographic_msgs::msg::GeoPoint & max_pt);

/// @brief Result of a `clearOverlappedDraft` call (ADR-0010 D8 anti-clobber).
///
/// `tiles_touched` is the display-cache invalidation seam (ADR-0008; camp#171/#172):
/// the `Draft` tiles that had ≥1 cell cleared (each appears once, ascending). Nothing
/// is removed on disk under cell-wise clearing — a cleared cell is written no-data
/// (NaN) in place and persisted through the normal dirty-tile save path — so these
/// tiles are **touched** (dirtied), not deleted.
struct DraftClearResult
{
  /// `Draft` cells transitioned from data → no-data by the clear.
  std::size_t cells_cleared = 0;
  /// `Draft` tiles with ≥1 cleared cell (each once, ascending). Cache-invalidation seam.
  std::vector<gggs::GridIndex> tiles_touched;
  /// **Distinct** `Draft` cells **coarser** than the processed data that overlap
  /// it but were deliberately kept, because the processed data does not fully
  /// supersede them: part of the draft cell's ground lies outside every processed
  /// tile in this call, or a processed cell under it holds no data. Keeping them
  /// is the shoal-safe direction (a retained draft blunder is a false alarm; a
  /// wrongly-cleared draft cell is a lost hazard), but the residue is real, so it
  /// is **counted rather than silent** — a caller can see that its imports still
  /// owe the rest of that draft cell's ground.
  ///
  /// Counted once per draft cell, not once per processed tile that saw it: the
  /// coverage decision is taken against every tile in the call at once (see
  /// `clearOverlappedDraft`), so a cell reported here is one the whole call
  /// genuinely does not supersede. 0 whenever `draft` is at or finer than every
  /// processed level in the call — with a fixed-level `draft` and a
  /// depth-adaptive `processed` (uma#369) that is the *deep* end of the ladder;
  /// where `processed` goes finer than `draft` (levels 11-14, shallow water) the
  /// coarse path is live.
  ///
  /// **Expect this to be large in shallow water, and expect `draft/` not to
  /// shrink there** (round 3). `draft` stays fixed-level while `processed` goes
  /// to 11-14, so coarse-draft-under-finer-processed is the NORMAL case for a
  /// shallow survey, not an edge residue — and supersession requires *every* one
  /// of the 16-256 fine cells under a draft cell to hold data, while a CUBE
  /// `processed` surface is gappy by construction (gated-drop holes,
  /// between-lines gaps). One hole keeps the whole draft cell. The direction is
  /// shoal-safe and the count makes it visible, but a caller should not read a
  /// large residue as a malfunction, and a draft blunder over ground the re-run
  /// does speak for can keep winning `shallowestReliable` until the draft cell is
  /// fully covered.
  ///
  /// It can also read *lower* than a naive expectation, and that is correct: a
  /// draft cell at or finer than the processed level covering it is decided by a
  /// single containing cell, so a gated-drop hole there is an ordinary hole, not
  /// coarse residue, and is not counted.
  std::size_t coarse_draft_cells_retained = 0;
};

/// @brief In-memory, GGGS-tiled, multi-layer, multi-level bathymetric store.
///
/// Holds, per `SourceLayer`, **one fused** map of tiles keyed by
/// `gggs::GridIndex` (which itself carries its level). The per-day epoch
/// dimension of ADR-0002 Amendment A1 was dropped (#221): a UTC calendar day is
/// a weak proxy for "a survey" and the single-coverage use case does not need
/// change detection, so each layer is one surface again. The store is
/// **level-agnostic**: tiles at heterogeneous GGGS levels coexist within a layer
/// (ADR-0002 §D2, amendment #151/#153). The constructor's `gggs_level` is
/// retained only as a **default for `cellIndex(lat,lon)`** (the write/query
/// convenience that turns coordinates into a cell) — it is *not* an invariant on
/// stored tiles.
///
/// Source priority is a **non-destructive query-time overlay** (see
/// `query.hpp`): the layers are independent, so a live CUBE (`Draft`) write never
/// clobbers the read-only prior. The single fused surface within a layer is
/// last-write-wins per cell (there is no provenance ordering); `processed > draft
/// > reference > chart` layer priority is the only provenance axis (ADR-0010 D8
/// re-split `survey` into `Processed`+`Draft`; `Chart` added #275 per ADR-0010
/// D3/D7, lowest under the D4 placeholder order). Cross-layer anti-clobber (a
/// `Processed` write clears overlapped `Draft` cells, cell-wise) is a public store
/// operation (`clearOverlappedDraft`): the store owns the semantics so every
/// `Processed` producer applies them identically — the GeoTIFF importer
/// (`geotiff_import.hpp`) and cube's replay/regen paths that write processed tiles
/// directly via `saveTile`. It is deliberately separate from this query overlay.
///
/// This phase has no ROS interface or distribution — just storage, in-process
/// queries (`query.hpp`), per-tile GeoTIFF persistence (`tile_io.hpp`), and the
/// GeoTIFF importer (`geotiff_import.hpp`).
class BathymetryStore
{
public:
  /// @brief Construct a store with a default level of GGGS quadtree
  ///        @p gggs_level.
  ///
  /// The default level only governs `cellIndex(lat,lon)` (the coordinate→cell
  /// convenience); stored tiles may be at any level (this store is multi-level,
  /// ADR-0002 §D2).
  ///
  /// @param reference_writable Opt in to per-cell `set()` writes on the
  ///   read-only `Reference` prior layer. Default `false` — only the prior
  ///   importer (which converts the contour prior to ellipsoidal at import)
  ///   should pass `true`. Runtime consumers leave it `false` so live CUBE ingest
  ///   can never mutate the prior. `load()` (a friend) may still populate
  ///   Reference from disk regardless of this flag — the read-only gate is on
  ///   per-cell mutation (`set`), not on loading the prior. Direct tile mutation
  ///   is impossible from outside: `getOrCreateTile` is private (persistence-only).
  /// @param chart_staging_writable Opt in to writes on the `Chart` layer
  ///   (ADR-0010 D3/D7). Default `false` — `Chart` is writable only by the
  ///   wholesale-regeneration workflow: a **staging** store passes `true`,
  ///   imports the exported chart tiles, saves to a staged directory, and the
  ///   updater swaps it in via `replaceChartLayer`. Runtime stores never write
  ///   Chart directly; as with Reference, `load()` may still populate the
  ///   layer from disk (the gate is on mutation, not loading).
  /// @throws std::out_of_range if the level is invalid (via gggs::Level).
  explicit BathymetryStore(
    uint8_t gggs_level, bool reference_writable = false,
    bool chart_staging_writable = false)
  : level_(gggs_level), reference_writable_(reference_writable),
    chart_staging_writable_(chart_staging_writable) {}

  /// @brief Construct a store whose **default** level is the coarsest GGGS level
  ///        whose cells are no larger than @p cell_size_m (clamped to level 20).
  static BathymetryStore fromCellSize(
    float cell_size_m, bool reference_writable = false,
    bool chart_staging_writable = false)
  {
    return BathymetryStore(
      gggs::Level::fromCellSize(cell_size_m).level(), reference_writable,
      chart_staging_writable);
  }

  /// @brief Whether per-cell `set()` may write the read-only `Reference` layer.
  bool referenceWritable() const noexcept {return reference_writable_;}

  /// @brief Whether writes may target the `Chart` layer (staging stores only).
  bool chartStagingWritable() const noexcept {return chart_staging_writable_;}

  /// @brief The store's **default** level (used by `cellIndex(lat,lon)` only).
  ///
  /// Stored tiles are not pinned to this level — the store is multi-level
  /// (ADR-0002 §D2). This is the level the coordinate convenience resolves at.
  const gggs::Level & level() const noexcept {return level_;}

  /// @brief CellIndex at this store's **default** level for a geographic position.
  ///
  /// Convenience for callers who think in coordinates. The returned CellIndex is
  /// at the default level; callers wanting a specific level should build the
  /// CellIndex from a `gggs::Level` of their choosing (the store accepts any).
  gggs::CellIndex cellIndex(double latitude, double longitude) const
  {
    return level_.cellIndex(gggs::geoPoint(latitude, longitude));
  }

  /// @brief Write @p value into @p layer at @p cell.
  ///
  /// Creates the backing tile on first write. The cell may be at **any** valid
  /// GGGS level — the store is multi-level (ADR-0002 §D2). Within a layer the
  /// surface is **last-write-wins** per cell: a second write to the same cell
  /// overwrites the first (there is no per-day epoch ordering since #221).
  /// @return `true` always (no provenance guard rejects a write). The bool
  ///         return is retained for source/API stability with the prior epoch
  ///         model and possible future write gates.
  /// @throws std::invalid_argument if @p cell is invalid.
  /// @throws std::logic_error if a write-gated prior's gate is shut: `Reference`
  ///   on a store not constructed `reference_writable` (the prior is read-only,
  ///   ADR-0002 §D3), or `Chart` on a store not constructed
  ///   `chart_staging_writable` (Chart is writable only via the wholesale
  ///   regeneration workflow, ADR-0010 D7).
  bool set(SourceLayer layer, const gggs::CellIndex & cell, const BathyCell & value);

  /// @brief Read the raw cell of one @p layer (no priority overlay).
  /// @return The cell, or `std::nullopt` if the cell's tile is absent.
  ///         A returned cell may still be no-data (`!hasData()`).
  std::optional<BathyCell> get(SourceLayer layer, const gggs::CellIndex & cell) const;

  /// @brief Bulk-insert @p tiles into @p layer (importer path).
  ///
  /// Merges the supplied tile map into the layer's single tile map: each grid's
  /// tile replaces any tile already resident at that grid, and grids not in
  /// @p tiles are left untouched (no wholesale clear — there is no epoch to
  /// supersede). Every inserted tile is marked dirty so it persists. The
  /// `Reference` and `Chart` write gates apply, identical to `set()`. Tiles may
  /// be at heterogeneous levels (multi-level, ADR-0002 §D2).
  ///
  /// @note Additive-merge footgun: because this never clears and `save()` never
  ///   deletes on-disk tiles, a *shrinking* re-import (a corrected coverage with
  ///   fewer grids than a prior import) does NOT remove the now-orphaned `.tif`
  ///   files — `load()` will resurrect them. This matches the deliberate
  ///   single-fused-grid merge contract (#221), but a true replace requires
  ///   clearing the on-disk layer dir first. A `--replace` import path is a
  ///   future addition if/when corrected re-imports become a workflow.
  /// @return The number of grids inserted/replaced.
  /// @throws std::invalid_argument if any grid key is invalid or a tile's own
  ///   GridIndex does not match its map key.
  /// @throws std::logic_error if a write-gated prior's gate is shut: `Reference`
  ///   without `reference_writable`, or `Chart` without `chart_staging_writable`
  ///   (writable only via the wholesale regeneration workflow, ADR-0010 D7).
  std::size_t importTiles(
    SourceLayer layer, std::map<gggs::GridIndex, BathymetryTile> tiles);

  /// @brief Clear overlapped `Draft` cells where @p processed_tiles has data
  ///        (ADR-0010 D8 cross-layer anti-clobber).
  ///
  /// The store owns this semantics so **every** producer of `Processed` data applies
  /// it identically: the GeoTIFF importer (`importGeoTiff`, which calls this after
  /// building its tile map) and cube's replay/regen paths that write processed tiles
  /// directly via `saveTile` and bypass the importer. Call it with the processed
  /// tiles in hand — the single-tile overload lets a live/replay writer clear
  /// incrementally right after each direct `saveTile`, or pass a whole tile-map for a
  /// bulk import.
  ///
  /// For each processed grid the `Draft` layer **already holds a tile for** (this
  /// never creates a spurious empty draft tile), clears exactly the `Draft` cells
  /// that (a) the processed data populated **and** (b) `Draft` currently holds data
  /// at, by writing them no-data via `set(SourceLayer::Draft, …, {})` — persisted
  /// through the normal dirty-tile save path (there is no tile/cell-erase API and
  /// `save()` never deletes on-disk tiles). A processed **no-data** cell (a
  /// gated-drop hole) leaves the overlapping draft cell intact: harmless under
  /// `Processed > Draft` and strictly more coverage than clearing by footprint, so
  /// stale gap-striping never accumulates under the authoritative surface.
  ///
  /// **Level-aware (uma#369).** `processed` is depth-adaptive and mixed-level
  /// while `draft` stays fixed-level, so the two no longer share a level and a
  /// clear keyed on the processed tile's own `GridIndex` would match no draft
  /// tile, clear nothing, and say nothing. The clear instead walks **every GGGS
  /// level the `draft` layer holds**, and per draft cell:
  /// - `draft` at or finer than `processed`: the draft cell lies inside exactly
  ///   one processed cell (levels nest exactly), which decides it.
  /// - `draft` **coarser** than `processed`: the draft cell is cleared only when
  ///   the processed data fully supersedes it — every point of its ground is
  ///   covered by a processed cell that has data. Otherwise it is kept (clearing
  ///   would discard draft data over ground this call does not speak for) and
  ///   counted in `DraftClearResult::coarse_draft_cells_retained`, so the residue
  ///   is reported rather than silent.
  ///
  /// **The coverage decision spans the whole call, not one tile at a time.**
  /// A coarse draft cell whose ground is split across several processed tiles —
  /// straddling a tile seam, or split between a shallow level-13 tile and its
  /// deeper level-12 neighbour — is superseded by their **union** and is
  /// cleared, even though no single tile covers it. Deciding tile-by-tile would
  /// retain it for every tile that saw part of it, which both keeps a superseded
  /// draft blunder winning `shallowestReliable` and reports the same cell in
  /// `coarse_draft_cells_retained` once per tile. The retained count is
  /// therefore a count of **distinct draft cells**, and a cell counted by the
  /// map overload is one that this whole import genuinely does not supersede.
  /// (The single-tile overload sees a set of one, so for an incremental writer
  /// the union is exactly that tile — a draft cell its neighbours will complete
  /// is retained now and cleared when they arrive.)
  ///
  /// This mutates only `Draft` — an ungated, freely-writable layer — so it needs no
  /// write-gate opt-in; `Reference`/`Chart` are never touched.
  ///
  /// @return The cells cleared and the draft tiles touched (cache-invalidation seam).
  /// @throws std::invalid_argument if any processed tile carries an invalid
  ///   `gggs::GridIndex`. Validation happens up front, before any `Draft` cell is
  ///   written, so a throw from *this* overload leaves `Draft` untouched.
  DraftClearResult clearOverlappedDraft(
    const std::map<gggs::GridIndex, BathymetryTile> & processed_tiles);

  /// @brief Single-tile overload of `clearOverlappedDraft` for incremental,
  ///        per-tile callers (a live/replay writer clearing draft after each direct
  ///        `saveTile` of a processed tile).
  ///
  /// @p processed_tile is treated as keyed by its own `index()` grid. Semantics are
  /// identical to the tile-map overload — both delegate to one implementation
  /// that sees the whole set at once; a single tile is a set of one.
  ///
  /// @throws std::invalid_argument if @p processed_tile carries an invalid
  ///   `gggs::GridIndex`, before any `Draft` cell is written.
  ///
  /// @note An incremental caller that clears after each `saveTile` gets the
  ///   single-tile union, which is narrower than a bulk import's: a coarse draft
  ///   cell whose remaining ground a *later* tile will cover is retained now and
  ///   cleared when that tile arrives. That is the shoal-safe order — nothing is
  ///   cleared before something supersedes it — but it means
  ///   `coarse_draft_cells_retained` from an incremental sequence is a running
  ///   view, not a final one.
  DraftClearResult clearOverlappedDraft(const BathymetryTile & processed_tile);

  /// @brief A layer's tiles, keyed by `gggs::GridIndex` (ascending). Empty map
  ///        if the layer holds no data.
  const std::map<gggs::GridIndex, BathymetryTile> & tiles(SourceLayer layer) const
  {
    return layerMap(layer);
  }

private:
  // Persistence must reach getOrCreateTile to populate any layer (including the
  // read-only `Reference` prior) from disk and to clear dirty flags after a
  // save, so the free functions in tile_io.cpp are friends. With getOrCreateTile
  // private, this is what makes the Reference read-only guarantee hold by
  // construction, not just by convention: the only public per-cell mutator is
  // `set` (which gates `Reference`), and external code cannot obtain a mutable
  // tile. `loadWindow` needs the same private access as `load`; `evictOutside`
  // needs the non-const layerMap() to erase tiles without const_cast.
  friend std::size_t save(
    BathymetryStore & store, const std::string & dir, const StoreMetadata * metadata);
  friend std::size_t load(
    BathymetryStore & store, const std::string & dir, StoreMetadata * metadata);
  friend std::size_t loadWindow(
    BathymetryStore & store, const std::string & dir,
    const geographic_msgs::msg::GeoPoint & min_pt,
    const geographic_msgs::msg::GeoPoint & max_pt,
    StoreMetadata * metadata);
  friend std::size_t evictOutside(
    BathymetryStore & store,
    const geographic_msgs::msg::GeoPoint & min_pt,
    const geographic_msgs::msg::GeoPoint & max_pt);

  /// @brief The shared implementation of both `clearOverlappedDraft` overloads.
  ///
  /// @p processed is every processed tile the call was given, as non-owning
  /// pointers (the single-tile overload passes one). The decision for each draft
  /// cell is taken against the **whole set at once**, never tile-by-tile — see
  /// `clearOverlappedDraft`'s contract for why that matters for a coarse draft
  /// cell, and for the double-count it removes.
  DraftClearResult clearOverlappedDraftImpl(
    const std::vector<const BathymetryTile *> & processed);

  /// @brief Find or create the tile for @p grid in @p layer.
  ///
  /// Private: the only mutable tile access. Public mutation goes through `set`
  /// (which gates `Reference`) or `importTiles`; persistence reaches this via
  /// friendship. @p grid may be at any valid level (multi-level store, ADR-0002
  /// §D2).
  /// @throws std::invalid_argument if @p grid is invalid.
  BathymetryTile & getOrCreateTile(SourceLayer layer, const gggs::GridIndex & grid);

  std::map<gggs::GridIndex, BathymetryTile> & layerMap(SourceLayer layer)
  {
    return layers_[static_cast<std::size_t>(layer)];
  }
  const std::map<gggs::GridIndex, BathymetryTile> & layerMap(SourceLayer layer) const
  {
    return layers_[static_cast<std::size_t>(layer)];
  }

  gggs::Level level_;
  bool reference_writable_ = false;
  bool chart_staging_writable_ = false;
  std::array<std::map<gggs::GridIndex, BathymetryTile>, source_layer_count> layers_;
};

}  // namespace marine_bathymetry_store

#endif  // MARINE_BATHYMETRY_STORE__BATHYMETRY_STORE_HPP_
