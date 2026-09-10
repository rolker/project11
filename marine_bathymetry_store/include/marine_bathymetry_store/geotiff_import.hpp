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

#ifndef MARINE_BATHYMETRY_STORE__GEOTIFF_IMPORT_HPP_
#define MARINE_BATHYMETRY_STORE__GEOTIFF_IMPORT_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "marine_autonomy/gggs.h"
#include "marine_bathymetry_store/bathymetry_store.hpp"

/// @file
/// @brief Import a depth/uncertainty GeoTIFF into a store layer's single fused
/// surface (ADR-0002 Phase 2, §D4). Footprint fill, datum conversion at import,
/// and lowest-uncertainty contention resolution. Per-cell provenance stamping was
/// dropped in #248 (coarse provenance is store-level now, ADR-0005 #248 amendment);
/// the importer bulk-inserts its tiles via `BathymetryStore::importTiles`.

namespace marine_bathymetry_store
{

/// @brief Options governing a GeoTIFF import.
struct GeoTiffImportOptions
{
  /// 1-based raster band holding depth as **ellipsoidal height** (WGS84, m,
  /// up-positive — the store convention; `bag_to_geotiff` band 1). Non-finite
  /// pixels are no-data and skipped.
  int depth_band = 1;
  /// 1-based raster band holding 1-sigma vertical uncertainty (m), or 0 if
  /// the file has none (cells then get `default_uncertainty`).
  int uncertainty_band = 2;
  /// Uncertainty assigned when `uncertainty_band == 0` or the band reads
  /// non-finite **or non-positive** (zero uncertainty is treated as missing,
  /// not perfect — it would pass every reliability gate and carry infinite
  /// weight in 1/sigma^2 fusion). NaN (the default) makes such cells
  /// never-reliable in the safety query — the conservative choice.
  double default_uncertainty = std::numeric_limits<double>::quiet_NaN();
  /// Vertical conversion applied at import (ADR-0002 §D4):
  /// `height = depth_scale * pixel + depth_offset`. The defaults pass
  /// store-convention inputs (ellipsoidal height, up-positive) through
  /// unchanged. A positive-down depths-below-lake-surface product imports
  /// with `depth_scale = -1` and `depth_offset` = the lake surface's
  /// ellipsoidal height.
  double depth_scale = 1.0;
  double depth_offset = 0.0;
  /// Per-point vertical-datum resolver (#315): when set, the source datum's
  /// ellipsoidal height at each pixel's (lat, lon) replaces the constant
  /// `depth_offset` — `height = depth_scale * pixel + fn(lat, lon)`. This is
  /// how a tidal-datum-referenced grid (e.g. depths below MLLW) imports
  /// without a hand-computed constant: the offset varies across the extent
  /// and comes from the VDatum grids (the CLI builds this from
  /// marine_vertical_datum::make_vdatum_query, keeping this library
  /// PROJ-free). Returning std::nullopt means the datum grids have no
  /// coverage at that point — a HARD ERROR (`std::runtime_error`), never a
  /// silent skip or a fall-back to `depth_offset`: importing an
  /// unconverted (or partially converted) surface would corrupt the layer.
  /// Mutually exclusive with a non-zero `depth_offset`
  /// (`std::invalid_argument`); `depth_scale` still applies (a
  /// positive-down depth-below-datum product uses `depth_scale = -1`).
  /// Threading contract: the importer invokes this SERIALLY from the one
  /// calling thread, so a single-threaded-only callable (e.g. a
  /// marine_vertical_datum query, whose copies share one PROJ context) is
  /// safe to supply. A future parallel pixel loop must not share one
  /// callable across threads — build one query per thread instead (see
  /// vdatum_query.hpp).
  std::function<std::optional<double>(double lat, double lon)> vertical_datum_fn;
  /// GGGS level to import at. The store is multi-level (ADR-0002 §D2): a coarse
  /// chart prior imports at a coarse level, a fine survey grid at a fine level,
  /// and both can share the store. `std::nullopt` (the default) uses the
  /// store's default level (`store.level()`), preserving the single-level
  /// caller's behaviour; pass a level to match the source resolution.
  std::optional<uint8_t> level = std::nullopt;
  /// Merge into the layer's resident tiles instead of replacing them per tile.
  ///
  /// `importTiles` inserts whole tiles, so by default a second source touching a
  /// tile another source already wrote **discards that source's cells**. That is
  /// intended for a re-import that supersedes (a shrinking survey must drop the
  /// cells it no longer covers), and wrong for a layer assembled from many
  /// disjoint sources — a chart layer built cell-by-cell from ENC cells, where
  /// two neighbouring cells legitimately share a GGGS tile at their seam
  /// (#339).
  ///
  /// With this set, each touched tile is seeded from the resident tile first, so
  /// the import fills gaps and the existing lowest-uncertainty contention rule
  /// arbitrates any cell both sources claim. Contentions against resident data
  /// are counted in `ProcessedImportResult::resident_contentions`.
  bool merge_into_resident = false;
};

/// @brief Result of a GeoTIFF import, including the anti-clobber side effect.
///
/// `draft_tiles_touched` is the cross-layer coordination seam for the display
/// cache (camp#171/#172, ADR-0008): after a `Processed` import clears overlapped
/// `Draft` cells cell-wise (ADR-0010 D8), it lists the `Draft` tiles that were
/// **touched** (marked dirty) — NOT removed. Nothing is removed on disk under
/// cell-wise clearing: a cleared draft cell is written no-data (NaN) in place and
/// persisted through the normal dirty-tile save path. A cache consumer invalidates
/// exactly these tiles. For non-`Processed` imports the draft fields stay
/// zero/empty (only a `Processed` import clears `Draft`).
struct ProcessedImportResult
{
  /// Distinct cells written into @p layer (a new cell; replacements don't recount).
  std::size_t cells_imported = 0;
  /// `Draft` cells transitioned from data → no-data (cell-wise anti-clobber).
  /// Only cells the processed import populated AND that `Draft` already held data
  /// at are counted — a processed no-data cell (gated-drop hole) leaves the
  /// overlapping draft cell intact.
  std::size_t draft_cells_cleared = 0;
  /// `Draft` tiles with ≥1 cleared cell (each appears once). Cache-invalidation
  /// seam for camp#171/#172; these tiles are touched (dirtied), not removed.
  std::vector<gggs::GridIndex> draft_tiles_touched;
  /// Distinct `Draft` cells **coarser** than this import's processed data that
  /// overlap it but were deliberately KEPT, because the import does not fully
  /// supersede them (part of the cell's ground lies outside every tile of this
  /// import, or a processed cell under it holds no data). Verbatim
  /// `DraftClearResult::coarse_draft_cells_retained` — see
  /// `BathymetryStore::clearOverlappedDraft` for the rule and its counting.
  ///
  /// Carried out to the caller deliberately: keeping the cell is the shoal-safe
  /// direction, but it means a superseded draft blunder is still in the store and
  /// still winning `shallowestReliable` over that ground, so the operator running
  /// the import has to be able to SEE it. `import_geotiff` prints it. 0 for
  /// non-`Processed` imports, and 0 whenever `draft` is at or finer than every
  /// processed level in the import.
  std::size_t coarse_draft_cells_retained = 0;
  /// Contention **events** against data already resident in the layer, when
  /// `GeoTiffImportOptions::merge_into_resident` is set (always 0 otherwise).
  ///
  /// An event is a source pixel landing on a cell the resident tile already
  /// holds a *different* value for. It counts events, not distinct cells: when
  /// the source is finer than the store level, several pixels fall in one cell
  /// and each is counted. That is deliberate — this is an alarm, not an
  /// accounting.
  ///
  /// **Expected to be zero for sources that do not overlap.** ENC cells
  /// partition their usage band, so a chart regeneration reporting a non-zero
  /// count means two sources disagree about the same ground: either NOAA has
  /// rescheme-overlapped, or the tool has been pointed at genuinely overlapping
  /// products (S-102 — see
  /// [#316](https://github.com/rolker/unh_marine_autonomy/issues/316), where the
  /// arbitration rule is still an open decision).
  std::size_t resident_contentions = 0;
};

/// @brief Import @p path into @p layer's single fused surface.
///
/// Reads the GeoTIFF, fills each pixel's **footprint** of GGGS cells at the
/// target level (every store cell a pixel covers, so a coarser-than-store source
/// still produces coverage), converts the vertical datum at import (§D4), keeps
/// the lowest-uncertainty value on contention, and bulk-inserts the resulting
/// tiles into @p layer via `BathymetryStore::importTiles`. That insert replaces
/// **per tile**: each touched tile replaces any resident tile at that grid, so a
/// partial GeoTIFF patch drops previously stored cells inside a touched tile
/// rather than merging per cell — the lowest-uncertainty-on-contention rule
/// applies only among this import's own pixels. See `importTiles`' additive-merge
/// footgun note for the shrinking-re-import consequence.
///
/// **Anti-clobber (ADR-0010 D8):** when @p layer is `Processed`, the importer clears
/// overlapped `Draft` cells **cell-wise — only where this import has data** — by
/// delegating to the store's public `BathymetryStore::clearOverlappedDraft` (the
/// store owns this semantics so cube's direct-`saveTile` regen paths clear draft
/// identically; the clear writes no-data via `set(Draft, …, {})`, persisted through
/// the normal dirty-tile save path — there is no tile/cell-erase API and `save()`
/// never deletes on-disk tiles). Draft cells in the re-run's gated-drop holes (cells
/// this import left no-data) survive — the query overlay resolves them under
/// `Processed > Draft` where the processed surface has data, and shows the surviving
/// draft where it does not.
///
/// **Level-aware (uma#369).** The clear is NOT limited to this import's own GGGS
/// level — `processed` is depth-adaptive and mixed-level while `draft` stays
/// fixed-level, so a level-keyed clear would match no draft tile and clear
/// nothing, silently. `clearOverlappedDraft` walks **every** level the `Draft`
/// layer holds. A draft cell at or finer than the processed tile is decided by
/// the one processed cell containing it; a **coarser** draft cell is cleared only
/// where this tile fully supersedes it (entirely inside the tile, every processed
/// cell under it holding data) and is otherwise kept and reported in
/// `ProcessedImportResult::coarse_draft_cells_retained`. See
/// `BathymetryStore::clearOverlappedDraft` for the full contract.
/// Non-`Processed` imports clear nothing.
///
/// @return A `ProcessedImportResult` (cells imported + anti-clobber side effect).
/// @throws std::invalid_argument on a bad band index, or if a tile built by this
///         import carries an invalid `gggs::GridIndex` (propagated from
///         `clearOverlappedDraft` / `importTiles`);
///         std::logic_error if @p layer is `Reference` and the store is not
///         `reference_writable` (the read-only-prior gate);
///         std::runtime_error on GDAL failure, a non-WGS84 / rotated raster, or
///         a missing geotransform.
///
/// @note **Exception safety is BASIC, not strong.** For a `Processed` import the
///   draft clear runs *before* `importTiles`. `clearOverlappedDraft` validates
///   every tile index up front, so a throw from the clear itself leaves `Draft`
///   untouched — but a throw from `importTiles` afterwards does not undo the
///   clear, leaving `Draft` cells erased and `Processed` never inserted. The
///   store is not rolled back: treat a throw from this function as "the store's
///   contents are indeterminate for this import's footprint" and reload from disk
///   rather than continuing to write. In practice the importer builds every tile
///   index itself from the raster geotransform, so the invalid-index path is a
///   corruption signal, not an input-validation path; the reachable case is the
///   write-gate `std::logic_error` for a `Reference` layer, which is checked
///   before any clear happens (a `Reference` import clears nothing).
ProcessedImportResult importGeoTiff(
  BathymetryStore & store, SourceLayer layer,
  const std::string & path,
  const GeoTiffImportOptions & options = GeoTiffImportOptions{});

}  // namespace marine_bathymetry_store

#endif  // MARINE_BATHYMETRY_STORE__GEOTIFF_IMPORT_HPP_
