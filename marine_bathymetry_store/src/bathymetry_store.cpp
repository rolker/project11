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

#include "marine_bathymetry_store/bathymetry_store.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

#include "cell_geometry.hpp"

namespace marine_bathymetry_store
{

bool BathymetryStore::set(
  SourceLayer layer, const gggs::CellIndex & cell, const BathyCell & value)
{
  // Validate the input first, then the layer permission — so a malformed cell
  // always reports invalid_argument (the more actionable error), and the
  // read-only logic_error fires only for an otherwise-valid Reference write.
  // The cell may be at any valid level — the store is multi-level (ADR-0002 §D2).
  if (!cell.valid()) {
    throw std::invalid_argument("BathymetryStore::set: invalid CellIndex");
  }
  if (layer == SourceLayer::Reference && !reference_writable_) {
    throw std::logic_error(
            "BathymetryStore::set: Reference is a read-only prior layer; construct "
            "the store with reference_writable=true (importer only) to write it");
  }
  if (layer == SourceLayer::Chart && !chart_staging_writable_) {
    throw std::logic_error(
            "BathymetryStore::set: Chart is writable only via the regeneration "
            "workflow (ADR-0010 D7); construct a staging store with "
            "chart_staging_writable=true, then swap via replaceChartLayer");
  }
  // Last-write-wins per cell: there is no per-day epoch ordering since #221.
  BathymetryTile & tile = getOrCreateTile(layer, cell.grid());
  tile.set(cell.row(), cell.column(), value);
  return true;
}

std::optional<BathyCell> BathymetryStore::get(
  SourceLayer layer, const gggs::CellIndex & cell) const
{
  if (!cell.valid()) {
    return std::nullopt;
  }
  const auto & layer_tiles = layerMap(layer);
  const auto tile_it = layer_tiles.find(cell.grid());
  if (tile_it == layer_tiles.end()) {
    return std::nullopt;
  }
  return tile_it->second.get(cell.row(), cell.column());
}

std::size_t BathymetryStore::importTiles(
  SourceLayer layer, std::map<gggs::GridIndex, BathymetryTile> tiles)
{
  // Reference is a read-only prior (ADR-0002 §D3). importTiles is a public
  // mutator just like set(), so it must honor the same gate -- otherwise the CLI
  // or any library consumer could overwrite the prior on a default store,
  // defeating the read-only guarantee. Only an importer that explicitly opted in
  // (reference_writable=true) may write Reference.
  if (layer == SourceLayer::Reference && !reference_writable_) {
    throw std::logic_error(
            "BathymetryStore::importTiles: Reference is a read-only prior layer; "
            "construct the store with reference_writable=true (importer only) to write it");
  }
  // Chart mirrors the gate: only a staging store (chart_staging_writable=true)
  // may bulk-import Chart tiles; runtime stores receive Chart solely via
  // load() after a replaceChartLayer swap (ADR-0010 D7).
  if (layer == SourceLayer::Chart && !chart_staging_writable_) {
    throw std::logic_error(
            "BathymetryStore::importTiles: Chart is writable only via the regeneration "
            "workflow (ADR-0010 D7); construct a staging store with "
            "chart_staging_writable=true, then swap via replaceChartLayer");
  }
  for (const auto & [grid, tile] : tiles) {
    if (!grid.valid()) {
      throw std::invalid_argument("BathymetryStore::importTiles: invalid GridIndex key");
    }
    // The tile must have been built for the grid it is keyed under: a mismatch
    // would write a tile under one grid's filename but with another grid's
    // georeference, corrupting the store on the next load (harvested #148
    // Copilot med-fix, preserved across the epoch removal).
    if (!(tile.index() == grid)) {
      throw std::invalid_argument(
              "BathymetryStore::importTiles: tile GridIndex does not match its map key");
    }
  }

  auto & m = layerMap(layer);
  std::size_t inserted = 0;
  for (auto & [grid, tile] : tiles) {
    // A bulk import is a fresh surface: mark every inserted tile dirty so it
    // persists on the next save. insert_or_assign (not operator[]) because
    // BathymetryTile is not default-constructible.
    tile.markDirty();
    m.insert_or_assign(grid, std::move(tile));
    ++inserted;
  }
  return inserted;
}

namespace
{

/// The distinct GGGS levels present in a layer's tile map, coarsest first.
std::set<uint8_t> levelsPresent(const std::map<gggs::GridIndex, BathymetryTile> & tiles)
{
  std::set<uint8_t> levels;
  for (const auto & [grid, tile] : tiles) {
    (void)tile;
    levels.insert(grid.level());
  }
  return levels;
}

/// Every processed tile one `clearOverlappedDraft` call was given, indexed for
/// point lookup: tiles by grid, plus the distinct levels present (finest first).
///
/// `processed` is depth-adaptive and mixed-level (uma#369), so a coverage
/// question ("does the processed data have anything at this point?") has to be
/// asked once per level present, each answered by one map lookup — never by a
/// scan over the tiles.
struct ProcessedIndex
{
  std::map<gggs::GridIndex, const BathymetryTile *> by_grid;
  std::set<uint8_t, std::greater<uint8_t>> levels;   ///< finest first

  uint8_t finest() const {return *levels.begin();}
};

/// Does ANY processed tile in @p index hold data at @p point?
bool processedHasDataAt(
  const ProcessedIndex & index, const geographic_msgs::msg::GeoPoint & point)
{
  for (const uint8_t level : index.levels) {
    const gggs::CellIndex cell = gggs::Level(level).cellIndex(point);
    const auto it = index.by_grid.find(cell.grid());
    if (it != index.by_grid.end() && it->second->get(cell.row(), cell.column()).hasData()) {
      return true;
    }
  }
  return false;
}

/// Does the processed data in @p index fully supersede @p draft_cell — is every
/// point of the draft cell's ground covered by a processed cell that has data?
///
/// Level-aware in both directions, and decided against the WHOLE index rather
/// than one tile:
/// - Every processed level at or coarser than the draft cell: the draft cell
///   lies inside exactly one cell of each (GGGS levels nest exactly), so its
///   centre resolves them all.
/// - Some processed level FINER than the draft cell: the draft cell covers many
///   processed cells, possibly spread across several tiles and several levels.
///   Walk it at the finest level present — the finest partition, and every
///   coarser level's boundaries fall on its cell boundaries — and require every
///   one of those cells to be covered. A processed no-data cell (a gated-drop
///   hole) or ground outside every tile leaves the draft cell intact: strictly
///   more coverage than clearing by footprint, so stale gap-striping never
///   accumulates under the authoritative surface.
///
/// A kept cell is recorded in @p retained (a set, so it is one cell of residue
/// however many tiles' walks reach it) rather than counted.
bool processedSupersedesDraftCell(
  const ProcessedIndex & index, const gggs::CellIndex & draft_cell,
  std::set<gggs::CellIndex> & retained)
{
  const GeoBox draft_box = cellBox(draft_cell);

  if (index.finest() <= draft_cell.level()) {
    // Nothing finer than the draft cell: one containing processed cell per
    // level decides it, and a kept cell here is NOT coarse-retained residue —
    // it is an ordinary gated-drop hole, which the counter has never counted.
    return processedHasDataAt(index, boxCenter(draft_box));
  }

  const GeoBox walk = insetForIteration(draft_box, index.finest());
  const gggs::Level fine(index.finest());
  for (gggs::GridAreaIterator grid_it(fine.gridIndex(walk.min), fine.gridIndex(walk.max));
    grid_it.valid(); grid_it.next())
  {
    for (gggs::CellAreaIterator cell_it(*grid_it, walk.min, walk.max);
      cell_it.valid(); cell_it.next())
    {
      if (!processedHasDataAt(index, cellCenter(*cell_it))) {
        retained.insert(draft_cell);
        return false;
      }
    }
  }
  return true;
}

}  // namespace

DraftClearResult BathymetryStore::clearOverlappedDraft(
  const BathymetryTile & processed_tile)
{
  return clearOverlappedDraftImpl({&processed_tile});
}

DraftClearResult BathymetryStore::clearOverlappedDraft(
  const std::map<gggs::GridIndex, BathymetryTile> & processed_tiles)
{
  // Pass every tile to one decision pass rather than delegating per tile: a
  // coarse draft cell split across two processed tiles is superseded by their
  // UNION, and a per-tile delegation would retain it for each tile that saw part
  // of it — keeping a superseded blunder alive and counting the same cell twice.
  std::vector<const BathymetryTile *> processed;
  processed.reserve(processed_tiles.size());
  for (const auto & [grid, tile] : processed_tiles) {
    (void)grid;   // the tiles are re-keyed off tile.index()
    processed.push_back(&tile);
  }
  return clearOverlappedDraftImpl(processed);
}

DraftClearResult BathymetryStore::clearOverlappedDraftImpl(
  const std::vector<const BathymetryTile *> & processed)
{
  DraftClearResult result;
  // Index the processed tiles by grid, and record the distinct levels present.
  // `processed` is depth-adaptive and mixed-level (uma#369), so a single call
  // can carry several native levels; every coverage question below is answered
  // by a map lookup per level rather than a scan over the tiles.
  ProcessedIndex index;
  for (const BathymetryTile * tile : processed) {
    const gggs::GridIndex & grid = tile->index();
    if (!grid.valid()) {
      throw std::invalid_argument(
              "BathymetryStore::clearOverlappedDraft: processed tile has an invalid GridIndex");
    }
    index.by_grid.emplace(grid, tile);
    index.levels.insert(grid.level());
  }
  if (index.by_grid.empty()) {
    return result;   // nothing to clear with
  }

  const auto & draft_tiles = layerMap(SourceLayer::Draft);
  if (draft_tiles.empty()) {
    return result;   // nothing to clear anywhere
  }

  // Draft tiles touched, deduplicated: a coarse draft tile can be reached by
  // several processed tiles, and one processed tile can reach several fine ones.
  std::set<gggs::GridIndex> touched;
  // Coarse draft cells KEPT, deduplicated: the same cell can be visited from
  // more than one processed tile, and it is one cell of residue however many
  // tiles saw part of it. The decision is identical each time (it is taken
  // against the whole index), so this is a set, not a counter.
  std::set<gggs::CellIndex> retained;

  for (const BathymetryTile * processed_tile : processed) {
    const gggs::GridIndex & processed_grid = processed_tile->index();
    const uint8_t processed_level = processed_grid.level();
    const GeoBox processed_box = gridBox(processed_grid);

    // Level-aware (uma#369): `processed` is depth-adaptive and mixed-level while
    // `draft` stays fixed-level, so keying the clear on the processed tile's own
    // GridIndex — which carries its level — would match no draft tile at all and
    // clear nothing, silently, leaving superseded draft blunders to keep winning
    // shallowestReliable. Walk EVERY level the draft layer actually holds.
    for (const uint8_t draft_level : levelsPresent(draft_tiles)) {
      const gggs::Level level(draft_level);
      // Inset by the finer of the two levels so the inclusive area iterators never
      // step onto ground outside the processed tile.
      const uint8_t finest = std::max(draft_level, processed_level);
      const GeoBox walk = insetForIteration(processed_box, finest);

      for (gggs::GridAreaIterator grid_it(level.gridIndex(walk.min), level.gridIndex(walk.max));
        grid_it.valid(); grid_it.next())
      {
        // Never create a draft tile where none exists: nothing to clear there, and
        // an all-NaN draft tile would be a spurious on-disk artifact.
        const auto draft_tile_it = draft_tiles.find(*grid_it);
        if (draft_tile_it == draft_tiles.end()) {
          continue;
        }
        // Hoist the draft tile out of the cell loop: the grid is already
        // resolved, so the per-cell read is a direct raster access instead of a
        // map find plus an optional<BathyCell> copy — 921,600 of those per
        // same-level tile otherwise. It is now the CHEAPEST test available, so
        // it stays first: the processed-side coverage decision (which for a
        // coarse draft cell walks the cells under it) runs only for draft cells
        // that actually hold something to clear.
        const BathymetryTile & draft_tile = draft_tile_it->second;
        for (gggs::CellAreaIterator cell_it(*grid_it, walk.min, walk.max);
          cell_it.valid(); cell_it.next())
        {
          const gggs::CellIndex draft_cell = *cell_it;
          if (!draft_tile.get(draft_cell.row(), draft_cell.column()).hasData()) {
            continue;   // draft has nothing here — nothing to clear
          }
          if (!processedSupersedesDraftCell(index, draft_cell, retained)) {
            continue;
          }
          // Write no-data in place: reads as no-data thereafter, tile marked dirty
          // so the clear persists on the next save.
          set(SourceLayer::Draft, draft_cell, BathyCell{});
          ++result.cells_cleared;
          touched.insert(draft_cell.grid());
        }
      }
    }
  }

  result.tiles_touched.assign(touched.begin(), touched.end());
  result.coarse_draft_cells_retained = retained.size();
  return result;
}

BathymetryTile & BathymetryStore::getOrCreateTile(
  SourceLayer layer, const gggs::GridIndex & grid)
{
  // Any valid level is accepted — the store is multi-level (ADR-0002 §D2). The
  // GridIndex carries its own level, so tiles at different levels coexist.
  if (!grid.valid()) {
    throw std::invalid_argument("BathymetryStore::getOrCreateTile: invalid GridIndex");
  }
  auto & layer_tiles = layerMap(layer);
  auto it = layer_tiles.find(grid);
  if (it == layer_tiles.end()) {
    it = layer_tiles.emplace(grid, BathymetryTile(grid)).first;
  }
  return it->second;
}

}  // namespace marine_bathymetry_store
