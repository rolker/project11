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

}  // namespace

DraftClearResult BathymetryStore::clearOverlappedDraft(
  const BathymetryTile & processed_tile)
{
  DraftClearResult result;
  const gggs::GridIndex & processed_grid = processed_tile.index();
  if (!processed_grid.valid()) {
    throw std::invalid_argument(
            "BathymetryStore::clearOverlappedDraft: processed tile has an invalid GridIndex");
  }
  const auto & draft_tiles = layerMap(SourceLayer::Draft);
  if (draft_tiles.empty()) {
    return result;   // nothing to clear anywhere
  }

  const uint8_t processed_level = processed_grid.level();
  const GeoBox processed_box = gridBox(processed_grid);
  // Draft tiles touched, deduplicated: a coarse draft tile can be reached by
  // several processed tiles, and one processed tile can reach several fine ones.
  std::set<gggs::GridIndex> touched;

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
      if (draft_tiles.count(*grid_it) == 0) {
        continue;
      }
      for (gggs::CellAreaIterator cell_it(*grid_it, walk.min, walk.max);
        cell_it.valid(); cell_it.next())
      {
        const gggs::CellIndex draft_cell = *cell_it;
        const std::optional<BathyCell> draft = get(SourceLayer::Draft, draft_cell);
        if (!draft.has_value() || !draft->hasData()) {
          continue;   // draft has nothing here — nothing to clear
        }
        if (!processedCoversDraftCell(processed_tile, draft_cell, result)) {
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

  result.tiles_touched.assign(touched.begin(), touched.end());
  return result;
}

bool BathymetryStore::processedCoversDraftCell(
  const BathymetryTile & processed_tile, const gggs::CellIndex & draft_cell,
  DraftClearResult & result) const
{
  const gggs::GridIndex & processed_grid = processed_tile.index();
  const uint8_t processed_level = processed_grid.level();
  const GeoBox draft_box = cellBox(draft_cell);

  if (draft_cell.level() >= processed_level) {
    // The draft cell is contained in exactly ONE processed cell (GGGS levels
    // nest exactly), so that cell alone decides. A processed no-data cell (a
    // gated-drop hole) leaves the draft cell intact — strictly more coverage
    // than clearing by footprint, so stale gap-striping never accumulates under
    // the authoritative surface.
    const gggs::CellIndex processed_cell =
      gggs::Level(processed_level).cellIndex(boxCenter(draft_box));
    if (!(processed_cell.grid() == processed_grid)) {
      return false;   // outside this processed tile
    }
    return processed_tile.get(processed_cell.row(), processed_cell.column()).hasData();
  }

  // The draft cell is COARSER than the processed tile: it covers many processed
  // cells, and clearing it would discard draft data over ground this processed
  // tile does not speak for. Clear it only when this tile fully supersedes it —
  // the draft cell lies entirely inside the tile AND every processed cell under
  // it has data. Anything short of that keeps the draft cell (the shoal-safe
  // direction: a retained draft blunder is a false alarm, a wrongly-cleared
  // draft cell is a lost hazard) and is COUNTED, so the caller sees residue
  // rather than a silent no-op.
  if (!boxContains(gridBox(processed_grid), draft_box)) {
    ++result.coarse_draft_cells_retained;
    return false;
  }
  const GeoBox walk = insetForIteration(draft_box, processed_level);
  for (gggs::CellAreaIterator cell_it(processed_grid, walk.min, walk.max);
    cell_it.valid(); cell_it.next())
  {
    if (!processed_tile.get(cell_it->row(), cell_it->column()).hasData()) {
      ++result.coarse_draft_cells_retained;
      return false;
    }
  }
  return true;
}

DraftClearResult BathymetryStore::clearOverlappedDraft(
  const std::map<gggs::GridIndex, BathymetryTile> & processed_tiles)
{
  // Delegate per tile, then re-deduplicate: with a level-aware clear one coarse
  // draft tile can be touched by several processed tiles, so the per-tile
  // results can name the same grid more than once. The contract is each grid
  // once, ascending.
  DraftClearResult result;
  std::set<gggs::GridIndex> touched;
  for (const auto & [grid, tile] : processed_tiles) {
    (void)grid;   // the per-tile overload re-keys off tile.index()
    const DraftClearResult one = clearOverlappedDraft(tile);
    result.cells_cleared += one.cells_cleared;
    result.coarse_draft_cells_retained += one.coarse_draft_cells_retained;
    touched.insert(one.tiles_touched.begin(), one.tiles_touched.end());
  }
  result.tiles_touched.assign(touched.begin(), touched.end());
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
