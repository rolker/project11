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

#include "marine_bathymetry_store/query.hpp"

#include <cmath>
#include <map>
#include <set>
#include <vector>

#include "cell_geometry.hpp"

namespace marine_bathymetry_store
{

namespace
{

/// The distinct GGGS levels present in one layer's tile map, **finest first**
/// (largest level number = finest resolution). Empty if the layer has no tiles.
std::set<uint8_t, std::greater<uint8_t>> levelsPresent(
  const std::map<gggs::GridIndex, BathymetryTile> & tiles)
{
  std::set<uint8_t, std::greater<uint8_t>> levels;
  for (const auto & [grid, tile] : tiles) {
    (void)tile;
    levels.insert(grid.level());
  }
  return levels;
}

/// Look up @p cell in a layer's tile map, or nullopt if its grid is absent.
std::optional<BathyCell> cellIn(
  const std::map<gggs::GridIndex, BathymetryTile> & tiles, const gggs::CellIndex & cell)
{
  const auto it = tiles.find(cell.grid());
  if (it == tiles.end()) {
    return std::nullopt;
  }
  return it->second.get(cell.row(), cell.column());
}

/// Visit EVERY cell at @p level that @p query_cell covers, for the cells whose
/// tile the layer actually holds.
///
/// This is what makes `uma-ADR-0013` D8 ("a safety query reads the finest data
/// **for the region**") true when `processed` is finer than the query level: a
/// level-10 costmap cell covers 16 level-12 cells, 64 level-13 and 256
/// level-14, and a 0.2-0.5 m rock — exactly what those levels exist to resolve
/// — can sit in any one of them. Point-sampling the query cell's centre reads
/// one of 256 and can walk past the rock; the caller must see all of them and
/// keep the shoalest.
///
/// Only levels FINER than the query cell need this. At or coarser than the
/// query level, one cell contains the whole query cell, so its centre resolves
/// it exactly — see the callers' fast path.
///
/// Only TILES are data-gated: a grid with no tile in this layer is skipped
/// without touching a cell. Inside a tile that IS present the walk is purely
/// geometric — 4^(fine - query) cells — whether or not those cells hold
/// anything, so a sparse level-14 tile holding ten values still costs 4096
/// visits under a level-8 query cell. The bound is the level gap, not the data.
/// A survey covering the query cell at level 14 under a level-10 query
/// genuinely is 256 cells of finest data, and reading fewer of them is the
/// defect this exists to prevent.
///
/// @p visit returns `true` to continue and `false` to stop the walk; this
/// function returns `false` when a visitor stopped it. An existence probe
/// (`hasAnyData`) stops at the first hit, so it costs one cell in the common
/// case instead of the full fan-out — the ordinary safety queries, which must
/// see every cell, always return `true` and walk it all.
template<typename Visitor>
bool forEachCoveredCell(
  const std::map<gggs::GridIndex, BathymetryTile> & tiles,
  const gggs::CellIndex & query_cell, uint8_t level, const Visitor & visit)
{
  const gggs::Level fine(level);
  // Inset both corners: the GGGS area iterators are inclusive, so the unmodified
  // NE corner would also visit the neighbouring cells that merely touch the
  // query cell's edge, and an exactly-on-boundary SW corner can round to the
  // neighbour below (see cell_geometry.hpp).
  const GeoBox box = insetForIteration(cellBox(query_cell), level);
  for (gggs::GridAreaIterator grid_it(fine.gridIndex(box.min), fine.gridIndex(box.max));
    grid_it.valid(); grid_it.next())
  {
    const auto tile_it = tiles.find(*grid_it);
    if (tile_it == tiles.end()) {
      continue;   // this layer holds no tile at this level here
    }
    for (gggs::CellAreaIterator cell_it(*grid_it, box.min, box.max);
      cell_it.valid(); cell_it.next())
    {
      if (!visit(*cell_it, tile_it->second.get(cell_it->row(), cell_it->column()))) {
        return false;
      }
    }
  }
  return true;
}

/// The reliability gate shared by `shallowestReliable` and `reliableSamples`:
/// the cell must hold data, and a NaN uncertainty is never reliable.
bool reliable(const BathyCell & c, double max_uncertainty)
{
  return c.hasData() && !std::isnan(c.uncertainty) && !(c.uncertainty > max_uncertainty);
}

/// Run @p visit over every cell of @p layer at @p level that bears on
/// @p query_cell: the single containing cell when the level is at or coarser
/// than the query, every covered cell when it is finer (see
/// `forEachCoveredCell`). @p center is the query cell's centre.
///
/// @p visit returns `true` to continue, `false` to stop; this function returns
/// `false` when a visitor stopped the walk.
template<typename Visitor>
bool forEachBearingCell(
  const std::map<gggs::GridIndex, BathymetryTile> & tiles,
  const gggs::CellIndex & query_cell, uint8_t level,
  const geographic_msgs::msg::GeoPoint & center, const Visitor & visit)
{
  if (level <= query_cell.level()) {
    const gggs::CellIndex lvl_cell =
      (level == query_cell.level()) ? query_cell : gggs::Level(level).cellIndex(center);
    if (const auto c = cellIn(tiles, lvl_cell)) {
      return visit(lvl_cell, *c);
    }
    return true;
  }
  return forEachCoveredCell(tiles, query_cell, level, visit);
}

/// Resolve a single layer's best-available sample at a cell across the levels it
/// holds, finest-first (ADR-0002 §D2). @p center is the query position.
std::optional<DepthSample> sampleFor(
  const BathymetryStore & store, SourceLayer layer, const gggs::CellIndex & cell,
  const geographic_msgs::msg::GeoPoint & center)
{
  const auto & tiles = store.tiles(layer);
  for (const uint8_t lvl : levelsPresent(tiles)) {
    const gggs::CellIndex lvl_cell =
      (lvl == cell.level()) ? cell : gggs::Level(lvl).cellIndex(center);
    const auto c = cellIn(tiles, lvl_cell);
    if (c && c->hasData()) {
      return DepthSample{c->depth, c->uncertainty, layer, lvl};
    }
  }
  return std::nullopt;
}

}  // namespace

std::optional<DepthSample> bestSource(
  const BathymetryStore & store, const gggs::CellIndex & cell)
{
  const auto center = cellCenter(cell);
  // Nav-safety precondition (#276 / ADR-0010 D7): source_layers_by_priority now
  // includes SourceLayer::Chart, so a loaded chart/ layer participates in these
  // depth queries — which feed navigation. load() populates Chart regardless of
  // the write-gate (the gate only blocks runtime writes, not reads), so there is
  // NO mechanical block on chart data driving navigation before the #276
  // cost-model rework lands. The standing precondition is that no deployed store
  // carries a chart/ layer until #276 is done; it is tracked there, not here.
  //
  // Finite-σ contract (ADR-0010 D7): a chart layer's cells MUST carry a large but
  // *finite* σ for low-confidence CATZOC classes (D/U). The consumer buckets a
  // non-finite σ = ∞ with no-data as unknown quality → conservative LETHAL
  // (bathymetry_layer::evaluateCell), so exporting D/U as σ = ∞ would make those
  // cells keepout-grade — the opposite of D7's "D/U never keepout-grade". The S57
  // exporter owns this; σ = ∞ stays reserved for genuinely-unknown quality (D4).
  for (const SourceLayer layer : source_layers_by_priority) {
    if (auto sample = sampleFor(store, layer, cell, center)) {
      return sample;
    }
  }
  return std::nullopt;
}

std::optional<DepthSample> shallowestReliable(
  const BathymetryStore & store, const gggs::CellIndex & cell, double max_uncertainty)
{
  std::optional<DepthSample> shallowest;
  const auto center = cellCenter(cell);

  // depth is ellipsoidal height (up-positive): shallower == greater height. With
  // one fused surface per layer (#221), there is no newest-epoch-first walk —
  // examine every level present in every layer and keep the shallowest reliable
  // value across all of them. Chart participates here too once loaded — see the
  // #276 nav-safety precondition note in bestSource above.
  for (const SourceLayer layer : source_layers_by_priority) {
    const auto & tiles = store.tiles(layer);
    // Within a layer, examine EVERY level so a coarse-but-reliable value can win
    // where a finer one is too uncertain — and, at levels FINER than the query,
    // every native cell the query cell covers rather than one sample from its
    // centre (uma-ADR-0013 D8: the finest data for the REGION). The bias is
    // deliberate and one-directional: of everything covering this ground, this
    // query keeps the SHOALEST reliable value, because a missed shoal is the
    // failure that grounds the boat.
    for (const uint8_t lvl : levelsPresent(tiles)) {
      forEachBearingCell(
        tiles, cell, lvl, center,
        [&](const gggs::CellIndex &, const BathyCell & c) {
          if (!reliable(c, max_uncertainty)) {
            return true;
          }
          if (!shallowest || c.depth > shallowest->depth) {
            shallowest = DepthSample{c.depth, c.uncertainty, layer, lvl};
          }
          return true;   // every covered cell is read: no early exit
        });
    }
  }
  return shallowest;
}

bool hasAnyData(const BathymetryStore & store, const gggs::CellIndex & cell)
{
  const auto center = cellCenter(cell);
  // Region-aware existence probe (uma#369). This is the SAFETY gate in
  // bathymetry_layer::evaluateCell — it decides unsurveyed (NO_INFORMATION, or
  // LETHAL under unsurveyed_is_lethal) versus surveyed-but-unusable (LETHAL) —
  // so it must cover the query cell's whole ground, not one sample from its
  // centre. Point-sampling here would let a single no-data native cell under the
  // centre (a gated-drop hole, a between-lines gap, an absent fine tile beside a
  // present one) declare the whole query cell unsurveyed and drop a rock in any
  // of the other 255 covered cells.
  //
  // Quality-blind by design: `hasData()` only, no reliability gate. The caller
  // separates "no data at all" from "data whose quality is unusable"; folding a
  // σ test in here would collapse that distinction (review M1).
  for (const SourceLayer layer : source_layers_by_priority) {
    const auto & tiles = store.tiles(layer);
    for (const uint8_t lvl : levelsPresent(tiles)) {
      const bool completed = forEachBearingCell(
        tiles, cell, lvl, center,
        [](const gggs::CellIndex &, const BathyCell & c) {
          return !c.hasData();   // false stops the walk: data found
        });
      if (!completed) {
        return true;
      }
    }
  }
  return false;
}

std::vector<DepthSample> reliableSamples(
  const BathymetryStore & store, const gggs::CellIndex & cell, double max_uncertainty)
{
  std::vector<DepthSample> samples;
  const auto center = cellCenter(cell);

  // Same walk as shallowestReliable, but collect EVERY passing sample instead of
  // keeping only the shallowest — the caller costs each and takes the most
  // hazardous (ADR-0010 §D7: a shallower untrusted sample must not mask a
  // co-located trusted keepout). Chart participates here too once loaded — see the
  // #276 nav-safety precondition note in bestSource above.
  // Region-aware for the same reason as shallowestReliable: at a level finer
  // than the query, EVERY covered native cell is a candidate hazard the caller
  // must cost. Dropping 255 of 256 by point-sampling the centre would hide the
  // most hazardous sample this function exists to surface.
  for (const SourceLayer layer : source_layers_by_priority) {
    const auto & tiles = store.tiles(layer);
    for (const uint8_t lvl : levelsPresent(tiles)) {
      forEachBearingCell(
        tiles, cell, lvl, center,
        [&](const gggs::CellIndex &, const BathyCell & c) {
          if (!reliable(c, max_uncertainty)) {
            return true;
          }
          samples.push_back(DepthSample{c.depth, c.uncertainty, layer, lvl});
          return true;   // every covered cell is read: no early exit
        });
    }
  }
  return samples;
}

void forEachCellBestSource(
  const BathymetryStore & store,
  const geographic_msgs::msg::GeoPoint & minimum,
  const geographic_msgs::msg::GeoPoint & maximum,
  const std::function<void(const gggs::CellIndex &,
  const std::optional<DepthSample> &)> & visitor)
{
  const gggs::Level & level = store.level();
  gggs::GridAreaIterator grid_it(
    level.gridIndex(minimum.latitude, minimum.longitude),
    level.gridIndex(maximum.latitude, maximum.longitude));
  for (; grid_it.valid(); grid_it.next()) {
    for (gggs::CellAreaIterator cell_it(*grid_it, minimum, maximum);
      cell_it.valid(); cell_it.next())
    {
      visitor(*cell_it, bestSource(store, *cell_it));
    }
  }
}

}  // namespace marine_bathymetry_store
