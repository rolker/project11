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

#include <gtest/gtest.h>

#include <cstddef>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>

#include "marine_autonomy/gggs.h"
#include "marine_bathymetry_store/bathymetry_store.hpp"
#include "marine_bathymetry_store/bathymetry_tile.hpp"

using marine_bathymetry_store::BathyCell;
using marine_bathymetry_store::BathymetryStore;
using marine_bathymetry_store::SourceLayer;

// Tile count of one layer (one fused surface per layer since #221).
static std::size_t tilesIn(const BathymetryStore & store, SourceLayer layer)
{
  return store.tiles(layer).size();
}

// The point at fractional position (@p lat_fraction, @p lon_fraction) inside a
// cell — (0.5, 0.5) is its centre. Re-derived here rather than reusing the
// implementation's own helper, so the cross-level tests do not lean on the code
// they check.
static geographic_msgs::msg::GeoPoint pointInCell(
  const gggs::CellIndex & cell, double lat_fraction, double lon_fraction)
{
  const gggs::GridIndex & grid = cell.grid();
  const double lat_per_cell = grid.latitudinalSpan() / gggs::cell_rows_per_grid;
  const double lon_per_cell = grid.longitudinalSpan() / gggs::cell_columns_per_grid;
  const auto sw = cell.position();
  return gggs::geoPoint(
    sw.latitude + lat_fraction * lat_per_cell, sw.longitude + lon_fraction * lon_per_cell);
}

TEST(Store, SetGetRoundTrip)
{
  BathymetryStore store(5);
  const auto cell = store.cellIndex(43.0, -70.5);
  store.set(SourceLayer::Draft, cell, BathyCell{-30.0, 0.5});

  const auto got = store.get(SourceLayer::Draft, cell);
  ASSERT_TRUE(got.has_value());
  EXPECT_DOUBLE_EQ(got->depth, -30.0);
  EXPECT_DOUBLE_EQ(got->uncertainty, 0.5);
}

TEST(Store, GetEmptyLayerIsNullopt)
{
  BathymetryStore store(5);
  EXPECT_FALSE(store.get(SourceLayer::Draft, store.cellIndex(43.0, -70.5)).has_value());
}

TEST(Store, LayersAreIndependent)
{
  // A survey write must not touch the reference layer at the same cell.
  BathymetryStore store(5, /*reference_writable=*/true);
  const auto cell = store.cellIndex(43.0, -70.5);
  store.set(SourceLayer::Reference, cell, BathyCell{-10.0, 0.1});
  store.set(SourceLayer::Draft, cell, BathyCell{-12.0, 2.0});
  EXPECT_DOUBLE_EQ(store.get(SourceLayer::Reference, cell)->depth, -10.0);
  EXPECT_DOUBLE_EQ(store.get(SourceLayer::Draft, cell)->depth, -12.0);
}

TEST(Store, DoubleWriteSameCellLastWriteWins)
{
  // Single fused surface (#221): a second write to the same cell overwrites the
  // first — there is no per-day epoch ordering or provenance guard.
  BathymetryStore store(5);
  const auto cell = store.cellIndex(43.0, -70.5);
  EXPECT_TRUE(store.set(SourceLayer::Draft, cell, BathyCell{-10.0, 0.1}));
  EXPECT_TRUE(store.set(SourceLayer::Draft, cell, BathyCell{-12.0, 0.2}));
  const auto got = store.get(SourceLayer::Draft, cell);
  ASSERT_TRUE(got.has_value());
  EXPECT_DOUBLE_EQ(got->depth, -12.0);
  EXPECT_DOUBLE_EQ(got->uncertainty, 0.2);
  // Still one tile (same grid).
  EXPECT_EQ(tilesIn(store, SourceLayer::Draft), 1u);
}

TEST(Store, TilesAllocatedLazilyPerLayer)
{
  BathymetryStore store(5);
  EXPECT_TRUE(store.tiles(SourceLayer::Draft).empty());
  EXPECT_TRUE(store.tiles(SourceLayer::Reference).empty());

  store.set(SourceLayer::Draft, store.cellIndex(43.0, -70.5), BathyCell{-30.0, 0.5});
  EXPECT_EQ(tilesIn(store, SourceLayer::Draft), 1u);
  EXPECT_TRUE(store.tiles(SourceLayer::Reference).empty());
}

TEST(Store, NoDataCellReadsBackAsNoData)
{
  BathymetryStore store(5);
  const auto cell = store.cellIndex(43.0, -70.5);
  store.set(SourceLayer::Draft, cell, BathyCell{});  // depth NaN
  const auto got = store.get(SourceLayer::Draft, cell);
  ASSERT_TRUE(got.has_value());      // tile exists
  EXPECT_FALSE(got->hasData());      // but the cell carries no usable depth
}

TEST(Store, MultiLevelTilesCoexist)
{
  // The store is multi-level (ADR-0002 §D2): a cell at a level other than the
  // store's default level is accepted, stored, and read back independently.
  BathymetryStore store(5);
  gggs::Level fine(6);
  const auto fine_cell = fine.cellIndex(gggs::geoPoint(43.0, -70.5));
  const auto default_cell = store.cellIndex(43.0, -70.5);
  ASSERT_NE(fine_cell.level(), default_cell.level());

  EXPECT_NO_THROW(store.set(SourceLayer::Draft, fine_cell, BathyCell{-22.0, 0.3}));
  store.set(SourceLayer::Draft, default_cell, BathyCell{-20.0, 0.5});

  // Both tiles exist in the same layer at different levels.
  EXPECT_EQ(tilesIn(store, SourceLayer::Draft), 2u);
  const auto fine_got = store.get(SourceLayer::Draft, fine_cell);
  ASSERT_TRUE(fine_got.has_value());
  EXPECT_DOUBLE_EQ(fine_got->depth, -22.0);
  const auto default_got = store.get(SourceLayer::Draft, default_cell);
  ASSERT_TRUE(default_got.has_value());
  EXPECT_DOUBLE_EQ(default_got->depth, -20.0);
}

TEST(Store, InvalidCellThrows)
{
  BathymetryStore store(5);
  EXPECT_THROW(store.set(SourceLayer::Draft, gggs::CellIndex{}, BathyCell{}),
    std::invalid_argument);
}

TEST(Store, FromCellSizeChoosesAFiniteLevel)
{
  const auto store = BathymetryStore::fromCellSize(30.0f);
  // The chosen level must produce valid cells for a normal position.
  EXPECT_TRUE(store.cellIndex(43.0, -70.5).valid());
}

TEST(Store, ReferenceIsReadOnlyByDefault)
{
  // The prior must be unclobberable by live ingest: set(Reference) throws
  // unless the store was explicitly opened reference_writable (ADR-0002 §D3).
  BathymetryStore store(5);
  EXPECT_FALSE(store.referenceWritable());
  EXPECT_THROW(
    store.set(SourceLayer::Reference, store.cellIndex(43.0, -70.5), BathyCell{-10.0, 3.0}),
    std::logic_error);
  // Other layers are unaffected by the guard.
  const auto cell = store.cellIndex(43.0, -70.5);
  EXPECT_NO_THROW(store.set(SourceLayer::Draft, cell, BathyCell{-12.0, 2.0}));
}

TEST(Store, ReferenceWritableStoreAllowsSet)
{
  // The importer opts in; the converted prior then writes and reads back.
  BathymetryStore store(5, /*reference_writable=*/true);
  EXPECT_TRUE(store.referenceWritable());
  const auto cell = store.cellIndex(43.0, -70.5);
  store.set(SourceLayer::Reference, cell, BathyCell{38.58, 3.0});
  const auto got = store.get(SourceLayer::Reference, cell);
  ASSERT_TRUE(got.has_value());
  EXPECT_DOUBLE_EQ(got->depth, 38.58);
}

TEST(Store, FromCellSizePropagatesReferenceWritable)
{
  auto store = BathymetryStore::fromCellSize(30.0f, /*reference_writable=*/true);
  EXPECT_TRUE(store.referenceWritable());
  EXPECT_NO_THROW(
    store.set(SourceLayer::Reference, store.cellIndex(43.0, -70.5), BathyCell{40.0, 3.0}));
}

namespace
{
// Build a one-cell tile map for importTiles from a (cell, value) pair.
std::map<gggs::GridIndex, marine_bathymetry_store::BathymetryTile> oneTile(
  const gggs::CellIndex & cell, const BathyCell & value)
{
  std::map<gggs::GridIndex, marine_bathymetry_store::BathymetryTile> tiles;
  marine_bathymetry_store::BathymetryTile tile(cell.grid());
  tile.set(cell.row(), cell.column(), value);
  tiles.emplace(cell.grid(), std::move(tile));
  return tiles;
}
}  // namespace

TEST(Store, ImportTilesBulkInsert)
{
  // The importer's bulk-insert path merges a tile map into the layer, marks
  // tiles dirty, and reads back. A second import replaces the tile at that grid
  // (last-write-wins) while leaving other grids untouched.
  BathymetryStore store(5);
  const auto cell = store.cellIndex(43.0, -70.5);
  EXPECT_EQ(store.importTiles(SourceLayer::Draft, oneTile(cell, BathyCell{-30.0, 0.5})), 1u);
  const auto got = store.get(SourceLayer::Draft, cell);
  ASSERT_TRUE(got.has_value());
  EXPECT_DOUBLE_EQ(got->depth, -30.0);

  // Re-importing the same grid replaces it (no provenance ordering since #221).
  EXPECT_EQ(store.importTiles(SourceLayer::Draft, oneTile(cell, BathyCell{-28.0, 0.4})), 1u);
  EXPECT_DOUBLE_EQ(store.get(SourceLayer::Draft, cell)->depth, -28.0);
  EXPECT_EQ(tilesIn(store, SourceLayer::Draft), 1u);

  // A grid not in the import is left untouched: import a second, distinct grid.
  const auto other = store.cellIndex(44.0, -71.0);
  ASSERT_FALSE(cell.grid() == other.grid());
  store.importTiles(SourceLayer::Draft, oneTile(other, BathyCell{-15.0, 0.6}));
  EXPECT_EQ(tilesIn(store, SourceLayer::Draft), 2u);
  EXPECT_DOUBLE_EQ(store.get(SourceLayer::Draft, cell)->depth, -28.0);   // first grid intact
}

TEST(Store, ImportTilesRejectsTileKeyMismatch)
{
  // A tile must be built for the grid it is keyed under (harvested #148 fix,
  // preserved across the epoch removal).
  BathymetryStore store(5);
  const auto cell_a = store.cellIndex(43.0, -70.5);
  const auto cell_b = store.cellIndex(44.0, -71.0);
  ASSERT_FALSE(cell_a.grid() == cell_b.grid());

  std::map<gggs::GridIndex, marine_bathymetry_store::BathymetryTile> tiles;
  // Tile built for grid_b but inserted under grid_a's key.
  marine_bathymetry_store::BathymetryTile mismatched(cell_b.grid());
  tiles.emplace(cell_a.grid(), std::move(mismatched));
  EXPECT_THROW(
    store.importTiles(SourceLayer::Draft, std::move(tiles)),
    std::invalid_argument);
}

TEST(Store, ImportTilesHonorsReferenceReadOnlyGate)
{
  // importTiles is a public mutator like set(), so it must honor the Reference
  // read-only gate (ADR-0002 §D3) — otherwise a bulk import would overwrite the
  // prior on a default store. Only a reference_writable (importer) store may.
  const auto cell = BathymetryStore(5).cellIndex(43.0, -70.5);

  BathymetryStore read_only(5);
  EXPECT_THROW(
    read_only.importTiles(SourceLayer::Reference, oneTile(cell, BathyCell{-10.0, 3.0})),
    std::logic_error);

  BathymetryStore importer(5, /*reference_writable=*/true);
  EXPECT_EQ(
    importer.importTiles(SourceLayer::Reference, oneTile(cell, BathyCell{-10.0, 3.0})), 1u);
  EXPECT_DOUBLE_EQ(importer.get(SourceLayer::Reference, cell)->depth, -10.0);
}

TEST(Store, ImportTilesEmptyIsNoOp)
{
  // An empty import (e.g. an entirely no-data GeoTIFF) inserts nothing and adds
  // no tiles to the layer.
  BathymetryStore store(5);
  EXPECT_EQ(store.importTiles(SourceLayer::Draft, {}), 0u);
  EXPECT_TRUE(store.tiles(SourceLayer::Draft).empty());
}

TEST(Store, ChartIsReadOnlyByDefault)
{
  // Chart is writable only via the regeneration workflow (ADR-0010 D7): both
  // mutators throw on a normal runtime store.
  BathymetryStore store(5);
  EXPECT_FALSE(store.chartStagingWritable());
  EXPECT_THROW(
    store.set(SourceLayer::Chart, store.cellIndex(43.0, -70.5), BathyCell{-20.0, 1.5}),
    std::logic_error);
}

TEST(Store, ChartStagingWritableStoreAllowsSet)
{
  // The staging workflow opts in; a staged chart cell writes and reads back.
  BathymetryStore store(5, /*reference_writable=*/false, /*chart_staging_writable=*/true);
  EXPECT_TRUE(store.chartStagingWritable());
  const auto cell = store.cellIndex(43.0, -70.5);
  store.set(SourceLayer::Chart, cell, BathyCell{-20.0, 1.5});
  const auto got = store.get(SourceLayer::Chart, cell);
  ASSERT_TRUE(got.has_value());
  EXPECT_DOUBLE_EQ(got->depth, -20.0);
}

TEST(Store, FromCellSizePropagatesChartStagingWritable)
{
  auto store = BathymetryStore::fromCellSize(
    30.0f, /*reference_writable=*/false, /*chart_staging_writable=*/true);
  EXPECT_TRUE(store.chartStagingWritable());
  EXPECT_NO_THROW(
    store.set(SourceLayer::Chart, store.cellIndex(43.0, -70.5), BathyCell{-20.0, 1.5}));
}

TEST(Store, ImportTilesHonorsChartGate)
{
  // Bulk import must honor the same Chart gate as set(): a runtime store
  // refuses, a staging store accepts.
  BathymetryStore runtime_store(5);
  const auto cell = runtime_store.cellIndex(43.0, -70.5);
  EXPECT_THROW(
    runtime_store.importTiles(SourceLayer::Chart, oneTile(cell, BathyCell{-20.0, 1.5})),
    std::logic_error);

  BathymetryStore staging(5, /*reference_writable=*/false, /*chart_staging_writable=*/true);
  EXPECT_EQ(staging.importTiles(SourceLayer::Chart, oneTile(cell, BathyCell{-20.0, 1.5})), 1u);
}

// --- Public cell-wise anti-clobber API: clearOverlappedDraft (ADR-0010 D8) ---
// Direct-call coverage of the store operation extracted from importGeoTiff (#308),
// so cube's `saveTile`-based regen paths that bypass the importer exercise the same
// semantics. importGeoTiff's own anti-clobber tests (test_geotiff_import.cpp) cover
// the importer wiring; these hit the store API on the direct path.

TEST(Store, ClearOverlappedDraftIsCellWiseAndPreservesGatedDropHoles)
{
  // The invariant that mandates CELL-WISE (not tile-wise) clearing on the direct
  // path: a processed tile clears only draft cells it has data for; a processed
  // no-data cell (a gated-drop hole) leaves the overlapping draft cell intact, and a
  // processed cell where draft has no data clears nothing.
  BathymetryStore store(11);
  const gggs::GridIndex grid = store.level().gridIndex(43.0, -70.5);
  const gggs::CellIndex cell_a(grid, 0, 0);   // draft data + processed data -> cleared
  const gggs::CellIndex cell_b(grid, 0, 1);   // draft data + processed hole  -> survives
  const gggs::CellIndex cell_c(grid, 0, 2);   // draft empty + processed data -> nothing
  ASSERT_TRUE(cell_a.valid() && cell_b.valid() && cell_c.valid());
  store.set(SourceLayer::Draft, cell_a, BathyCell{-9.0, 0.4});
  store.set(SourceLayer::Draft, cell_b, BathyCell{-8.0, 0.4});
  // cell_c intentionally left with no draft data.

  marine_bathymetry_store::BathymetryTile processed(grid);
  processed.set(0, 0, BathyCell{-30.0, 0.1});  // covers A
  processed.set(0, 2, BathyCell{-31.0, 0.1});  // covers C (draft empty there)
  // (0, 1) left no-data: the gated-drop hole over B.

  const auto result = store.clearOverlappedDraft(processed);

  EXPECT_EQ(result.cells_cleared, 1u);            // only cell A
  ASSERT_EQ(result.tiles_touched.size(), 1u);
  EXPECT_TRUE(result.tiles_touched.front() == grid);

  // Cell A: draft cleared (reads no-data).
  const auto draft_a = store.get(SourceLayer::Draft, cell_a);
  ASSERT_TRUE(draft_a.has_value());
  EXPECT_FALSE(draft_a->hasData());
  // Cell B: processed no-data hole -> draft SURVIVES.
  const auto draft_b = store.get(SourceLayer::Draft, cell_b);
  ASSERT_TRUE(draft_b.has_value());
  ASSERT_TRUE(draft_b->hasData());
  EXPECT_DOUBLE_EQ(draft_b->depth, -8.0);
  // Cell C: draft never had data, so nothing to clear (still no-data).
  const auto draft_c = store.get(SourceLayer::Draft, cell_c);
  ASSERT_TRUE(draft_c.has_value());  // the A/B writes created the tile
  EXPECT_FALSE(draft_c->hasData());
}

TEST(Store, ClearOverlappedDraftNeverCreatesDraftTile)
{
  // Clearing must never CREATE a Draft tile where none existed: a processed tile
  // over an empty Draft layer clears nothing and leaves Draft empty (no spurious
  // all-NaN on-disk artifact).
  BathymetryStore store(11);
  const gggs::GridIndex grid = store.level().gridIndex(43.0, -70.5);
  marine_bathymetry_store::BathymetryTile processed(grid);
  processed.set(0, 0, BathyCell{-30.0, 0.1});

  const auto result = store.clearOverlappedDraft(processed);

  EXPECT_EQ(result.cells_cleared, 0u);
  EXPECT_TRUE(result.tiles_touched.empty());
  EXPECT_TRUE(store.tiles(SourceLayer::Draft).empty());
}

TEST(Store, ClearOverlappedDraftTileMapAggregatesAcrossTiles)
{
  // The tile-map overload clears across several grids in one call (a bulk regen),
  // aggregating the count and listing each touched grid once.
  BathymetryStore store(11);
  const gggs::GridIndex grid1 = store.level().gridIndex(43.0, -70.5);
  const gggs::GridIndex grid2 = store.level().gridIndex(44.0, -69.0);
  ASSERT_FALSE(grid1 == grid2);
  const gggs::CellIndex c1(grid1, 0, 0);
  const gggs::CellIndex c2(grid2, 0, 0);
  store.set(SourceLayer::Draft, c1, BathyCell{-9.0, 0.4});
  store.set(SourceLayer::Draft, c2, BathyCell{-7.0, 0.4});

  std::map<gggs::GridIndex, marine_bathymetry_store::BathymetryTile> processed;
  marine_bathymetry_store::BathymetryTile t1(grid1);
  t1.set(0, 0, BathyCell{-30.0, 0.1});
  marine_bathymetry_store::BathymetryTile t2(grid2);
  t2.set(0, 0, BathyCell{-31.0, 0.1});
  processed.emplace(grid1, std::move(t1));
  processed.emplace(grid2, std::move(t2));

  const auto result = store.clearOverlappedDraft(processed);

  EXPECT_EQ(result.cells_cleared, 2u);
  EXPECT_EQ(result.tiles_touched.size(), 2u);
  EXPECT_FALSE(store.get(SourceLayer::Draft, c1)->hasData());
  EXPECT_FALSE(store.get(SourceLayer::Draft, c2)->hasData());
}

// --- clearOverlappedDraft ACROSS GGGS LEVELS (uma#369) ---
// `processed` became depth-adaptive and mixed-level while `draft` stays
// fixed-level, so the two layers no longer share a level. Keyed on the processed
// tile's own GridIndex — which carries its level — the clear matched no draft
// tile, cleared zero cells and reported nothing, leaving superseded draft
// blunders to keep winning shallowestReliable. These pin the level-aware
// behaviour in both directions.

TEST(Store, ClearOverlappedDraftClearsFinerDraftUnderACoarserProcessedTile)
{
  // draft FINER than processed: each processed cell covers 16 level-12 draft
  // cells. Every draft cell under a processed data cell clears; those under a
  // processed no-data hole survive, exactly as at equal levels.
  BathymetryStore store(10);
  const gggs::GridIndex pgrid = gggs::Level(10).gridIndex(43.0, -70.5);
  const gggs::Level fine(12);

  const gggs::CellIndex covered(pgrid, 5, 7);    // processed has data here
  const gggs::CellIndex hole(pgrid, 5, 8);       // processed no-data hole

  std::vector<gggs::CellIndex> covered_draft;
  std::vector<gggs::CellIndex> hole_draft;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      const double lat_f = (i + 0.5) / 4.0;
      const double lon_f = (j + 0.5) / 4.0;
      covered_draft.push_back(fine.cellIndex(pointInCell(covered, lat_f, lon_f)));
      hole_draft.push_back(fine.cellIndex(pointInCell(hole, lat_f, lon_f)));
      store.set(SourceLayer::Draft, covered_draft.back(), BathyCell{-9.0, 0.4});
      store.set(SourceLayer::Draft, hole_draft.back(), BathyCell{-8.0, 0.4});
    }
  }

  marine_bathymetry_store::BathymetryTile processed(pgrid);
  processed.set(covered.row(), covered.column(), BathyCell{-30.0, 0.1});
  // `hole` deliberately left no-data.

  const auto result = store.clearOverlappedDraft(processed);

  EXPECT_EQ(result.cells_cleared, 16u) << "a finer draft tile was not reached";
  EXPECT_EQ(result.coarse_draft_cells_retained, 0u);
  EXPECT_FALSE(result.tiles_touched.empty());
  for (const auto & cell : covered_draft) {
    EXPECT_FALSE(store.get(SourceLayer::Draft, cell)->hasData()) << "not cleared";
  }
  for (const auto & cell : hole_draft) {
    EXPECT_TRUE(store.get(SourceLayer::Draft, cell)->hasData()) <<
      "cleared under a processed no-data hole";
  }
}

TEST(Store, ClearOverlappedDraftClearsCoarserDraftOnlyWhereFullySuperseded)
{
  // draft COARSER than processed: one level-10 draft cell covers 2x2 level-11
  // processed cells. Clearing it when only part of it is superseded would
  // discard draft data over ground this tile does not speak for, so it clears
  // only under full coverage — and the retained ones are COUNTED, not silent.
  BathymetryStore store(10);
  const gggs::GridIndex pgrid = gggs::Level(11).gridIndex(43.0, -70.5);
  const gggs::Level draft_level(10);
  marine_bathymetry_store::BathymetryTile processed(pgrid);

  // Two draft cells well inside the processed tile: one fully covered, one with
  // a single processed no-data cell under it.
  const auto inside = pointInCell(gggs::CellIndex(pgrid, 400, 400), 0.5, 0.5);
  const gggs::CellIndex full = draft_level.cellIndex(inside);
  const gggs::CellIndex partial =
    draft_level.cellIndex(pointInCell(full, 3.5, 0.5));    // three draft cells north
  ASSERT_FALSE(full == partial);
  store.set(SourceLayer::Draft, full, BathyCell{-9.0, 0.4});
  store.set(SourceLayer::Draft, partial, BathyCell{-8.0, 0.4});

  int full_cells = 0;
  int partial_cells = 0;
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      const double lat_f = (i + 0.5) / 2.0;
      const double lon_f = (j + 0.5) / 2.0;
      const auto pf = gggs::Level(11).cellIndex(pointInCell(full, lat_f, lon_f));
      ASSERT_TRUE(pf.grid() == pgrid);
      processed.set(pf.row(), pf.column(), BathyCell{-30.0, 0.1});
      ++full_cells;
      // Leave ONE of the four cells under `partial` no-data.
      if (i == 0 && j == 0) {
        continue;
      }
      const auto pp = gggs::Level(11).cellIndex(pointInCell(partial, lat_f, lon_f));
      ASSERT_TRUE(pp.grid() == pgrid);
      processed.set(pp.row(), pp.column(), BathyCell{-31.0, 0.1});
      ++partial_cells;
    }
  }
  ASSERT_EQ(full_cells, 4);
  ASSERT_EQ(partial_cells, 3);

  const auto result = store.clearOverlappedDraft(processed);

  EXPECT_EQ(result.cells_cleared, 1u) << "the fully superseded coarse draft cell";
  EXPECT_EQ(result.coarse_draft_cells_retained, 1u) <<
    "the partially covered one must be reported, not silently skipped";
  EXPECT_FALSE(store.get(SourceLayer::Draft, full)->hasData());
  ASSERT_TRUE(store.get(SourceLayer::Draft, partial)->hasData());
  EXPECT_DOUBLE_EQ(store.get(SourceLayer::Draft, partial)->depth, -8.0);
}

TEST(Store, ClearOverlappedDraftKeepsCoarseDraftStraddlingTheTileEdge)
{
  // A draft cell that extends PAST the processed tile's edge is kept and
  // counted: this tile does not speak for the rest of its ground, and a
  // wrongly-cleared draft cell is a lost hazard. Tile edges land mid-cell only
  // once the level gap exceeds 6 (960 cells per grid; 960/2^7 = 7.5), so this
  // is not a configuration the depth-adaptive ladder produces — it is here so
  // the rule is enforced rather than assumed away.
  BathymetryStore store(10);
  const gggs::GridIndex pgrid = gggs::Level(11).gridIndex(43.0, -70.5);
  const gggs::Level draft_level(4);     // 7 levels coarser: 7.5 cells per tile
  marine_bathymetry_store::BathymetryTile processed(pgrid);

  // Fill the whole processed tile with data — coverage is then limited only by
  // the tile's own extent.
  for (uint16_t r = 0; r < marine_bathymetry_store::BathymetryTile::edge; ++r) {
    for (uint16_t c = 0; c < marine_bathymetry_store::BathymetryTile::edge; ++c) {
      processed.set(r, c, BathyCell{-30.0, 0.1});
    }
  }

  // Does this coarse draft cell reach outside the processed tile?
  const auto straddles = [&pgrid](const gggs::CellIndex & cell) {
      const gggs::GridIndex & g = cell.grid();
      const double lat_per_cell = g.latitudinalSpan() / gggs::cell_rows_per_grid;
      const double lon_per_cell = g.longitudinalSpan() / gggs::cell_columns_per_grid;
      const auto sw = cell.position();
      return sw.latitude < pgrid.southLatitude() ||
             sw.longitude < pgrid.westLongitude() ||
             sw.latitude + lat_per_cell > pgrid.northLatitude() ||
             sw.longitude + lon_per_cell > pgrid.eastLongitude();
    };

  // The draft cells at the processed tile's four corners. With a 7.5-cell tile
  // at least one corner cell must overhang, whichever way the grid aligns.
  const double lat_nudge = 1e-9;
  const double lon_nudge = 1e-9;
  std::vector<gggs::CellIndex> corners;
  for (const double lat :
    {pgrid.southLatitude() + lat_nudge, pgrid.northLatitude() - lat_nudge})
  {
    for (const double lon :
      {pgrid.westLongitude() + lon_nudge, pgrid.eastLongitude() - lon_nudge})
    {
      corners.push_back(draft_level.cellIndex(gggs::geoPoint(lat, lon)));
    }
  }

  std::size_t expected_retained = 0;
  for (const auto & cell : corners) {
    store.set(SourceLayer::Draft, cell, BathyCell{-5.0, 0.4});
    if (straddles(cell)) {
      ++expected_retained;
    }
  }
  ASSERT_GT(expected_retained, 0u) << "no corner cell overhangs — test proves nothing";

  const auto result = store.clearOverlappedDraft(processed);

  EXPECT_EQ(result.coarse_draft_cells_retained, expected_retained) <<
    "a straddling coarse draft cell must be reported, not silently skipped";
  for (const auto & cell : corners) {
    const auto draft = store.get(SourceLayer::Draft, cell);
    ASSERT_TRUE(draft.has_value());
    if (straddles(cell)) {
      EXPECT_TRUE(draft->hasData()) << "cleared a draft cell this tile only partly covers";
      EXPECT_DOUBLE_EQ(draft->depth, -5.0);
    } else {
      EXPECT_FALSE(draft->hasData()) << "a fully superseded draft cell survived";
    }
  }
}
