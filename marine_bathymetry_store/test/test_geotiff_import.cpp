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

#include <gdal_priv.h>
#include <ogr_spatialref.h>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "marine_autonomy/gggs.h"
#include "marine_bathymetry_store/bathymetry_store.hpp"
#include "marine_bathymetry_store/geotiff_import.hpp"
#include "marine_bathymetry_store/tile_io.hpp"

using marine_bathymetry_store::BathyCell;
using marine_bathymetry_store::BathymetryStore;
using marine_bathymetry_store::GeoTiffImportOptions;
using marine_bathymetry_store::importGeoTiff;
using marine_bathymetry_store::SourceLayer;

namespace fs = std::filesystem;

namespace
{

/// Write a small north-up WGS84 GeoTIFF whose lattice starts at the north-west
/// corner of the GGGS grid containing (43.0, -70.5) at @p level, with
/// @p pixels_per_cell input pixels per store cell along each axis. Band 1 =
/// depth, band 2 = uncertainty (Float32, NaN no-data).
std::string writeTestTiff(
  const fs::path & path, const gggs::Level & level, int width, int height,
  const std::vector<float> & depth, const std::vector<float> & uncertainty,
  int pixels_per_cell = 1)
{
  const gggs::GridIndex grid = level.gridIndex(43.0, -70.5);
  const double cell_x = grid.longitudinalSpan() / gggs::cell_rows_per_grid;
  const double cell_y = grid.latitudinalSpan() / gggs::cell_rows_per_grid;
  const double px = cell_x / pixels_per_cell;
  const double py = cell_y / pixels_per_cell;

  GDALAllRegister();
  GDALDriver * driver = GetGDALDriverManager()->GetDriverByName("GTiff");
  if (driver == nullptr) {
    throw std::runtime_error("writeTestTiff: GTiff driver unavailable");
  }
  GDALDataset * ds =
    driver->Create(path.string().c_str(), width, height, 2, GDT_Float32, nullptr);
  if (ds == nullptr) {
    throw std::runtime_error("writeTestTiff: could not create " + path.string());
  }
  double gt[6] = {grid.westLongitude(), px, 0.0, grid.northLatitude(), 0.0, -py};
  ds->SetGeoTransform(gt);
  OGRSpatialReference srs;
  srs.SetWellKnownGeogCS("WGS84");
  char * wkt = nullptr;
  srs.exportToWkt(&wkt);
  ds->SetProjection(wkt);
  CPLFree(wkt);
  if (ds->GetRasterBand(1)->RasterIO(
      GF_Write, 0, 0, width, height, const_cast<float *>(depth.data()), width, height,
      GDT_Float32, 0, 0) != CE_None ||
    ds->GetRasterBand(2)->RasterIO(
      GF_Write, 0, 0, width, height, const_cast<float *>(uncertainty.data()), width, height,
      GDT_Float32, 0, 0) != CE_None)
  {
    GDALClose(ds);
    throw std::runtime_error("writeTestTiff: band write failed");
  }
  GDALClose(ds);
  return path.string();
}

/// The NW-corner store cell of the grid containing (43.0, -70.5) at @p level
/// (the cell whose centre is half a cell in from the grid's NW corner).
gggs::CellIndex nwCell(const BathymetryStore & store, const gggs::Level & level)
{
  const gggs::GridIndex grid = level.gridIndex(43.0, -70.5);
  const double cell_x = grid.longitudinalSpan() / gggs::cell_rows_per_grid;
  const double cell_y = grid.latitudinalSpan() / gggs::cell_rows_per_grid;
  return store.cellIndex(
    grid.northLatitude() - 0.5 * cell_y, grid.westLongitude() + 0.5 * cell_x);
}

}  // namespace

class GeoTiffImportTest : public ::testing::Test
{
protected:
  fs::path dir_;

  void SetUp() override
  {
    const std::string name = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    dir_ = fs::temp_directory_path() / ("marine_bathy_import_" + name);
    fs::remove_all(dir_);
    fs::create_directories(dir_);
  }

  void TearDown() override
  {
    fs::remove_all(dir_);
  }
};

TEST_F(GeoTiffImportTest, AlignedImportIsCellExact)
{
  BathymetryStore store(11);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  // 3x3 pixels at the grid's NW corner; centre pixel is no-data.
  const std::vector<float> depth{
    -30.0f, -31.0f, -32.0f,
    -33.0f, nan, -35.0f,
    -36.0f, -37.0f, -38.0f};
  const std::vector<float> unc{
    0.1f, 0.2f, 0.3f,
    0.4f, nan, 0.6f,
    0.7f, 0.8f, 0.9f};
  const auto tif = writeTestTiff(dir_ / "aligned.tif", store.level(), 3, 3, depth, unc);

  GeoTiffImportOptions options;
  const auto imported = importGeoTiff(store, SourceLayer::Draft, tif, options).cells_imported;
  EXPECT_EQ(imported, 8u);   // 9 pixels minus the no-data centre

  const auto nw = store.get(SourceLayer::Draft, nwCell(store, store.level()));
  ASSERT_TRUE(nw.has_value());
  EXPECT_FLOAT_EQ(static_cast<float>(nw->depth), -30.0f);
  EXPECT_FLOAT_EQ(static_cast<float>(nw->uncertainty), 0.1f);

  // The import landed in the layer's single fused surface (#221).
  EXPECT_FALSE(store.tiles(SourceLayer::Draft).empty());
}

TEST_F(GeoTiffImportTest, ImportsAtCallerSpecifiedLevel)
{
  // Multi-level (ADR-0002 §D2): import at a level other than the store default.
  // Importing to the read-only Reference prior requires the importer opt-in.
  BathymetryStore store(11, /*reference_writable=*/true);   // default level 11
  const std::vector<float> depth{-30.0f};
  const std::vector<float> unc{0.1f};
  gggs::Level coarse(8);
  const auto tif = writeTestTiff(dir_ / "lvl.tif", coarse, 1, 1, depth, unc);

  GeoTiffImportOptions options;
  options.level = 8;   // import at level 8, not the store's default 11
  EXPECT_EQ(importGeoTiff(store, SourceLayer::Reference, tif, options).cells_imported, 1u);

  // The imported tile is at level 8, queryable at that level. The fixture's
  // single pixel sits at the level-8 grid's NW corner, so query that cell.
  const gggs::GridIndex grid8 = coarse.gridIndex(43.0, -70.5);
  const double cell_x = grid8.longitudinalSpan() / gggs::cell_rows_per_grid;
  const double cell_y = grid8.latitudinalSpan() / gggs::cell_rows_per_grid;
  const auto cell = coarse.cellIndex(gggs::geoPoint(
      grid8.northLatitude() - 0.5 * cell_y, grid8.westLongitude() + 0.5 * cell_x));
  const auto got = store.get(SourceLayer::Reference, cell);
  ASSERT_TRUE(got.has_value());
  EXPECT_FLOAT_EQ(static_cast<float>(got->depth), -30.0f);
  // And the stored grid carries the level-8 prefix.
  const auto & tiles = store.tiles(SourceLayer::Reference);
  ASSERT_FALSE(tiles.empty());
  EXPECT_EQ(tiles.begin()->first.level(), 8u);
}

TEST_F(GeoTiffImportTest, FinerInputKeepsLowestUncertainty)
{
  BathymetryStore store(11);
  // 2x2 input pixels per store cell, covering exactly one cell: four candidate
  // values; the lowest-uncertainty one must win.
  const std::vector<float> depth{-30.0f, -31.0f, -32.0f, -33.0f};
  const std::vector<float> unc{0.5f, 0.2f, 0.9f, 0.7f};
  const auto tif = writeTestTiff(dir_ / "fine.tif", store.level(), 2, 2, depth, unc, 2);

  const auto imported = importGeoTiff(store, SourceLayer::Draft, tif).cells_imported;
  EXPECT_EQ(imported, 1u);   // four pixels, one cell

  const auto got = store.get(SourceLayer::Draft, nwCell(store, store.level()));
  ASSERT_TRUE(got.has_value());
  EXPECT_FLOAT_EQ(static_cast<float>(got->depth), -31.0f);       // unc 0.2 wins
  EXPECT_FLOAT_EQ(static_cast<float>(got->uncertainty), 0.2f);
}

TEST_F(GeoTiffImportTest, ReimportMergesLastWriteWins)
{
  // #221: there is no per-day epoch or provenance ordering — a re-import simply
  // merges into the layer's single fused surface (last-write-wins per cell).
  BathymetryStore store(11);
  const std::vector<float> unc{0.1f};
  const auto first =
    writeTestTiff(dir_ / "first.tif", store.level(), 1, 1, {-30.0f}, unc);
  const auto second =
    writeTestTiff(dir_ / "second.tif", store.level(), 1, 1, {-28.0f}, unc);

  EXPECT_EQ(importGeoTiff(store, SourceLayer::Draft, first).cells_imported, 1u);
  EXPECT_DOUBLE_EQ(
    store.get(SourceLayer::Draft, nwCell(store, store.level()))->depth, -30.0);

  // A second import of the same cell overwrites the first.
  EXPECT_EQ(importGeoTiff(store, SourceLayer::Draft, second).cells_imported, 1u);
  EXPECT_DOUBLE_EQ(
    store.get(SourceLayer::Draft, nwCell(store, store.level()))->depth, -28.0);
}

TEST_F(GeoTiffImportTest, MissingUncertaintyBandUsesDefault)
{
  BathymetryStore store(11);
  const std::vector<float> depth{-30.0f};
  const std::vector<float> unc{0.1f};   // present in the file but ignored
  const auto tif = writeTestTiff(dir_ / "nounc.tif", store.level(), 1, 1, depth, unc);

  GeoTiffImportOptions options;
  options.uncertainty_band = 0;
  options.default_uncertainty = 3.5;
  EXPECT_EQ(importGeoTiff(store, SourceLayer::Draft, tif, options).cells_imported, 1u);

  const auto got = store.get(SourceLayer::Draft, nwCell(store, store.level()));
  ASSERT_TRUE(got.has_value());
  EXPECT_DOUBLE_EQ(got->uncertainty, 3.5);
}

TEST_F(GeoTiffImportTest, DepthScaleOffsetAndFiniteNoData)
{
  // A lake-prior product: depths positive-down below the surface, with a FINITE
  // no-data value (-9999) that must not import as a 9 km depth.
  BathymetryStore store(11, /*reference_writable=*/true);  // imports to prior
  const std::vector<float> depth{4.0f, -9999.0f};   // 4 m of water; one no-data
  const std::vector<float> unc{0.0f, 0.0f};         // source has no real band
  const auto tif = writeTestTiff(dir_ / "lake.tif", store.level(), 2, 1, depth, unc);
  {
    GDALAllRegister();
    GDALDataset * ds = GDALDataset::FromHandle(GDALOpen(tif.c_str(), GA_Update));
    ASSERT_NE(ds, nullptr);
    ds->GetRasterBand(1)->SetNoDataValue(-9999.0);
    GDALClose(ds);
  }

  GeoTiffImportOptions options;
  options.uncertainty_band = 0;
  options.default_uncertainty = 3.0;
  options.depth_scale = -1.0;       // positive-down -> up-positive
  options.depth_offset = -20.0;     // lake surface ellipsoidal height
  const auto imported = importGeoTiff(store, SourceLayer::Reference, tif, options).cells_imported;
  EXPECT_EQ(imported, 1u);         // the -9999 pixel was skipped

  const auto got = store.get(SourceLayer::Reference, nwCell(store, store.level()));
  ASSERT_TRUE(got.has_value());
  EXPECT_DOUBLE_EQ(got->depth, -24.0);          // -1 * 4 + (-20)
  EXPECT_DOUBLE_EQ(got->uncertainty, 3.0);
}

TEST_F(GeoTiffImportTest, CoarserInputFillsPixelFootprint)
{
  // A 5x-coarser pixel must fill its whole footprint of store cells — coverage,
  // not isolated dots (the contour-prior case). The helper only does integer
  // pixels-per-cell, so build the 1x1 raster with a 5-cell pixel directly.
  BathymetryStore store(11, /*reference_writable=*/true);  // imports to prior
  const std::vector<float> depth{-30.0f};
  const std::vector<float> unc{3.0f};
  const gggs::GridIndex grid = store.level().gridIndex(43.0, -70.5);
  const double cell_x = grid.longitudinalSpan() / gggs::cell_rows_per_grid;
  const double cell_y = grid.latitudinalSpan() / gggs::cell_rows_per_grid;
  const auto path = (dir_ / "coarse.tif").string();
  {
    GDALAllRegister();
    GDALDriver * driver = GetGDALDriverManager()->GetDriverByName("GTiff");
    ASSERT_NE(driver, nullptr);
    GDALDataset * ds = driver->Create(path.c_str(), 1, 1, 2, GDT_Float32, nullptr);
    ASSERT_NE(ds, nullptr);
    double gt[6] = {grid.westLongitude(), 5.0 * cell_x, 0.0,
      grid.northLatitude(), 0.0, -5.0 * cell_y};
    ds->SetGeoTransform(gt);
    OGRSpatialReference srs;
    srs.SetWellKnownGeogCS("WGS84");
    char * wkt = nullptr;
    srs.exportToWkt(&wkt);
    ds->SetProjection(wkt);
    CPLFree(wkt);
    ASSERT_EQ(
      ds->GetRasterBand(1)->RasterIO(
        GF_Write, 0, 0, 1, 1, const_cast<float *>(depth.data()), 1, 1, GDT_Float32, 0, 0),
      CE_None);
    ASSERT_EQ(
      ds->GetRasterBand(2)->RasterIO(
        GF_Write, 0, 0, 1, 1, const_cast<float *>(unc.data()), 1, 1, GDT_Float32, 0, 0),
      CE_None);
    GDALClose(ds);
  }

  const auto imported = importGeoTiff(store, SourceLayer::Reference, path).cells_imported;
  EXPECT_EQ(imported, 25u);   // 5x5 store cells under the one pixel

  // Every cell of the footprint carries the value — spot-check a middle one.
  const auto mid = store.cellIndex(
    grid.northLatitude() - 2.5 * cell_y, grid.westLongitude() + 2.5 * cell_x);
  const auto got = store.get(SourceLayer::Reference, mid);
  ASSERT_TRUE(got.has_value());
  EXPECT_FLOAT_EQ(static_cast<float>(got->depth), -30.0f);
}

TEST_F(GeoTiffImportTest, ZeroUncertaintyIsMissingNotPerfect)
{
  // A source cell with uncertainty 0 must not import as "perfectly certain" —
  // it gets default_uncertainty (NaN here), so it never passes a gate.
  BathymetryStore store(11);
  const std::vector<float> depth{-30.0f, -31.0f};
  const std::vector<float> unc{0.0f, 0.4f};
  const auto tif = writeTestTiff(dir_ / "zerounc.tif", store.level(), 2, 1, depth, unc);

  const auto imported = importGeoTiff(store, SourceLayer::Draft, tif).cells_imported;
  EXPECT_EQ(imported, 2u);   // both cells import (depth is real)

  const gggs::GridIndex grid = store.level().gridIndex(43.0, -70.5);
  const double cell_x = grid.longitudinalSpan() / gggs::cell_rows_per_grid;
  const double cell_y = grid.latitudinalSpan() / gggs::cell_rows_per_grid;
  const auto zero_cell = store.cellIndex(
    grid.northLatitude() - 0.5 * cell_y, grid.westLongitude() + 0.5 * cell_x);
  const auto good_cell = store.cellIndex(
    grid.northLatitude() - 0.5 * cell_y, grid.westLongitude() + 1.5 * cell_x);

  const auto zero = store.get(SourceLayer::Draft, zero_cell);
  ASSERT_TRUE(zero.has_value());
  EXPECT_TRUE(std::isnan(zero->uncertainty));   // missing, not 0

  const auto good = store.get(SourceLayer::Draft, good_cell);
  ASSERT_TRUE(good.has_value());
  EXPECT_FLOAT_EQ(static_cast<float>(good->uncertainty), 0.4f);
}

TEST_F(GeoTiffImportTest, MissingFileThrows)
{
  BathymetryStore store(11);
  EXPECT_THROW(
    importGeoTiff(store, SourceLayer::Draft, (dir_ / "absent.tif").string()),
    std::runtime_error);
}

namespace
{
/// Count `.tif` tiles directly under @p layer_dir (non-recursive).
std::size_t countTiles(const fs::path & layer_dir)
{
  std::size_t n = 0;
  if (!fs::is_directory(layer_dir)) {
    return 0;
  }
  for (const auto & entry : fs::directory_iterator(layer_dir)) {
    if (entry.path().extension() == ".tif") {
      ++n;
    }
  }
  return n;
}
}  // namespace

// Synthetic in-tree stand-in for the issue's "Lewes corpus" acceptance test:
// the full `--stage` -> `--commit` chart round-trip at the library level, with
// explicit pass/fail criteria (exact tile count staged and committed; per-cell
// depth and sigma equal the source within tolerance). The manual Lewes corpus
// procedure is documented in the PR (no in-tree NOAA fixture — kept deterministic
// and CI-runnable). See plan.md, must-fix 2.
TEST_F(GeoTiffImportTest, ChartStageCommitRoundTripPreservesDepthAndSigma)
{
  // A staging store (chart_staging_writable) is the only store that may write
  // the Chart layer (ADR-0010 D7). Import a fully-valid 3x3 tile at the store
  // level: nine known depths, uniform sigma.
  BathymetryStore staging(11, /*reference_writable=*/false, /*chart_staging_writable=*/true);
  const std::vector<float> depth{
    -30.0f, -31.0f, -32.0f,
    -33.0f, -34.0f, -35.0f,
    -36.0f, -37.0f, -38.0f};
  const std::vector<float> unc{
    1.5f, 1.5f, 1.5f,
    1.5f, 1.5f, 1.5f,
    1.5f, 1.5f, 1.5f};
  const auto tif = writeTestTiff(dir_ / "chart.tif", staging.level(), 3, 3, depth, unc);

  const auto imported = importGeoTiff(staging, SourceLayer::Chart, tif).cells_imported;
  EXPECT_EQ(imported, 9u);   // all nine pixels are valid

  // --stage: save the chart layer into a staged directory. The 3x3 import sits
  // within one GGGS grid, so exactly one value tile is written under chart/.
  const fs::path staged = dir_ / "staged";
  const std::size_t saved = marine_bathymetry_store::save(staging, staged.string());
  EXPECT_EQ(saved, 1u);
  EXPECT_EQ(countTiles(staged / "chart"), 1u);   // exact staged tile count

  // --commit: atomic wholesale swap into a fresh store dir. The same single tile
  // must land under <store>/chart/.
  const fs::path store_dir = dir_ / "store";
  fs::create_directories(store_dir);
  marine_bathymetry_store::replaceChartLayer(
    (staged / "chart").string(), store_dir.string());
  EXPECT_EQ(countTiles(store_dir / "chart"), 1u);   // exact committed tile count

  // Reload through a normal (non-staging) runtime store and assert every
  // imported cell's depth and sigma equal the source within the Float64
  // round-trip tolerance.
  BathymetryStore runtime(11);
  EXPECT_EQ(marine_bathymetry_store::load(runtime, store_dir.string()), 1u);

  const gggs::GridIndex grid = runtime.level().gridIndex(43.0, -70.5);
  const double cell_x = grid.longitudinalSpan() / gggs::cell_rows_per_grid;
  const double cell_y = grid.latitudinalSpan() / gggs::cell_rows_per_grid;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      const auto cell = runtime.cellIndex(
        grid.northLatitude() - (row + 0.5) * cell_y,
        grid.westLongitude() + (col + 0.5) * cell_x);
      const auto got = runtime.get(SourceLayer::Chart, cell);
      ASSERT_TRUE(got.has_value()) << "missing chart cell at row " << row << " col " << col;
      EXPECT_NEAR(got->depth, depth[row * 3 + col], 1e-6);
      EXPECT_NEAR(got->uncertainty, unc[row * 3 + col], 1e-6);
    }
  }
}

// --- Cell-wise anti-clobber: a Processed import clears overlapped Draft (D8) ---

namespace
{
// The store cell one column east of the NW-corner cell of the grid containing
// (43.0, -70.5) at @p level (shares the grid/tile with nwCell).
gggs::CellIndex nwCellEast(const BathymetryStore & store, const gggs::Level & level)
{
  const gggs::GridIndex grid = level.gridIndex(43.0, -70.5);
  const double cell_x = grid.longitudinalSpan() / gggs::cell_rows_per_grid;
  const double cell_y = grid.latitudinalSpan() / gggs::cell_rows_per_grid;
  return store.cellIndex(
    grid.northLatitude() - 0.5 * cell_y, grid.westLongitude() + 1.5 * cell_x);
}
}  // namespace

/// [#339] The merge flag must be refused for Processed, not merely unused there.
///
/// Seeding from the resident layer puts cells OUTSIDE the import footprint into
/// the tiles map, and a Processed import hands that same map to
/// clearOverlappedDraft (ADR-0010 D8) — which would clear Draft under
/// previously-imported Processed data the import never covered. Unreachable via
/// the CLI today (the flag is set only on the chart-only --stage path), so this
/// pins the guard against a future caller rather than a current bug.
TEST_F(GeoTiffImportTest, MergeIntoResidentIsRefusedForProcessed)
{
  BathymetryStore store(11);
  const std::vector<float> depth{-30.0f, -31.0f, -32.0f, -33.0f};
  const std::vector<float> unc{0.1f, 0.2f, 0.3f, 0.4f};
  const auto tif = writeTestTiff(dir_ / "merge_guard.tif", store.level(), 2, 2, depth, unc);

  GeoTiffImportOptions options;
  options.merge_into_resident = true;
  EXPECT_THROW(
    importGeoTiff(store, SourceLayer::Processed, tif, options),
    std::invalid_argument);

  // Chart is the intended use and must still be accepted.
  BathymetryStore chart_store =
    BathymetryStore::fromCellSize(store.level().cellSize(), false, true);
  EXPECT_NO_THROW(importGeoTiff(chart_store, SourceLayer::Chart, tif, options));
}

TEST_F(GeoTiffImportTest, AntiClobberCellWiseClearsOnlyCoveredDraftCells)
{
  // ADR-0010 D8: a Processed import clears overlapped Draft cells CELL-WISE — only
  // where the import has data. Draft cell A (covered by the import) is cleared;
  // Draft cell B (same tile, not covered) survives.
  BathymetryStore store(11);
  const auto cell_a = nwCell(store, store.level());       // NW-corner cell
  const auto cell_b = nwCellEast(store, store.level());   // one east, same tile
  ASSERT_TRUE(cell_a.grid() == cell_b.grid());
  store.set(SourceLayer::Draft, cell_a, BathyCell{-9.0, 0.4});
  store.set(SourceLayer::Draft, cell_b, BathyCell{-8.0, 0.4});

  // A 1x1 processed raster at the NW corner covers exactly cell A.
  const auto tif = writeTestTiff(dir_ / "proc.tif", store.level(), 1, 1, {-30.0f}, {0.1f});
  const auto result = importGeoTiff(store, SourceLayer::Processed, tif);

  EXPECT_EQ(result.cells_imported, 1u);
  EXPECT_EQ(result.draft_cells_cleared, 1u);
  ASSERT_EQ(result.draft_tiles_touched.size(), 1u);
  EXPECT_TRUE(result.draft_tiles_touched.front() == cell_a.grid());

  // Cell A: Draft cleared (reads no-data), Processed now holds the imported value.
  const auto draft_a = store.get(SourceLayer::Draft, cell_a);
  ASSERT_TRUE(draft_a.has_value());
  EXPECT_FALSE(draft_a->hasData());
  EXPECT_DOUBLE_EQ(store.get(SourceLayer::Processed, cell_a)->depth, -30.0);

  // Cell B: not covered by the import, so the Draft value survives untouched.
  const auto draft_b = store.get(SourceLayer::Draft, cell_b);
  ASSERT_TRUE(draft_b.has_value());
  ASSERT_TRUE(draft_b->hasData());
  EXPECT_DOUBLE_EQ(draft_b->depth, -8.0);
}

TEST_F(GeoTiffImportTest, AntiClobberSurfacesTheRetainedCoarseResidue)
{
  // uma#369 round 2: `coarse_draft_cells_retained` is counted in the store; it
  // has to REACH the caller, or "counted rather than silent" stops at the API
  // boundary and the first depth-adaptive import — exactly when someone needs to
  // see the residue — reports nothing. A coarse draft cell partly covered by a
  // finer processed import must appear in ProcessedImportResult.
  BathymetryStore store(11);
  // Draft one level COARSER than the import: the level-10 draft cell covers four
  // level-11 processed cells, and the 1x1 raster fills only one of them, so the
  // import does not fully supersede it — kept, and reported.
  const gggs::Level draft_level(10);
  const auto nw = nwCell(store, store.level());
  const auto draft_cell = draft_level.cellIndex(nw.position());
  store.set(SourceLayer::Draft, draft_cell, BathyCell{-9.0, 0.4});

  const auto tif = writeTestTiff(dir_ / "partial.tif", store.level(), 1, 1, {-30.0f}, {0.1f});
  const auto result = importGeoTiff(store, SourceLayer::Processed, tif);

  EXPECT_EQ(result.cells_imported, 1u);
  EXPECT_EQ(result.draft_cells_cleared, 0u) << "a partly-covered coarse cell must not clear";
  EXPECT_EQ(result.coarse_draft_cells_retained, 1u)
    << "the retained-residue count was dropped at the importer's API boundary";

  const auto draft = store.get(SourceLayer::Draft, draft_cell);
  ASSERT_TRUE(draft.has_value());
  EXPECT_TRUE(draft->hasData()) << "cleared a draft cell this import only partly covers";
}

TEST_F(GeoTiffImportTest, AntiClobberReportsNoResidueForANonProcessedImport)
{
  // Only a Processed import clears Draft, so the residue count stays 0 — pinned
  // so the new field cannot pick up a value on a path that clears nothing.
  BathymetryStore store(11);
  const gggs::Level draft_level(10);
  const auto draft_cell = draft_level.cellIndex(nwCell(store, store.level()).position());
  store.set(SourceLayer::Draft, draft_cell, BathyCell{-9.0, 0.4});

  const auto tif = writeTestTiff(dir_ / "draft.tif", store.level(), 1, 1, {-30.0f}, {0.1f});
  const auto result = importGeoTiff(store, SourceLayer::Draft, tif);

  EXPECT_EQ(result.draft_cells_cleared, 0u);
  EXPECT_EQ(result.coarse_draft_cells_retained, 0u);
}

TEST_F(GeoTiffImportTest, AntiClobberGatedDropHolePreservesDraft)
{
  // The invariant that mandates CELL-WISE (not tile-wise) clearing: a processed
  // re-run leaves gated-drop holes (no-data cells). Draft data under such a hole
  // must SURVIVE — the query overlay shows it where Processed has nothing.
  BathymetryStore store(11);
  const auto cell_a = nwCell(store, store.level());        // will be covered (data)
  const auto cell_hole = nwCellEast(store, store.level());  // processed no-data hole
  store.set(SourceLayer::Draft, cell_a, BathyCell{-9.0, 0.4});
  store.set(SourceLayer::Draft, cell_hole, BathyCell{-8.0, 0.4});

  // 2x1 processed raster: col 0 has data (covers A), col 1 is no-data (the hole
  // over cell_hole).
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const auto tif = writeTestTiff(
    dir_ / "hole.tif", store.level(), 2, 1, {-30.0f, nan}, {0.1f, nan});
  const auto result = importGeoTiff(store, SourceLayer::Processed, tif);

  EXPECT_EQ(result.cells_imported, 1u);      // only the col-0 pixel has data
  EXPECT_EQ(result.draft_cells_cleared, 1u);  // only cell A cleared

  // Covered cell A: Draft cleared.
  EXPECT_FALSE(store.get(SourceLayer::Draft, cell_a)->hasData());
  // Gated-drop hole cell: Processed has no data there, so Draft SURVIVES.
  const auto draft_hole = store.get(SourceLayer::Draft, cell_hole);
  ASSERT_TRUE(draft_hole.has_value());
  ASSERT_TRUE(draft_hole->hasData());
  EXPECT_DOUBLE_EQ(draft_hole->depth, -8.0);
}

TEST_F(GeoTiffImportTest, ProcessedImportCreatesNoSpuriousDraftTile)
{
  // Clearing must never CREATE a Draft tile where none existed: importing
  // Processed into a store with no Draft data leaves the Draft layer empty.
  BathymetryStore store(11);
  const auto tif = writeTestTiff(dir_ / "proc.tif", store.level(), 1, 1, {-30.0f}, {0.1f});
  const auto result = importGeoTiff(store, SourceLayer::Processed, tif);

  EXPECT_EQ(result.cells_imported, 1u);
  EXPECT_EQ(result.draft_cells_cleared, 0u);
  EXPECT_TRUE(result.draft_tiles_touched.empty());
  EXPECT_TRUE(store.tiles(SourceLayer::Draft).empty());
}

TEST_F(GeoTiffImportTest, DraftImportDoesNotClearAnything)
{
  // Only a Processed import clears Draft. A Draft import returns zeroed anti-clobber
  // fields (and, trivially, does not clear itself).
  BathymetryStore store(11);
  const auto tif = writeTestTiff(dir_ / "draft.tif", store.level(), 1, 1, {-30.0f}, {0.1f});
  const auto result = importGeoTiff(store, SourceLayer::Draft, tif);

  EXPECT_EQ(result.cells_imported, 1u);
  EXPECT_EQ(result.draft_cells_cleared, 0u);
  EXPECT_TRUE(result.draft_tiles_touched.empty());
}

// --- Per-point vertical-datum conversion (#315) ---

TEST_F(GeoTiffImportTest, VerticalDatumFnAppliesPerPointOffset)
{
  // An MLLW-referenced grid: each pixel's offset is the datum's ellipsoidal
  // height AT THAT POINT, not one constant — inject a longitude-dependent
  // resolver and check two same-row cells convert with different offsets.
  BathymetryStore store(11, /*reference_writable=*/true);
  const std::vector<float> depth{10.0f, 10.0f};   // equal depths below datum
  const std::vector<float> unc{0.5f, 0.5f};
  const auto tif = writeTestTiff(dir_ / "mllw.tif", store.level(), 2, 1, depth, unc);

  GeoTiffImportOptions options;
  options.depth_scale = -1.0;   // positive-down depths below the datum
  const gggs::GridIndex grid = store.level().gridIndex(43.0, -70.5);
  const double cell_x = grid.longitudinalSpan() / gggs::cell_rows_per_grid;
  const double split = grid.westLongitude() + cell_x;   // between the 2 pixels
  options.vertical_datum_fn =
    [split](double, double lon) -> std::optional<double> {
      return lon < split ? -28.0 : -29.0;   // western vs eastern mllw_z
    };
  const auto imported =
    importGeoTiff(store, SourceLayer::Reference, tif, options).cells_imported;
  EXPECT_EQ(imported, 2u);

  const auto west = store.get(SourceLayer::Reference, nwCell(store, store.level()));
  ASSERT_TRUE(west.has_value());
  EXPECT_DOUBLE_EQ(west->depth, -38.0);    // -1 * 10 + (-28)
  const auto east = store.get(
    SourceLayer::Reference,
    store.cellIndex(
      grid.northLatitude() - 0.5 * grid.latitudinalSpan() / gggs::cell_rows_per_grid,
      grid.westLongitude() + 1.5 * cell_x));
  ASSERT_TRUE(east.has_value());
  EXPECT_DOUBLE_EQ(east->depth, -39.0);    // -1 * 10 + (-29)
}

TEST_F(GeoTiffImportTest, VerticalDatumFnNoCoverageIsHardError)
{
  // A point outside the datum grids must abort the import — a partially
  // converted surface must never land in the store.
  BathymetryStore store(11, /*reference_writable=*/true);
  const std::vector<float> depth{10.0f};
  const std::vector<float> unc{0.5f};
  const auto tif = writeTestTiff(dir_ / "gap.tif", store.level(), 1, 1, depth, unc);

  GeoTiffImportOptions options;
  options.depth_scale = -1.0;
  options.vertical_datum_fn =
    [](double, double) -> std::optional<double> {return std::nullopt;};
  EXPECT_THROW(
    importGeoTiff(store, SourceLayer::Reference, tif, options), std::runtime_error);
  EXPECT_TRUE(store.tiles(SourceLayer::Reference).empty());
}

TEST_F(GeoTiffImportTest, VerticalDatumFnExcludesConstantOffset)
{
  // The datum query IS the offset; combining it with a constant one is a
  // caller bug and must fail loudly, not silently sum.
  BathymetryStore store(11, /*reference_writable=*/true);
  const std::vector<float> depth{10.0f};
  const std::vector<float> unc{0.5f};
  const auto tif = writeTestTiff(dir_ / "both.tif", store.level(), 1, 1, depth, unc);

  GeoTiffImportOptions options;
  options.depth_offset = -20.0;
  options.vertical_datum_fn =
    [](double, double) -> std::optional<double> {return -28.0;};
  EXPECT_THROW(
    importGeoTiff(store, SourceLayer::Reference, tif, options), std::invalid_argument);
}
