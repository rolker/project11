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

#include "marine_bathymetry_store/geotiff_import.hpp"

#include <gdal_priv.h>
#include <ogr_spatialref.h>

#include <cmath>
#include <cstdint>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace marine_bathymetry_store
{

namespace
{

/// RAII wrapper so a thrown exception still closes the GDAL dataset.
struct DatasetCloser
{
  GDALDataset * ds = nullptr;
  ~DatasetCloser() {if (ds) {GDALClose(ds);}}
};

}  // namespace

ProcessedImportResult importGeoTiff(
  BathymetryStore & store, SourceLayer layer,
  const std::string & path, const GeoTiffImportOptions & options)
{
  if (options.depth_band < 1) {
    throw std::invalid_argument("importGeoTiff: depth_band must be >= 1");
  }
  if (options.merge_into_resident && layer == SourceLayer::Processed) {
    // Seeding from the resident layer puts cells OUTSIDE this import's footprint
    // into `tiles`, and a Processed import hands that same map to
    // clearOverlappedDraft (ADR-0010 D8) — which would then clear Draft under
    // previously-imported Processed data, well beyond what this import touched.
    // Anti-clobber destroys data, so refuse rather than trust the caller.
    //
    // The flag is for assembling a layer from many disjoint sources (a chart
    // layer built cell-by-cell from ENC cells). A Processed re-import is meant
    // to supersede per tile — a shrinking survey must drop the cells it no
    // longer covers — so merging is the wrong semantics there anyway.
    throw std::invalid_argument(
      "importGeoTiff: merge_into_resident is not supported for the Processed "
      "layer — seeded cells outside the import footprint would drive "
      "clearOverlappedDraft to clear Draft data this import never covered");
  }
  if (options.uncertainty_band < 0) {
    throw std::invalid_argument("importGeoTiff: uncertainty_band must be >= 0 (0 = none)");
  }
  if (options.vertical_datum_fn && options.depth_offset != 0.0) {
    throw std::invalid_argument(
      "importGeoTiff: vertical_datum_fn and a non-zero depth_offset are "
      "mutually exclusive (the datum query IS the per-point offset)");
  }

  // Target import level: a caller-specified level (multi-level, ADR-0002 §D2) or
  // the store's default level.
  const gggs::Level level =
    options.level.has_value() ? gggs::Level(*options.level) : store.level();

  GDALAllRegister();
  DatasetCloser in;
  in.ds = GDALDataset::FromHandle(GDALOpen(path.c_str(), GA_ReadOnly));
  if (in.ds == nullptr) {
    throw std::runtime_error("importGeoTiff: could not open " + path);
  }
  const int width = in.ds->GetRasterXSize();
  const int height = in.ds->GetRasterYSize();
  const int band_count = in.ds->GetRasterCount();
  if (options.depth_band > band_count || options.uncertainty_band > band_count) {
    throw std::runtime_error("importGeoTiff: band index beyond raster bands in " + path);
  }

  // The pixel-center mapping below interprets the geotransform as degrees, so
  // the file must be a geographic WGS84 raster (same guard as tile_io's load).
  const OGRSpatialReference * srs = in.ds->GetSpatialRef();
  OGRErr axis_error = OGRERR_NONE;
  if (srs == nullptr || !srs->IsGeographic() ||
    std::abs(srs->GetSemiMajor(&axis_error) - 6378137.0) > 1.0 || axis_error != OGRERR_NONE)
  {
    throw std::runtime_error("importGeoTiff: not a geographic WGS84 raster: " + path);
  }

  double gt[6];
  if (in.ds->GetGeoTransform(gt) != CE_None) {
    throw std::runtime_error("importGeoTiff: missing geotransform in " + path);
  }
  if (gt[2] != 0.0 || gt[4] != 0.0) {
    throw std::runtime_error("importGeoTiff: rotated geotransform unsupported in " + path);
  }

  // Honor a declared no-data value: it may be finite (e.g. -9999), which the
  // isfinite() skip below would happily import as a 9 km depth.
  int has_nodata = 0;
  const double nodata =
    in.ds->GetRasterBand(options.depth_band)->GetNoDataValue(&has_nodata);
  // Honor the uncertainty band's no-data sentinel too: a positive finite
  // sentinel (e.g. 9999) would otherwise pass the isfinite/>0 gate below and
  // import as a real, huge-but-finite uncertainty rather than being treated as
  // missing -- the same trap the depth band is guarded against.
  int unc_has_nodata = 0;
  double unc_nodata = 0.0;
  if (options.uncertainty_band > 0) {
    unc_nodata =
      in.ds->GetRasterBand(options.uncertainty_band)->GetNoDataValue(&unc_has_nodata);
  }

  std::map<gggs::GridIndex, BathymetryTile> tiles;
  std::size_t imported = 0;
  std::size_t resident_contentions = 0;

  std::vector<double> depth_row(width);
  std::vector<double> uncertainty_row(width);
  for (int y = 0; y < height; ++y) {
    if (in.ds->GetRasterBand(options.depth_band)->RasterIO(
        GF_Read, 0, y, width, 1, depth_row.data(), width, 1, GDT_Float64, 0, 0) != CE_None)
    {
      throw std::runtime_error("importGeoTiff: depth read failed in " + path);
    }
    if (options.uncertainty_band > 0) {
      if (in.ds->GetRasterBand(options.uncertainty_band)->RasterIO(
          GF_Read, 0, y, width, 1, uncertainty_row.data(), width, 1, GDT_Float64, 0, 0) !=
        CE_None)
      {
        throw std::runtime_error("importGeoTiff: uncertainty read failed in " + path);
      }
    }
    const double latitude = gt[3] + (y + 0.5) * gt[5];
    for (int x = 0; x < width; ++x) {
      if (!std::isfinite(depth_row[x]) || (has_nodata && depth_row[x] == nodata)) {
        continue;   // no-data pixel
      }
      const double longitude = gt[0] + (x + 0.5) * gt[1];
      // Vertical conversion at import (§D4): pixel value -> ellipsoidal height.
      // With a per-point datum resolver (#315) the offset is the source
      // datum's ellipsoidal height at this pixel; no coverage there is a
      // hard error — a partially converted surface must never import.
      double offset = options.depth_offset;
      if (options.vertical_datum_fn) {
        const std::optional<double> datum_z =
          options.vertical_datum_fn(latitude, longitude);
        if (!datum_z.has_value()) {
          throw std::runtime_error(
            "importGeoTiff: vertical-datum grids have no coverage at (" +
            std::to_string(latitude) + ", " + std::to_string(longitude) +
            ") — refusing a partial datum conversion for " + path);
        }
        offset = *datum_z;
      }
      const double depth = options.depth_scale * depth_row[x] + offset;
      // A non-finite OR non-positive uncertainty is *missing*, not perfect:
      // zero would pass every reliability gate and carry infinite weight in
      // 1/sigma^2 fusion. Such cells get default_uncertainty (NaN by default
      // = never reliable — conservative).
      double uncertainty = options.default_uncertainty;
      if (options.uncertainty_band > 0 && std::isfinite(uncertainty_row[x]) &&
        uncertainty_row[x] > 0.0 &&
        !(unc_has_nodata && uncertainty_row[x] == unc_nodata))
      {
        uncertainty = uncertainty_row[x];
      }
      // Fill the pixel's full FOOTPRINT of store cells, not just the cell
      // under its centre — a coarser-than-store source (e.g. a 5 m contour
      // prior into a 0.5 m store) must produce coverage, not isolated dots.
      // The box is shrunk by a hair so adjacent pixels never contend for the
      // cells on their shared boundary; for aligned or finer inputs it
      // therefore covers exactly the one containing cell.
      const double half_lon = 0.495 * std::abs(gt[1]);
      const double half_lat = 0.495 * std::abs(gt[5]);
      const auto box_min = gggs::geoPoint(latitude - half_lat, longitude - half_lon);
      const auto box_max = gggs::geoPoint(latitude + half_lat, longitude + half_lon);
      gggs::GridAreaIterator grid_it(
        level.gridIndex(box_min.latitude, box_min.longitude),
        level.gridIndex(box_max.latitude, box_max.longitude));
      for (; grid_it.valid(); grid_it.next()) {
        auto tile_it = tiles.find(*grid_it);
        // [#339] The tile this grid may already hold in the LAYER (not in this
        // import's working set). importTiles inserts whole tiles, so without
        // seeding from it a second source touching this grid would discard the
        // first source's cells — which is what two adjacent ENC cells sharing a
        // GGGS tile at their seam do on every chart regeneration.
        const BathymetryTile * resident = nullptr;
        if (options.merge_into_resident) {
          const auto & resident_map = store.tiles(layer);
          const auto found = resident_map.find(*grid_it);
          if (found != resident_map.end()) {
            resident = &found->second;
          }
        }
        for (gggs::CellAreaIterator cell_it(*grid_it, box_min, box_max);
          cell_it.valid(); cell_it.next())
        {
          const gggs::CellIndex & cell = *cell_it;
          if (!cell.valid()) {
            continue;   // outside GGGS's usable envelope
          }
          if (tile_it == tiles.end()) {
            tile_it = tiles.emplace(
              cell.grid(),
              resident != nullptr ? *resident : BathymetryTile(cell.grid())).first;
          }
          if (resident != nullptr) {
            // Alarm, not accounting: a source pixel landing where the layer
            // already holds a DIFFERENT value means two sources disagree about
            // the same ground. Zero for sources that do not overlap.
            const BathyCell res = resident->get(cell.row(), cell.column());
            const bool same_sigma = res.uncertainty == uncertainty ||
              (std::isnan(res.uncertainty) && std::isnan(uncertainty));
            if (res.hasData() && !(res.depth == depth && same_sigma)) {
              ++resident_contentions;
            }
          }
          // Several input pixels can land in one cell when the input is finer
          // than the store level: keep the lowest-uncertainty value (a finite
          // uncertainty always beats NaN; ties keep the first read).
          const BathyCell existing = tile_it->second.get(cell.row(), cell.column());
          if (existing.hasData()) {
            const bool existing_unc_finite = std::isfinite(existing.uncertainty);
            const bool new_unc_finite = std::isfinite(uncertainty);
            if (!new_unc_finite && existing_unc_finite) {
              continue;
            }
            if (new_unc_finite && existing_unc_finite &&
              uncertainty >= existing.uncertainty)
            {
              continue;
            }
            if (!new_unc_finite && !existing_unc_finite) {
              continue;   // tie among NaNs: keep the first read
            }
          } else {
            ++imported;   // a new cell (replacements don't recount)
          }
          tile_it->second.set(cell.row(), cell.column(), BathyCell{depth, uncertainty});
        }
      }
    }
  }

  ProcessedImportResult result;
  result.cells_imported = imported;
  result.resident_contentions = resident_contentions;

  // Anti-clobber (ADR-0010 D8): a Processed import supersedes overlapped live
  // Draft cells. Delegate to the store's public `clearOverlappedDraft` (the store
  // owns this semantics so cube's direct-`saveTile` regen paths apply it
  // identically). Clear BEFORE the move below consumes `tiles` — Draft and
  // Processed are independent layer maps, so mutating Draft here does not disturb
  // the processed tiles still headed for importTiles(). Gated-drop holes (processed
  // no-data cells) leave the draft cell intact; see BathymetryStore::clearOverlappedDraft.
  if (layer == SourceLayer::Processed) {
    const DraftClearResult cleared = store.clearOverlappedDraft(tiles);
    result.draft_cells_cleared = cleared.cells_cleared;
    result.draft_tiles_touched = std::move(cleared.tiles_touched);
    // Carry the retained-residue count out to the caller. Counting it in the
    // store and dropping it here would make "counted rather than silent" true
    // only inside the store — the operator running the import is who needs to
    // see that a superseded draft blunder is still in there.
    result.coarse_draft_cells_retained = cleared.coarse_draft_cells_retained;
  }

  // Bulk-insert into the layer's single fused surface (#221). The Reference
  // read-only gate is enforced by importTiles (throws logic_error if the store
  // is not reference_writable).
  store.importTiles(layer, std::move(tiles));
  return result;
}

}  // namespace marine_bathymetry_store
