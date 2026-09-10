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

/// @file
/// @brief CLI: import a depth/uncertainty GeoTIFF into a store layer.
///
/// usage: import_geotiff <store_dir> <layer> <geotiff>          (draft/processed/reference)
///        import_geotiff --stage <staged_dir> chart <geotiff>   (build chart layer)
///        import_geotiff --commit <staged_dir> <store_dir>      (atomic chart swap)
///                       [--cell-size m] [--level N]
///                       [--uncertainty m] [--depth-scale s] [--depth-offset m]
///                       [--platform P] [--sensor S] [--survey K] [--date D]
///
/// For `draft`/`processed`/`reference` this loads any existing tiles under
/// <store_dir>, imports the GeoTIFF into <layer>'s single fused surface (#221 — no
/// per-day epoch), then saves. A `processed` import also clears overlapped `draft`
/// cells cell-wise (ADR-0010 D8). `survey` is a deprecated alias for `processed`.
///
/// The `chart` layer follows ADR-0010 D7's wholesale-regeneration semantics
/// (#275): it is never written into the live store incrementally. Instead
/// `--stage` builds the chart layer in a staging directory
/// (`chart_staging_writable=true`), and one `--commit` performs the atomic
/// `replaceChartLayer` swap into the live store. `--commit` is an
/// offline/maintenance operation — see its notes in usage().
///
/// Per-cell time / source provenance was dropped in #248; coarse store-level
/// provenance (StoreMetadata) may be set with the --platform/--sensor/--survey/
/// --date flags and is written to <store_dir>/registry.json.

#include <gdal_priv.h>
#include <ogr_spatialref.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>

#include <geographic_msgs/msg/geo_point.hpp>

#include "marine_bathymetry_store/bathymetry_store.hpp"
#include "marine_bathymetry_store/geotiff_import.hpp"
#include "marine_bathymetry_store/registry.hpp"
#include "marine_bathymetry_store/tile_io.hpp"
#include "marine_vertical_datum/vdatum_query.hpp"

namespace
{

namespace fs = std::filesystem;

void usage()
{
  std::cout <<
    "usage: import_geotiff <store_dir> <layer> <geotiff>          (draft/processed/reference)\n"
    "       import_geotiff --stage <staged_dir> chart <geotiff>   (build chart layer)\n"
    "       import_geotiff --commit <staged_dir> <store_dir>      (atomic chart swap)\n"
    "                      [--cell-size m] [--level N]\n"
    "                      [--uncertainty m] [--depth-scale s] [--depth-offset m]\n"
    "                      [--source-datum mllw --geoid G --vdatum-dir D [--source-up]]\n"
    "                      [--platform P] [--sensor S] [--survey K] [--date D]\n"
    "  layer:      draft | processed | reference | chart\n"
    "              ('survey' is a deprecated alias for 'processed'; ADR-0010 D8)\n"
    "              a 'processed' import clears overlapped 'draft' cells cell-wise\n"
    "  --stage <staged_dir>: build the write-gated chart layer into <staged_dir>\n"
    "               (ADR-0010 D7). Layer must be 'chart'. The live store is not\n"
    "               touched; tiles land in <staged_dir>/chart/. Point repeated\n"
    "               --stage calls at the SAME <staged_dir> to accumulate cells,\n"
    "               but START EACH regeneration cycle from a FRESH/EMPTY dir:\n"
    "               --stage accumulates per GGGS grid tile (re-staging a grid\n"
    "               overwrites its tile; tiles for grids not re-staged persist),\n"
    "               so a reused non-empty dir carries stale prior-cycle tiles into\n"
    "               the wholesale swap (a warning is printed when the dir is\n"
    "               non-empty).\n"
    "  --commit <staged_dir> <store_dir>: atomically swap <staged_dir>/chart into\n"
    "               <store_dir> via replaceChartLayer (wholesale, never a merge).\n"
    "               OFFLINE/MAINTENANCE ONLY: run only while navigation is not\n"
    "               consuming the store (ADR-0010 D7 nav-down precondition). This\n"
    "               CLI does NOT enforce a nav-liveness check — the enforced check\n"
    "               lives in the cron updater (rolker/s57_tools#28). <staged_dir>\n"
    "               and <store_dir> must be on the SAME filesystem (the atomic\n"
    "               rename rejects a cross-device staged dir).\n"
    "  note: importing chart data does NOT activate it in the costmap; that is\n"
    "               gated on the cost-model rework (uma#276).\n"
    "  --cell-size: store default cell size in metres (default 0.5)\n"
    "  --level:     GGGS level to import at (default: derived from --cell-size).\n"
    "               The store is multi-level; pass a level to match the source.\n"
    "  --uncertainty: ignore the file's uncertainty band and assign this\n"
    "               constant 1-sigma value (for sources without one)\n"
    "  --depth-scale / --depth-offset: vertical conversion at import,\n"
    "               height = scale*pixel + offset (defaults 1, 0). A\n"
    "               positive-down depths-below-lake-surface product imports\n"
    "               with scale -1 and offset = lake surface ellipsoidal height\n"
    "  --source-datum mllw --geoid <grid> --vdatum-dir <dir> (#315):\n"
    "               per-cell vertical-datum conversion for a tidal-datum-\n"
    "               referenced grid: each pixel's offset is the MLLW\n"
    "               ellipsoidal height at its (lat, lon) from the VDatum\n"
    "               grids (the same grids the enc_updater provisions under\n"
    "               world/datum/), never a hand-computed constant. Pixels\n"
    "               are assumed POSITIVE-DOWN depths below the datum (the\n"
    "               hydrographic norm); pass --source-up for up-positive\n"
    "               heights relative to the datum. Mutually exclusive with\n"
    "               --depth-scale/--depth-offset; a point outside the grids'\n"
    "               coverage is a HARD ERROR (never a partial conversion).\n"
    "               Draft/processed/reference imports only (the chart path\n"
    "               converts in s57_to_geotiff). Records the source datum in\n"
    "               registry.json provenance.\n"
    "  --platform/--sensor/--survey/--date: coarse store-level provenance\n"
    "               (StoreMetadata) written to registry.json (ADR-0005 #248)\n";
  exit(1);
}

marine_bathymetry_store::SourceLayer layerFromName(const std::string & name)
{
  if (name == "processed") {
    return marine_bathymetry_store::SourceLayer::Processed;
  }
  if (name == "draft") {
    return marine_bathymetry_store::SourceLayer::Draft;
  }
  if (name == "survey") {
    // Deprecated alias: the pre-D8 fused `survey` layer is re-classified as
    // `processed` (ADR-0010 D8). Accept it so existing operator workflows keep
    // working, but warn loudly rather than silently retargeting.
    std::cerr << "warning: layer name 'survey' is deprecated; use 'processed' "
              << "(ADR-0010 D8 split survey into processed/draft)\n";
    return marine_bathymetry_store::SourceLayer::Processed;
  }
  if (name == "reference") {
    return marine_bathymetry_store::SourceLayer::Reference;
  }
  if (name == "chart") {
    return marine_bathymetry_store::SourceLayer::Chart;
  }
  std::cerr << "unknown layer '" << name
            << "' (expected draft|processed|reference|chart; 'survey' is a "
            << "deprecated alias for processed)\n";
  exit(1);
}

/// Count `.tif` tiles already staged under <staged_dir>/chart. Used to warn that
/// `--stage` appends: a reused non-empty staged dir would carry stale tiles from
/// a prior regeneration cycle into the wholesale D7 swap (each cycle must start
/// from a fresh dir). A dir that does not exist yet counts as zero.
std::size_t countStagedChartTiles(const std::string & staged_dir)
{
  const fs::path chart_dir = fs::path(staged_dir) / "chart";
  std::error_code ec;
  if (!fs::is_directory(chart_dir, ec)) {
    return 0;
  }
  std::size_t tiles = 0;
  for (const auto & entry : fs::directory_iterator(chart_dir, ec)) {
    if (entry.path().extension() == ".tif") {
      ++tiles;
    }
  }
  return tiles;
}

/// Geographic bounding box of @p path, from its geotransform alone.
///
/// Used to window the staged-tile adoption (#339): only tiles this source can
/// possibly touch need loading. Opening the dataset here is one extra open of a
/// file the importer opens anyway — negligible against the whole-staged-set
/// `load()` it replaces.
std::pair<geographic_msgs::msg::GeoPoint, geographic_msgs::msg::GeoPoint>
sourceGeoBounds(const std::string & path)
{
  GDALAllRegister();
  std::unique_ptr<GDALDataset, void (*)(GDALDataset *)> ds(
    static_cast<GDALDataset *>(GDALOpen(path.c_str(), GA_ReadOnly)),
    [](GDALDataset * d) {if (d) {GDALClose(d);}});
  if (!ds) {
    throw std::runtime_error("could not open " + path);
  }
  // Same guards importGeoTiff enforces, applied BEFORE any adoption. Without
  // them a projected (metres) or rotated source yields a lon/lat box that is
  // wildly wrong, so loadWindow adopts far too many staged tiles — the very
  // O(total tiles) cost the windowing exists to avoid — and only then does the
  // import fail on its own guard. Fail fast instead.
  const OGRSpatialReference * srs = ds->GetSpatialRef();
  OGRErr axis_error = OGRERR_NONE;
  if (srs == nullptr || !srs->IsGeographic() ||
    std::abs(srs->GetSemiMajor(&axis_error) - 6378137.0) > 1.0 || axis_error != OGRERR_NONE)
  {
    throw std::runtime_error("not a geographic WGS84 raster: " + path);
  }
  double gt[6] = {0, 1, 0, 0, 0, 1};
  if (ds->GetGeoTransform(gt) != CE_None) {
    throw std::runtime_error("missing geotransform in " + path);
  }
  if (gt[2] != 0.0 || gt[4] != 0.0) {
    throw std::runtime_error("rotated geotransform unsupported in " + path);
  }
  const double x0 = gt[0];
  const double y0 = gt[3];
  const double x1 = x0 + gt[1] * ds->GetRasterXSize();
  const double y1 = y0 + gt[5] * ds->GetRasterYSize();
  geographic_msgs::msg::GeoPoint min_pt;
  geographic_msgs::msg::GeoPoint max_pt;
  min_pt.longitude = std::min(x0, x1);
  min_pt.latitude = std::min(y0, y1);
  max_pt.longitude = std::max(x0, x1);
  max_pt.latitude = std::max(y0, y1);
  return {min_pt, max_pt};
}

}  // namespace

int main(int argc, char * argv[])
{
  std::string positional[3];
  int n_positional = 0;
  const char * stage_dir = nullptr;    // set by --stage: build chart into here
  const char * commit_dir = nullptr;   // set by --commit: staged dir to swap in
  double cell_size = 0.5;
  int level = 0;                        // valid only when level_set is true
  bool level_set = false;               // unset: derive the level from --cell-size
  double constant_uncertainty = -1.0;   // sentinel: use the file's band
  double depth_scale = 1.0;
  double depth_offset = 0.0;
  bool depth_scale_set = false;         // for the --source-datum exclusion
  bool depth_offset_set = false;
  const char * source_datum = nullptr;  // --source-datum (only "mllw" today)
  const char * geoid_grid = nullptr;    // --geoid (with --source-datum)
  const char * vdatum_dir = nullptr;    // --vdatum-dir (with --source-datum)
  bool source_up = false;               // --source-up: datum-relative heights
  marine_bathymetry_store::StoreMetadata metadata;

  const auto need_arg = [&](int & i) -> const char * {
      if (i + 1 >= argc) {
        usage();
      }
      return argv[++i];
    };
  // std::stod/std::stoi throw on non-numeric input and there is no enclosing
  // try block in main, so an unguarded parse aborts via std::terminate.
  // Parse through these helpers instead: bad input is a clean usage error.
  // Finite-only because a NaN/inf here either hits undefined behavior
  // (--cell-size -> log2/int-cast in gggs::Level::fromCellSize) or silently
  // poisons imported depths (--depth-scale/--depth-offset) or silently
  // disables the flag (--uncertainty: NaN fails the >= 0 sentinel gate).
  const auto parse_finite = [](const char * flag, const char * val) -> double {
      double parsed = 0.0;
      try {
        parsed = std::stod(val);
      } catch (const std::exception &) {
        std::cerr << "bad " << flag << " '" << val << "'\n";
        std::exit(1);
      }
      if (!std::isfinite(parsed)) {
        std::cerr << "bad " << flag << " '" << val << "' (want a finite number)\n";
        std::exit(1);
      }
      return parsed;
    };

  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--stage") == 0) {
      stage_dir = need_arg(i);
    } else if (std::strcmp(argv[i], "--commit") == 0) {
      commit_dir = need_arg(i);
    } else if (std::strcmp(argv[i], "--cell-size") == 0) {
      const char * val = need_arg(i);
      cell_size = parse_finite("--cell-size", val);
      // Non-positive values flow into gggs::Level::fromCellSize() where
      // std::log2(<=0) yields inf/NaN and the following static_cast<int> is
      // undefined behavior — same guard as s102_import.
      if (cell_size <= 0.0) {
        std::cerr << "bad --cell-size '" << val << "' (want a positive finite number)\n";
        return 1;
      }
    } else if (std::strcmp(argv[i], "--level") == 0) {
      const char * val = need_arg(i);
      try {
        level = std::stoi(val);
      } catch (const std::exception &) {
        std::cerr << "bad --level '" << val << "'\n";
        return 1;
      }
      // Validate the range at parse time so a negative level is a clean error
      // rather than being silently ignored later (it no longer aliases a
      // "derive from --cell-size" sentinel — level_set carries that instead).
      if (level < 0 || level > 20) {
        std::cerr << "invalid --level " << level << " (GGGS levels are 0..20)\n";
        return 1;
      }
      level_set = true;
    } else if (std::strcmp(argv[i], "--uncertainty") == 0) {
      constant_uncertainty = parse_finite("--uncertainty", need_arg(i));
    } else if (std::strcmp(argv[i], "--depth-scale") == 0) {
      depth_scale = parse_finite("--depth-scale", need_arg(i));
      depth_scale_set = true;
    } else if (std::strcmp(argv[i], "--depth-offset") == 0) {
      depth_offset = parse_finite("--depth-offset", need_arg(i));
      depth_offset_set = true;
    } else if (std::strcmp(argv[i], "--source-datum") == 0) {
      source_datum = need_arg(i);
    } else if (std::strcmp(argv[i], "--geoid") == 0) {
      geoid_grid = need_arg(i);
    } else if (std::strcmp(argv[i], "--vdatum-dir") == 0) {
      vdatum_dir = need_arg(i);
    } else if (std::strcmp(argv[i], "--source-up") == 0) {
      source_up = true;
    } else if (std::strcmp(argv[i], "--platform") == 0) {
      metadata.platform = need_arg(i);
    } else if (std::strcmp(argv[i], "--sensor") == 0) {
      metadata.sensor = need_arg(i);
    } else if (std::strcmp(argv[i], "--survey") == 0) {
      metadata.survey = need_arg(i);
    } else if (std::strcmp(argv[i], "--date") == 0) {
      metadata.date = need_arg(i);
    } else if (n_positional < 3) {
      positional[n_positional++] = argv[i];
    } else {
      usage();
    }
  }

  // --stage and --commit are mutually exclusive modes, and neither combines with
  // a normal survey/reference import.
  if (stage_dir != nullptr && commit_dir != nullptr) {
    std::cerr << "--stage and --commit are mutually exclusive\n";
    usage();
  }

  // --source-datum validation (#315). Every mis-combination is a loud usage
  // error — a datum flag silently ignored is exactly how a wrong-datum
  // surface would reach the store.
  if (source_datum != nullptr) {
    if (std::strcmp(source_datum, "mllw") != 0) {
      std::cerr << "--source-datum '" << source_datum
                << "' unsupported (only 'mllw' today)\n";
      usage();
    }
    if (stage_dir != nullptr || commit_dir != nullptr) {
      std::cerr << "--source-datum applies to survey/reference imports only; "
                << "the chart path converts datums in s57_to_geotiff\n";
      usage();
    }
    if (depth_scale_set || depth_offset_set) {
      std::cerr << "--source-datum and --depth-scale/--depth-offset are "
                << "mutually exclusive (the datum query IS the conversion; "
                << "use --source-up for up-positive datum-relative heights)\n";
      usage();
    }
    if (geoid_grid == nullptr || vdatum_dir == nullptr) {
      std::cerr << "--source-datum requires both --geoid and --vdatum-dir\n";
      usage();
    }
  } else if (geoid_grid != nullptr || vdatum_dir != nullptr || source_up) {
    std::cerr << "--geoid/--vdatum-dir/--source-up require --source-datum\n";
    usage();
  }

  // --commit mode: atomic wholesale swap of the staged chart layer into a store.
  // No store loading or import; the staged dir is <staged>/chart and the sole
  // positional is <store_dir>.
  if (commit_dir != nullptr) {
    if (n_positional != 1) {
      usage();
    }
    const std::string & store_dir = positional[0];
    const std::string staged_chart = (fs::path(commit_dir) / "chart").string();
    try {
      marine_bathymetry_store::replaceChartLayer(staged_chart, store_dir);
    } catch (const std::exception & e) {
      std::cerr << "commit failed: " << e.what() << "\n";
      return 1;
    }
    std::cout << "committed chart layer " << staged_chart << " -> " << store_dir << "\n";
    return 0;
  }

  // --stage mode: build the write-gated chart layer into <staged_dir>. The live
  // store is untouched; positionals are <layer> <geotiff> and layer must be
  // chart. Nothing is loaded — each cycle accumulates via the OS directory.
  if (stage_dir != nullptr) {
    if (n_positional != 2) {
      usage();
    }
    // Provenance flags write registry.json into the staged dir, but --commit
    // swaps only <staged>/chart/ into the live store, so staged provenance never
    // reaches it. Reject them here rather than silently dropping the metadata at
    // commit time (fail loud over silent drop).
    if (!metadata.empty()) {
      std::cerr << "--platform/--sensor/--survey/--date are not supported with "
                << "--stage: --commit swaps only chart/, so staged provenance "
                << "would be silently dropped\n";
      usage();
    }
    const auto layer = layerFromName(positional[0]);
    if (layer != marine_bathymetry_store::SourceLayer::Chart) {
      std::cerr << "--stage requires layer 'chart' (got '" << positional[0]
                << "'); survey/reference import directly into <store_dir>\n";
      usage();
    }
    const std::string & geotiff = positional[1];

    const std::size_t already = countStagedChartTiles(stage_dir);
    if (already > 0) {
      std::cerr << "warning: " << (fs::path(stage_dir) / "chart").string() << " already holds "
                << already << " tile(s); --stage appends. Start each wholesale "
                << "regeneration cycle from a fresh/empty staged dir (ADR-0010 D7) "
                << "or stale tiles from a prior cycle will be committed.\n";
    }

    auto store = marine_bathymetry_store::BathymetryStore::fromCellSize(
      static_cast<float>(cell_size), /*reference_writable=*/false,
      /*chart_staging_writable=*/true);
    std::cout << "store default level: " << static_cast<int>(store.level().level()) << "\n";

    // [#339] Adopt what is already staged WHERE THIS SOURCE LANDS, so this cell
    // merges into it. Without any adoption the store starts empty and
    // importTiles replaces whole tiles, so a cell sharing a GGGS tile with a
    // previously staged neighbour discarded that neighbour's cells — silently,
    // and depending on staging order.
    //
    // Windowed, not wholesale: `load()` GDAL-opens every staged tile, so
    // staging N sources would cost O(N x total tiles) full-tile reads and a
    // real regional corpus (~80 ENC cells) would never finish. `loadWindow`
    // gates each tile on its FILENAME-derived bounding box before paying any
    // GDAL cost, and skips tiles already resident, so each source pays only for
    // the tiles it actually touches. Loaded tiles are clean, so `save` below
    // still writes only the tiles this import dirtied.
    // Mirror the normal import path's error contract (see the try/catch around
    // importGeoTiff/save below): a bad source must print a diagnosis and exit
    // non-zero, never terminate. This block gained four throw sites with the
    // windowed adoption (#339) — the two sourceGeoBounds guards and loadWindow
    // — so on a regional regeneration one unreadable or non-geographic cell out
    // of dozens would otherwise abort the whole run.
    try {
      const auto source_bounds = sourceGeoBounds(geotiff);
      const std::size_t adopted = marine_bathymetry_store::loadWindow(
      store, stage_dir, source_bounds.first, source_bounds.second);
      if (adopted > 0) {
        std::cout << "adopted " << adopted <<
          " already-staged tile(s) overlapping this source, to merge into\n";
      }

      marine_bathymetry_store::GeoTiffImportOptions options;
      options.depth_scale = depth_scale;
      options.depth_offset = depth_offset;
      if (level_set) {
        options.level = static_cast<uint8_t>(level);
      }
      if (constant_uncertainty >= 0.0) {
        options.uncertainty_band = 0;
        options.default_uncertainty = constant_uncertainty;
      }
      options.merge_into_resident = true;
      const auto staged = marine_bathymetry_store::importGeoTiff(
      store, layer, geotiff, options);
      std::cout << "imported " << staged.cells_imported << " cell(s) into chart\n";
      if (staged.resident_contentions > 0) {
      // Expected zero: ENC cells partition their usage band, so no two of them
      // should claim the same cell. A non-zero count means two sources disagree
      // about the same ground (a rescheme overlap, or genuinely overlapping
      // products such as S-102 — see #316, where the arbitration rule is still
      // an open decision). The lowest-uncertainty rule resolved them; say so.
        std::cout << "warning: " << staged.resident_contentions
                  << " contention(s) with already-staged data — two sources "
                  << "disagree about the same cell; kept the lower-uncertainty "
                  << "value\n";
      }

      // Provenance flags are rejected above, so the staged store never carries a
      // registry.json (it would be dropped by the chart-only --commit anyway).
      const std::size_t saved = marine_bathymetry_store::save(store, stage_dir, nullptr);
      std::cout << "saved " << saved << " tile(s) under " << stage_dir << "\n";
    } catch (const std::exception & e) {
      std::cerr << "stage failed: " << e.what() << "\n";
      return 1;
    }
    return 0;
  }

  // Normal mode: survey/reference import into the live store.
  if (n_positional != 3) {
    usage();
  }
  const std::string & store_dir = positional[0];
  const auto layer = layerFromName(positional[1]);
  const std::string & geotiff = positional[2];
  if (layer == marine_bathymetry_store::SourceLayer::Chart) {
    std::cerr << "chart imports must use --stage/--commit (ADR-0010 D7 wholesale "
              << "regeneration); a chart layer is never written into a live store "
              << "incrementally\n";
    usage();
  }

  // The importer is the one sanctioned writer of the read-only Reference prior,
  // so it opts into reference_writable only when the target layer is
  // Reference (ADR-0002 §D3). For Survey this stays false, so a typo'd layer
  // can't touch the prior.
  const bool reference_writable =
    (layer == marine_bathymetry_store::SourceLayer::Reference);
  auto store = marine_bathymetry_store::BathymetryStore::fromCellSize(
    static_cast<float>(cell_size), reference_writable);
  std::cout << "store default level: " << static_cast<int>(store.level().level()) << "\n";

  marine_bathymetry_store::StoreMetadata existing_metadata;
  const std::size_t loaded =
    marine_bathymetry_store::load(store, store_dir, &existing_metadata);
  std::cout << "loaded " << loaded << " existing tile(s) from " << store_dir << "\n";

  marine_bathymetry_store::GeoTiffImportOptions options;
  options.depth_scale = depth_scale;
  options.depth_offset = depth_offset;
  if (level_set) {
    options.level = static_cast<uint8_t>(level);
  }
  if (constant_uncertainty >= 0.0) {
    options.uncertainty_band = 0;
    options.default_uncertainty = constant_uncertainty;
  }
  if (source_datum != nullptr) {
    // Per-cell MLLW -> ellipsoid conversion (#315). Build the query ONCE
    // (it owns the PROJ context; per-point work is a single proj_trans) and
    // pass it in as the per-point offset resolver. Setup failure — missing
    // or unreadable grids — is a hard error here, before any pixel is read.
    marine_vertical_datum::VDatumConfig vconfig;
    vconfig.geoid_grid = geoid_grid;
    vconfig.vdatum_grid_dir = vdatum_dir;
    // Only mllw_z is consumed: skip the MHHW pipeline so a million-pixel
    // import doesn't pay a second, discarded proj_trans per pixel.
    vconfig.want_mhhw = false;
    const marine_vertical_datum::VDatumQueryFn vquery =
      marine_vertical_datum::make_vdatum_query(
      vconfig, [](const std::string & msg) {
        std::cerr << "vdatum: " << msg << "\n";
      });
    if (!vquery) {
      std::cerr << "--source-datum mllw: vdatum query setup failed "
                << "(see vdatum messages above; check --geoid/--vdatum-dir)\n";
      return 1;
    }
    options.vertical_datum_fn =
      [vquery](double lat, double lon) -> std::optional<double> {
        const auto result = vquery(lat, lon);
        if (!result.has_value()) {
          return std::nullopt;
        }
        return result->mllw_z;
      };
    // Positive-down depths below the datum by default (hydrographic norm):
    // h_ellip = mllw_z - depth. --source-up keeps +1 for up-positive
    // heights relative to the datum: h_ellip = mllw_z + height.
    options.depth_scale = source_up ? 1.0 : -1.0;
    metadata.datum = source_datum;
  }
  // Clean failure for runtime errors (#315 review): a vdatum-coverage gap a
  // few pixels past the grids' edge is a routine operational condition of
  // --source-datum, not misuse — it must print and exit 1, never abort via
  // std::terminate (matching the --commit path's catch). Nothing partial can
  // land either way: tiles accumulate in memory and the store save runs
  // after a successful import.
  marine_bathymetry_store::ProcessedImportResult import_result;
  try {
    import_result =
      marine_bathymetry_store::importGeoTiff(store, layer, geotiff, options);
  } catch (const std::exception & e) {
    std::cerr << "import failed: " << e.what() << "\n";
    return 1;
  }
  std::cout << "imported " << import_result.cells_imported << " cell(s) into "
            << positional[1] << "\n";
  // ADR-0010 D8 anti-clobber: a processed import supersedes overlapped live draft
  // cells. Report what it cleared (the draft_tiles_touched list is the cache-
  // invalidation seam for camp#171/#172).
  if (layer == marine_bathymetry_store::SourceLayer::Processed &&
    import_result.draft_cells_cleared > 0)
  {
    std::cout << "cleared " << import_result.draft_cells_cleared
              << " overlapped draft cell(s) across "
              << import_result.draft_tiles_touched.size() << " draft tile(s)\n";
  }
  // Residue: coarse draft cells this import overlaps but does not fully
  // supersede, so they were KEPT (uma#369). Shoal-safe, but not nothing — each
  // is a superseded draft value still winning shallowestReliable over ground
  // this import speaks for only in part. Report it, or "counted rather than
  // silent" stops at the store's API boundary.
  if (layer == marine_bathymetry_store::SourceLayer::Processed &&
    import_result.coarse_draft_cells_retained > 0)
  {
    std::cout << "kept " << import_result.coarse_draft_cells_retained
              << " coarser draft cell(s) this import only partly supersedes"
              << " (neighbouring imports still owe the rest of their ground)\n";
  }

  // Field-wise provenance merge (#315 review): CLI-supplied fields update
  // the registry, everything else carries forward from what was loaded. A
  // whole-record replace here would let a pure conversion run
  // (--source-datum with no provenance flags) blank previously recorded
  // platform/sensor/survey/date — or a later single-flag run erase the
  // datum stamp while the converted cells remain in the store.
  marine_bathymetry_store::StoreMetadata merged = existing_metadata;
  if (!metadata.platform.empty()) {merged.platform = metadata.platform;}
  if (!metadata.sensor.empty()) {merged.sensor = metadata.sensor;}
  if (!metadata.survey.empty()) {merged.survey = metadata.survey;}
  if (!metadata.date.empty()) {merged.date = metadata.date;}
  if (!metadata.datum.empty()) {merged.datum = metadata.datum;}
  const marine_bathymetry_store::StoreMetadata * to_write =
    merged.empty() ? nullptr : &merged;
  std::size_t saved = 0;
  try {
    saved = marine_bathymetry_store::save(store, store_dir, to_write);
  } catch (const std::exception & e) {
    std::cerr << "save failed: " << e.what() << "\n";
    return 1;
  }
  std::cout << "saved " << saved << " tile(s) under " << store_dir << "\n";
  return 0;
}
