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

#include "marine_sidescan_mosaic/bathy_dem.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <sstream>
#include <stdexcept>

#include "marine_tiled_raster_store/tile_io.hpp"

namespace marine_sidescan_mosaic
{

namespace fs = std::filesystem;
namespace mtrs = marine_tiled_raster_store;

// ADR-0010 D8 split the pre-D8 fused `survey/` layer into `processed/` (offline
// re-run, authoritative) and `draft/` (live CUBE). The old `survey,reference`
// default is preserved in behaviour by consulting both new layers in store
// priority order (`processed > draft`) plus the `reference` prior — pre-D8
// `survey/` fused live+re-run, so equivalent coverage post-D8 is
// `processed` ∪ `draft`.
const char kDefaultBathyLayers[] = "processed,draft,reference";

namespace
{

/// @brief Split @p s on @p sep, keeping empty fields (unlike @ref splitCsv).
std::vector<std::string> splitOn(const std::string & s, char sep)
{
  std::vector<std::string> out;
  std::string item;
  std::istringstream stream(s);
  while (std::getline(stream, item, sep)) {
    out.push_back(item);
  }
  return out;
}

/// @brief The GGGS level encoded as the leading integer of a tile filename
///   (`<level>_<row>_<col>.tif`), or `nullopt` if the name is not one.
///
/// This is the **only** filename parse the reader performs. The full
/// filename→`GridIndex` inverse is not available here: `gggs::GridIndex`'s
/// `(level,row,column)` constructor is private and `marine_bathymetry_store`
/// keeps its derive helper in an anonymous namespace. The lookup therefore runs
/// the other way — `Level::gridIndex(lat,lon)` → `tileFilename()` → membership
/// test against the scanned name set (see `BathyDem::tileFor`).
/// The whole stem must be exactly three unsigned integer fields. A looser prefix
/// test also accepts the store's companion rasters (`<level>_<row>_<col>_time.tif`,
/// `..._source.tif`) that `marine_bathymetry_store` itself skips: those carry no
/// depth band, so counting them would inflate the tile count — enough to satisfy
/// the zero-tile hard-fail for a store whose every VALUE tile is missing, while
/// every lookup misses (#297 review).
std::optional<int> levelFromName(const std::string & name)
{
  if (name.size() < 5 || name.compare(name.size() - 4, 4, ".tif") != 0) {
    return std::nullopt;
  }
  const std::string stem = name.substr(0, name.size() - 4);
  const auto fields = splitOn(stem, '_');
  if (fields.size() != 3) {
    return std::nullopt;
  }
  for (const auto & field : fields) {
    if (field.empty() || field.find_first_not_of("0123456789") != std::string::npos) {
      return std::nullopt;
    }
  }
  // `stoi` rather than `atoi`: an overflowing digit run is UB in `atoi`, while
  // `stoi` throws — and a 30-digit "level" is a name we simply do not recognise.
  int level = 0;
  try {
    level = std::stoi(fields[0]);
  } catch (const std::exception &) {
    return std::nullopt;
  }
  if (level < 0 || level >= static_cast<int>(gggs::levels.size())) {
    return std::nullopt;
  }
  return level;
}

}  // namespace

std::vector<std::string> splitCsv(const std::string & csv)
{
  std::vector<std::string> out;
  std::string item;
  std::istringstream stream(csv);
  while (std::getline(stream, item, ',')) {
    const auto begin = item.find_first_not_of(" \t");
    if (begin == std::string::npos) {
      continue;
    }
    const auto end = item.find_last_not_of(" \t");
    out.push_back(item.substr(begin, end - begin + 1));
  }
  return out;
}

BathyDem::BathyDem(
  const std::string & store_root,
  const std::vector<std::string> & layer_names,
  std::size_t cache_tiles)
: store_root_(store_root), cache_tiles_(std::max<std::size_t>(1, cache_tiles))
{
  if (layer_names.empty()) {
    throw std::runtime_error("BathyDem: no layer names requested (see --bathy-layers)");
  }
  if (!fs::exists(store_root_)) {
    throw std::runtime_error("BathyDem: bathy store root does not exist: " + store_root_);
  }

  // A name listed twice would otherwise be scanned twice: double-counted
  // `tile_count_`, described twice, and two `layers_` entries feeding one
  // name-keyed lookup counter. De-duplicate, keeping the first (highest-priority)
  // occurrence, and say so rather than silently accepting the list (#297 review).
  std::vector<std::string> requested;
  std::set<std::string> requested_seen;
  for (const auto & name : layer_names) {
    if (requested_seen.insert(name).second) {
      requested.push_back(name);
    } else {
      warnings_.push_back(
        "requested bathy layer '" + name + "/' appears more than once in the search order; "
        "the later occurrence is ignored (the first sets its priority).");
    }
  }

  std::set<int> levels_seen;
  std::vector<std::string> missing;
  std::vector<std::string> empty;
  for (const auto & name : requested) {
    const fs::path dir = fs::path(store_root_) / name;
    if (!fs::is_directory(dir)) {
      missing.push_back(name);
      continue;
    }
    Layer layer;
    layer.name = name;
    layer.dir = dir.string();
    std::set<int> layer_levels;
    for (const auto & entry : fs::directory_iterator(dir)) {
      if (!entry.is_regular_file()) {
        continue;
      }
      const std::string filename = entry.path().filename().string();
      const auto level = levelFromName(filename);
      if (!level) {
        continue;   // not a value tile (registry.json, overviews sidecar, ...).
      }
      layer.files.insert(filename);
      layer_levels.insert(*level);
      levels_seen.insert(*level);
      ++tile_count_;
    }
    // Finest (highest level number) first: a fine patch wins over the coarse
    // regional surface where both cover a position.
    layer.levels.assign(layer_levels.rbegin(), layer_levels.rend());
    if (layer.files.empty()) {
      empty.push_back(name);
      continue;
    }
    layer_names_.push_back(layer.name);
    layers_.push_back(std::move(layer));
  }

  // A layer that is absent or holds no tiles while ANOTHER requested layer does
  // is not an error — reference-only coverage is a legitimate configuration —
  // but it must never be silent: after ADR-0010 D8 re-classifies `survey/` as
  // `processed/` (+ a new `draft/`), a run asking for the old `survey,reference`
  // would otherwise quietly orthorectify against the coarse regional layer alone
  // (#297 review).
  //
  // Exception: `draft/` is optional-by-design under ADR-0010 D8. It starts empty
  // and only live CUBE ingest ever creates it, so on an offline or freshly
  // migrated store its directory legitimately does not exist — that is the
  // expected state, not reduced coverage. Suppress the absent-layer warning for
  // `draft` as long as some OTHER requested layer resolved (so we are not
  // silently down to a coarser prior alone); a missing NON-draft layer, and a
  // missing `draft` with no other coverage at all, still warn. `draft` stays in
  // the default search order (kDefaultBathyLayers) so a live store consults it at
  // its correct priority.
  const bool have_other_coverage = !layers_.empty();
  for (const auto & name : missing) {
    if (name == "draft" && have_other_coverage) {
      continue;
    }
    warnings_.push_back(
      "requested bathy layer '" + name + "/' does not exist under " + store_root_ +
      "; continuing with the layer(s) that do. If the store re-classified its layers "
      "(ADR-0010 D8 renames survey/ to processed/ and adds draft/), pass "
      "--bathy-layers instead of accepting the reduced coverage.");
  }
  for (const auto & name : empty) {
    warnings_.push_back(
      "requested bathy layer '" + name + "/' exists under " + store_root_ +
      " but holds no <level>_<row>_<col>.tif tiles; continuing without it.");
  }

  if (missing.size() == requested.size()) {
    std::ostringstream msg;
    msg << "BathyDem: none of the requested layer directories exists under " << store_root_
        << " (looked for:";
    for (const auto & name : requested) {
      msg << " " << name << "/";
    }
    msg << "). A bathy store that renamed its layers (ADR-0010 D8 re-classifies";
    msg << " survey/ as processed/ and adds draft/) is a --bathy-layers change, "
      "not an empty store.";
    throw std::runtime_error(msg.str());
  }
  if (tile_count_ == 0) {
    std::ostringstream msg;
    msg << "BathyDem: no <level>_<row>_<col>.tif tiles found under " << store_root_
        << " in layer(s):";
    for (const auto & name : requested) {
      msg << " " << name << "/";
    }
    throw std::runtime_error(msg.str());
  }

  all_levels_.assign(levels_seen.rbegin(), levels_seen.rend());
  for (const auto & layer : layers_) {
    lookups_by_layer_[layer.name] = 0;
  }
}

std::string BathyDem::describe() const
{
  std::ostringstream out;
  out << store_root_ << " [";
  for (std::size_t i = 0; i < layers_.size(); ++i) {
    if (i != 0) {
      out << ", ";
    }
    out << layers_[i].name << "/: " << layers_[i].files.size() << " tiles, level(s)";
    for (const int level : layers_[i].levels) {
      out << " " << level;
    }
  }
  out << "]";
  return out.str();
}

const BathyDem::Tile * BathyDem::tileFor(
  std::size_t layer, int level_n, const gggs::GridIndex & grid)
{
  const std::string filename = mtrs::tileFilename(grid);
  if (layers_[layer].files.count(filename) == 0) {
    return nullptr;   // no tile on disk for this grid — cheap negative test.
  }
  const CacheKey key{layer, level_n, grid.row(), grid.column()};
  const auto found = cache_index_.find(key);
  if (found != cache_index_.end()) {
    lru_.splice(lru_.begin(), lru_, found->second);
    return &lru_.begin()->second;
  }

  const std::string path = (fs::path(layers_[layer].dir) / filename).string();
  // A tile the scan listed but that cannot be read is an error, not a
  // no-coverage: silently degrading a corrupt store to flat-bottom placement is
  // exactly the stale-data path the quality standard forbids.
  Tile tile = mtrs::loadTile<double>(path, gggs::Level(static_cast<uint8_t>(level_n)), kBands);

  lru_.emplace_front(key, std::move(tile));
  cache_index_[key] = lru_.begin();
  while (lru_.size() > cache_tiles_) {
    cache_index_.erase(lru_.back().first);
    lru_.pop_back();
  }
  return &lru_.begin()->second;
}

std::optional<double> BathyDem::cellValue(
  std::size_t layer, int level_n, double lat_deg, double lon_deg)
{
  const gggs::Level level(static_cast<uint8_t>(level_n));
  gggs::GridIndex grid;
  try {
    grid = level.gridIndex(lat_deg, lon_deg);
  } catch (const std::out_of_range &) {
    return std::nullopt;   // off-ellipsoid query: no coverage, not a crash.
  }
  if (!grid.valid()) {
    return std::nullopt;
  }
  const Tile * tile = tileFor(layer, level_n, grid);
  if (tile == nullptr) {
    return std::nullopt;
  }
  const gggs::CellIndex cell(grid, gggs::geoPoint(lat_deg, lon_deg));
  const double depth = tile->get(cell.row(), cell.column(), kDepthBand);
  if (!std::isfinite(depth)) {
    return std::nullopt;   // NaN = no data (marine_bathymetry_store contract).
  }
  return depth;
}

std::optional<BathyDem::Source> BathyDem::resolveSource(double lat_deg, double lon_deg)
{
  for (std::size_t i = 0; i < layers_.size(); ++i) {
    for (const int level : layers_[i].levels) {
      if (cellValue(i, level, lat_deg, lon_deg)) {
        return Source{i, level};
      }
    }
  }
  return std::nullopt;
}

std::optional<double> BathyDem::depthAt(double lat_deg, double lon_deg)
{
  const auto source = resolveSource(lat_deg, lon_deg);
  if (!source) {
    return std::nullopt;
  }

  const gggs::Level level(static_cast<uint8_t>(source->level));
  const gggs::CellIndex cell = level.cellIndex(gggs::geoPoint(lat_deg, lon_deg));
  // CellIndex::position() is the cell's SOUTH-WEST corner, so the centre is the
  // corner plus half a cell span in each axis.
  const auto corner = cell.position();
  const double lat_span = cell.grid().latitudinalSpan() / gggs::cell_rows_per_grid;
  const double lon_span = cell.grid().longitudinalSpan() / gggs::cell_columns_per_grid;
  const double centre_lat = corner.latitude + 0.5 * lat_span;
  const double centre_lon = corner.longitude + 0.5 * lon_span;

  // The four bracketing cell centres: this cell's, plus its neighbours on the
  // side the query point falls toward. Each neighbour is resolved from its own
  // lat/lon (`cellValue` → `Level::gridIndex`), so one falling into an adjoining
  // GRID resolves there rather than clamping at the tile edge.
  //
  // The offsets are the QUERY cell's spans, which assumes the neighbour shares
  // them. GGGS longitudinal spans step by 3x at the 72-deg and 80-deg latitude
  // bands, so a probe across one of those two parallels can land back inside the
  // query cell (wider neighbour) or skip a cell (narrower one). The blend stays
  // well-formed either way — a re-sampled centre just contributes no gradient
  // along that axis, degrading toward nearest-cell — so this is an accuracy
  // caveat at two parallels, not a correctness hole. No survey here is near
  // them; a general reader would want per-neighbour spans.
  const double lat_dir = (lat_deg >= centre_lat) ? 1.0 : -1.0;
  const double lon_dir = (lon_deg >= centre_lon) ? 1.0 : -1.0;
  const double t = std::min(1.0, std::abs(lat_deg - centre_lat) / lat_span);
  const double u = std::min(1.0, std::abs(lon_deg - centre_lon) / lon_span);
  const double lats[2] = {centre_lat, centre_lat + lat_dir * lat_span};
  const double lons[2] = {centre_lon, centre_lon + lon_dir * lon_span};
  const double lat_weight[2] = {1.0 - t, t};
  const double lon_weight[2] = {1.0 - u, u};

  double blended = 0.0;
  double weight_sum = 0.0;
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      const double weight = lat_weight[i] * lon_weight[j];
      const auto value = cellValue(source->layer, source->level, lats[i], lons[j]);
      if (!value) {
        continue;
      }
      blended += weight * *value;
      weight_sum += weight;
    }
  }
  if (!(weight_sum > 0.0)) {
    // Unreachable: `resolveSource` proved the centre cell (i=j=0) has data, and
    // its weight (1-t)(1-u) >= 0.25 because t,u <= 0.5 by construction. Kept as a
    // guard so a future change to the stencil cannot divide by zero silently.
    return std::nullopt;
  }
  ++lookups_by_layer_[layers_[source->layer].name];
  // All four present ⇒ the weights sum to 1 and this is the plain bilinear value.
  // With one or more missing, the SAME expression renormalised by the weights
  // actually present is the bilinear interpolation over the neighbours that exist:
  // a missing far corner carrying a negligible weight barely moves the result,
  // where snapping to the nearest cell would discard an almost-exact
  // interpolation. Dividing is also what keeps a partial sum unbiased — an
  // un-renormalised partial blend pulls toward zero, which for ellipsoidal heights
  // means toward the ellipsoid, not toward a neighbouring depth.
  return blended / weight_sum;
}

}  // namespace marine_sidescan_mosaic
