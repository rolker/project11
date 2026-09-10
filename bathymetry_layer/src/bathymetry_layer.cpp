// Copyright (c) 2026 Roland Arsenault
// Licensed under BSD license

#include "bathymetry_layer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geodesy/ecef.h"
#include "geodesy/wgs84.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "marine_autonomy/gggs.h"
#include "marine_bathymetry_store/bathy_cell.hpp"
#include "marine_bathymetry_store/query.hpp"
#include "marine_bathymetry_store/tile_io.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace bathymetry_layer
{

using marine_bathymetry_store::BathymetryStore;
using marine_bathymetry_store::DepthSample;
using marine_bathymetry_store::hasAnyData;
using marine_bathymetry_store::reliableSamples;

// Expand a leading "~" or "~/" in a path to $HOME so one portable store_path
// value (e.g. "~/data/world/depths") resolves on both the boat (field user)
// and a dev/sim host with a different home. std::filesystem does no such
// expansion. A bare absolute/relative path, an empty string, or the unsupported
// "~user" form is returned unchanged (the latter would need getpwnam).
std::string BathymetryLayer::expandUserPath(const std::string & path)
{
  if (path.empty() || path[0] != '~') {
    return path;
  }
  if (path.size() == 1 || path[1] == '/') {
    const char * home = std::getenv("HOME");
    if (home != nullptr && home[0] != '\0') {
      return std::string(home) + path.substr(1);
    }
  }
  return path;
}

BathymetryLayer::BathymetryLayer() = default;

BathymetryLayer::~BathymetryLayer() = default;

void BathymetryLayer::onInitialize()
{
  auto node = node_.lock();
  current_ = false;

  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + ".enabled", enabled_);

  declareParameter("store_path", rclcpp::ParameterValue(store_path_));
  node->get_parameter(name_ + ".store_path", store_path_);
  // Expand a leading "~" so one portable value works on the boat and dev/sim.
  store_path_ = expandUserPath(store_path_);

  declareParameter("minimum_depth", rclcpp::ParameterValue(minimum_depth_));
  node->get_parameter(name_ + ".minimum_depth", minimum_depth_);

  declareParameter("maximum_caution_depth", rclcpp::ParameterValue(maximum_caution_depth_));
  node->get_parameter(name_ + ".maximum_caution_depth", maximum_caution_depth_);

  // A degenerate or inverted ramp would make computeCost divide by zero or
  // produce nonsense. Parameters are external input — validate.
  if (!(std::isfinite(minimum_depth_) && std::isfinite(maximum_caution_depth_) &&
    maximum_caution_depth_ > minimum_depth_))
  {
    RCLCPP_WARN_STREAM(
      logger_,
      "Invalid depth ramp (minimum_depth="  << minimum_depth_ << ", maximum_caution_depth="
                                            << maximum_caution_depth_
                                            << "); using defaults 1.0 / 2.5 m instead.");
    minimum_depth_ = 1.0;
    maximum_caution_depth_ = 2.5;
  }

  const double default_confidence_gate = confidence_gate_;
  declareParameter("confidence_gate", rclcpp::ParameterValue(confidence_gate_));
  node->get_parameter(name_ + ".confidence_gate", confidence_gate_);
  // confidence_gate is external input and gates ALL keepout: a sample is TRUSTED
  // (keepout-eligible) only when σ ≤ confidence_gate. A NaN, negative, or zero
  // gate makes `σ ≤ gate` false for every realistic sample (σ ≥ 0, and every NaN
  // comparison is false), silently disabling trusted keepout entirely — a
  // fail-quiet safety hole where the layer would never keep the vehicle out of a
  // trusted shoal. Validate finite & strictly positive (mirroring the depth-ramp
  // guard above); reset to the default on a bad value rather than run degraded.
  if (!(std::isfinite(confidence_gate_) && confidence_gate_ > 0.0)) {
    RCLCPP_WARN_STREAM(
      logger_,
      "Invalid confidence_gate (" << confidence_gate_ << "); it must be finite and > 0 "
        "(σ ≤ gate ⇒ trusted keepout-eligible). Using default "
                                  << default_confidence_gate << " m instead.");
    confidence_gate_ = default_confidence_gate;
  }

  // #276 deprecation guard: `max_uncertainty` was renamed to `confidence_gate`
  // AND its meaning was inverted — from a reject-filter (σ over the gate ⇒ the
  // cell collapses to LETHAL) to a trust-threshold (σ ≤ gate ⇒ the cell is
  // keepout-eligible; σ over the gate ⇒ costed caution, ADR-0010 D7). A boat
  // config still setting `max_uncertainty:` would be silently ignored and would
  // also carry the wrong mental model. Declare the old key with a NaN sentinel so
  // an operator-provided value (from YAML overrides, applied at declare time) is
  // detectable, and warn once (onInitialize runs once per lifecycle) naming both
  // the rename and the semantic change. The value is NOT honoured — a straight
  // remap would misapply the old reject semantics.
  const double deprecated_sentinel = std::numeric_limits<double>::quiet_NaN();
  declareParameter("max_uncertainty", rclcpp::ParameterValue(deprecated_sentinel));
  double deprecated_max_uncertainty = deprecated_sentinel;
  node->get_parameter(name_ + ".max_uncertainty", deprecated_max_uncertainty);
  if (std::isfinite(deprecated_max_uncertainty)) {
    deprecated_max_uncertainty_seen_ = true;
    RCLCPP_WARN_STREAM(
      logger_,
      "bathymetry_layer '" << name_ << "': parameter 'max_uncertainty' ("
                           << deprecated_max_uncertainty << ") is DEPRECATED and IGNORED. "
                           << "It was renamed to 'confidence_gate' and its meaning CHANGED: "
                           << "it is no longer a reject-filter (over-uncertain cells forced "
                           << "LETHAL) but a trust-threshold (sigma <= confidence_gate makes a "
                           << "cell keepout-eligible; higher-sigma cells are costed as caution, "
                           << "never LETHAL on uncertainty alone; ADR-0010 D7). Set "
                           << "'confidence_gate' instead (currently " << confidence_gate_
                           << " m) and remove 'max_uncertainty'.");
  }

  declareParameter("unsurveyed_is_lethal", rclcpp::ParameterValue(unsurveyed_is_lethal_));
  node->get_parameter(name_ + ".unsurveyed_is_lethal", unsurveyed_is_lethal_);

  declareParameter("map_tide_frame", rclcpp::ParameterValue(map_tide_frame_));
  node->get_parameter(name_ + ".map_tide_frame", map_tide_frame_);

  // The water-surface ellipsoidal height is read as the z of map_tide_frame
  // expressed in map_frame. map_frame MUST be the ellipsoid-referenced world
  // frame (REP-105 'map'): its z=0 is the WGS84 ellipsoid, matching the store's
  // ellipsoidal-height datum, so lookupTransform(map_frame, map_tide_frame).z is
  // the surface ellipsoidal height (~+48.9 m at Lake Massabesic full pool).
  // This must NOT be the costmap's own global frame: bizzy's costmaps render in
  // map_tide, so referencing global_frame_id_ (the previous behaviour) was a
  // degenerate self-lookup → identity → z=0 → every cell LETHAL (#220). Compare
  // s57_layer, which likewise references a dedicated datum frame
  // (chart_datum_frame), never the costmap global frame.
  declareParameter("map_frame", rclcpp::ParameterValue(map_frame_));
  node->get_parameter(name_ + ".map_frame", map_frame_);

  const double default_buffer_fraction = buffer_fraction_;
  declareParameter("buffer_fraction", rclcpp::ParameterValue(buffer_fraction_));
  node->get_parameter(name_ + ".buffer_fraction", buffer_fraction_);
  if (!std::isfinite(buffer_fraction_) || buffer_fraction_ < 0.0) {
    RCLCPP_WARN_STREAM(
      logger_,
      "Invalid buffer_fraction value " << buffer_fraction_ << "; using "
                                       << default_buffer_fraction << " instead.");
    buffer_fraction_ = default_buffer_fraction;
  }

  // Re-render the cost-tile cache only when the tide moves more than this (m).
  // Default 0.1 m clears realistic sea-surface-estimate jitter (~+/-0.02 m) so
  // noise doesn't constantly re-render and defeat the cache (#226 review).
  declareParameter("tide_invalidate_threshold", rclcpp::ParameterValue(tide_invalidate_threshold_));
  node->get_parameter(name_ + ".tide_invalidate_threshold", tide_invalidate_threshold_);
  if (!std::isfinite(tide_invalidate_threshold_) || tide_invalidate_threshold_ < 0.0) {
    RCLCPP_WARN_STREAM(
      logger_, "Invalid tide_invalidate_threshold " << tide_invalidate_threshold_ <<
        "; using 0.1 instead.");
    tide_invalidate_threshold_ = 0.1;
  }

  // Per-cycle wall-clock budget for tile (re)rendering (seconds), mirroring
  // s57_layer's update_timeout. updateBounds renders as many pending tiles as fit
  // in this budget each cycle, so a large global costmap fills over a few cycles
  // instead of blocking the costmap thread, yet drains fast because the
  // no-coverage short-circuit makes most tiles cheap. Default 0.5 s (= s57_layer).
  declareParameter("update_timeout", rclcpp::ParameterValue(update_timeout_));
  node->get_parameter(name_ + ".update_timeout", update_timeout_);
  if (!std::isfinite(update_timeout_) || update_timeout_ <= 0.0) {
    RCLCPP_WARN_STREAM(
      logger_, "Invalid update_timeout " << update_timeout_ << "; using 0.5 instead.");
    update_timeout_ = 0.5;
  }

  // Radius (m) around the vehicle whose tiles must be rendered before the layer
  // reports current_ (review A). Decouples readiness from the full window so a
  // large global doesn't stall the planner. Default 200 m (~the local-planning
  // neighbourhood); the rest of the global fills outward in the background.
  declareParameter("ready_radius", rclcpp::ParameterValue(ready_radius_));
  node->get_parameter(name_ + ".ready_radius", ready_radius_);
  if (!std::isfinite(ready_radius_) || ready_radius_ < 0.0) {
    RCLCPP_WARN_STREAM(
      logger_, "Invalid ready_radius " << ready_radius_ << "; using 200.0 instead.");
    ready_radius_ = 200.0;
  }

  global_frame_id_ = layered_costmap_->getGlobalFrameID();

  if (store_path_.empty()) {
    RCLCPP_WARN_STREAM(
      logger_,
      "bathymetry_layer '" << name_ << "' has no store_path; the layer will contribute "
        "no costs until one is configured.");
  }

  // Fail loud on a degenerate tide reference. If map_frame == map_tide_frame the
  // tide lookup is an identity transform (z=0): clearance collapses to
  // -seafloor_height and the whole survey reads LETHAL (#220). This is a config
  // error, not a runtime condition — surface it once, clearly, rather than
  // letting the MF1 gate silently accept the bogus z=0 (the gate in updateBounds
  // also rejects this case so no cost is written until it is fixed).
  if (!map_frame_.empty() && map_frame_ == map_tide_frame_) {
    RCLCPP_ERROR_STREAM(
      logger_,
      "bathymetry_layer '" << name_ << "': map_frame and map_tide_frame are both '"
                           << map_frame_ << "' — the tide lookup would be a degenerate "
                           << "identity (z=0) and every cell would read LETHAL. map_frame "
                           << "must be the ellipsoid-referenced world frame (REP-105 'map'), "
                           << "distinct from the tide frame. This layer will contribute no "
                           << "costs until the frames differ.");
  }

  const char * const unsurveyed_lethal_str = unsurveyed_is_lethal_ ? "true" : "false";
  RCLCPP_INFO_STREAM(
    logger_,
    "bathymetry_layer '" << name_ << "': store_path='" << store_path_ << "', minimum_depth="
                         << minimum_depth_ << " m, maximum_caution_depth=" << maximum_caution_depth_
                         << " m, confidence_gate=" << confidence_gate_
                         << " m, unsurveyed_is_lethal=" << unsurveyed_lethal_str
                         << ", map_frame='" << map_frame_ << "', map_tide_frame='"
                         << map_tide_frame_ << "'");

  matchSize();
}

void BathymetryLayer::reset()
{
  current_ = false;
  window_valid_ = false;
  map_tide_valid_ = false;
  // Drop the rendered-cost cache so it is rebuilt against the next tide/window.
  tiles_.clear();
  tide_rendered_ = false;
  core_ready_ = false;
}

void BathymetryLayer::openStore()
{
  // A new store path is being tried — reset the one-shot error flag so any
  // subsequent loadWindow failure gets a fresh ERROR log.
  store_path_error_logged_ = false;
  if (store_path_.empty()) {
    store_.reset();
    return;
  }
  try {
    // The default level only governs cellIndex(lat,lon); the store is otherwise
    // multi-level. reference_writable=false: a navigation consumer never
    // mutates the read-only prior.
    store_ = std::make_unique<BathymetryStore>(
      BathymetryStore::fromCellSize(static_cast<float>(resolution_), false));
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(
      logger_, "bathymetry_layer '" << name_ << "': failed to construct store: " << e.what());
    store_.reset();
  }
}

void BathymetryLayer::matchSize()
{
  auto parent = layered_costmap_->getCostmap();
  resolution_ = parent->getResolution();

  // Resolution may have changed; rebuild the store at the new default level and
  // force a fresh window load on the next updateBounds. The cost tiles are sized
  // to the old resolution, so drop them too — they are rebuilt on the next pass.
  openStore();
  window_valid_ = false;
  current_ = false;
  tiles_.clear();
  tide_rendered_ = false;
  core_ready_ = false;
}

geographic_msgs::msg::GeoPoint BathymetryLayer::worldToLatLon(double x, double y)
{
  geometry_msgs::msg::PointStamped world;
  world.point.x = x;
  world.point.y = y;
  world.header.frame_id = global_frame_id_;
  geometry_msgs::msg::PointStamped ecef;
  tf_->transform(world, ecef, "earth");
  geodesy::ECEFPoint ecef_point(ecef.point);
  return geodesy::toMsg(ecef_point);
}

geographic_msgs::msg::GeoPoint BathymetryLayer::worldToLatLon(
  double x, double y, const geometry_msgs::msg::TransformStamped & to_earth) const
{
  // Hoisted-transform path: apply a pre-looked-up global_frame->earth transform
  // instead of a per-cell tf buffer query. Used by generateTile so a tile's cells
  // share one transform lookup rather than tile_size_^2 of them (#226).
  geometry_msgs::msg::PointStamped world;
  world.point.x = x;
  world.point.y = y;
  world.header.frame_id = global_frame_id_;
  geometry_msgs::msg::PointStamped ecef;
  tf2::doTransform(world, ecef, to_earth);
  geodesy::ECEFPoint ecef_point(ecef.point);
  return geodesy::toMsg(ecef_point);
}

void BathymetryLayer::injectTile(
  int ti, int tj, std::shared_ptr<nav2_costmap_2d::Costmap2D> tile)
{
  TileInfo info;
  info.costmap = std::move(tile);
  info.generated = true;
  info.needs_update = false;
  tiles_[std::make_pair(ti, tj)] = std::move(info);
  core_ready_ = true;
}

void BathymetryLayer::markTileNeedsUpdate(int ti, int tj)
{
  const auto it = tiles_.find(std::make_pair(ti, tj));
  if (it != tiles_.end()) {
    it->second.needs_update = true;
  }
}

BathymetryLayer::TileID BathymetryLayer::worldToTile(double x, double y) const
{
  const double tile_meters = resolution_ * tile_size_;
  return std::make_pair(
    static_cast<int>(std::floor((x - x_origin_) / tile_meters)),
    static_cast<int>(std::floor((y - y_origin_) / tile_meters)));
}

bool BathymetryLayer::tileHasCoverage(
  double min_lat, double min_lon, double max_lat, double max_lon,
  const std::vector<CoverageBox> & coverage) const
{
  // ~1e-4 deg (~11 m) margin: a generous superset so a tile straddling the
  // coverage edge is treated as covered (full render) rather than skipped —
  // never the reverse, so no covered cell is ever dropped.
  constexpr double kMargin = 1e-4;
  const double lo_lat = min_lat - kMargin;
  const double lo_lon = min_lon - kMargin;
  const double hi_lat = max_lat + kMargin;
  const double hi_lon = max_lon + kMargin;
  for (const auto & box : coverage) {
    const bool lat_overlap = (hi_lat >= box.min_lat) && (lo_lat <= box.max_lat);
    const bool lon_overlap = (hi_lon >= box.min_lon) && (lo_lon <= box.max_lon);
    if (lat_overlap && lon_overlap) {
      return true;
    }
  }
  return false;
}

bool BathymetryLayer::windowFullyRendered(
  int ti_lo, int tj_lo, int ti_hi, int tj_hi) const
{
  // "Rendered at least once" = the tile is resident and generated. needs_update
  // (a stale-but-serving tile awaiting re-render) does NOT count as un-rendered:
  // its cached costs are valid to serve, so it must not hold current_ false (#3).
  for (int ti = ti_lo; ti <= ti_hi; ++ti) {
    for (int tj = tj_lo; tj <= tj_hi; ++tj) {
      const auto it = tiles_.find(std::make_pair(ti, tj));
      if (it == tiles_.end() || !it->second.generated) {
        return false;
      }
    }
  }
  return true;
}

std::vector<BathymetryLayer::CoverageBox> BathymetryLayer::buildCoverage() const
{
  std::vector<CoverageBox> coverage;
  if (!store_) {
    return coverage;
  }
  // The store holds only the windowed coverage (a handful of GGGS tiles for a
  // lake), so this is cheap. One AABB per resident tile across all source layers.
  for (const auto layer : marine_bathymetry_store::source_layers_by_priority) {
    for (const auto & entry : store_->tiles(layer)) {
      const gggs::GridIndex & grid = entry.first;
      coverage.push_back(
        CoverageBox{
          grid.southLatitude(), grid.westLongitude(),
          grid.northLatitude(), grid.eastLongitude()});
    }
  }
  return coverage;
}

void BathymetryLayer::generateTile(
  const TileID & id, const geometry_msgs::msg::TransformStamped & to_earth,
  const std::vector<CoverageBox> & coverage)
{
  const double tile_meters = resolution_ * tile_size_;
  const double world_min_x = x_origin_ + id.first * tile_meters;
  const double world_min_y = y_origin_ + id.second * tile_meters;
  const double world_max_x = world_min_x + tile_meters;
  const double world_max_y = world_min_y + tile_meters;

  // Whole-tile coverage short-circuit (mirrors s57_layer testing a tile against
  // its loaded chart grids before sampling). Project the four corners to lat/lon:
  // a convex rectangle's lat/lon extent is bounded by the min/max of its corner
  // images (Earth curvature over a ~100 m tile edge is sub-mm), so this AABB is a
  // superset of the tile's true extent. If it overlaps NONE of the store's
  // resident-tile AABBs, the tile has no store data and is filled uniformly —
  // skipping the 10,000-cell projection+query that dominates a large global
  // costmap's first pass (the bulk of tiles on a 4 km global over a small lake).
  bool corners_ok = true;
  double clat[4];
  double clon[4];
  const double corners_x[4] = {world_min_x, world_max_x, world_min_x, world_max_x};
  const double corners_y[4] = {world_min_y, world_min_y, world_max_y, world_max_y};
  for (int c = 0; c < 4; ++c) {
    try {
      const auto geo = worldToLatLon(corners_x[c], corners_y[c], to_earth);
      clat[c] = geo.latitude;
      clon[c] = geo.longitude;
    } catch (const tf2::TransformException &) {
      corners_ok = false;
      break;
    }
  }

  if (corners_ok) {
    const double min_lat = std::min(std::min(clat[0], clat[1]), std::min(clat[2], clat[3]));
    const double max_lat = std::max(std::max(clat[0], clat[1]), std::max(clat[2], clat[3]));
    const double min_lon = std::min(std::min(clon[0], clon[1]), std::min(clon[2], clon[3]));
    const double max_lon = std::max(std::max(clon[0], clon[1]), std::max(clon[2], clon[3]));

    if (!tileHasCoverage(min_lat, min_lon, max_lat, max_lon, coverage)) {
      // No store data anywhere in this tile. generateTile only runs once
      // map_tide_valid_ (MF1), so for a closed basin whose prior fills the
      // interior a no-data cell is land: LETHAL when unsurveyed_is_lethal_, else
      // left fully NO_INFORMATION (nullptr, which updateCosts fast-skips so
      // another prior can fill it). Either path does zero per-cell projection.
      auto & info = tiles_[id];
      if (unsurveyed_is_lethal_) {
        info.costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(
          tile_size_, tile_size_, resolution_, world_min_x, world_min_y,
          nav2_costmap_2d::LETHAL_OBSTACLE);
      } else {
        info.costmap = nullptr;
      }
      info.generated = true;
      info.needs_update = false;
      return;
    }

    // Covered: full render with per-cell lat/lon BILINEARLY interpolated from the
    // four corner geos (review B). The world->lat/lon map is near-affine over a
    // ~100 m tile (sub-mm curvature, <<1 store cell), so this drops the dominant
    // per-cell cost — an ECEF/tf round-trip per cell — that made a large global's
    // first pass take minutes. cellIndex + the two-query evaluateCell decision
    // still run per cell (the actual store lookup). All four corners projected,
    // so no per-cell tf throw is possible here.
    auto tile = std::make_shared<nav2_costmap_2d::Costmap2D>(
      tile_size_, tile_size_, resolution_, world_min_x, world_min_y,
      nav2_costmap_2d::NO_INFORMATION);
    const double inv = 1.0 / static_cast<double>(tile_size_);
    // Corner LATTICE, not cell centres (round 3): the cost of a costmap cell is
    // decided over the cell's whole ground, so each cell needs its four corner
    // geos, and adjacent cells share them. One (tile_size+1)^2 lattice is both
    // cheaper than four bilinear evaluations per cell and exactly consistent at
    // the shared edges.
    const int lattice_n = tile_size_ + 1;
    std::vector<double> lat_lattice(static_cast<size_t>(lattice_n) * lattice_n);
    std::vector<double> lon_lattice(lat_lattice.size());
    for (int iy = 0; iy < lattice_n; ++iy) {
      const double fy = static_cast<double>(iy) * inv;
      for (int ix = 0; ix < lattice_n; ++ix) {
        const double fx = static_cast<double>(ix) * inv;
        const double w00 = (1.0 - fx) * (1.0 - fy);
        const double w10 = fx * (1.0 - fy);
        const double w01 = (1.0 - fx) * fy;
        const double w11 = fx * fy;
        const size_t k = static_cast<size_t>(iy) * lattice_n + ix;
        lat_lattice[k] = w00 * clat[0] + w10 * clat[1] + w01 * clat[2] + w11 * clat[3];
        lon_lattice[k] = w00 * clon[0] + w10 * clon[1] + w01 * clon[2] + w11 * clon[3];
      }
    }
    for (int ty = 0; ty < tile_size_; ++ty) {
      for (int tx = 0; tx < tile_size_; ++tx) {
        const size_t sw = static_cast<size_t>(ty) * lattice_n + tx;
        const size_t se = sw + 1;
        const size_t nw = sw + lattice_n;
        const size_t ne = nw + 1;
        const std::optional<unsigned char> evaluated = evaluateCostmapCell(
          std::min(
            std::min(lat_lattice[sw], lat_lattice[se]),
            std::min(lat_lattice[nw], lat_lattice[ne])),
          std::min(
            std::min(lon_lattice[sw], lon_lattice[se]),
            std::min(lon_lattice[nw], lon_lattice[ne])),
          std::max(
            std::max(lat_lattice[sw], lat_lattice[se]),
            std::max(lat_lattice[nw], lat_lattice[ne])),
          std::max(
            std::max(lon_lattice[sw], lon_lattice[se]),
            std::max(lon_lattice[nw], lon_lattice[ne])));
        if (evaluated) {
          tile->setCost(
            static_cast<unsigned int>(tx), static_cast<unsigned int>(ty), *evaluated);
        }
      }
    }
    auto & info = tiles_[id];
    info.costmap = tile;
    info.generated = true;
    info.needs_update = false;
    return;
  }

  // corners_ok == false (a corner is unprojectable): fall back to the per-cell
  // worldToLatLon path, which skips individual unprojectable cells (continue)
  // rather than failing the whole tile. Rare (a TF gap at a tile corner).
  auto tile = std::make_shared<nav2_costmap_2d::Costmap2D>(
    tile_size_, tile_size_, resolution_, world_min_x, world_min_y,
    nav2_costmap_2d::NO_INFORMATION);
  for (int ty = 0; ty < tile_size_; ++ty) {
    for (int tx = 0; tx < tile_size_; ++tx) {
      double wx;
      double wy;
      tile->mapToWorld(
        static_cast<unsigned int>(tx), static_cast<unsigned int>(ty), wx, wy);
      // The cell's four corners, not its centre: the cost is decided over the
      // cell's whole ground (round 3). A throw at any corner skips the cell,
      // exactly as the centre-only projection did.
      const double half = 0.5 * resolution_;
      double min_lat = 0.0;
      double min_lon = 0.0;
      double max_lat = 0.0;
      double max_lon = 0.0;
      try {
        bool first = true;
        for (int c = 0; c < 4; ++c) {
          const auto geo = worldToLatLon(
            wx + ((c & 1) ? half : -half), wy + ((c & 2) ? half : -half), to_earth);
          if (first) {
            min_lat = max_lat = geo.latitude;
            min_lon = max_lon = geo.longitude;
            first = false;
            continue;
          }
          min_lat = std::min(min_lat, geo.latitude);
          max_lat = std::max(max_lat, geo.latitude);
          min_lon = std::min(min_lon, geo.longitude);
          max_lon = std::max(max_lon, geo.longitude);
        }
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 10000,
          "bathymetry_layer '%s': cannot project tile cell to lat/lon: %s",
          name_.c_str(), e.what());
        continue;
      }
      const std::optional<unsigned char> evaluated =
        evaluateCostmapCell(min_lat, min_lon, max_lat, max_lon);
      if (evaluated) {
        tile->setCost(
          static_cast<unsigned int>(tx), static_cast<unsigned int>(ty), *evaluated);
      }
    }
  }
  auto & info = tiles_[id];
  info.costmap = tile;
  info.generated = true;
  info.needs_update = false;
}

void BathymetryLayer::refreshWindow()
{
  if (!store_) {
    return;
  }

  auto parent = layered_costmap_->getCostmap();
  const double world_min_x = parent->getOriginX();
  const double world_min_y = parent->getOriginY();
  const double world_max_x = world_min_x + parent->getSizeInMetersX();
  const double world_max_y = world_min_y + parent->getSizeInMetersY();

  // Buffer the costmap window so tiles are resident slightly ahead of the
  // rolling window edge (same convention as s57_layer's buffer_fraction_).
  const double buffer =
    std::max(world_max_x - world_min_x, world_max_y - world_min_y) * buffer_fraction_;

  geographic_msgs::msg::GeoPoint min_geo;
  geographic_msgs::msg::GeoPoint max_geo;
  try {
    // The four buffered corners; project each to lat/lon and take the AABB. The
    // global frame need not be axis-aligned with north, so corner-projection is
    // required (a two-corner box would clip the window under rotation).
    const double xs[4] = {
      world_min_x - buffer, world_max_x + buffer, world_min_x - buffer, world_max_x + buffer};
    const double ys[4] = {
      world_min_y - buffer, world_min_y - buffer, world_max_y + buffer, world_max_y + buffer};
    double lat_min = 90.0;
    double lat_max = -90.0;
    double lon_min = 180.0;
    double lon_max = -180.0;
    for (int k = 0; k < 4; ++k) {
      const auto geo = worldToLatLon(xs[k], ys[k]);
      lat_min = std::min(lat_min, geo.latitude);
      lat_max = std::max(lat_max, geo.latitude);
      lon_min = std::min(lon_min, geo.longitude);
      lon_max = std::max(lon_max, geo.longitude);
    }
    min_geo = gggs::geoPoint(lat_min, lon_min);
    max_geo = gggs::geoPoint(lat_max, lon_max);
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 10000,
      "bathymetry_layer '%s': cannot project costmap bounds to lat/lon: %s",
      name_.c_str(), e.what());
    return;
  }

  // Skip redundant reloads if the window has not changed materially.
  // S5: use a cell-width tolerance (~resolution_ in degrees, ≈ 1e-5° at mid
  // latitudes for 1 m cells) rather than 1e-9° (~0.1 mm). Sub-mm TF round-trip
  // jitter in the world→lat/lon projection would otherwise force
  // evict+loadWindow disk I/O on every costmap cycle.
  const double tol = resolution_ * 1e-5;  // ~1 cell in degrees at mid-lat
  if (window_valid_ &&
    std::abs(min_geo.latitude - window_min_.latitude) < tol &&
    std::abs(min_geo.longitude - window_min_.longitude) < tol &&
    std::abs(max_geo.latitude - window_max_.latitude) < tol &&
    std::abs(max_geo.longitude - window_max_.longitude) < tol)
  {
    return;
  }

  try {
    // Evict tiles outside the new buffered window first, then load the window.
    // Both use the SAME buffered box (review S2) so margin tiles are not
    // immediately evicted after loading.
    marine_bathymetry_store::evictOutside(*store_, min_geo, max_geo);
    marine_bathymetry_store::loadWindow(*store_, store_path_, min_geo, max_geo);
    window_min_ = min_geo;
    window_max_ = max_geo;
    window_valid_ = true;
  } catch (const std::exception & e) {
    // A persistent failure here (bad store_path or corrupt tiles) means this
    // layer contributes nothing — that must be visible to Nav2 as a degraded
    // cycle. Emit a one-shot ERROR so operators can see the root cause without
    // being spammed; current_ is kept false (MF2 / S2: updateCosts gates on
    // window_valid_).
    if (!store_path_error_logged_) {
      RCLCPP_ERROR_STREAM(
        logger_,
        "bathymetry_layer '" << name_ << "': failed to load store window from '"
                             << store_path_ << "': " << e.what()
                             << " — this layer will contribute no costs until the "
                             << "store is reachable.");
      store_path_error_logged_ = true;
    }
    window_valid_ = false;
  }
}

void BathymetryLayer::updateBounds(
  double robot_x, double robot_y, double, double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  // Cache the water-surface ellipsoidal height as the z of map_tide_frame
  // expressed in map_frame (the REP-105 'map' frame, whose z=0 is the WGS84
  // ellipsoid). lookupTransform(map_frame, map_tide_frame).z is then the surface
  // ellipsoidal height (~+48.9 m at Lake Massabesic full pool).
  //
  // #220: this previously referenced global_frame_id_ (the costmap's global
  // frame). bizzy's costmaps render in map_tide, so that was a degenerate
  // self-lookup (lookupTransform(map_tide, map_tide) → identity → z=0), making
  // clearance = 0 − seafloor_height ≈ −47 m and reading the whole survey LETHAL.
  // map_frame must be the tide-free ellipsoid frame, distinct from map_tide_frame
  // — exactly as s57_layer references a dedicated chart_datum_frame rather than
  // the costmap global frame.
  //
  // Sign (MF1): store depths are WGS84 ellipsoidal heights (up-positive); the
  // seafloor sits below the surface, so clearance = map_tide_z_ − seafloor_height
  // > 0. The MF1 gate refuses to write any cost until a valid tide is received;
  // a degenerate frame config (map_frame == map_tide_frame) is treated as no
  // valid tide so a misconfigured z=0 can never be accepted as a real surface.
  const bool tide_frames_ok =
    !map_frame_.empty() && !map_tide_frame_.empty() && map_frame_ != map_tide_frame_;
  if (tide_frames_ok) {
    try {
      auto transform =
        tf_->lookupTransform(map_frame_, map_tide_frame_, tf2::TimePointZero);
      map_tide_z_ = transform.transform.translation.z;
      map_tide_valid_ = true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 10000,
        "bathymetry_layer '%s': cannot look up %s in %s: %s",
        name_.c_str(), map_tide_frame_.c_str(), map_frame_.c_str(), e.what());
    }
  } else {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 10000,
      "bathymetry_layer '%s': no usable tide reference (map_frame='%s', "
      "map_tide_frame='%s'); contributing no costs.",
      name_.c_str(), map_frame_.c_str(), map_tide_frame_.c_str());
  }

  refreshWindow();

  auto parent = layered_costmap_->getCostmap();
  const double world_min_x = parent->getOriginX();
  const double world_min_y = parent->getOriginY();
  const double world_max_x = world_min_x + parent->getSizeInMetersX();
  const double world_max_y = world_min_y + parent->getSizeInMetersY();

  // --- Cost-tile cache management (#226) ---
  // Render the static prior into world-anchored tiles HERE (once per tile), so
  // updateCosts is a cheap blit instead of a per-cell store query every cycle.
  core_ready_ = false;
  coverage_empty_lethal_ = false;
  if (store_ && window_valid_ && map_tide_valid_) {
    const double buffer =
      std::max(world_max_x - world_min_x, world_max_y - world_min_y) * buffer_fraction_;

    // Tide-CHANGE invalidation: re-render when the surface MOVED materially.
    // Cached costs keep being served until each tile regenerates (no flicker).
    // NOTE: this handles a tide that *moves*, not a tide that *freezes/stops*
    // (lookupTransform(TimePointZero) returns the latest regardless of age, and
    // map_tide_valid_ is a latch) — detecting a stale/absent tide is the separate
    // #223 work, NOT resolved here.
    if (!tide_rendered_ ||
      std::abs(map_tide_z_ - last_tide_z_) > tide_invalidate_threshold_)
    {
      for (auto & entry : tiles_) {
        entry.second.needs_update = true;
      }
      last_tide_z_ = map_tide_z_;
      tide_rendered_ = true;
    }

    // Tiles overlapping the buffered window.
    const TileID lo = worldToTile(world_min_x - buffer, world_min_y - buffer);
    const TileID hi = worldToTile(world_max_x + buffer, world_max_y + buffer);

    // Evict cached tiles fully outside the window (bound memory on long surveys).
    for (auto it = tiles_.begin(); it != tiles_.end(); ) {
      const TileID & t = it->first;
      if (t.first < lo.first || t.first > hi.first ||
        t.second < lo.second || t.second > hi.second)
      {
        it = tiles_.erase(it);
      } else {
        ++it;
      }
    }

    // Hoist the global_frame->earth transform: one lookup for the whole pass,
    // not one per cell (the costly part of the old per-cell path).
    geometry_msgs::msg::TransformStamped to_earth;
    bool have_xf = false;
    try {
      to_earth = tf_->lookupTransform("earth", global_frame_id_, tf2::TimePointZero);
      have_xf = true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 10000,
        "bathymetry_layer '%s': cannot look up earth<-%s: %s",
        name_.c_str(), global_frame_id_.c_str(), e.what());
    }

    // The store holds only the windowed coverage (a handful of GGGS tiles for a
    // lake); collect their lat/lon AABBs once so generateTile can cheaply skip the
    // per-cell projection on tiles that fall entirely outside coverage.
    const std::vector<CoverageBox> coverage = buildCoverage();

    // #2 safety: an EMPTY store window under unsurveyed_is_lethal_ would make
    // every tile short-circuit to LETHAL — the whole costmap, including the
    // vehicle's own cell, reads as obstacle. Rather than assert that fabricated
    // all-lethal grid as a usable ("current") costmap, flag it so updateCosts
    // holds current_ false, and warn so the operator sees a misconfig (wrong/empty
    // store_path) or the vehicle being outside the surveyed extent, instead of a
    // silent box-in. When unsurveyed_is_lethal_ is false, empty coverage just
    // means "no data here" — tiles stay NO_INFORMATION for other priors to fill,
    // so this does not fire and exploration is not frozen.
    coverage_empty_lethal_ = unsurveyed_is_lethal_ && coverage.empty();
    if (coverage_empty_lethal_) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 10000,
        "bathymetry_layer '%s': no store coverage in the current window and "
        "unsurveyed_is_lethal is set; the whole costmap would read LETHAL. "
        "Reporting NOT current instead of an all-lethal grid — check store_path "
        "and that the vehicle is within the store's surveyed extent.",
        name_.c_str());
    }

    // Render ROBOT-FIRST: order pending tiles by distance to the vehicle's tile.
    // (For a rolling costmap the vehicle is near the window centre; the core
    // readiness region below is clamped to the window regardless.)
    const TileID robot_tile = worldToTile(robot_x, robot_y);

    // Render pending tiles nearest-the-vehicle first, time-boxed per cycle
    // (#226 review A+B). Robot-first ordering means the planning-relevant
    // neighbourhood is correct within a cycle or two; the rest of a large global
    // fills outward in the background. Skipped when empty-lethal (#2). A tile is
    // (re)rendered when never generated OR stale (needs_update after a tide move);
    // a stale-but-generated tile keeps serving its cached costs until it
    // regenerates (no flicker).
    if (have_xf && !coverage_empty_lethal_) {
      std::vector<TileID> pending;
      for (int ti = lo.first; ti <= hi.first; ++ti) {
        for (int tj = lo.second; tj <= hi.second; ++tj) {
          const auto it = tiles_.find(std::make_pair(ti, tj));
          if (it == tiles_.end() || !it->second.generated || it->second.needs_update) {
            pending.emplace_back(ti, tj);
          }
        }
      }
      std::sort(
        pending.begin(), pending.end(),
        [robot_tile](const TileID & a, const TileID & b) {
          const int64_t ax = a.first - robot_tile.first;
          const int64_t ay = a.second - robot_tile.second;
          const int64_t bx = b.first - robot_tile.first;
          const int64_t by = b.second - robot_tile.second;
          return (ax * ax + ay * ay) < (bx * bx + by * by);
        });
      const rclcpp::Time gen_start = clock_->now();
      const rclcpp::Duration budget = rclcpp::Duration::from_seconds(update_timeout_);
      for (const TileID & id : pending) {
        generateTile(id, to_earth, coverage);
        if (clock_->now() - gen_start > budget) {
          break;
        }
      }
    }

    // current_ readiness (#226 review A): gate on a robot-CENTRED CORE region
    // being rendered, NOT the whole window. Rendering an entire large global
    // (e.g. 4 km ~= 1900 tiles) takes far longer than the planner's
    // costmap_update_timeout, so gating readiness on the full window stalls the
    // planner. With robot-first rendering the core (within ready_radius_ of the
    // vehicle) is ready within a cycle or two: the planner can plan, the rest
    // fills outward before the vehicle reaches it, and the (small, fast) local
    // costmap covers the immediate surroundings meanwhile. Scale-independent — a
    // bigger global for longer transits does not change time-to-ready.
    // Also #3: a generated-but-stale (needs_update) tile still counts as rendered,
    // so a tide drift does not drop current_ for a full re-render.
    const int core_r = std::max(
      0, static_cast<int>(std::ceil(ready_radius_ / (resolution_ * tile_size_))));
    const int core_lo_i = std::max(lo.first, robot_tile.first - core_r);
    const int core_hi_i = std::min(hi.first, robot_tile.first + core_r);
    const int core_lo_j = std::max(lo.second, robot_tile.second - core_r);
    const int core_hi_j = std::min(hi.second, robot_tile.second + core_r);
    const bool core_in_window = (core_lo_i <= core_hi_i) && (core_lo_j <= core_hi_j);
    core_ready_ =
      have_xf && !coverage_empty_lethal_ && core_in_window &&
      windowFullyRendered(core_lo_i, core_lo_j, core_hi_i, core_hi_j);
  }

  // This layer contributes data over the whole parent window; expand the update
  // bounds to the parent costmap extent so updateCosts is invoked across it.
  *min_x = std::min(*min_x, world_min_x);
  *min_y = std::min(*min_y, world_min_y);
  *max_x = std::max(*max_x, world_max_x);
  *max_y = std::max(*max_y, world_max_y);
}

unsigned char BathymetryLayer::computeCost(double worst_case_clearance, bool trusted) const
{
  // ADR-0010 D7: only TRUSTED data (σ ≤ confidence_gate) may keep a cell out
  // (LETHAL). High-σ (untrusted) data that reads shallow is capped at the top of
  // the caution band (MAX_NON_OBSTACLE) — costed go-slow, never hard-forbidden on
  // its own. This single cap encodes the trusted/untrusted split; the ramp itself
  // is trust-independent, so cost is continuous across the gate boundary (only the
  // below-minimum_depth_ verdict changes with trust).
  const unsigned char keepout_cost =
    trusted ? nav2_costmap_2d::LETHAL_OBSTACLE : nav2_costmap_2d::MAX_NON_OBSTACLE;

  // A non-finite worst-case clearance is INVALID INPUT (e.g. a malformed store
  // tile with a non-finite depth → clearance ±∞/NaN even with a finite σ), NOT
  // "shallow but untrusted". Unknown geometry must stay LETHAL independent of
  // trust — the untrusted caution cap applies only to a genuinely-computed
  // finite-but-shallow clearance. Keep these two verdicts separate.
  if (!std::isfinite(worst_case_clearance)) {
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  }
  if (worst_case_clearance < minimum_depth_) {
    return keepout_cost;
  }
  if (worst_case_clearance >= maximum_caution_depth_) {
    return nav2_costmap_2d::FREE_SPACE;
  }
  // S4: guard div-by-zero when parameters are degenerate (maximum_caution_depth_
  // == minimum_depth_). onInitialize() validates and resets to defaults, but the
  // protected test setters (test_bathymetry_layer.cpp) write the members directly
  // and bypass that validation — the only non-onInitialize path (there is no
  // add_on_set_parameters_callback in this layer). At the ramp boundary
  // worst_case_clearance == minimum_depth_ with a zero-width window,
  // any cost in [LETHAL+1, MAX_NON_OBSTACLE] is defensible; the keepout cost
  // (LETHAL when trusted, else the caution cap) is the conservative choice.
  const double range = maximum_caution_depth_ - minimum_depth_;
  if (range <= 0.0) {
    return keepout_cost;
  }
  // Linear ramp between minimum_depth_ (keepout boundary) and maximum_caution_depth_
  // (FREE boundary), matching s57_layer::get_cost_from_grid.
  const double scaled =
    nav2_costmap_2d::MAX_NON_OBSTACLE * (1.0 - (worst_case_clearance - minimum_depth_) / range);
  return static_cast<unsigned char>(scaled);
}

std::optional<unsigned char> BathymetryLayer::evaluateCostmapCell(
  double min_lat, double min_lon, double max_lat, double max_lon) const
{
  if (!store_) {
    return std::nullopt;
  }
  const gggs::Level & level = store_->level();

  // The GGGS query cell is SMALLER than the costmap cell it costs (round 3).
  // `BathymetryStore::fromCellSize(resolution_)` returns the coarsest level whose
  // cells are AT OR FINER than the costmap resolution, so a query cell's latitude
  // extent lies in (resolution/2, resolution] and its longitude extent is that
  // times cos(latitude). At 1 m resolution and 43.5°N that is 0.906 m × 0.657 m =
  // 0.60 m² of a 1.00 m² costmap cell; just under a level boundary (1.80 m
  // resolution, still level 10) it is 0.60 m² of 3.24 m². Costing the cell from
  // the single GGGS cell under its CENTRE therefore left 40-82% of its ground
  // unread — a 0.3 m rock in the remainder was never queried, no matter how fine
  // `processed` was, which defeated the region-aware walk uma#369 exists for.
  //
  // So: read EVERY GGGS cell overlapping the costmap cell and keep the most
  // hazardous verdict. Two consequences, both deliberate:
  //
  //  - The lat/lon box is the AABB of a cell that is axis-aligned in the MAP
  //    frame, so under a rotated map->WGS84 relation it is slightly larger than
  //    the cell. Over-reading neighbouring ground can only ADD a hazard, never
  //    hide one; under-reading is the failure that grounds the boat.
  //  - A cell the box covers that is UNSURVEYED makes the whole costmap cell
  //    LETHAL when unsurveyed_is_lethal_ is set (its evaluateCell returns
  //    LETHAL, and this fold keeps the max). That is a stricter rule than the
  //    one INSIDE a query cell, where partial no-data still counts as surveyed
  //    (see evaluateCell) — deliberately: there the no-data cells are routine
  //    gaps inside one fused surface, here they are whole query cells of a
  //    closed basin's prior, which is what the flag calls land.
  //
  // Cost: 4-6 query cells per costmap cell at typical resolutions, multiplying
  // the per-query fan-out recorded on unh_marine_autonomy#371.
  if (max_lon < min_lon) {
    // Antimeridian straddle: not iterable as a lat/lon range. Fall back to the
    // centre cell — the pre-round-3 behaviour, and the one place the partial
    // coverage above survives. No boat this store serves operates there.
    return evaluateCell(
      store_->cellIndex(0.5 * (min_lat + max_lat), min_lon));
  }
  const auto south_west = gggs::geoPoint(min_lat, min_lon);
  const auto north_east = gggs::geoPoint(max_lat, max_lon);
  std::optional<unsigned char> worst;
  for (gggs::GridAreaIterator grid_it(
      level.gridIndex(min_lat, min_lon), level.gridIndex(max_lat, max_lon));
    grid_it.valid(); grid_it.next())
  {
    for (gggs::CellAreaIterator cell_it(*grid_it, south_west, north_east);
      cell_it.valid(); cell_it.next())
    {
      const std::optional<unsigned char> cost = evaluateCell(*cell_it);
      if (!cost) {
        continue;   // unknown: leave it to another prior, as the single-cell path did
      }
      if (!worst || *cost > *worst) {
        worst = cost;
      }
    }
  }
  return worst;
}

std::optional<unsigned char> BathymetryLayer::evaluateCell(
  const gggs::CellIndex & cell) const
{
  if (!store_) {
    return std::nullopt;
  }

  // MF1: refuse to compute clearance from an invalid (never-received) tide.
  // The clearance direction is SITE-DEPENDENT (see comment in updateBounds), so
  // map_tide_z_=0.0 (the default) cannot be treated as a safe fallback. Leave the
  // cell as NO_INFORMATION (do not write) so the costmap does not assert bogus
  // FREE or spurious LETHAL costs before the first valid tide arrives.
  if (!map_tide_valid_) {
    return std::nullopt;
  }

  // Two-query no-data policy (review M1, ADR-0002 §D7), BOTH queries
  // region-aware (uma#369).
  //
  //   1. reliableSamples(∞) — EVERY sample WITHOUT a finite-σ reject filter
  //   (ADR-0010 D7). Passing infinity keeps every finite-σ sample eligible so σ
  //   drives *cost*, not rejection; only NaN-σ samples are dropped (∞ > ∞ is
  //   false, so a literal σ=∞ sample IS returned and the isfinite branch below
  //   folds it into the same conservative path as NaN).
  //   (The pre-#248 per-cell staleness gate was retired — ADR-0002 A2.4.)
  //
  //   This runs FIRST, and it is the whole point of uma#369. `processed` is
  //   depth-adaptive, so a level-14 store tile sits under a level-10 costmap
  //   query cell: 256 native cells to one costmap cell, and a 0.2-0.5 m rock can
  //   be in any of them. `reliableSamples` reads every one (uma-ADR-0013 D8);
  //   the previous order gated it behind `bestSource`, a POINT lookup at the
  //   query cell's centre, so one no-data native cell there — a gated-drop hole,
  //   a between-lines gap, an absent fine tile beside a present one — returned
  //   early and dropped the rock in the other 255. Never gate the region-aware
  //   query behind a point query.
  const std::vector<DepthSample> samples =
    reliableSamples(*store_, cell, std::numeric_limits<double>::infinity());

  if (samples.empty()) {
    //   2. hasAnyData — quality-blind, region-aware "is there ANY data here?".
    //   Only reached when nothing usable covers the cell, and it is what
    //   separates the two very different no-sample causes (review M1):
    //     - NO data anywhere under the cell ⇒ truly UNSURVEYED. By default leave
    //       the master cost untouched (NO_INFORMATION) so another prior (e.g.
    //       s57_layer) can contribute. When unsurveyed_is_lethal_ is set, treat
    //       no-data as an obstacle instead — for a closed basin whose prior fills
    //       the whole interior, the only no-data cells are land (see the class
    //       doc). This still sits behind the MF1 tide gate above, so no
    //       lethal-land is written before a valid tide arrives.
    //     - Data exists but every sample has a NaN σ ⇒ SURVEYED BUT UNUSABLE ⇒
    //       conservative LETHAL. A surveyed-but-unusable cell must NOT be
    //       treated as unsurveyed.
    if (!hasAnyData(*store_, cell)) {
      if (unsurveyed_is_lethal_) {
        return nav2_costmap_2d::LETHAL_OBSTACLE;
      }
      return std::nullopt;
    }
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  }

  // Cost by the MAX over ALL reliable samples, NOT a single shallowest-depth
  // pick. Multiple samples arise when several source layers (Survey/Reference/
  // Chart) or GGGS levels cover one cell (they no longer "coincide" once Chart is
  // loaded — ADR-0010 D7). Selecting the shallowest point estimate would let a
  // shallower but UNTRUSTED sample (high finite σ → capped at the caution band,
  // 252) MASK a co-located TRUSTED sample whose worst-case clearance is
  // keepout-grade (LETHAL, 254) — a safety regression. Taking the max cost keeps
  // the trusted keepout: LETHAL > caution numerically.
  unsigned char cost = nav2_costmap_2d::FREE_SPACE;
  for (const DepthSample & sample : samples) {
    unsigned char sample_cost;
    if (!std::isfinite(sample.uncertainty)) {
      // σ = ∞: genuinely unknown quality (ADR-0010 D4), not high-but-finite-σ
      // caution — bucketed with no-data → conservative LETHAL. (The future chart
      // exporter must emit a large *finite* σ for CATZOC D/U so those cells cost
      // as caution, never keepout; ADR-0010 D7 finite-σ contract.)
      sample_cost = nav2_costmap_2d::LETHAL_OBSTACLE;
    } else {
      // depth is ellipsoidal HEIGHT (WGS84, up-positive): a seafloor 5 m below the
      // ellipsoid has depth=-5.0, so a SHALLOWER (more hazardous) seafloor has a
      // LARGER (less-negative) depth, hence a SMALLER clearance and a HIGHER cost.
      // Worst-case clearance subtracts one σ (clearance − σ) — the shallowest the
      // seafloor plausibly is. `trusted` (σ ≤ confidence_gate_) gates keepout: only
      // trusted data can drive LETHAL; high-σ data is costed (caution), never
      // keepout on its own.
      const double clearance = map_tide_z_ - sample.depth;
      const double worst_case_clearance = clearance - sample.uncertainty;
      const bool trusted = sample.uncertainty <= confidence_gate_;
      sample_cost = computeCost(worst_case_clearance, trusted);
    }
    cost = std::max(cost, sample_cost);
    if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
      break;  // nothing costs above LETHAL — no need to examine the rest.
    }
  }
  return cost;
}

void BathymetryLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_ || !store_) {
    // A disabled or store-less cycle contributes nothing; it must not leave a
    // stale current_=true behind for Nav2's staleness monitor (MF2).
    current_ = false;
    return;
  }

  // Blit the pre-rendered cost tiles into the master grid (#226). The expensive
  // store query + projection already happened once per tile in
  // updateBounds::generateTile; here it is a bounded memory copy with this
  // layer's max-combine semantics (raise-only; skip NO_INFORMATION). Mirrors
  // s57_layer::updateCosts (tile_offset_* index convention).
  double world_min_x;
  double world_min_y;
  double world_max_x;
  double world_max_y;
  master_grid.mapToWorld(
    static_cast<unsigned int>(min_i), static_cast<unsigned int>(min_j),
    world_min_x, world_min_y);
  master_grid.mapToWorld(
    static_cast<unsigned int>(max_i), static_cast<unsigned int>(max_j),
    world_max_x, world_max_y);

  const TileID start_tile = worldToTile(world_min_x, world_min_y);
  const TileID end_tile = worldToTile(world_max_x, world_max_y);

  unsigned char * const master = master_grid.getCharMap();
  const double inv_res = 1.0 / resolution_;

  for (int ti = start_tile.first; ti <= end_tile.first; ++ti) {
    // Offset from tile-local x to master-local x (costmap origins are
    // resolution-aligned; lround absorbs sub-cell TF round-trip jitter).
    const int tile_offset_x =
      -ti * tile_size_ +
      static_cast<int>(std::lround((master_grid.getOriginX() - x_origin_) * inv_res));
    const int start_i = std::max(min_i, -tile_offset_x);
    const int stop_i = std::min(max_i, tile_size_ - tile_offset_x);
    if (start_i >= stop_i) {
      continue;
    }
    for (int tj = start_tile.second; tj <= end_tile.second; ++tj) {
      const auto entry = tiles_.find(std::make_pair(ti, tj));
      if (entry == tiles_.end() || !entry->second.costmap) {
        // Not generated yet (incremental fill / post-invalidation), or fully
        // unsurveyed: leave the master untouched. COVERAGE-GAP CONTRACT: while the
        // first pass fills in, an un-generated tile contributes nothing — under
        // unsurveyed_is_lethal a land cell here is NOT yet lethal. current_ is
        // held false (core_ready_ below) for exactly this window, so a
        // consumer that respects costmap currentness will not plan on it. Nav2's
        // controller/planner gate on costmap currentness, so the gap is not acted
        // on; it self-closes within a few cycles for the (small) local costmap.
        continue;
      }
      const int tile_offset_y =
        -tj * tile_size_ +
        static_cast<int>(std::lround((master_grid.getOriginY() - y_origin_) * inv_res));
      const unsigned char * const src = entry->second.costmap->getCharMap();

      for (int j = std::max(min_j, -tile_offset_y);
        j < max_j && j + tile_offset_y < tile_size_; ++j)
      {
        const unsigned int target_row = master_grid.getIndex(0, static_cast<unsigned int>(j));
        const unsigned int source_row =
          entry->second.costmap->getIndex(0, static_cast<unsigned int>(j + tile_offset_y));
        for (int i = start_i; i < stop_i; ++i) {
          const unsigned char cost = src[source_row + i + tile_offset_x];
          if (cost == nav2_costmap_2d::NO_INFORMATION) {
            continue;
          }
          unsigned char & dst = master[target_row + i];
          if (dst == nav2_costmap_2d::NO_INFORMATION || cost > dst) {
            dst = cost;
          }
        }
      }
    }
  }

  // MF2: only report this cycle "current" when the MF1 tide gate and the store
  // window gate held AND the robot-centred CORE region has been rendered
  // (core_ready_; see updateBounds for the #3 stale-vs-rendered distinction and
  // the review-A core-readiness rationale). coverage_empty_lethal_ (#2) also
  // forces not-current so an all-LETHAL grid from an empty store window is never
  // asserted as usable. While the core is still filling in, current_=false so
  // Nav2's staleness monitor sees a not-yet-ready costmap.
  current_ = map_tide_valid_ && window_valid_ && core_ready_ &&
    !coverage_empty_lethal_;

  // Diagnostic breadcrumb: while the layer withholds readiness, say WHICH gate is
  // responsible, so a "planner can't plan / costmap timed out waiting for update"
  // symptom maps to a cause (no tide TF, store window not loaded, first-pass tile
  // fill still in progress, or empty-coverage-lethal #2) without re-running blind.
  // Only fires when not current; in steady state the layer is current and silent.
  // clock_ guard: unit tests drive updateCosts without onInitialize (no clock_),
  // and RCLCPP_WARN_THROTTLE dereferences the clock.
  if (!current_ && clock_) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000,
      "bathymetry_layer '%s' NOT current: map_tide_valid=%d window_valid=%d "
      "core_rendered=%d coverage_empty_lethal=%d (resident tiles=%zu)",
      name_.c_str(),
      static_cast<int>(map_tide_valid_), static_cast<int>(window_valid_),
      static_cast<int>(core_ready_),
      static_cast<int>(coverage_empty_lethal_), tiles_.size());
  }
}

}  // namespace bathymetry_layer

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(bathymetry_layer::BathymetryLayer, nav2_costmap_2d::Layer)
