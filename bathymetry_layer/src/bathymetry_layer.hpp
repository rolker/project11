// Copyright (c) 2026 Roland Arsenault
// Licensed under BSD license

#ifndef BATHYMETRY_LAYER_HPP_
#define BATHYMETRY_LAYER_HPP_

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "geographic_msgs/msg/geo_point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "marine_autonomy/gggs.h"
#include "marine_bathymetry_store/bathymetry_store.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace bathymetry_layer
{

/// @brief Nav2 costmap layer fed by the bathymetric store (marine_bathymetry_store).
///
/// Reads the store's persisted layers (`processed/` + `draft/` and the read-only
/// `reference/` prior — ADR-0010 D8 split the pre-D8 `survey/` into processed/draft)
/// from disk — transparently, via the store's best-source query overlay
/// (`hasAnyData`/`reliableSamples`, data-driven over `source_layers_by_priority`),
/// so this layer needs no per-layer knowledge — and turns *clearance*
/// — the water-surface ellipsoidal height minus the seafloor ellipsoidal height —
/// into occupancy cost so the planner routes around shoals. This is the D1
/// (prior-only, static) deliverable of issue #164: no live re-read of the store
/// (deferred to D2, which depends on the atomic-tile-write work in #189).
///
/// **No-data policy (ADR-0002 §D7, two-query safety pattern):** per cell,
/// `reliableSamples` collects every usable sample first; if none, `hasAnyData`
/// answers "is there ANY data here?" (quality-blind). If not,
/// the cell is truly unsurveyed and this layer leaves the master cost untouched
/// (NO_INFORMATION) so another prior — e.g. `s57_layer` — can fill it in. The
/// opt-in `unsurveyed_is_lethal` parameter (default false) overrides this: when
/// set, a truly-unsurveyed cell is written `LETHAL_OBSTACLE` instead. This suits a
/// closed-basin water body whose prior covers the whole navigable interior (so the
/// only no-data cells are land), letting the layer mark the shoreline lethal
/// without a separate land-mask. It is a blanket rule (every no-data cell, not just
/// "land") and, via max-cost combine, overrides other priors on those cells — so
/// choose it per deployment. It is still held behind the tide gate: no cost,
/// lethal-land included, is emitted before a valid `map_tide` arrives.
/// If there *is* data, the cost comes from the **worst-case clearance**
/// (clearance − σ) via a confidence-gated ramp (ADR-0010 D7): keepout (LETHAL) is
/// reserved for **trusted** data (σ ≤ `confidence_gate`) whose worst-case
/// clearance is below `minimum_depth`; a high-σ (chart-grade) cell is *costed*
/// (caution ramp, capped at `MAX_NON_OBSTACLE`), never hard-forbidden on its own.
/// **Both** queries are REGION-aware (uma#369, `uma-ADR-0013` D8): where the store
/// holds data finer than the costmap's query level, every covered native cell is
/// read, not one sample from the query cell's centre. A point-sampled gate walks
/// past a rock sitting anywhere but the centre — see `query.hpp`'s `hasAnyData`.
/// And the region is the whole COSTMAP cell, not one GGGS cell inside it: a query
/// cell is smaller than the costmap cell it costs (0.60 m² of 1.00 m² at 1 m and
/// 43.5°N, and as little as 18% of it just under a level boundary), so
/// `evaluateCostmapCell` reads every query cell the costmap cell overlaps and
/// keeps the most hazardous verdict. Reading one of them left 40-82% of the
/// cell's ground unqueried at every store resolution.
///
/// **Runtime consequence — the fan-out is a config parameter (uma#369).** The
/// query level is `BathymetryStore::fromCellSize(resolution_)`, so the gap
/// between it and the store's native level is set by the costmap's resolution
/// against whatever the store holds: a 2 m or 4 m global costmap over today's
/// uniform level-10 `processed` is already a 4x or 16x fan-out per cell, and a
/// 1 m global over level-14 depth-adaptive tiles would be 256 covered cells per
/// costmap cell — ~2.56 M cell visits for one 100x100 tile, each a map find and
/// a `push_back` — multiplied again by the 4-6 query cells each costmap cell
/// overlaps (`evaluateCostmapCell`). `generateTile`'s time budget is checked only *between* tiles,
/// so one tile is uninterruptible once started. Reducing the work by
/// point-sampling is exactly the defect uma#369 fixed, so the remedy is a bound
/// or an interruption point, not a narrower query; tracked in
/// [#371](https://github.com/rolker/unh_marine_autonomy/issues/371).
/// `hasAnyData` short-circuits at the first cell holding data, so the existence
/// gate itself costs one cell over surveyed ground and only an empty region pays
/// its full walk.
///
/// A cell whose only data carries σ = ∞ / NaN (genuinely unknown quality) has no
/// usable magnitude of uncertainty and stays conservatively `LETHAL_OBSTACLE`
/// (the "data exists but no reliable sample" path) — same as before. This
/// supersedes the pre-#276 reject-filter, where any σ over `max_uncertainty`
/// collapsed the cell to LETHAL, which would wholesale-keepout CATZOC-grade chart
/// regions. (The pre-#248 per-cell staleness gate was retired — ADR-0002
/// Amendment A2.4: the bathy store is a surveyed static bottom, not a live feed,
/// so per-cell age is not a meaningful costmap hazard.)
///
/// **Layer coexistence:** writes use max-cost semantics (a cell is only raised,
/// never lowered, and an unsurveyed cell is skipped), so this layer composes with
/// `s57_layer` in a layered costmap: the higher cost per cell wins, and where this
/// layer has no data `s57_layer` is the sole contributor. See the package README.
class BathymetryLayer : public nav2_costmap_2d::Layer
{
public:
  BathymetryLayer();
  ~BathymetryLayer() override;

  void onInitialize() override;

  void reset() override;

  bool isClearable() override {return false;}

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y,
    double * max_x, double * max_y) override;

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  void matchSize() override;

protected:
  // Exposed for unit testing (test/test_bathymetry_layer.cpp): the pure
  // worst-case-clearance→cost ramp, decoupled from the store/TF pipeline
  // (ADR-0010 D7). @p worst_case_clearance is in metres — the point-estimate
  // clearance (water-surface height minus seafloor height; see evaluateCell)
  // minus one standard deviation of vertical uncertainty (clearance − σ), i.e.
  // the shallowest the seafloor plausibly is. @p trusted is whether the sample's
  // σ passed the confidence gate (σ ≤ confidence_gate_): only trusted data may
  // drive a cell to LETHAL. When @p trusted is false the same ramp applies but
  // the below-`minimum_depth_` verdict is capped at MAX_NON_OBSTACLE (top of the
  // caution band) instead of LETHAL — high-σ data is costed (go-slow), never
  // hard-forbidden on its own (D7: keepout only on trusted data).
  //
  // NOTE: the trust flag is NOT the whole safety story. A sample with a *non-finite*
  // σ (σ = ∞ / NaN → genuinely unknown quality, ADR-0010 D4) is bucketed with
  // no-data → conservative LETHAL by `evaluateCell` *before* it ever reaches
  // computeCost, so computeCost only ever sees a usable finite σ reduced to
  // (worst_case_clearance, trusted). Do not read a `trusted == false → caution`
  // guarantee here as covering unknown-quality data — that path never gets here.
  // Separately, a *non-finite* @p worst_case_clearance (invalid geometry — e.g. a
  // non-finite sample depth even with a finite σ) is treated as unknown and
  // returns LETHAL unconditionally, independent of @p trusted: the untrusted
  // caution cap applies only to a finite-but-shallow clearance.
  unsigned char computeCost(double worst_case_clearance, bool trusted) const;

  // Exposed for unit testing: the full two-query per-cell decision (review M1),
  // decoupled from the costmap/TF pipeline. Reads store_ and map_tide_z_.
  // Returns std::nullopt when the cell is truly unsurveyed (the layer leaves the
  // master cost untouched); otherwise the cost to combine into the master grid.
  std::optional<unsigned char> evaluateCell(const gggs::CellIndex & cell) const;

  // The cost of one COSTMAP cell, given the geographic bounding box of its
  // ground. Reads every GGGS query cell the box overlaps and keeps the most
  // hazardous verdict; std::nullopt when none of them has a verdict (all
  // unsurveyed, and unsurveyed_is_lethal_ is not set). A query cell is smaller
  // than a costmap cell, so costing from the single cell under the centre left
  // 40-82% of the ground unread (round 3, uma#369) — see the implementation for
  // the geometry and for the two deliberate asymmetries it carries.
  // Exposed for unit testing.
  std::optional<unsigned char> evaluateCostmapCell(
    double min_lat, double min_lon, double max_lat, double max_lon) const;

  // Expand a leading "~"/"~/" in @p path to $HOME (so one portable store_path
  // resolves on both the boat and dev/sim). Absolute, empty, and "~user" paths
  // are returned unchanged. Exposed for unit testing.
  static std::string expandUserPath(const std::string & path);

  // Test seam (test_bathymetry_layer.cpp): inject a pre-rendered cost tile at
  // grid coordinate (ti, tj) and mark the cache complete, so the updateCosts
  // blit can be unit-tested without the store/TF/projection pipeline. tileSize()
  // exposes the tile edge (cells) so a test can size a matching tile.
  void injectTile(int ti, int tj, std::shared_ptr<nav2_costmap_2d::Costmap2D> tile);
  int tileSize() const {return tile_size_;}

  // Test seam: force window_valid_ (normally set by refreshWindow on a
  // successful loadWindow) so the current_ gate can be unit-tested without TF.
  void setWindowValidForTest(bool v) {window_valid_ = v;}

  // Test seam: mark an already-injected tile stale (needs_update) without a tide
  // move, so windowFullyRendered's "stale-but-rendered still counts as ready"
  // property (#3) can be unit-tested. No-op if the tile is not resident.
  void markTileNeedsUpdate(int ti, int tj);

  // A lat/lon axis-aligned bounding box of one store tile, used as a cheap
  // whole-tile coverage test in generateTile (mirrors s57_layer testing a tile
  // against its loaded chart grids before sampling). Built once per generation
  // pass by buildCoverage() from the store's resident tiles across all layers.
  struct CoverageBox
  {
    double min_lat;
    double min_lon;
    double max_lat;
    double max_lon;
  };

  // Whether a tile's projected lat/lon AABB (@p min_lat..@p max_lat,
  // @p min_lon..@p max_lon) overlaps any store coverage box, after padding by a
  // safety margin so a tile straddling the coverage edge counts as covered
  // (full render) — never the reverse, so a covered cell is never classified
  // no-coverage and dropped. Exposed for unit testing the generateTile
  // no-coverage short-circuit decision.
  bool tileHasCoverage(
    double min_lat, double min_lon, double max_lat, double max_lon,
    const std::vector<CoverageBox> & coverage) const;

  // Whether every tile in the inclusive grid range [ (ti_lo,tj_lo) .. (ti_hi,tj_hi) ]
  // has been rendered at least once (resident and `generated`). This is the
  // current_ readiness signal: a tile that is generated but `needs_update`
  // (stale-but-serving after a tide move) still counts as rendered, so a tide
  // drift does not drop the costmap to not-current (#3). A never-rendered tile
  // (first fill / freshly scrolled-in region) returns false. Exposed for testing.
  bool windowFullyRendered(int ti_lo, int tj_lo, int ti_hi, int tj_hi) const;

  // Test seams: the store and the cached water-surface height, so a test fixture
  // can populate a synthetic store and drive cost evaluation without TF.
  std::unique_ptr<marine_bathymetry_store::BathymetryStore> store_;
  double map_tide_z_ = 0.0;
  // True only after at least one successful map_tide TF lookup. When false,
  // evaluateCell() refuses to compute clearance — tide height 0.0 (the default)
  // is not a valid water-surface datum and would produce arbitrary clearance
  // values. updateCosts() also gates current_=true on this flag (MF1/MF2).
  bool map_tide_valid_ = false;

  // #2 safety latch (set each updateBounds pass): the current window has NO store
  // coverage AND unsurveyed_is_lethal_ is set, so every tile would short-circuit
  // to LETHAL — an all-obstacle grid including the vehicle's own cell. updateCosts
  // gates current_=false on this so that fabricated all-lethal grid is never
  // asserted as a usable costmap (the layer warns instead). Protected so the test
  // fixture can drive the gate directly.
  bool coverage_empty_lethal_ = false;

  // Parameters (protected so the test fixture can configure them directly).
  double minimum_depth_ = 1.0;
  double maximum_caution_depth_ = 2.5;
  // Confidence gate (was `max_uncertainty` pre-#276, with inverted role): the
  // 1-sigma vertical-uncertainty threshold (m) at/below which a sample is
  // TRUSTED. Trusted samples may drive a cell to LETHAL when the worst-case
  // clearance (clearance − σ) falls below minimum_depth_; a sample with σ above
  // this gate is high-σ chart-grade data that is COSTED (caution ramp), never
  // hard-forbidden on its own (ADR-0010 D7). This is NOT the old reject-filter,
  // where any σ over the gate collapsed the cell to LETHAL.
  double confidence_gate_ = 0.5;
  // When true, a truly-unsurveyed (no-data) cell is written LETHAL_OBSTACLE
  // instead of being left untouched (NO_INFORMATION). Opt-in (default false):
  // a blanket rule suited to a closed basin whose prior covers the whole
  // interior, so the only no-data cells are land. Still subject to the tide gate.
  bool unsurveyed_is_lethal_ = false;

  // #276 deprecation latch: set true by onInitialize when a config still declares
  // the removed `max_uncertainty` key (renamed to `confidence_gate` with an
  // inverted, trust-threshold meaning). Records that the one-shot migration WARN
  // fired, so the silent-config-drift guard can be unit-tested without scraping
  // logs. Protected for the test seam.
  bool deprecated_max_uncertainty_seen_ = false;

private:
  // Convert a costmap world coordinate to WGS84 lat/lon via the `earth` TF frame
  // (mirrors s57_layer::worldToLatLon). Per-cell tf buffer lookup — used only for
  // the buffered-window AABB; tile rendering uses the hoisted transform below.
  geographic_msgs::msg::GeoPoint worldToLatLon(double x, double y);

  // Apply a cached global_frame->earth transform to a world point and convert to
  // lat/lon — the hoisted-transform inner loop of generateTile(). No per-cell tf
  // buffer lookup (the costly part of worldToLatLon); the transform is looked up
  // once per generation pass and reused for every cell.
  geographic_msgs::msg::GeoPoint worldToLatLon(
    double x, double y, const geometry_msgs::msg::TransformStamped & to_earth) const;

  // Recompute the buffered geographic window from the parent costmap bounds and
  // (re)load / evict store tiles to keep memory bounded. Returns false if the
  // store is not open or a TF lookup fails (logged, throttled).
  void refreshWindow();

  // Open the on-disk store at store_path_ with the parent costmap's resolution.
  void openStore();

  // World-anchored cost-tile cache (mirrors s57_layer). The prior is static in
  // world space, so a tile is rendered once and reused as the rolling window
  // scrolls — turning per-cycle O(cells x (tf + store query)) into an O(cells)
  // blit in updateCosts. TileID is the integer tile grid coordinate; a tile
  // covers tile_size_ x tile_size_ cells of resolution_ metres, anchored at
  // (x_origin_, y_origin_) in the costmap global frame.
  typedef std::pair<int, int> TileID;
  struct TileInfo
  {
    // Pre-rendered costs for this tile (nullptr until generated). NO_INFORMATION
    // where the cell is unsurveyed/untouched; otherwise the bathymetry cost.
    std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap;
    // True once the tile has been rendered against the current tide.
    bool generated = false;
    // Marked when the tide moved past the threshold; the tile keeps serving its
    // cached costs until it is regenerated, so the costmap never flickers.
    bool needs_update = false;
  };

  TileID worldToTile(double x, double y) const;

  // Collect the lat/lon AABBs of every tile currently resident in the store
  // (all source layers). Cheap: the store holds only the windowed coverage (a
  // handful of GGGS tiles for a lake). Empty when the store has no data.
  std::vector<CoverageBox> buildCoverage() const;

  // Render (or re-render) a tile's costs from the store using @p to_earth (the
  // hoisted global_frame->earth transform). @p coverage is the store's resident
  // tile AABBs (buildCoverage()): a tile whose projected lat/lon extent overlaps
  // none of them has no store data, so it is filled uniformly without the
  // per-cell projection+query (LETHAL_OBSTACLE when unsurveyed_is_lethal_, else
  // left fully NO_INFORMATION) — the bulk of tiles on a large global costmap.
  // Always marks the tile generated: a cell whose projection throws is skipped
  // (left NO_INFORMATION) rather than failing the tile, so a single bad cell
  // cannot pin the tile ungenerated and hold current_ false forever.
  void generateTile(
    const TileID & id, const geometry_msgs::msg::TransformStamped & to_earth,
    const std::vector<CoverageBox> & coverage);

  std::string store_path_;
  std::string map_tide_frame_ = "map_tide";
  // Ellipsoid-referenced world frame (REP-105 'map'): z=0 is the WGS84 ellipsoid.
  // The water-surface height is read as map_tide_frame's z in this frame. MUST be
  // distinct from both map_tide_frame_ and the costmap global frame (#220).
  std::string map_frame_ = "map";
  double buffer_fraction_ = 0.05;

  std::string global_frame_id_;
  double resolution_ = 1.0;

  // Cost-tile cache + its fixed world anchor (cells, metres). x_origin_/y_origin_
  // stay 0 so tile boundaries are stable in the global frame across cycles (a
  // rolling window then reuses the same tiles). tile_size_ is the tile edge in
  // cells. update_timeout_ time-boxes tile generation per updateBounds call
  // (mirrors s57_layer): as many pending tiles as fit in this wall-clock budget
  // (seconds) are rendered each cycle, so the first full pass over a large
  // (e.g. 4000x4000) global costmap is spread across cycles instead of blocking
  // the costmap thread. With the no-coverage short-circuit (most tiles cheap),
  // the whole window drains in ~1-2 cycles so current_ goes true promptly.
  std::map<TileID, TileInfo> tiles_;
  double x_origin_ = 0.0;
  double y_origin_ = 0.0;
  int tile_size_ = 100;
  double update_timeout_ = 0.5;
  // Readiness radius (metres) around the vehicle for the current_ gate (review A).
  // current_ is reported once the tiles within this radius of the vehicle are
  // rendered — NOT the whole window — so a large global (which takes far longer
  // than the planner's costmap_update_timeout to render fully) does not stall the
  // planner. Robot-first rendering fills the core first; the rest fills outward in
  // the background. Scale-independent: a bigger global does not change time-to-ready.
  double ready_radius_ = 200.0;
  // True when the robot-centred CORE region (within ready_radius_) has been
  // rendered at least once. Gates current_ (MF2) alongside the tide/window flags
  // and coverage_empty_lethal_ (#2), so Nav2 sees "not yet current" only until the
  // vehicle's neighbourhood is ready — seconds, regardless of total global size —
  // rather than until the entire window is rendered (minutes on a 4 km global).
  bool core_ready_ = false;

  // Tide-change invalidation: re-render tiles only when the water surface moves
  // more than this (metres) from the value the cache was rendered against.
  // Default 0.1 m: above realistic sea-surface estimate jitter (~+/-0.01-0.02 m
  // in sim, from averaging odom z) so normal noise does not constantly re-render
  // and defeat the cache, while still re-rendering when the tide moves enough to
  // shift cost (the clearance ramp spans ~1.5 m). Parameterized (tunable).
  double last_tide_z_ = 0.0;
  bool tide_rendered_ = false;
  double tide_invalidate_threshold_ = 0.1;

  // Last buffered geographic window passed to loadWindow/evictOutside. Used to
  // skip redundant reloads when the costmap has not scrolled past the buffer.
  geographic_msgs::msg::GeoPoint window_min_;
  geographic_msgs::msg::GeoPoint window_max_;
  bool window_valid_ = false;

  // One-shot flag: the first loadWindow/evictOutside failure is logged at ERROR
  // level; subsequent failures from the same root cause (e.g. bad store_path_)
  // are suppressed to avoid log spam. Reset in openStore() so a path reconfigure
  // gets a fresh error if it also fails.
  bool store_path_error_logged_ = false;
};

}  // namespace bathymetry_layer

#endif  // BATHYMETRY_LAYER_HPP_
