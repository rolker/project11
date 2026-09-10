// Copyright (c) 2026 Roland Arsenault
// Licensed under BSD license

#include <gtest/gtest.h>

#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "marine_autonomy/gggs.h"
#include "marine_bathymetry_store/bathy_cell.hpp"
#include "marine_bathymetry_store/bathymetry_store.hpp"
#include "marine_bathymetry_store/tile_io.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "bathymetry_layer.hpp"

using bathymetry_layer::BathymetryLayer;
using marine_bathymetry_store::BathyCell;
using marine_bathymetry_store::BathymetryStore;
using marine_bathymetry_store::SourceLayer;

namespace
{
// Survey position used across the per-cell tests.
constexpr double kLat = 43.0;
constexpr double kLon = -70.5;
}  // namespace

// Test subclass exposing the protected store / water-surface seam and the
// two-query decision helpers, so the cost logic can be exercised directly
// against a synthetic store with no costmap or TF pipeline.
class BathymetryLayerForTest : public BathymetryLayer
{
public:
  void setStore(std::unique_ptr<BathymetryStore> store) {store_ = std::move(store);}
  BathymetryStore & store() {return *store_;}
  void setMapTideZ(double z) {map_tide_z_ = z;}
  double getMapTideZ() const {return map_tide_z_;}
  // Explicitly mark the tide as valid (as if a successful TF lookup arrived).
  void setMapTideValid(bool v) {map_tide_valid_ = v;}
  bool isMapTideValid() const {return map_tide_valid_;}
  void setMinimumDepth(double v) {minimum_depth_ = v;}
  void setMaximumCautionDepth(double v) {maximum_caution_depth_ = v;}
  // Renamed from setMaxUncertainty (#276): the parameter is now a trust-threshold
  // (σ ≤ gate ⇒ keepout-eligible), not a reject-filter.
  void setConfidenceGate(double v) {confidence_gate_ = v;}
  double getConfidenceGate() const {return confidence_gate_;}
  void setUnsurveyedIsLethal(bool v) {unsurveyed_is_lethal_ = v;}
  // #276 deprecation guard: whether onInitialize saw a config still setting the
  // removed `max_uncertainty` key (and fired the one-shot migration WARN).
  bool deprecatedMaxUncertaintySeen() const {return deprecated_max_uncertainty_seen_;}
  void setWindowValid(bool v) {setWindowValidForTest(v);}
  void setCoverageEmptyLethal(bool v) {coverage_empty_lethal_ = v;}
  // enabled_ is set from the "enabled" param in onInitialize(); the blit tests
  // skip initialize(), so set it directly.
  void setEnabled(bool v) {enabled_ = v;}

  using BathymetryLayer::computeCost;
  using BathymetryLayer::evaluateCell;
  using BathymetryLayer::evaluateCostmapCell;
  using BathymetryLayer::expandUserPath;
  using BathymetryLayer::injectTile;
  using BathymetryLayer::tileSize;
  using BathymetryLayer::CoverageBox;
  using BathymetryLayer::tileHasCoverage;
  using BathymetryLayer::windowFullyRendered;
  using BathymetryLayer::markTileNeedsUpdate;
};

// ---------------------------------------------------------------------------
// Test case 0 (#218): store_path "~"/"~/" expansion to $HOME — one portable
// value resolves on both the boat and dev/sim; other paths are untouched.
// ---------------------------------------------------------------------------
TEST(BathymetryLayer, ExpandUserPathHandlesTilde)
{
  ::setenv("HOME", "/home/tester", 1);

  EXPECT_EQ(
    BathymetryLayerForTest::expandUserPath("~/data/world/depths"),
    "/home/tester/data/world/depths");
  EXPECT_EQ(BathymetryLayerForTest::expandUserPath("~"), "/home/tester");
  // Absolute, relative, and empty paths are returned unchanged.
  EXPECT_EQ(
    BathymetryLayerForTest::expandUserPath("/home/field/data/world/depths"),
    "/home/field/data/world/depths");
  EXPECT_EQ(BathymetryLayerForTest::expandUserPath("relative/path"), "relative/path");
  EXPECT_EQ(BathymetryLayerForTest::expandUserPath(""), "");
  // The unsupported "~user" form is left untouched (not expanded).
  EXPECT_EQ(BathymetryLayerForTest::expandUserPath("~field/x"), "~field/x");
}

// ---------------------------------------------------------------------------
// Test case 1: worst-case-clearance → cost ramp boundaries (lethal / caution /
// free) for TRUSTED data (σ ≤ confidence_gate). The ramp shape is unchanged from
// the pre-#276 clearance ramp; only the argument is now worst-case clearance and
// a trust flag (ADR-0010 D7). Trusted below-minimum_depth → LETHAL.
// ---------------------------------------------------------------------------
TEST(BathymetryLayer, ClearanceRampBoundaries)
{
  BathymetryLayerForTest layer;
  layer.setMinimumDepth(1.0);
  layer.setMaximumCautionDepth(3.0);

  // Below minimum_depth → LETHAL (trusted).
  EXPECT_EQ(layer.computeCost(0.5, true), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(layer.computeCost(1.0 - 1e-6, true), nav2_costmap_2d::LETHAL_OBSTACLE);

  // At / above maximum_caution_depth → FREE_SPACE (trust-independent).
  EXPECT_EQ(layer.computeCost(3.0, true), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(layer.computeCost(10.0, true), nav2_costmap_2d::FREE_SPACE);

  // Midpoint of the [1, 3] ramp → half of MAX_NON_OBSTACLE.
  const unsigned char mid = layer.computeCost(2.0, true);
  const unsigned char expected_mid =
    static_cast<unsigned char>(nav2_costmap_2d::MAX_NON_OBSTACLE * 0.5);
  EXPECT_EQ(mid, expected_mid);

  // Monotonic: shallower clearance must never cost less than deeper clearance.
  EXPECT_GE(layer.computeCost(1.2, true), layer.computeCost(2.5, true));

  // Non-finite worst-case clearance is INVALID INPUT (unknown geometry), not
  // "shallow but untrusted": it stays LETHAL regardless of the trust flag. The
  // untrusted caution cap (MAX_NON_OBSTACLE) applies only to a finite-but-shallow
  // clearance — never to NaN / ±∞ (#276 Integrated Review, Copilot must-fix).
  for (const double bad :
    {std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity()})
  {
    EXPECT_EQ(layer.computeCost(bad, true), nav2_costmap_2d::LETHAL_OBSTACLE)
      << "Non-finite clearance (trusted) → LETHAL.";
    EXPECT_EQ(layer.computeCost(bad, false), nav2_costmap_2d::LETHAL_OBSTACLE)
      << "Non-finite clearance (UNTRUSTED) → LETHAL, not the caution cap: invalid "
      "geometry is unknown, not merely uncertain.";
  }
}

// ---------------------------------------------------------------------------
// Test case 2: truly-unsurveyed cell → NO_INFORMATION (layer leaves it alone).
// ---------------------------------------------------------------------------
TEST(BathymetryLayer, UnsurveyedCellLeavesMasterUntouched)
{
  BathymetryLayerForTest layer;
  layer.setStore(std::make_unique<BathymetryStore>(5));
  layer.setMapTideZ(0.0);
  layer.setMapTideValid(true);

  // A cell with no data anywhere in the store.
  const auto cell = layer.store().cellIndex(kLat, kLon);
  const auto result = layer.evaluateCell(cell);
  EXPECT_FALSE(result.has_value()) << "Unsurveyed cells must not be written.";
}

// ---------------------------------------------------------------------------
// Test case 2b (#216): with unsurveyed_is_lethal enabled, a no-data cell is
// LETHAL instead of skipped — the closed-basin "no data = land" mode.
// ---------------------------------------------------------------------------
TEST(BathymetryLayer, UnsurveyedCellLethalWhenParamEnabled)
{
  BathymetryLayerForTest layer;
  layer.setStore(std::make_unique<BathymetryStore>(5));
  layer.setMapTideZ(48.88);
  layer.setMapTideValid(true);
  layer.setUnsurveyedIsLethal(true);

  const auto cell = layer.store().cellIndex(kLat, kLon);
  const auto result = layer.evaluateCell(cell);
  ASSERT_TRUE(result.has_value())
    << "With unsurveyed_is_lethal, a no-data cell must be written.";
  EXPECT_EQ(*result, nav2_costmap_2d::LETHAL_OBSTACLE);
}

// ---------------------------------------------------------------------------
// Test case 2c (#216): unsurveyed_is_lethal stays behind the tide gate (option
// A). Even with the param set, no-data cells are NOT written before a valid
// tide — the layer never emits a half-initialised "lethal land + unknown water"
// costmap at startup.
// ---------------------------------------------------------------------------
TEST(BathymetryLayer, UnsurveyedLethalStillGatedByTide)
{
  BathymetryLayerForTest layer;
  layer.setStore(std::make_unique<BathymetryStore>(5));
  layer.setUnsurveyedIsLethal(true);
  // Deliberately do NOT mark the tide valid (map_tide_valid_ defaults false).

  const auto cell = layer.store().cellIndex(kLat, kLon);
  const auto result = layer.evaluateCell(cell);
  EXPECT_FALSE(result.has_value())
    << "No tide yet: unsurveyed-lethal must still be gated (MF1).";
}

// ---------------------------------------------------------------------------
// Test case 3: surveyed shallow cell → LETHAL; surveyed deep cell → FREE.
// ---------------------------------------------------------------------------
TEST(BathymetryLayer, SurveyedCellMapsClearanceToCost)
{
  BathymetryLayerForTest layer;
  layer.setMinimumDepth(1.0);
  layer.setMaximumCautionDepth(3.0);
  layer.setConfidenceGate(0.5);
  layer.setMapTideZ(0.0);  // water surface at ellipsoidal height 0.

  auto store = std::make_unique<BathymetryStore>(5);
  const auto cell = store->cellIndex(kLat, kLon);

  // Shoal: seafloor 0.5 m below the surface, trusted (σ 0.1 ≤ gate). Worst-case
  // clearance = 0.5 − 0.1 = 0.4 m < 1.0 → LETHAL (trusted keepout).
  store->set(SourceLayer::Draft, cell, BathyCell{-0.5, 0.1});
  layer.setStore(std::move(store));
  layer.setMapTideValid(true);
  auto shoal = layer.evaluateCell(cell);
  ASSERT_TRUE(shoal.has_value());
  EXPECT_EQ(*shoal, nav2_costmap_2d::LETHAL_OBSTACLE);

  // Deep: seafloor 5 m down, trusted. Worst-case clearance = 5 − 0.1 = 4.9 m
  // >= 3.0 → FREE_SPACE.
  auto deep_store = std::make_unique<BathymetryStore>(5);
  const auto deep_cell = deep_store->cellIndex(kLat, kLon);
  deep_store->set(SourceLayer::Draft, deep_cell, BathyCell{-5.0, 0.1});
  layer.setStore(std::move(deep_store));
  layer.setMapTideValid(true);
  auto deep = layer.evaluateCell(deep_cell);
  ASSERT_TRUE(deep.has_value());
  EXPECT_EQ(*deep, nav2_costmap_2d::FREE_SPACE);
}

// ---------------------------------------------------------------------------
// Test case 4 (#276, issue-named "chart-σ shallow ⇒ caution not LETHAL"):
// REWRITTEN for ADR-0010 D7. Pre-#276 this asserted that any finite σ over the
// gate collapsed the cell to LETHAL (the reject-filter). Under the new model a
// high-σ (untrusted, chart-grade) cell that reads shallow is COSTED as caution,
// never keepout on uncertainty alone. The cell still HAS data (bestSource
// non-null) and must be written — not skipped as unsurveyed (review M1).
// ---------------------------------------------------------------------------
TEST(BathymetryLayer, UntrustedShallowCellIsCautionNotLethal)
{
  BathymetryLayerForTest layer;
  layer.setMinimumDepth(1.0);
  layer.setMaximumCautionDepth(3.0);
  layer.setConfidenceGate(0.5);
  layer.setMapTideZ(0.0);

  auto store = std::make_unique<BathymetryStore>(5);
  const auto cell = store->cellIndex(kLat, kLon);
  // Chart-grade cell: seafloor 1.0 m down but σ 2.0 m (CATZOC-C-like) is above
  // the 0.5 m confidence gate → UNTRUSTED. Worst-case clearance = 1.0 − 2.0 =
  // −1.0 m < minimum_depth, so pre-#276 this would be LETHAL; under D7 an
  // untrusted below-threshold cell is capped at the caution band, NOT LETHAL.
  store->set(SourceLayer::Draft, cell, BathyCell{-1.0, 2.0});
  layer.setStore(std::move(store));
  layer.setMapTideValid(true);

  const auto result = layer.evaluateCell(cell);
  ASSERT_TRUE(result.has_value())
    << "Over-uncertain surveyed cell must be written (not skipped as unsurveyed).";
  EXPECT_EQ(*result, nav2_costmap_2d::MAX_NON_OBSTACLE)
    << "High-σ shallow cell is costed at the top of the caution band, not LETHAL "
       "(ADR-0010 D7: keepout only on trusted data).";
  EXPECT_NE(*result, nav2_costmap_2d::LETHAL_OBSTACLE);
}

// ---------------------------------------------------------------------------
// Test case 6 (MF1): invalid tide → evaluateCell returns nullopt even for a
// well-surveyed, reliable, fresh cell. This pins the safety contract: the
// layer must not compute clearance from an uninitialised tide value.
// ---------------------------------------------------------------------------
TEST(BathymetryLayer, InvalidTideYieldsNoInformation)
{
  BathymetryLayerForTest layer;
  layer.setMinimumDepth(1.0);
  layer.setMaximumCautionDepth(3.0);
  layer.setConfidenceGate(0.5);
  // Deliberately do NOT call setMapTideValid(true) — map_tide_valid_ defaults false.
  layer.setMapTideZ(52.3);  // Even a "correct" z must be rejected without a valid flag.

  auto store = std::make_unique<BathymetryStore>(5);
  const auto cell = store->cellIndex(kLat, kLon);
  // A perfectly good surveyed cell (deep, reliable, fresh) must still be skipped.
  store->set(SourceLayer::Draft, cell, BathyCell{47.0, 0.1});
  layer.setStore(std::move(store));

  const auto result = layer.evaluateCell(cell);
  EXPECT_FALSE(result.has_value())
    << "No tide yet: layer must not write any cost (MF1 safety gate).";
}

// ---------------------------------------------------------------------------
// Test case 7 (#276, issue-named "σ=∞/no-data unchanged"): REWRITTEN for
// ADR-0010 D7. A cell whose only record carries σ = ∞ or σ = NaN has NO usable
// magnitude of uncertainty (genuinely unknown quality, ADR-0010 D4). That is
// bucketed with no-data, NOT with high-but-finite-σ caution, so it stays
// conservatively LETHAL — the pre-existing "data exists but no reliable sample"
// path, unchanged. This also pins the σ=∞ correctness detail: reliableSamples
// called with ∞ does NOT filter a literal σ=∞ sample (∞ > ∞ is false), so it is
// retained in the returned set and evaluateCell's own isfinite guard is what
// makes σ=∞ conservative. The confident
// control confirms the LETHAL came from the unknown-σ guard, not absent data.
// ---------------------------------------------------------------------------
TEST(BathymetryLayer, UnknownUncertaintyCellStaysConservativeLethal)
{
  BathymetryLayerForTest layer;
  layer.setMinimumDepth(1.0);
  layer.setMaximumCautionDepth(3.0);
  layer.setConfidenceGate(0.5);
  layer.setMapTideZ(0.0);
  layer.setMapTideValid(true);

  // σ = ∞ (unknown quality): deep depth, but the magnitude of uncertainty is
  // unknown → conservative LETHAL, NOT costed as caution. (A finite large σ here
  // would instead be caution — see UntrustedShallowCellIsCautionNotLethal.)
  {
    auto store = std::make_unique<BathymetryStore>(5);
    const auto cell = store->cellIndex(kLat, kLon);
    store->set(
      SourceLayer::Draft, cell,
      BathyCell{-5.0, std::numeric_limits<double>::infinity()});
    layer.setStore(std::move(store));

    const auto result = layer.evaluateCell(cell);
    ASSERT_TRUE(result.has_value())
      << "Cell has data → must write something (not skip as unsurveyed).";
    EXPECT_EQ(*result, nav2_costmap_2d::LETHAL_OBSTACLE)
      << "σ=∞ (unknown quality) → conservative LETHAL, unchanged from pre-#276.";
  }

  // σ = NaN: every sample is dropped by reliableSamples → empty set → the same
  // conservative LETHAL path.
  {
    auto store = std::make_unique<BathymetryStore>(5);
    const auto cell = store->cellIndex(kLat, kLon);
    store->set(
      SourceLayer::Draft, cell,
      BathyCell{-5.0, std::numeric_limits<double>::quiet_NaN()});
    layer.setStore(std::move(store));

    const auto result = layer.evaluateCell(cell);
    ASSERT_TRUE(result.has_value())
      << "NaN-σ cell still HAS data (depth present) → must be written.";
    EXPECT_EQ(*result, nav2_costmap_2d::LETHAL_OBSTACLE)
      << "NaN-σ → empty reliableSamples → conservative LETHAL.";
  }

  // The same confident record is FREE — confirming the LETHAL verdicts above came
  // from the unknown-σ guard, not from the data being absent.
  {
    auto store = std::make_unique<BathymetryStore>(5);
    const auto cell = store->cellIndex(kLat, kLon);
    store->set(SourceLayer::Draft, cell, BathyCell{-5.0, 0.1});
    layer.setStore(std::move(store));

    const auto result = layer.evaluateCell(cell);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(*result, nav2_costmap_2d::FREE_SPACE)
      << "Deep, reliable record → FREE.";
  }
}

// ---------------------------------------------------------------------------
// #276 Integrated Review (Copilot must-fix): a sample with a FINITE σ but a
// non-finite DEPTH (malformed store tile) yields a non-finite worst-case
// clearance. Its σ can still be large enough to be UNTRUSTED (σ > confidence
// gate), so before the fix computeCost folded it into the trust-gated keepout
// branch and capped it at MAX_NON_OBSTACLE (caution) instead of LETHAL. Invalid
// geometry is UNKNOWN, not "shallow but untrusted", and must stay LETHAL
// regardless of trust — store tiles are on-disk external input and cannot be
// assumed well-formed in the field. Distinct from UnknownUncertaintyCell above
// (that path is driven by a non-finite σ; here σ is finite and the DEPTH is bad).
//
// Depth here is an INFINITE, not NaN, value: BathyCell::hasData() is
// `!isnan(depth)`, so a NaN depth reads as NO-DATA (the unsurveyed path), whereas
// a ±∞ depth HAS data, survives reliableSamples (finite σ), and reaches
// computeCost with a non-finite clearance — the exact reachable malformed-tile
// path the finding is about.
// ---------------------------------------------------------------------------
TEST(BathymetryLayer, NonFiniteDepthWithUntrustedFiniteSigmaStaysLethal)
{
  BathymetryLayerForTest layer;
  layer.setMinimumDepth(1.0);
  layer.setMaximumCautionDepth(3.0);
  layer.setConfidenceGate(0.5);
  layer.setMapTideZ(0.0);
  layer.setMapTideValid(true);

  // σ = 2.0 > confidence_gate (0.5) → UNTRUSTED; the ±∞ depth makes the worst-case
  // clearance non-finite. Both signs of infinity must be LETHAL, not the caution
  // cap: invalid geometry is unknown, so trust does not soften it.
  for (const double bad_depth :
    {std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity()})
  {
    auto store = std::make_unique<BathymetryStore>(5);
    const auto cell = store->cellIndex(kLat, kLon);
    store->set(SourceLayer::Draft, cell, BathyCell{bad_depth, 2.0});
    layer.setStore(std::move(store));

    const auto result = layer.evaluateCell(cell);
    ASSERT_TRUE(result.has_value())
      << "±∞ depth still HAS data (not NaN) → must write, not skip as unsurveyed.";
    EXPECT_EQ(*result, nav2_costmap_2d::LETHAL_OBSTACLE)
      << "Non-finite depth + finite untrusted σ → LETHAL, not the caution cap.";
  }
}

// ---------------------------------------------------------------------------
// Test case 8 (S4): computeCost boundary guard — degenerate ramp
// (maximum_caution_depth_ == minimum_depth_) must not divide by zero or
// produce a NaN cast. Also pins computeCost(minimum_depth_) == LETHAL.
// ---------------------------------------------------------------------------
TEST(BathymetryLayer, ComputeCostDegenerateRampAndBoundary)
{
  BathymetryLayerForTest layer;
  layer.setMinimumDepth(2.0);
  layer.setMaximumCautionDepth(2.0);  // degenerate: range == 0

  // Degenerate range: any clearance < maximum_caution_depth_ falls through to
  // the ramp but the range guard returns the keepout cost (LETHAL for trusted).
  EXPECT_EQ(layer.computeCost(1.9, true), nav2_costmap_2d::LETHAL_OBSTACLE)
    << "Below the (degenerate) boundary → LETHAL (trusted)";
  // At exactly the boundary both the < minimum_depth_ check and the range guard
  // fire in sequence; the result must be LETHAL or FREE — not UB.
  const unsigned char at_boundary = layer.computeCost(2.0, true);
  EXPECT_TRUE(
    at_boundary == nav2_costmap_2d::LETHAL_OBSTACLE ||
    at_boundary == nav2_costmap_2d::FREE_SPACE)
    << "At minimum_depth_ with degenerate range: must be LETHAL or FREE, not UB.";

  // With a normal ramp, computeCost(minimum_depth_) must equal MAX_NON_OBSTACLE
  // (the top of the caution band, just inside LETHAL).
  layer.setMinimumDepth(1.0);
  layer.setMaximumCautionDepth(3.0);
  EXPECT_EQ(layer.computeCost(1.0, true), nav2_costmap_2d::MAX_NON_OBSTACLE)
    << "At minimum_depth_ on a normal ramp → MAX_NON_OBSTACLE (highest caution).";
}

// ---------------------------------------------------------------------------
// Test case (#276, issue-named "keepout only on trusted-shallow"): two cells at
// the same shoal depth, one TRUSTED (σ ≤ confidence_gate) and one UNTRUSTED
// (σ > gate). Both have a worst-case clearance below minimum_depth, but only the
// trusted cell is keepout (LETHAL); the untrusted cell — even though it reads
// shallower after subtracting its larger σ — is costed as caution. This is the
// core ADR-0010 D7 guarantee: keepout is reserved for trusted data.
// ---------------------------------------------------------------------------
TEST(BathymetryLayer, KeepoutOnlyOnTrustedShallow)
{
  BathymetryLayerForTest layer;
  layer.setMinimumDepth(1.0);
  layer.setMaximumCautionDepth(3.0);
  layer.setConfidenceGate(0.5);
  layer.setMapTideZ(0.0);
  layer.setMapTideValid(true);

  // Trusted shoal: seafloor 0.5 m down, σ 0.2 ≤ gate. worst_case = 0.5 − 0.2 =
  // 0.3 < minimum_depth → LETHAL (keepout is allowed for trusted data).
  {
    auto store = std::make_unique<BathymetryStore>(5);
    const auto cell = store->cellIndex(kLat, kLon);
    store->set(SourceLayer::Draft, cell, BathyCell{-0.5, 0.2});
    layer.setStore(std::move(store));
    const auto result = layer.evaluateCell(cell);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(*result, nav2_costmap_2d::LETHAL_OBSTACLE)
      << "trusted (σ ≤ gate) worst-case-shallow cell → keepout";
  }

  // Same shoal depth but UNTRUSTED (σ 0.8 > gate): worst_case = 0.5 − 0.8 = −0.3
  // is even shallower, yet the cell is caution, NOT LETHAL — keepout is reserved
  // for trusted data (never keepout on uncertainty alone).
  {
    auto store = std::make_unique<BathymetryStore>(5);
    const auto cell = store->cellIndex(kLat, kLon);
    store->set(SourceLayer::Draft, cell, BathyCell{-0.5, 0.8});
    layer.setStore(std::move(store));
    const auto result = layer.evaluateCell(cell);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(*result, nav2_costmap_2d::MAX_NON_OBSTACLE)
      << "untrusted (σ > gate) worst-case-shallow cell → caution, never keepout";
    EXPECT_NE(*result, nav2_costmap_2d::LETHAL_OBSTACLE);
  }
}

// ---------------------------------------------------------------------------
// Test case (#276 local-review must-fix): trusted-keepout must NOT be masked on a
// MULTI-SAMPLE cell. When several source layers cover one cell, evaluateCell must
// cost by the MAX over all reliable samples — not select one by shallowest point
// estimate. Here a shallower UNTRUSTED Chart sample (caution) sits over a slightly
// deeper TRUSTED Survey sample whose worst-case clearance is keepout-grade
// (LETHAL). A shallowest-depth pick would return the Chart caution and hide the
// Survey LETHAL; the max keeps the keepout (ADR-0010 D7: keepout on trusted data).
// ---------------------------------------------------------------------------
TEST(BathymetryLayer, TrustedKeepoutNotMaskedByShallowerUntrustedSample)
{
  BathymetryLayerForTest layer;
  layer.setMinimumDepth(1.0);
  layer.setMaximumCautionDepth(3.0);
  layer.setConfidenceGate(0.5);
  layer.setMapTideZ(0.0);
  layer.setMapTideValid(true);

  // Staging-writable so the Chart layer can be populated directly in the test.
  auto store = std::make_unique<BathymetryStore>(
    5, /*reference_writable=*/false, /*chart_staging_writable=*/true);
  const auto cell = store->cellIndex(kLat, kLon);
  // TRUSTED Survey shoal: seafloor 0.5 m down, σ 0.2 ≤ gate. worst_case =
  // 0.5 − 0.2 = 0.3 < minimum_depth → LETHAL (trusted keepout).
  store->set(SourceLayer::Draft, cell, BathyCell{-0.5, 0.2});
  // UNTRUSTED Chart sample, SHALLOWER point estimate (−0.2 > −0.5) so it would win
  // a shallowest-depth pick, but σ 2.0 > gate → caution cap, never keepout.
  store->set(SourceLayer::Chart, cell, BathyCell{-0.2, 2.0});
  layer.setStore(std::move(store));

  const auto result = layer.evaluateCell(cell);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(*result, nav2_costmap_2d::LETHAL_OBSTACLE)
    << "the trusted Survey keepout must survive a shallower untrusted Chart sample "
       "(cost = MAX over reliable samples, not a shallowest-depth pick)";

  // Control: the Chart sample ALONE is caution, confirming the LETHAL above came
  // from the trusted Survey sample via the max — not from the Chart sample.
  auto chart_only = std::make_unique<BathymetryStore>(
    5, /*reference_writable=*/false, /*chart_staging_writable=*/true);
  const auto chart_cell = chart_only->cellIndex(kLat, kLon);
  chart_only->set(SourceLayer::Chart, chart_cell, BathyCell{-0.2, 2.0});
  layer.setStore(std::move(chart_only));
  const auto chart_result = layer.evaluateCell(chart_cell);
  ASSERT_TRUE(chart_result.has_value());
  EXPECT_EQ(*chart_result, nav2_costmap_2d::MAX_NON_OBSTACLE)
    << "the untrusted Chart sample on its own is caution, not LETHAL";
}

// ---------------------------------------------------------------------------
// Test case (#276, issue-named "ramp continuity at gate boundaries"): within the
// [minimum_depth, maximum_caution_depth] ramp band, cost does NOT depend on
// trust, so as a sample's σ crosses confidence_gate (trusted ⇄ untrusted) at a
// fixed worst-case clearance the cost is unchanged — no discontinuity. The only
// trust-dependent difference is strictly below minimum_depth (trusted → LETHAL,
// untrusted → the caution cap), which is the intended keepout cliff for trusted
// data; an untrusted cell has no cliff at minimum_depth at all.
// ---------------------------------------------------------------------------
TEST(BathymetryLayer, RampContinuityAcrossConfidenceGate)
{
  BathymetryLayerForTest layer;
  layer.setMinimumDepth(1.0);
  layer.setMaximumCautionDepth(3.0);

  // In the ramp band the cost is trust-independent → identical across the split.
  for (const double w : {1.0 + 1e-6, 1.5, 2.0, 2.5, 3.0 - 1e-6}) {
    EXPECT_EQ(layer.computeCost(w, true), layer.computeCost(w, false))
      << "ramp cost must not depend on trust at worst_case=" << w;
  }
  // At/above the FREE boundary both are FREE (trust-independent).
  EXPECT_EQ(layer.computeCost(3.0, true), layer.computeCost(3.0, false));
  EXPECT_EQ(layer.computeCost(3.0, false), nav2_costmap_2d::FREE_SPACE);

  // The single trust-dependent split is strictly below minimum_depth.
  EXPECT_EQ(layer.computeCost(0.5, true), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(layer.computeCost(0.5, false), nav2_costmap_2d::MAX_NON_OBSTACLE);
  EXPECT_LT(
    static_cast<int>(nav2_costmap_2d::MAX_NON_OBSTACLE),
    static_cast<int>(nav2_costmap_2d::LETHAL_OBSTACLE))
    << "the untrusted caution cap must be strictly below LETHAL";

  // Untrusted continuity across minimum_depth: the ramp top (at minimum_depth)
  // and the below-threshold cap are the SAME value (MAX_NON_OBSTACLE), so an
  // untrusted cell has no cost cliff at minimum_depth.
  EXPECT_EQ(layer.computeCost(1.0, false), nav2_costmap_2d::MAX_NON_OBSTACLE);
  EXPECT_EQ(layer.computeCost(1.0 - 1e-6, false), nav2_costmap_2d::MAX_NON_OBSTACLE);
}

// ---------------------------------------------------------------------------
// Test case 9 (acceptance criterion): memory-bound windowed residency. After
// loading a small window and then evicting outside a small box, the resident
// tile count stays bounded — the layer never holds the whole store.
// ---------------------------------------------------------------------------
namespace
{
// Sum the resident tiles across all layers of a store (one fused surface per
// layer since #221).
std::size_t residentTileCount(const BathymetryStore & store)
{
  std::size_t total = 0;
  for (const auto layer : marine_bathymetry_store::source_layers_by_priority) {
    total += store.tiles(layer).size();
  }
  return total;
}
}  // namespace

TEST(BathymetryLayer, WindowedResidencyStaysBounded)
{
  namespace fs = std::filesystem;
  const fs::path dir =
    fs::temp_directory_path() /
    ("bathymetry_layer_window_" + std::to_string(::getpid()));
  fs::remove_all(dir);
  fs::create_directories(dir);

  // Build a store spanning a wide geographic strip so its tiles land in
  // distinct GGGS grids, then persist it to disk. reference_writable=true so
  // the synthetic prior can be written on the read-only Reference layer.
  {
    BathymetryStore store(12, /*reference_writable=*/true);
    for (int k = 0; k < 8; ++k) {
      const double lon = -70.5 + 0.05 * static_cast<double>(k);
      const auto cell = store.cellIndex(43.0, lon);
      store.set(SourceLayer::Reference, cell, BathyCell{-10.0, 0.2});
    }
    const std::size_t written = marine_bathymetry_store::save(store, dir.string(), nullptr);
    ASSERT_GT(written, 0u);
  }

  // A fresh consumer store loads only a small window, leaving most tiles on disk.
  BathymetryStore consumer(12, /*reference_writable=*/false);
  const auto small_min = gggs::geoPoint(42.999, -70.51);
  const auto small_max = gggs::geoPoint(43.001, -70.49);
  marine_bathymetry_store::loadWindow(consumer, dir.string(), small_min, small_max);
  const std::size_t small_resident = residentTileCount(consumer);
  ASSERT_GT(small_resident, 0u) << "small window should load at least one tile";

  // Expand the window to cover the whole strip, then evict back to the small
  // box. Residency must shrink back to the small-window count — the consumer is
  // never forced to hold the full store at once.
  const auto big_min = gggs::geoPoint(42.99, -70.55);
  const auto big_max = gggs::geoPoint(43.01, -70.05);
  marine_bathymetry_store::loadWindow(consumer, dir.string(), big_min, big_max);
  const std::size_t big_resident = residentTileCount(consumer);
  EXPECT_GT(big_resident, small_resident)
    << "expanding the window should load additional tiles";

  marine_bathymetry_store::evictOutside(consumer, small_min, small_max);
  const std::size_t evicted_resident = residentTileCount(consumer);
  EXPECT_LE(evicted_resident, small_resident)
    << "eviction must drop residency back to the small-window bound";
  EXPECT_LT(evicted_resident, big_resident)
    << "eviction must actually free the out-of-window tiles";

  fs::remove_all(dir);
}

// ---------------------------------------------------------------------------
// Cost-tile cache / blit (#226). updateCosts no longer queries the store per
// cell every cycle; it blits pre-rendered, world-anchored tiles into the master
// grid. These exercise the blit (mapping + max-combine + cheapness) directly via
// the injectTile seam, without the store/TF/projection pipeline (generateTile's
// projection reuses the already-tested worldToLatLon + evaluateCell).
// ---------------------------------------------------------------------------

TEST(BathymetryLayer, TileBlitMaxCombineAndOffsets)
{
  BathymetryLayerForTest layer;
  layer.setStore(std::make_unique<BathymetryStore>(10));  // non-null for the guard
  layer.setEnabled(true);

  const int ts = layer.tileSize();
  // Tile at grid (0,0): world origin (0,0), 1 m cells, NO_INFORMATION default.
  auto tile = std::make_shared<nav2_costmap_2d::Costmap2D>(
    ts, ts, 1.0, 0.0, 0.0, nav2_costmap_2d::NO_INFORMATION);
  tile->setCost(5, 5, 100);
  tile->setCost(10, 20, 200);
  tile->setCost(7, 7, 80);
  tile->setCost(50, 50, nav2_costmap_2d::FREE_SPACE);
  // (40, 40) left NO_INFORMATION.
  layer.injectTile(0, 0, tile);

  nav2_costmap_2d::Costmap2D master(60, 60, 1.0, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);
  master.setCost(5, 5, 150);   // higher than the tile's 100 -> keep existing
  master.setCost(10, 20, 50);  // lower than the tile's 200 -> tile wins
  master.setCost(50, 50, nav2_costmap_2d::NO_INFORMATION);  // tile FREE fills it

  layer.updateCosts(master, 0, 0, 60, 60);

  EXPECT_EQ(static_cast<int>(master.getCost(5, 5)), 150)
    << "max-combine must keep the higher existing master cost";
  EXPECT_EQ(static_cast<int>(master.getCost(10, 20)), 200)
    << "tile raises a lower master cost (and the (10,20) offset maps correctly)";
  EXPECT_EQ(static_cast<int>(master.getCost(7, 7)), 80)
    << "tile cost fills over FREE_SPACE";
  EXPECT_EQ(
    static_cast<int>(master.getCost(40, 40)),
    static_cast<int>(nav2_costmap_2d::FREE_SPACE))
    << "tile NO_INFORMATION is skipped -- master left unchanged";
  EXPECT_EQ(
    static_cast<int>(master.getCost(50, 50)),
    static_cast<int>(nav2_costmap_2d::FREE_SPACE))
    << "tile cost fills a NO_INFORMATION master cell";
}

TEST(BathymetryLayer, TileBlitTracksRollingMasterOrigin)
{
  BathymetryLayerForTest layer;
  layer.setStore(std::make_unique<BathymetryStore>(10));
  layer.setEnabled(true);

  const int ts = layer.tileSize();
  auto tile = std::make_shared<nav2_costmap_2d::Costmap2D>(
    ts, ts, 1.0, 0.0, 0.0, nav2_costmap_2d::NO_INFORMATION);
  tile->setCost(35, 25, 170);
  layer.injectTile(0, 0, tile);

  // Master origin shifted to (30, 10): master cell (5, 15) is world ~(35.5,25.5),
  // i.e. tile cell (35, 25). Exercises the tile_offset_* index math (#226).
  nav2_costmap_2d::Costmap2D master(40, 40, 1.0, 30.0, 10.0, nav2_costmap_2d::FREE_SPACE);
  layer.updateCosts(master, 0, 0, 40, 40);

  EXPECT_EQ(static_cast<int>(master.getCost(5, 15)), 170)
    << "tile cell (35,25) must blit to master cell (5,15) under a (30,10) origin";
  EXPECT_EQ(
    static_cast<int>(master.getCost(6, 15)),
    static_cast<int>(nav2_costmap_2d::FREE_SPACE))
    << "the neighbouring master cell maps to an un-set tile cell";
}

TEST(BathymetryLayer, TileBlitIsCheapOverLargeGrid)
{
  BathymetryLayerForTest layer;
  layer.setStore(std::make_unique<BathymetryStore>(10));
  layer.setEnabled(true);

  const int ts = layer.tileSize();
  const int n = 500;  // 500x500 = 250k master cells
  const int tiles_per_side = (n + ts - 1) / ts;
  for (int ti = 0; ti < tiles_per_side; ++ti) {
    for (int tj = 0; tj < tiles_per_side; ++tj) {
      auto tile = std::make_shared<nav2_costmap_2d::Costmap2D>(
        ts, ts, 1.0,
        static_cast<double>(ti) * ts, static_cast<double>(tj) * ts,
        nav2_costmap_2d::FREE_SPACE);
      layer.injectTile(ti, tj, tile);
    }
  }

  nav2_costmap_2d::Costmap2D master(n, n, 1.0, 0.0, 0.0, nav2_costmap_2d::NO_INFORMATION);

  const auto t0 = std::chrono::steady_clock::now();
  layer.updateCosts(master, 0, 0, n, n);
  const auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - t0).count();

  // A memory-copy blit of 250k cells is sub-ms to a few ms. This guards the blit
  // staying O(cells) — no per-cell projection/tf creeping back into the copy. It
  // does NOT exercise the store (none is loaded; generateTile owns the store
  // query), so it cannot catch a per-cell *store-query* regression — that path is
  // covered end to end in sim. The 1 s bound is generous against CI noise.
  std::cout << "[ PERF     ] tile blit over " << (n * n) << " cells: "
            << dt << " ms" << std::endl;
  EXPECT_LT(dt, 1000)
    << "updateCosts must be an O(cells) blit, not a per-cell store query";
  EXPECT_EQ(
    static_cast<int>(master.getCost(250, 250)),
    static_cast<int>(nav2_costmap_2d::FREE_SPACE))
    << "interior master cell got the tile's cost";
}

// ---------------------------------------------------------------------------
// Test case (#220): the water-surface height is read from map_frame, NOT the
// costmap's global frame. bizzy's costmaps render in map_tide, so referencing
// the global frame was a degenerate self-lookup (identity → z=0) that read the
// whole survey LETHAL. These tests pin the frame the tide is measured against.
// ---------------------------------------------------------------------------

// Publish a map_frame -> map_tide transform with the sea surface at the given
// ellipsoidal height (z of map_tide in map). map_frame's z=0 is the ellipsoid.
static void publishMapTide(
  std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string & map_frame,
  double surface_z)
{
  static int64_t stamp_ns = 1;
  geometry_msgs::msg::TransformStamped mt;
  mt.header.stamp = rclcpp::Time(stamp_ns++);
  mt.header.frame_id = map_frame;
  mt.child_frame_id = "map_tide";
  mt.transform.translation.z = surface_z;
  mt.transform.rotation.w = 1.0;
  tf_buffer->setTransform(mt, "test_authority", false);
}

class TideFrameTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    // ExpandUserPathHandlesTilde overwrites HOME to a non-existent path; without
    // a writable log directory rclcpp::init() throws while configuring logging.
    // Pin ROS_LOG_DIR to a writable temp dir so init succeeds regardless of the
    // HOME another test may have left behind (test-ordering isolation).
    ::setenv("ROS_LOG_DIR", "/tmp/bathymetry_layer_test_log", 1);
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

// The costmap global frame is map_tide (as on bizzy), map_frame is the distinct
// ellipsoid frame. updateBounds must read the surface height from map_frame ->
// map_tide (≈ +48.88 m), not from a global-frame self-lookup (which would be 0).
TEST_F(TideFrameTest, MapTideZReadFromMapFrameNotGlobalFrame)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("test_tide_frame");
  node->declare_parameter("bathymetry_layer.map_frame", std::string("map"));
  node->declare_parameter("bathymetry_layer.map_tide_frame", std::string("map_tide"));

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  publishMapTide(tf_buffer, "map", 48.88);

  // Global frame is map_tide — the exact deployment that triggered #220.
  nav2_costmap_2d::LayeredCostmap layered_costmap("map_tide", false, false);
  layered_costmap.resizeMap(10, 10, 1.0, 0.0, 0.0);

  BathymetryLayerForTest layer;
  layer.initialize(&layered_costmap, "bathymetry_layer", tf_buffer.get(), node, nullptr);

  double minx = 1e30, miny = 1e30, maxx = -1e30, maxy = -1e30;
  layer.updateBounds(0.0, 0.0, 0.0, &minx, &miny, &maxx, &maxy);

  EXPECT_TRUE(layer.isMapTideValid())
    << "a valid map_frame -> map_tide transform must open the tide gate";
  EXPECT_NEAR(layer.getMapTideZ(), 48.88, 1e-6)
    << "map_tide_z_ must be the surface height in map_frame, not 0 from a "
       "global-frame self-lookup";

  // Negative control for the #276 deprecation guard: this config does NOT set the
  // removed `max_uncertainty` key, so the guard must not fire.
  EXPECT_FALSE(layer.deprecatedMaxUncertaintySeen())
    << "no deprecated key set → deprecation guard must stay quiet";
}

// #276 deprecation guard: a config that still declares the removed
// `max_uncertainty` key must trip the one-shot migration WARN (recorded via the
// latch) rather than be silently ignored — the rename ALSO inverted the meaning
// (reject-filter → trust-threshold), so silent drift would carry the wrong model.
TEST_F(TideFrameTest, DeprecatedMaxUncertaintyKeyIsDetected)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("test_deprecated_key");
  node->declare_parameter("bathymetry_layer.map_frame", std::string("map"));
  node->declare_parameter("bathymetry_layer.map_tide_frame", std::string("map_tide"));
  // A stale config still carrying the removed key (from YAML overrides).
  node->declare_parameter("bathymetry_layer.max_uncertainty", 0.5);

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  publishMapTide(tf_buffer, "map", 48.88);

  nav2_costmap_2d::LayeredCostmap layered_costmap("map_tide", false, false);
  layered_costmap.resizeMap(10, 10, 1.0, 0.0, 0.0);

  BathymetryLayerForTest layer;
  layer.initialize(&layered_costmap, "bathymetry_layer", tf_buffer.get(), node, nullptr);

  EXPECT_TRUE(layer.deprecatedMaxUncertaintySeen())
    << "a config still setting max_uncertainty must trip the deprecation guard "
       "(one-shot WARN), not be silently ignored";
}

// #276 local-review must-fix: an invalid confidence_gate (NaN / negative / zero)
// silently disables ALL trusted keepout, because `σ ≤ gate` is then false for
// every realistic sample — a fail-quiet safety hole. onInitialize must validate
// it (finite & > 0) and reset to the default, mirroring the depth-ramp guard.
TEST_F(TideFrameTest, InvalidConfidenceGateResetsToDefault)
{
  auto make_node = [](const std::string & name, double gate) {
      auto node = std::make_shared<nav2_util::LifecycleNode>(name);
      node->declare_parameter("bathymetry_layer.map_frame", std::string("map"));
      node->declare_parameter("bathymetry_layer.map_tide_frame", std::string("map_tide"));
      node->declare_parameter("bathymetry_layer.confidence_gate", gate);
      return node;
    };

  // Each bad value (negative, zero, NaN) must be rejected and reset to the 0.5 m
  // default so trusted keepout keeps working.
  for (const auto & [suffix, gate] : std::vector<std::pair<std::string, double>>{
    {"neg", -1.0}, {"zero", 0.0},
    {"nan", std::numeric_limits<double>::quiet_NaN()}})
  {
    auto node = make_node("test_bad_confidence_gate_" + suffix, gate);
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    publishMapTide(tf_buffer, "map", 48.88);

    nav2_costmap_2d::LayeredCostmap layered_costmap("map_tide", false, false);
    layered_costmap.resizeMap(10, 10, 1.0, 0.0, 0.0);

    BathymetryLayerForTest layer;
    layer.initialize(&layered_costmap, "bathymetry_layer", tf_buffer.get(), node, nullptr);

    EXPECT_DOUBLE_EQ(layer.getConfidenceGate(), 0.5)
      << "confidence_gate=" << gate << " must reset to the 0.5 m default, not "
      "silently disable all keepout";
  }

  // A valid positive gate is honoured (not clobbered by the guard).
  auto node = make_node("test_good_confidence_gate", 1.25);
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  publishMapTide(tf_buffer, "map", 48.88);
  nav2_costmap_2d::LayeredCostmap layered_costmap("map_tide", false, false);
  layered_costmap.resizeMap(10, 10, 1.0, 0.0, 0.0);
  BathymetryLayerForTest layer;
  layer.initialize(&layered_costmap, "bathymetry_layer", tf_buffer.get(), node, nullptr);
  EXPECT_DOUBLE_EQ(layer.getConfidenceGate(), 1.25)
    << "a valid confidence_gate must be preserved";
}

// A degenerate config (map_frame == map_tide_frame) must NOT be accepted: the
// lookup would be an identity (z=0). The tide gate stays closed so no bogus
// LETHAL flood is written — fail loud, do not paper over it.
TEST_F(TideFrameTest, DegenerateTideFrameConfigRejected)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("test_tide_frame_degenerate");
  node->declare_parameter("bathymetry_layer.map_frame", std::string("map_tide"));
  node->declare_parameter("bathymetry_layer.map_tide_frame", std::string("map_tide"));

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  nav2_costmap_2d::LayeredCostmap layered_costmap("map_tide", false, false);
  layered_costmap.resizeMap(10, 10, 1.0, 0.0, 0.0);

  BathymetryLayerForTest layer;
  layer.initialize(&layered_costmap, "bathymetry_layer", tf_buffer.get(), node, nullptr);

  double minx = 1e30, miny = 1e30, maxx = -1e30, maxy = -1e30;
  layer.updateBounds(0.0, 0.0, 0.0, &minx, &miny, &maxx, &maxy);

  EXPECT_FALSE(layer.isMapTideValid())
    << "map_frame == map_tide_frame is a degenerate identity lookup and must be "
       "rejected as an invalid tide";
}

// ---------------------------------------------------------------------------
// generateTile's whole-tile coverage short-circuit (#226 perf fix part 2):
// a tile whose projected lat/lon AABB overlaps no resident store tile is filled
// uniformly without the per-cell projection+query. The safety-critical property
// is that a tile is NEVER wrongly classified no-coverage (which would drop real
// bathymetry when unsurveyed_is_lethal is false), so the test pins the margin
// behaviour: straddling/within-margin tiles count as covered; only clearly
// disjoint tiles are skipped.
TEST(BathymetryLayer, TileCoverageOverlapDecision)
{
  BathymetryLayerForTest layer;
  using Box = BathymetryLayerForTest::CoverageBox;
  // One resident store tile, roughly a Lake Massabesic patch.
  const std::vector<BathymetryLayerForTest::CoverageBox> coverage{
    Box{42.98, -71.40, 43.00, -71.38}};

  // Fully inside coverage → covered (full per-cell render).
  EXPECT_TRUE(layer.tileHasCoverage(42.985, -71.395, 42.990, -71.390, coverage));
  // Straddling the coverage edge → covered (conservative).
  EXPECT_TRUE(layer.tileHasCoverage(42.995, -71.385, 43.005, -71.375, coverage));
  // Just outside the top edge but within the ~11 m (1e-4 deg) margin → still
  // covered: a covered cell must never be dropped.
  EXPECT_TRUE(layer.tileHasCoverage(43.00005, -71.390, 43.00010, -71.385, coverage));
  // Clearly beyond the margin (~22 m above) → no coverage (eligible for the
  // uniform-fill short-circuit).
  EXPECT_FALSE(layer.tileHasCoverage(43.00030, -71.390, 43.00040, -71.385, coverage));
  // Far away → no coverage.
  EXPECT_FALSE(layer.tileHasCoverage(42.000, -72.000, 42.001, -71.999, coverage));
  // Empty store (no data loaded) → nothing is covered, so every tile takes the
  // short-circuit (uniform lethal land / NO_INFORMATION).
  EXPECT_FALSE(layer.tileHasCoverage(42.985, -71.395, 42.990, -71.390, {}));
}

// #2 safety gate: when the store window has NO coverage and unsurveyed_is_lethal
// is set, the whole costmap would read LETHAL (incl. the boat's own cell). The
// layer must report NOT current in that case rather than asserting the fabricated
// all-lethal grid as a usable costmap. Drives the gate through updateCosts (which
// sets current_) + isCurrent().
TEST(BathymetryLayer, EmptyCoverageWithUnsurveyedLethalHoldsNotCurrent)
{
  BathymetryLayerForTest layer;
  layer.setStore(std::make_unique<BathymetryStore>(10));  // non-null for the guard
  layer.setEnabled(true);
  layer.setUnsurveyedIsLethal(true);

  const int ts = layer.tileSize();
  auto tile = std::make_shared<nav2_costmap_2d::Costmap2D>(
    ts, ts, 1.0, 0.0, 0.0, nav2_costmap_2d::NO_INFORMATION);
  layer.injectTile(0, 0, tile);   // marks core_ready_ true
  layer.setMapTideValid(true);
  layer.setWindowValid(true);

  nav2_costmap_2d::Costmap2D master(ts, ts, 1.0, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);

  // Coverage present (flag false): all gates satisfied → current.
  layer.setCoverageEmptyLethal(false);
  layer.updateCosts(master, 0, 0, ts, ts);
  EXPECT_TRUE(layer.isCurrent())
    << "tide + window valid and tiles rendered → costmap is current";

  // Empty-coverage-lethal: must drop to not-current even though every other gate
  // still holds, so the planner does not act on an all-lethal grid.
  layer.setCoverageEmptyLethal(true);
  layer.updateCosts(master, 0, 0, ts, ts);
  EXPECT_FALSE(layer.isCurrent())
    << "empty store window + unsurveyed_is_lethal must report NOT current";
}

// #3: a tide drift past the invalidation threshold marks every tile needs_update;
// gating current_ on that would drop the costmap to not-current for a full-window
// re-render (and re-trip the planner timeout) even though the cached costs are
// only sub-threshold stale. windowFullyRendered must therefore treat a
// generated-but-needs_update tile as STILL rendered (ready), and only a
// never-rendered (absent) tile as not-ready.
TEST(BathymetryLayer, WindowReadyTreatsStaleTileAsRendered)
{
  BathymetryLayerForTest layer;
  layer.setStore(std::make_unique<BathymetryStore>(10));
  const int ts = layer.tileSize();
  auto make_tile = [ts]() {
      return std::make_shared<nav2_costmap_2d::Costmap2D>(
        ts, ts, 1.0, 0.0, 0.0, nav2_costmap_2d::NO_INFORMATION);
    };
  layer.injectTile(0, 0, make_tile());
  layer.injectTile(0, 1, make_tile());
  layer.injectTile(1, 0, make_tile());
  layer.injectTile(1, 1, make_tile());

  EXPECT_TRUE(layer.windowFullyRendered(0, 0, 1, 1))
    << "every window tile generated → ready";

  // Mark one tile stale (as a tide move would): still rendered, still ready.
  layer.markTileNeedsUpdate(1, 1);
  EXPECT_TRUE(layer.windowFullyRendered(0, 0, 1, 1))
    << "a generated-but-stale (needs_update) tile must NOT hold current_ false (#3)";

  // A window region with a never-rendered (absent) tile is not ready.
  EXPECT_FALSE(layer.windowFullyRendered(0, 0, 2, 2))
    << "an absent (never-rendered) tile → not ready";
}

// ---------------------------------------------------------------------------
// Test case 14 (uma#369 round 2, must-fix 1): the region-aware safety query
// must reach the costmap.
//
// `processed` is depth-adaptive (uma#369), so a level-14 tile sits under a
// level-10 costmap query cell: 256 native cells, one costmap cell. While
// evaluateCell gated on `bestSource` — a point lookup at the query cell's
// CENTRE — a single no-data native cell there (a gated-drop hole, a
// between-lines gap, an absent fine tile beside a present one) returned
// nullopt, `reliableSamples` was never called, and the rock in one of the other
// 255 cells was dropped. That is the walk-past-the-rock failure the region-aware
// query exists to prevent, in the one consumer that steers the boat.
//
// Both of these FAIL against the `bestSource` gate and pass against `hasAnyData`.
// ---------------------------------------------------------------------------
TEST(BathymetryLayer, ShoalOffCentreIsCostedWhenTheCentreCellIsNoData)
{
  BathymetryLayerForTest layer;
  layer.setMinimumDepth(1.0);
  layer.setMaximumCautionDepth(3.0);
  layer.setConfidenceGate(0.5);
  layer.setMapTideZ(0.0);
  layer.setMapTideValid(true);

  auto store = std::make_unique<BathymetryStore>(10);
  const gggs::Level query_level(10);
  const gggs::Level fine(12);              // 4x4 = 16 native cells per query cell
  const auto query_cell = query_level.cellIndex(gggs::geoPoint(kLat, kLon));

  const auto cell_box_min = query_cell.position();
  const gggs::GridIndex & qgrid = query_cell.grid();
  const double lat_span = qgrid.latitudinalSpan() / gggs::cell_rows_per_grid;
  const double lon_span = qgrid.longitudinalSpan() / gggs::cell_columns_per_grid;
  const auto point_in_query_cell = [&](double lat_f, double lon_f) {
      return gggs::geoPoint(
        cell_box_min.latitude + lat_f * lat_span,
        cell_box_min.longitude + lon_f * lon_span);
    };

  // A gated-drop hole exactly under the query cell's centre: written, no data.
  const auto centre_cell = fine.cellIndex(point_in_query_cell(0.5, 0.5));
  store->set(SourceLayer::Processed, centre_cell, BathyCell{});

  // A 0.4 m rock in the SW-most covered cell — trusted (σ 0.1 ≤ gate 0.5), so
  // worst-case clearance 0.4 − 0.1 = 0.3 m < minimum_depth 1.0 → LETHAL.
  const auto rock_cell = fine.cellIndex(point_in_query_cell(0.125, 0.125));
  ASSERT_NE(rock_cell, centre_cell) << "the rock must not sit under the centre, "
    "or a point-sampling gate would pass this test and prove nothing";
  store->set(SourceLayer::Processed, rock_cell, BathyCell{-0.4, 0.1});

  layer.setStore(std::move(store));

  const auto result = layer.evaluateCell(query_cell);
  ASSERT_TRUE(result.has_value())
    << "the layer declared a surveyed cell unsurveyed because its CENTRE native "
    "cell holds no data";
  EXPECT_EQ(*result, nav2_costmap_2d::LETHAL_OBSTACLE)
    << "a trusted 0.4 m rock inside the query cell was not costed";
}

TEST(BathymetryLayer, CentreNoDataDoesNotSuppressUnsurveyedIsLethal)
{
  // The same geometry under the closed-basin policy: the cell IS surveyed
  // (a covered native cell holds data), so the unsurveyed-is-lethal "no data
  // means land" branch must not claim it. Here the covered data is deep and
  // reliable, so the correct answer is FREE_SPACE — the opposite verdict from
  // the LETHAL a point-sampling gate would have written.
  BathymetryLayerForTest layer;
  layer.setMinimumDepth(1.0);
  layer.setMaximumCautionDepth(3.0);
  layer.setConfidenceGate(0.5);
  layer.setMapTideZ(0.0);
  layer.setMapTideValid(true);
  layer.setUnsurveyedIsLethal(true);

  auto store = std::make_unique<BathymetryStore>(10);
  const gggs::Level query_level(10);
  const gggs::Level fine(12);
  const auto query_cell = query_level.cellIndex(gggs::geoPoint(kLat, kLon));
  const auto sw = query_cell.position();
  const gggs::GridIndex & qgrid = query_cell.grid();
  const double lat_span = qgrid.latitudinalSpan() / gggs::cell_rows_per_grid;
  const double lon_span = qgrid.longitudinalSpan() / gggs::cell_columns_per_grid;

  store->set(
    SourceLayer::Processed,
    fine.cellIndex(gggs::geoPoint(sw.latitude + 0.5 * lat_span, sw.longitude + 0.5 * lon_span)),
    BathyCell{});
  store->set(
    SourceLayer::Processed,
    fine.cellIndex(
      gggs::geoPoint(sw.latitude + 0.875 * lat_span, sw.longitude + 0.875 * lon_span)),
    BathyCell{-20.0, 0.1});

  layer.setStore(std::move(store));

  const auto result = layer.evaluateCell(query_cell);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(*result, nav2_costmap_2d::FREE_SPACE)
    << "a surveyed, deep, reliable cell was written LETHAL as if it were land, "
    "because its centre native cell holds no data";
}

// ---------------------------------------------------------------------------
// Test case 15 (uma#369 round 3, must-fix 1): a costmap cell is costed over its
// WHOLE ground, not over the one GGGS cell under its centre.
//
// `BathymetryStore::fromCellSize(resolution)` returns the coarsest level whose
// cells are at or finer than the costmap resolution, so a query cell is always
// SMALLER than the costmap cell it costs — 0.60 m² of 1.00 m² at 1 m and
// 43.5°N, and as little as 18% of it just under a level boundary. Costing from
// the centre query cell alone therefore left 40-82% of every costmap cell
// unread at every store resolution, which is the same walk-past-the-rock
// failure as test case 14, one level up.
//
// All three build a box spanning 2x2 query cells whose CENTRE lies in the
// south-west one, so a centre-sampling implementation reads only that cell.
// ---------------------------------------------------------------------------
namespace
{

// The lat/lon box of a 2x2 block of query cells whose south-west cell contains
// (kLat, kLon), plus the block's cell spans. The box stops at 1.5 spans so its
// centre (0.75 of a span) stays inside the south-west cell.
struct QueryCellBlock
{
  gggs::CellIndex south_west;
  double lat_span;
  double lon_span;
  double min_lat;
  double min_lon;
  double max_lat;
  double max_lon;
};

QueryCellBlock queryCellBlock(const gggs::Level & level)
{
  const auto south_west = level.cellIndex(gggs::geoPoint(kLat, kLon));
  const gggs::GridIndex & grid = south_west.grid();
  const double lat_span = grid.latitudinalSpan() / gggs::cell_rows_per_grid;
  const double lon_span = grid.longitudinalSpan() / gggs::cell_columns_per_grid;
  const auto corner = south_west.position();
  return QueryCellBlock{
    south_west, lat_span, lon_span,
    corner.latitude, corner.longitude,
    corner.latitude + 1.5 * lat_span, corner.longitude + 1.5 * lon_span};
}

}  // namespace

TEST(BathymetryLayer, ShoalOutsideTheCentreQueryCellIsCosted)
{
  BathymetryLayerForTest layer;
  layer.setMinimumDepth(1.0);
  layer.setMaximumCautionDepth(3.0);
  layer.setConfidenceGate(0.5);
  layer.setMapTideZ(0.0);
  layer.setMapTideValid(true);

  auto store = std::make_unique<BathymetryStore>(10);
  const gggs::Level query_level(10);
  const auto block = queryCellBlock(query_level);

  // A trusted 0.4 m rock in the block's NORTH-EAST query cell: inside the
  // costmap cell, outside the query cell holding the costmap cell's centre.
  const auto rock_cell = query_level.cellIndex(
    gggs::geoPoint(
      block.min_lat + 1.25 * block.lat_span, block.min_lon + 1.25 * block.lon_span));
  ASSERT_NE(rock_cell, block.south_west)
    << "the rock must not sit in the centre query cell, or centre-sampling "
    "would pass this test and prove nothing";
  store->set(SourceLayer::Processed, rock_cell, BathyCell{-0.4, 0.1});

  layer.setStore(std::move(store));

  const auto result = layer.evaluateCostmapCell(
    block.min_lat, block.min_lon, block.max_lat, block.max_lon);
  ASSERT_TRUE(result.has_value())
    << "a rock inside the costmap cell was not read at all";
  EXPECT_EQ(*result, nav2_costmap_2d::LETHAL_OBSTACLE)
    << "a 0.4 m rock inside the costmap cell but outside its centre query cell "
    "was not costed";
}

TEST(BathymetryLayer, TheMostHazardousQueryCellInTheCostmapCellWins)
{
  BathymetryLayerForTest layer;
  layer.setMinimumDepth(1.0);
  layer.setMaximumCautionDepth(3.0);
  layer.setConfidenceGate(0.5);
  layer.setMapTideZ(0.0);
  layer.setMapTideValid(true);

  auto store = std::make_unique<BathymetryStore>(10);
  const gggs::Level query_level(10);
  const auto block = queryCellBlock(query_level);

  // Deep, reliable water in the centre query cell — FREE_SPACE on its own.
  store->set(SourceLayer::Processed, block.south_west, BathyCell{-20.0, 0.1});
  // A rock in a cell the costmap cell also covers.
  store->set(
    SourceLayer::Processed,
    query_level.cellIndex(
      gggs::geoPoint(
        block.min_lat + 1.25 * block.lat_span, block.min_lon + 1.25 * block.lon_span)),
    BathyCell{-0.4, 0.1});

  layer.setStore(std::move(store));

  const auto result = layer.evaluateCostmapCell(
    block.min_lat, block.min_lon, block.max_lat, block.max_lon);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(*result, nav2_costmap_2d::LETHAL_OBSTACLE)
    << "deep water under the costmap cell's centre masked a rock elsewhere in "
    "the same costmap cell";
}

TEST(BathymetryLayer, AnUnsurveyedQueryCellMakesTheCostmapCellLandUnderTheBasinFlag)
{
  // The deliberate asymmetry (round 3): INSIDE a query cell, partial no-data
  // still counts as surveyed — those are routine gaps in one fused surface.
  // ACROSS the query cells of a costmap cell, an unsurveyed one is a whole cell
  // of the basin's prior with nothing in it, which is what unsurveyed_is_lethal
  // calls land, so the costmap cell goes LETHAL even though its centre is deep
  // open water.
  BathymetryLayerForTest layer;
  layer.setMinimumDepth(1.0);
  layer.setMaximumCautionDepth(3.0);
  layer.setConfidenceGate(0.5);
  layer.setMapTideZ(0.0);
  layer.setMapTideValid(true);
  layer.setUnsurveyedIsLethal(true);

  auto store = std::make_unique<BathymetryStore>(10);
  const gggs::Level query_level(10);
  const auto block = queryCellBlock(query_level);
  store->set(SourceLayer::Processed, block.south_west, BathyCell{-20.0, 0.1});

  layer.setStore(std::move(store));

  const auto centre_only = layer.evaluateCell(block.south_west);
  ASSERT_TRUE(centre_only.has_value());
  ASSERT_EQ(*centre_only, nav2_costmap_2d::FREE_SPACE)
    << "the centre query cell must be free water, or this test proves nothing "
    "about the cells around it";

  const auto result = layer.evaluateCostmapCell(
    block.min_lat, block.min_lon, block.max_lat, block.max_lon);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(*result, nav2_costmap_2d::LETHAL_OBSTACLE)
    << "an unsurveyed query cell inside the costmap cell was not treated as "
    "land under unsurveyed_is_lethal";
}
