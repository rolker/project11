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

// [uma#369] Tests for the depth -> GGGS level policy (uma-ADR-0010 D9 as
// amended). The load-bearing assertions are the LEVEL TRANSITION DEPTHS and the
// two clamps — the policy's actual content. An earlier draft proposed asserting
// `cell * sqrt(2)/2 < capture radius` as the headline invariant; that is
// near-tautological given fromCellSize's at-or-finer contract (it tests the GGGS
// API, not this policy), so it survives here only as a regression guard.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include "marine_autonomy/gggs.h"
#include "marine_bathymetry_store/depth_adaptive_level.hpp"

using marine_bathymetry_store::DepthAdaptiveLevelPolicy;
using marine_bathymetry_store::depthAdaptiveLevel;

namespace
{

/// The shallowest depth that still maps to @p level under @p policy:
/// `Level(level).cellSize() / capture_distance_scale`. Derived from the GGGS API
/// rather than hard-coded so the test tracks the level table, not a transcription
/// of it.
double transitionDepth(uint8_t level, const DepthAdaptiveLevelPolicy & policy = {})
{
  return gggs::Level(level).cellSize() / policy.capture_distance_scale;
}

}  // namespace

// Every ladder transition, from the coarse clamp's crossover up to the fine
// clamp: at the boundary the coarser level applies; a hair below it, the next
// finer one. These pin the mapping the ADR amendment publishes.
TEST(DepthAdaptiveLevel, TransitionDepths)
{
  const DepthAdaptiveLevelPolicy policy;
  for (uint8_t level = policy.coarsest_level + 1; level <= policy.finest_level; ++level) {
    const double boundary = transitionDepth(level);
    SCOPED_TRACE("level " + std::to_string(level) + " boundary " + std::to_string(boundary));
    // Just deeper than the boundary, this level applies. (Exactly ON the
    // boundary is deliberately not asserted: the requested cell size is then
    // bit-for-bit this level's cell size, and fromCellSize takes a float, so
    // which side of the ceil() it lands on is a rounding accident rather than a
    // policy statement.)
    EXPECT_EQ(depthAdaptiveLevel(boundary * 1.001, policy).level(), level);
    // Just shallower, one level finer (clamped at finest_level).
    const uint8_t finer = std::min<uint8_t>(level + 1, policy.finest_level);
    EXPECT_EQ(depthAdaptiveLevel(boundary * 0.999, policy).level(), finer);
  }
}

// A handful of literal depth -> level expectations, so the suite is not purely
// self-referential against the same API it is testing.
TEST(DepthAdaptiveLevel, LiteralExpectations)
{
  EXPECT_EQ(depthAdaptiveLevel(25.0).level(), 10);   // 1.25 m requested
  EXPECT_EQ(depthAdaptiveLevel(12.0).level(), 11);   // 0.60 m requested
  EXPECT_EQ(depthAdaptiveLevel(6.0).level(), 12);    // 0.30 m requested
  EXPECT_EQ(depthAdaptiveLevel(3.0).level(), 13);    // 0.15 m requested
  EXPECT_EQ(depthAdaptiveLevel(50.0).level(), 9);    // 2.50 m requested
}

// Both clamps genuinely bind under the pinned constants — unlike the first
// draft's finest_level = 11, which the 0.5 m floor made unreachable.
TEST(DepthAdaptiveLevel, ClampsBind)
{
  const DepthAdaptiveLevelPolicy policy;

  // Coarse end: below the level-8 crossover (~72.5 m) the ladder would keep
  // coarsening; the clamp holds it at 8.
  ASSERT_GT(transitionDepth(policy.coarsest_level), 72.0);
  EXPECT_EQ(depthAdaptiveLevel(100.0).level(), policy.coarsest_level);
  EXPECT_EQ(depthAdaptiveLevel(1000.0).level(), policy.coarsest_level);
  EXPECT_EQ(depthAdaptiveLevel(1.0e9).level(), policy.coarsest_level);

  // Fine end: below ~2.26 m the ladder would keep refining without bound.
  EXPECT_EQ(depthAdaptiveLevel(1.0).level(), policy.finest_level);
  EXPECT_EQ(depthAdaptiveLevel(0.2).level(), policy.finest_level);
  EXPECT_EQ(depthAdaptiveLevel(1.0e-9).level(), policy.finest_level);
  // Zero depth is guarded before fromCellSize (whose log2(0) path is UB).
  EXPECT_EQ(depthAdaptiveLevel(0.0).level(), policy.finest_level);
}

// The narrowing to float must not invert the ladder. `fromCellSize` takes a
// float, so a positive DOUBLE request can underflow to 0.0f (or overflow to
// +inf) on the way in and hit the log2(0)/log2(inf) path whose cast to int is
// undefined. Before the guard was moved onto the narrowed value,
// depthAdaptiveLevel(1e-44) returned the COARSEST level -- the exact inversion
// of "shallower water gets a finer lattice" -- for the shallowest input there
// is. Both ends must land on the shoal-biased side of the ladder.
TEST(DepthAdaptiveLevel, FloatNarrowingCannotInvertTheLadder)
{
  const DepthAdaptiveLevelPolicy policy;

  // Underflow end: 0.05 * 1e-44 is a positive double that is 0.0f as a float.
  ASSERT_GT(policy.capture_distance_scale * 1.0e-44, 0.0) << "test input is not a positive double";
  ASSERT_EQ(static_cast<float>(policy.capture_distance_scale * 1.0e-44), 0.0f) <<
    "test input no longer underflows in float";
  EXPECT_EQ(depthAdaptiveLevel(1.0e-44).level(), policy.finest_level);
  EXPECT_EQ(depthAdaptiveLevel(-1.0e-44).level(), policy.finest_level);
  // Smallest positive double at all: still the shallow end.
  EXPECT_EQ(
    depthAdaptiveLevel(std::numeric_limits<double>::denorm_min()).level(),
    policy.finest_level);

  // Overflow end: a finite double whose request exceeds FLT_MAX (~3.4e38).
  ASSERT_TRUE(std::isfinite(policy.capture_distance_scale * 1.0e40)) <<
    "test input is not a finite double";
  ASSERT_FALSE(std::isfinite(static_cast<float>(policy.capture_distance_scale * 1.0e40))) <<
    "test input no longer overflows in float";
  EXPECT_EQ(depthAdaptiveLevel(1.0e40).level(), policy.coarsest_level);
  EXPECT_EQ(depthAdaptiveLevel(std::numeric_limits<double>::max()).level(),
    policy.coarsest_level);

  // The same must hold for a custom policy: the clamps, not hard-coded levels.
  DepthAdaptiveLevelPolicy narrow;
  narrow.coarsest_level = 6;
  narrow.finest_level = 12;
  EXPECT_EQ(depthAdaptiveLevel(1.0e-44, narrow).level(), narrow.finest_level);
  EXPECT_EQ(depthAdaptiveLevel(1.0e40, narrow).level(), narrow.coarsest_level);
}

// The published policy IS the defaults. Every other test derives its
// expectations from the same struct, so changing a default would keep the whole
// suite green while contradicting the ladder the ADR and README publish to
// operators. Pin the operator-decided values themselves (uma#369).
TEST(DepthAdaptiveLevel, DefaultPolicyIsThePublishedOne)
{
  const DepthAdaptiveLevelPolicy policy;
  EXPECT_DOUBLE_EQ(policy.capture_distance_scale, 0.05) <<
    "mirrors cube_bathymetry::Parameters::capture_distance_scale — there is no "
    "automated link between the two repos, so a change here must be deliberate";
  EXPECT_EQ(policy.finest_level, 14u) << "operator-pinned fine clamp (~0.057 m cells)";
  EXPECT_EQ(policy.coarsest_level, 8u) << "operator-pinned coarse clamp (~3.624 m cells)";
}

// The level must never get finer as the water gets deeper: a regression inside
// one band would silently reintroduce the original bug at that band only.
TEST(DepthAdaptiveLevel, MonotonicInDepth)
{
  uint8_t previous = depthAdaptiveLevel(0.05).level();
  for (double depth = 0.05; depth <= 200.0; depth += 0.01) {
    const uint8_t level = depthAdaptiveLevel(depth).level();
    ASSERT_LE(level, previous) << "level rose (got finer) at depth " << depth;
    previous = level;
  }
  EXPECT_EQ(previous, DepthAdaptiveLevelPolicy().coarsest_level);
}

// Sign convention differs between the CUBE node and store consumers, so the
// policy decides on magnitude.
TEST(DepthAdaptiveLevel, SignIgnored)
{
  for (const double depth : {0.5, 2.3, 4.6, 9.1, 18.2, 40.0, 90.0}) {
    EXPECT_EQ(depthAdaptiveLevel(-depth).level(), depthAdaptiveLevel(depth).level()) <<
      "depth " << depth;
  }
}

// The constants are a policy, not a hard-coding: a different scale and a
// different clamp pair must both take effect.
TEST(DepthAdaptiveLevel, CustomPolicy)
{
  DepthAdaptiveLevelPolicy policy;
  policy.capture_distance_scale = 0.10;
  policy.coarsest_level = 6;
  policy.finest_level = 12;

  // Twice the scale asks for twice the cell size, i.e. one level coarser.
  EXPECT_EQ(
    depthAdaptiveLevel(12.0, policy).level(),
    depthAdaptiveLevel(24.0, DepthAdaptiveLevelPolicy()).level());
  // The narrowed clamps bind where the defaults would not.
  EXPECT_EQ(depthAdaptiveLevel(1.0, policy).level(), 12);
  EXPECT_EQ(depthAdaptiveLevel(500.0, policy).level(), 6);
}

// Configuration and input errors fail loud rather than clamping to a plausible
// but wrong answer.
TEST(DepthAdaptiveLevel, InvalidInputsThrow)
{
  DepthAdaptiveLevelPolicy inverted;
  inverted.coarsest_level = 14;
  inverted.finest_level = 8;
  EXPECT_THROW(depthAdaptiveLevel(10.0, inverted), std::invalid_argument);

  DepthAdaptiveLevelPolicy out_of_range;
  out_of_range.finest_level = 21;
  EXPECT_THROW(depthAdaptiveLevel(10.0, out_of_range), std::invalid_argument);

  DepthAdaptiveLevelPolicy bad_scale;
  bad_scale.capture_distance_scale = 0.0;
  EXPECT_THROW(depthAdaptiveLevel(10.0, bad_scale), std::invalid_argument);
  bad_scale.capture_distance_scale = -0.05;
  EXPECT_THROW(depthAdaptiveLevel(10.0, bad_scale), std::invalid_argument);

  // A NaN that clamped silently would write a whole survey at the fine clamp.
  EXPECT_THROW(depthAdaptiveLevel(std::nan("")), std::invalid_argument);
  EXPECT_THROW(depthAdaptiveLevel(std::numeric_limits<double>::infinity()),
    std::invalid_argument);
  EXPECT_THROW(depthAdaptiveLevel(-std::numeric_limits<double>::infinity()),
    std::invalid_argument);
}

// Regression guard for the defect in #369: at 9 m the fixed level 10 gives
// 0.906 m cells against a 0.45 m capture radius, so the cell a sounding lands in
// can be coarser than the radius that would accept it. The new policy never
// returns a cell coarser than the requested capture distance across the surveyed
// range (this follows from fromCellSize's contract — kept as a guard, not as the
// proof of the policy) and specifically returns finer than 10 at 9 m.
TEST(DepthAdaptiveLevel, RegressionAgainstFixedLevelTen)
{
  // A finer lattice is a HIGHER GGGS level number (level 11 = 0.453 m cells).
  EXPECT_GT(depthAdaptiveLevel(9.0).level(), 10) << "still as coarse as today's fixed level";
  EXPECT_GT(gggs::Level(10).cellSize(), 0.05 * 9.0) << "the defect this issue reports";

  const DepthAdaptiveLevelPolicy policy;
  // Start a hair ABOVE the fine clamp's boundary: exactly on it, which side of
  // fromCellSize's ceil() the request lands on is a rounding accident that
  // TransitionDepths deliberately declines to assert on.
  for (double depth = transitionDepth(policy.finest_level) * 1.001; depth <= 60.0;
    depth += 0.05)
  {
    const double capture_radius_m = policy.capture_distance_scale * depth;
    ASSERT_LE(depthAdaptiveLevel(depth).cellSize(), capture_radius_m + 1e-9) <<
      "cell coarser than the capture radius at depth " << depth;
  }
}
