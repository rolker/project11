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

// [uma-ADR-0010 D9 / uma#369] Depth-driven GGGS level selection for the
// `processed` survey layer. Pure policy: no I/O, no store state. See
// depth_adaptive_level.hpp for the ladder, the storage cost and the decision
// unit; the writer that will call this is cube_bathymetry#143.

#include "marine_bathymetry_store/depth_adaptive_level.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace marine_bathymetry_store
{

namespace
{

/// Highest level `gggs::Level` accepts (level.h clamps `fromCellSize` to this
/// and the constructor throws above it).
constexpr uint8_t kMaxGggsLevel = 20;

}  // namespace

gggs::Level depthAdaptiveLevel(double depth_m, const DepthAdaptiveLevelPolicy & policy)
{
  // Validate the policy before the depth: an inverted or out-of-range clamp is a
  // configuration error that would otherwise silently return the wrong end of
  // the ladder for every depth, not just this one.
  if (policy.coarsest_level > kMaxGggsLevel || policy.finest_level > kMaxGggsLevel) {
    throw std::invalid_argument(
            "depthAdaptiveLevel: clamp levels must be <= " + std::to_string(kMaxGggsLevel) +
            " (got coarsest " + std::to_string(policy.coarsest_level) + ", finest " +
            std::to_string(policy.finest_level) + ")");
  }
  if (policy.finest_level < policy.coarsest_level) {
    throw std::invalid_argument(
            "depthAdaptiveLevel: inverted clamp — finest_level " +
            std::to_string(policy.finest_level) + " is coarser than coarsest_level " +
            std::to_string(policy.coarsest_level));
  }
  if (!std::isfinite(policy.capture_distance_scale) || policy.capture_distance_scale <= 0.0) {
    throw std::invalid_argument(
            "depthAdaptiveLevel: capture_distance_scale must be finite and positive (got " +
            std::to_string(policy.capture_distance_scale) + ")");
  }

  // A non-finite depth must fail loud rather than clamp: a NaN reaching the
  // clamp silently writes an entire survey at the finest level.
  if (!std::isfinite(depth_m)) {
    throw std::invalid_argument(
            "depthAdaptiveLevel: depth_m must be finite (got " + std::to_string(depth_m) + ")");
  }

  // Sign convention differs between the CUBE node (which takes std::abs at
  // node.cpp:154) and store consumers, so decide on magnitude.
  const double requested_cell_size_m = policy.capture_distance_scale * std::abs(depth_m);

  // fromCellSize takes a FLOAT, so guard the narrowed value, not the double: a
  // positive double that underflows (or overflows) in float would otherwise
  // reach exactly the std::log2(0) / log2(inf) path whose cast to int is
  // undefined -- and did, returning the coarsest level for a near-zero depth,
  // the inversion of the documented shallow-to-finest behaviour.
  const float requested_cell_size = static_cast<float>(requested_cell_size_m);

  // Requested finer than float can represent (including an exactly-zero depth):
  // the request is finer than any GGGS level, so the fine clamp is the answer.
  if (!(requested_cell_size > 0.0f)) {
    return gggs::Level(policy.finest_level);
  }
  // Requested coarser than float can represent (|depth| above ~6.8e39 with the
  // default scale): coarser than any GGGS level, so the coarse clamp is the
  // answer. Both ends stay on the shoal-biased side: a depth that cannot be
  // represented never silently picks the opposite end of the ladder.
  if (!std::isfinite(requested_cell_size)) {
    return gggs::Level(policy.coarsest_level);
  }

  // fromCellSize returns the level whose cells are AT OR FINER than the request,
  // so a tile sized this way never has cells coarser than the capture radius
  // that produced them.
  const uint8_t unclamped = gggs::Level::fromCellSize(requested_cell_size).level();

  return gggs::Level(std::clamp(unclamped, policy.coarsest_level, policy.finest_level));
}

}  // namespace marine_bathymetry_store
