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

#ifndef MARINE_BATHYMETRY_STORE__DEPTH_ADAPTIVE_LEVEL_HPP_
#define MARINE_BATHYMETRY_STORE__DEPTH_ADAPTIVE_LEVEL_HPP_

#include <cstdint>

#include "marine_autonomy/gggs.h"

/// @file
/// @brief Depth-driven GGGS level selection for the `processed` survey layer
///        (`uma-ADR-0010` D9, amended by
///        [uma#369](https://github.com/rolker/unh_marine_autonomy/issues/369)).
///
/// CUBE's capture radius is depth-adaptive — `node.cpp:154` accepts a sounding
/// into a node only within `max(capture_distance_scale * |depth|, 0.5 m)` — but
/// the lattice it is written onto has been one fixed GGGS level for the whole
/// survey (level 10, ~0.906 m cells, across a 1-15 m+ depth range). This is the
/// policy that makes the lattice follow the same driver.
///
/// **Requested cell size is `capture_distance_scale * |depth|`, with no floor.**
/// CUBE's own 0.5 m term is a minimum *acceptance distance*, not a statement
/// about achievable resolution; inheriting it here as a *resolution* floor
/// collapses the policy to a single level (11) everywhere shallower than ~18 m,
/// i.e. across the entire surveyed range — a new fixed level rather than an
/// adaptive one.
///
/// **Level ladder** under the default policy (boundaries are exactly
/// `gggs::Level(L).cellSize() / capture_distance_scale`):
///
/// | Level | Cell size | Tile extent | Applies when |
/// |---|---|---|---|
/// | 8 (coarse clamp) | 3.624 m | 3478.7 m | depth >= 72.47 m |
/// | 9 | 1.812 m | 1739.4 m | 36.24 m <= depth < 72.47 m |
/// | 10 | 0.906 m | 869.7 m | 18.12 m <= depth < 36.24 m |
/// | 11 | 0.453 m | 434.8 m | 9.06 m <= depth < 18.12 m |
/// | 12 | 0.227 m | 217.4 m | 4.53 m <= depth < 9.06 m |
/// | 13 | 0.113 m | 108.7 m | 2.26 m <= depth < 4.53 m |
/// | 14 (fine clamp) | 0.057 m | 54.4 m | depth < 2.26 m |
///
/// Level 10 is what every `processed` tile is written at today, so relative to
/// today each step finer is a **4x** cell- and tile-count increase over the same
/// ground: 4x at level 11, 16x at 12, 64x at 13 and **256x** at the level-14
/// clamp. Tiles are dense 960x960 rasters whether or not the survey fills them,
/// so shallow water also pays a partial-tile overhead (a level-14 tile spans
/// 54.4 m). Deeper than ~36 m the ladder returns levels *coarser* than today's
/// 10; that is deeper than the operator platforms currently survey.
///
/// **Decision unit: one level per store tile, sized from the SHALLOWEST depth in
/// that tile.** The shallowest sounding has the tightest capture radius, so
/// sizing the lattice from it is what keeps every sounding in the unit within
/// its own capture distance of a node. The scalar signature encodes exactly that
/// ("one depth decides one level") and may change — to a depth range, say —
/// when the writer lands.
///
/// **Nothing calls this yet.** The writer is
/// [cube_bathymetry#143](https://github.com/rolker/cube_bathymetry/issues/143):
/// `import_bag_main.cpp:1030` builds one `cube::GeoMapSheet` for an entire run
/// and `:1130-1133` pins the store cell size to that sheet, so CUBE's estimation
/// grid and the store tiling are the same resolution by construction. Varying
/// the store level with depth is a re-architecture of that pipeline, not a call
/// site.

namespace marine_bathymetry_store
{

/// @brief Tunables for `depthAdaptiveLevel`.
///
/// The defaults are the decided policy (uma#369); the struct exists so tests and
/// future callers can vary it without editing the function.
struct DepthAdaptiveLevelPolicy
{
  /// @brief Requested cell size per metre of depth.
  ///
  /// Mirrors `cube_bathymetry::Parameters::capture_distance_scale`
  /// (`cube_bathymetry/include/cube_bathymetry/parameters.h:205`, 0.05).
  /// **There is no automated link between the two repos' constants** — a change
  /// to CUBE's value must be reviewed against this default, and vice versa.
  double capture_distance_scale = 0.05;

  /// @brief Finest level the policy may return (~0.057 m cells).
  ///
  /// Operator-pinned. Without it, shallow water requests unbounded resolution:
  /// the ladder has no natural fine end once the 0.5 m acceptance floor is not
  /// treated as a resolution floor.
  uint8_t finest_level = 14;

  /// @brief Coarsest level the policy may return (~3.624 m cells).
  ///
  /// Binds only below ~72 m depth. Note this is **not** a "never coarser than
  /// today" guarantee: the ladder already returns level 9 (coarser than today's
  /// 10) between ~36 m and ~72 m. It bounds the coarse end; it does not pin it
  /// to today's resolution.
  uint8_t coarsest_level = 8;
};

/// @brief Choose the GGGS level for a store tile from the depth it covers.
///
/// Computes `capture_distance_scale * |depth_m|` and maps it through
/// `gggs::Level::fromCellSize` (which returns the level at or finer than the
/// requested cell size), then clamps into
/// `[policy.coarsest_level, policy.finest_level]`.
///
/// @param depth_m Depth in metres for the decision unit — by contract the
///        **shallowest** depth in the tile. Sign is ignored (magnitude is used),
///        since the sign convention differs between the CUBE node and store
///        consumers. A depth of exactly 0 (or any depth whose requested cell
///        size is non-positive) returns `finest_level` rather than reaching
///        `fromCellSize`, whose `log2` path is undefined there.
/// @param policy Tunables; defaults to the decided uma#369 policy.
/// @return The GGGS level to write the tile at.
/// @throws std::invalid_argument if @p depth_m is not finite (a silently clamped
///         NaN would write an entire survey at the fine clamp), or if the policy
///         is inverted (`finest_level < coarsest_level`), if either clamp is
///         outside the GGGS level range, or if `capture_distance_scale` is not
///         finite and positive.
gggs::Level depthAdaptiveLevel(
  double depth_m, const DepthAdaptiveLevelPolicy & policy = DepthAdaptiveLevelPolicy());

}  // namespace marine_bathymetry_store

#endif  // MARINE_BATHYMETRY_STORE__DEPTH_ADAPTIVE_LEVEL_HPP_
