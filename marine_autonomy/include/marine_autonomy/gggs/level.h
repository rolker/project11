// Copyright 2016-2020 Roland Arsenault
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Roland Arsenault nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef PROJECT11_GGGS_LEVEL_H
#define PROJECT11_GGGS_LEVEL_H

#include <algorithm>
#include <stdexcept>
#include <string>
#include "cell_index.h"

namespace gggs
{

/// @brief Represents a quadtree level and provides geographic lookups.
///
/// Level is the primary entry point for converting geographic coordinates to
/// grid and cell indices. Level 0 has 8-degree grids (~928 m cells); each
/// higher level halves the angular span.
class Level
{
public:
  /// @brief Construct a Level.
  /// @param level Quadtree level (0-20).
  /// @throws std::out_of_range if level >= 21.
  Level(uint8_t level):level_(level)
  {
    if (level_ >= levels.size())
      throw std::out_of_range("GGGS level " + std::to_string(level_) +
        " exceeds maximum level " + std::to_string(levels.size() - 1));
  }

  /// @brief Find the level whose cell size is at most @p cell_size meters.
  ///
  /// The result is clamped to [0, 20].
  /// @param cell_size Desired maximum cell size in meters.
  /// @return The coarsest Level whose cells are AT OR FINER than @p cell_size,
  ///         or 20 if even the finest level's cells are coarser than the
  ///         request. (The previous wording, "smallest cells that are >=
  ///         cell_size", said the opposite of what this returns; consumers
  ///         depend on the at-or-finer direction — marine_bathymetry_store's
  ///         depth-adaptive level policy and s102_import both do.)
  static Level fromCellSize(float cell_size)
  {
    auto grid_size = cell_size * cell_rows_per_grid;
    auto raw = static_cast<int>(std::ceil(std::log2(level_0_grid_size / grid_size)));
    uint8_t clamped = static_cast<uint8_t>(std::clamp(raw, 0, static_cast<int>(levels.size() - 1)));
    return Level(clamped);
  }

  /// @brief The quadtree level number (0-20).
  constexpr uint8_t level() const noexcept
  {
    return level_;
  }

  /// Approximate cell size in meters
  double cellSize() const noexcept
  {
    return levels[level_].nominal_cell_size;
  }

  /// @brief Get the GridIndex containing the given coordinates.
  /// @param latitude Latitude in degrees.
  /// @param longitude Longitude in degrees.
  /// @return GridIndex for the grid containing (latitude, longitude).
  /// @throws std::out_of_range if latitude is more than 1e-6° outside [-90, 90].
  GridIndex gridIndex(double latitude, double longitude) const
  {
    // Wrap longitude into [-180, 180).
    longitude = normalizeLongitude(longitude);

    // Clamp latitude for small floating-point overshoot; throw for clearly invalid values.
    constexpr double lat_epsilon = 1e-6;
    if (latitude > 90.0 + lat_epsilon || latitude < -90.0 - lat_epsilon)
      throw std::out_of_range("GGGS latitude " + std::to_string(latitude) +
        " is out of range [-90, 90]");
    latitude = std::clamp(latitude, -90.0, 90.0);

    uint32_t row = (latitude + 96.0) / levels[level_].grid_angular_span;
    uint32_t column = (longitude + 180.0) / (levels[level_].grid_angular_span * latitudeScaleFactor(latitude));

    return GridIndex(level_, row, column);
  }

  /// @brief Get the GridIndex containing the given position.
  GridIndex gridIndex(const geographic_msgs::msg::GeoPoint & position) const
  {
    return gridIndex(position.latitude, position.longitude);
  }

  /// @brief Get the CellIndex for a position (grid + cell within that grid).
  CellIndex cellIndex(const geographic_msgs::msg::GeoPoint & position) const
  {
    return CellIndex(gridIndex(position), position);
  }

  /// @brief Angular span of a cell
  /// @return Degrees (in latitude direction, not accounting for polar scales)
  double cellAngularSpan() const noexcept
  {
    return levels[level_].cell_angular_span;
  }

protected:
  /// Level of the quad tree where 0 is top with 8 degree tiles
  uint8_t level_ = 0;
};

}

#endif

