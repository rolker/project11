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


#ifndef CELL_GEOMETRY_HPP_
#define CELL_GEOMETRY_HPP_

#include <algorithm>
#include <cstdint>

#include "geographic_msgs/msg/geo_point.hpp"
#include "marine_autonomy/gggs.h"

/// @file
/// @brief Internal (not installed) geographic-extent helpers shared by the
///        query walk and the store's cross-layer draft clear.
///
/// Both have the same job: relate a cell or tile at one GGGS level to the cells
/// another level covers the same ground with. GGGS levels are exactly nested —
/// spans halve per level from fixed origins (-96° latitude, -180° longitude),
/// and the polar longitude scale factor changes at fixed latitudes — so a
/// coarser cell's extent is tiled exactly by finer cells, with no partial cells
/// at the seams.

namespace marine_bathymetry_store
{

/// The geographic extent of one cell or grid: SW corner (`min`) and NE corner
/// (`max`).
struct GeoBox
{
  geographic_msgs::msg::GeoPoint min;
  geographic_msgs::msg::GeoPoint max;
};

/// Geographic extent of @p cell, from its SW corner plus one cell each way.
inline GeoBox cellBox(const gggs::CellIndex & cell)
{
  const gggs::GridIndex & grid = cell.grid();
  const double lat_per_cell = grid.latitudinalSpan() / gggs::cell_rows_per_grid;
  const double lon_per_cell = grid.longitudinalSpan() / gggs::cell_columns_per_grid;
  const auto sw = cell.position();   // SW corner of the cell
  return GeoBox{sw, gggs::geoPoint(sw.latitude + lat_per_cell, sw.longitude + lon_per_cell)};
}

/// Geographic extent of a whole grid (one tile's ground).
inline GeoBox gridBox(const gggs::GridIndex & grid)
{
  return GeoBox{grid.southWestPosition(), grid.northEastPosition()};
}

/// Geographic center of @p box.
inline geographic_msgs::msg::GeoPoint boxCenter(const GeoBox & box)
{
  return gggs::geoPoint(
    0.5 * (box.min.latitude + box.max.latitude),
    0.5 * (box.min.longitude + box.max.longitude));
}

/// Geographic center of a cell (its SW corner plus half a cell each way). Used
/// to re-resolve a query position at another GGGS level (multi-level store).
inline geographic_msgs::msg::GeoPoint cellCenter(const gggs::CellIndex & cell)
{
  return boxCenter(cellBox(cell));
}

/// A quarter of a cell's angular span at @p level — the amount by which a
/// maximum corner is pulled back inside a box before handing it to the GGGS
/// area iterators.
///
/// Those iterators are **inclusive** of the cell containing the maximum corner,
/// and a box's NE corner is the SW corner of the neighbouring cell, so passing
/// the corner unmodified would visit a whole row and column of cells outside the
/// box. Use the finest level involved so the inset is always smaller than one
/// cell of every level being walked.
inline double cellInset(uint8_t level)
{
  return 0.25 * gggs::Level(level).cellAngularSpan();
}

/// @p box with its maximum corner pulled `cellInset(level)` inside — ready for
/// `gggs::GridAreaIterator` / `gggs::CellAreaIterator`, which are inclusive.
inline GeoBox insetForIteration(const GeoBox & box, uint8_t level)
{
  const double inset = cellInset(level);
  return GeoBox{
    box.min,
    gggs::geoPoint(box.max.latitude - inset, box.max.longitude - inset)};
}

/// True when @p inner lies entirely within @p outer (edges may coincide).
inline bool boxContains(const GeoBox & outer, const GeoBox & inner)
{
  return inner.min.latitude >= outer.min.latitude &&
         inner.max.latitude <= outer.max.latitude &&
         inner.min.longitude >= outer.min.longitude &&
         inner.max.longitude <= outer.max.longitude;
}

}  // namespace marine_bathymetry_store

#endif  // CELL_GEOMETRY_HPP_
