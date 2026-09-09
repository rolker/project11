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
///
/// One qualification, handled in `cellInset` rather than assumed away: grid
/// latitudes are **clamped to ±90**, so a row the clamp truncates would hold
/// cells shorter than the level's nominal span. That does not occur for any
/// level ≥ 2 (90° falls on a row boundary), and so not for the [8, 14] ladder
/// these helpers serve — but the inset is derived from the grid's actual span,
/// not the level's nominal one, so the claim holds by construction.

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

/// A quarter of a cell's angular span at @p level, measured **at @p near** — the
/// amount by which a box's corners are pulled inward before it is handed to the
/// GGGS area iterators.
///
/// Use the finest level involved, so the inset is always well under one cell of
/// every level being walked.
///
/// Measured at a position rather than taken from the level's nominal span,
/// because the two can disagree. `cellBox` derives a cell's height from
/// `grid.latitudinalSpan()`, and `GridIndex::northLatitude`/`southLatitude`
/// **clamp to ±90**; a grid row that the clamp truncates therefore holds cells
/// shorter than the level's nominal `cellAngularSpan()`, and a nominal
/// quarter-cell inset could then exceed a quarter — in the limit, a whole one —
/// of the real cells there, dropping a cell the box genuinely covers. Reading
/// the span from the grid that actually contains @p near keeps the inset a
/// quarter of the cells being walked wherever they are, and the `std::min`
/// against the nominal span keeps it conservative if @p near falls outside the
/// valid grid range.
///
/// (This is defensive, not reachable today: 90° falls on a row boundary at every
/// level ≥ 2, so no row of the [8, 14] ladder is partially clamped. It is
/// written this way so the file-header claim below — "no partial cells at the
/// seams" — is true of the code rather than of an assumption about the levels
/// callers happen to use. Longitude needs no equivalent: the polar scale factor
/// makes cells *wider*, so a latitude-sized inset is a smaller fraction of a
/// longitude cell, never a larger one.)
inline double cellInset(uint8_t level, const geographic_msgs::msg::GeoPoint & near)
{
  const gggs::Level lvl(level);
  const double nominal = lvl.cellAngularSpan();
  const gggs::GridIndex grid = lvl.gridIndex(near);
  if (!grid.valid()) {
    return 0.25 * nominal;
  }
  const double actual = grid.latitudinalSpan() / gggs::cell_rows_per_grid;
  return 0.25 * std::min(nominal, actual);
}

/// @p box shrunk by `cellInset(level)` on **every** side — ready for
/// `gggs::GridAreaIterator` / `gggs::CellAreaIterator`.
///
/// Both corners move, for two different reasons:
/// - The **maximum** corner, because those iterators are *inclusive* of the cell
///   containing it and a box's NE corner is the SW corner of its neighbour;
///   passing it unmodified visits a whole row and column of cells outside the
///   box.
/// - The **minimum** corner, because a box corner that coincides exactly with a
///   cell boundary (which it does whenever the box is itself a cell or a grid at
///   another level — GGGS nests exactly) can land an ulp on the wrong side once
///   the two levels' spans are computed by different routes, starting the walk
///   one cell short and silently reading no-data outside the box.
///
/// Shrinking never drops a cell the box fully covers: such a cell spans at least
/// one whole cell of @p level, four times the inset. A cell the box merely clips
/// by less than a quarter of a cell of @p level can be dropped — an overlap
/// finer than the finest level being reasoned about.
inline GeoBox insetForIteration(const GeoBox & box, uint8_t level)
{
  // Measure at BOTH corners and take the smaller: a box can span a clamped row
  // boundary, and the inset must be under a quarter of the shortest cell it
  // touches.
  const double inset =
    std::min(cellInset(level, box.min), cellInset(level, box.max));
  return GeoBox{
    gggs::geoPoint(box.min.latitude + inset, box.min.longitude + inset),
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
