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

#ifndef MARINE_BATHYMETRY_STORE__QUERY_HPP_
#define MARINE_BATHYMETRY_STORE__QUERY_HPP_

#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

#include "geographic_msgs/msg/geo_point.hpp"
#include "marine_autonomy/gggs.h"
#include "marine_bathymetry_store/bathy_cell.hpp"
#include "marine_bathymetry_store/bathymetry_store.hpp"

namespace marine_bathymetry_store
{

/// @brief A depth value resolved from the store, tagged with its winning layer.
///
/// `depth` is **ellipsoidal height** (WGS84, metres, up-positive): a seafloor
/// below the ellipsoid has a negative value, and a *shallower* seafloor has a
/// *greater* (less negative) value. See `shallowestReliable`.
struct DepthSample
{
  double depth;            ///< Ellipsoidal height (WGS84, m, up-positive).
  double uncertainty;      ///< 1-sigma vertical uncertainty (m).
  SourceLayer source;      ///< Which layer this value came from.
  /// GGGS level of the cell this value was resolved at. The store is
  /// multi-level (ADR-0002 §D2); the query resolves the best-available level
  /// per cell, so a single region scan can return samples at mixed levels.
  uint8_t level = 0;
};

/// @brief Best-available depth at @p cell across layers and levels.
///
/// Walks layers in `source_layers_by_priority` order; within a layer it resolves
/// the **finest GGGS level present** that has data at the query position (the
/// store is multi-level, ADR-0002 §D2). Returns the first (layer, level) that
/// resolves. `std::nullopt` means **unknown** — no layer has data here; a
/// safety-conscious caller must treat that as not-safe (ADR-0002 §D7), not as
/// deep water. (The per-day epoch walk of ADR-0002 §A1.3 was dropped in #221;
/// each layer is one fused surface.)
///
/// @warning **This is a point query, not a safety query.** Where a layer holds
///   data FINER than @p cell, it resolves the single native cell containing the
///   query cell's centre — one representative value, which is what a display or
///   best-available lookup wants. A caller that must not miss a shoal inside the
///   query cell (a costmap over depth-adaptive `processed` tiles, uma#369) must
///   use `shallowestReliable` or `reliableSamples` for the depth, and
///   `hasAnyData` for the "is anything surveyed here?" gate — all three read
///   **every** covered native cell (`uma-ADR-0013` D8). `bathymetry_layer` used
///   this function as that gate until uma#369; because it returns early on
///   `nullopt`, one no-data native cell under the centre suppressed the
///   region-aware depth query entirely. Do not reintroduce it on a safety path.
std::optional<DepthSample> bestSource(
  const BathymetryStore & store, const gggs::CellIndex & cell);

/// @brief Shallowest reliable depth at @p cell (navigation-safety query).
///
/// Within each layer, takes the value passing the reliability gate (uncertainty
/// ≤ @p max_uncertainty; a NaN uncertainty is never reliable). Across all levels
/// present and all layers, returns the **shallowest** — the greatest ellipsoidal
/// height, the value closest to the surface and therefore the most hazardous.
/// `std::nullopt` means no reliable data covers the cell; the caller must treat
/// that as not-safe.
///
/// **The finest data for the REGION, not a sample from its centre**
/// (`uma-ADR-0013` D8). Levels at or coarser than @p cell contain the whole
/// query cell, so one cell of theirs bears on it. A level FINER than @p cell
/// covers it with many native cells — 16 at one level finer, 256 at four, which
/// is what a level-14 depth-adaptive `processed` tile (uma#369) is under a
/// level-10 costmap query — and **every one of them is read**, with the
/// shoalest reliable value winning. Point-sampling the centre would read 1 of
/// 256 and could walk past exactly the 0.2-0.5 m rock those fine levels exist to
/// resolve. Cost is bounded by data: native cells with no tile are never
/// visited.
///
/// @note Since #221 there is one fused surface per layer, so the ADR-0002 §A1.3
///   safety walk (a noisy newest epoch falling through to a prior confident
///   epoch) is gone — if the only data over a cell is over-uncertain, the query
///   returns `nullopt` (unknown → obstacle, §D7) rather than a stale prior
///   value. This is a deliberate tradeoff for the single-survey use case (see
///   ADR-0002 Amendment A1 supersession note).
std::optional<DepthSample> shallowestReliable(
  const BathymetryStore & store, const gggs::CellIndex & cell, double max_uncertainty);

/// @brief EVERY reliable sample at @p cell across all layers and levels.
///
/// Same reliability gate as `shallowestReliable` (uncertainty ≤ @p
/// max_uncertainty; a NaN uncertainty is never reliable), but returns **all**
/// passing samples rather than collapsing to the shallowest. A cell can carry
/// more than one sample when several source layers (Processed/Draft/Reference/Chart) or
/// several GGGS levels cover it — and, since a level finer than @p cell covers it
/// with many native cells, one per **covered native cell** (`uma-ADR-0013` D8;
/// see `shallowestReliable`). A query cell over a level-14 `processed` tile can
/// therefore return up to 256 samples from that layer alone; the caller costs
/// each and takes the most hazardous, which is the point — dropping all but the
/// centre sample would hide the hazard.
///
/// @note This exists because a *shallowest-depth* pick is unsafe for cost: a
///   shallower but high-σ (untrusted) sample would mask a co-located trusted
///   sample whose worst-case clearance is keepout-grade (ADR-0010 §D7). A
///   safety-conscious caller must cost each sample and take the **most hazardous**
///   (max cost), not select one by point-estimate depth. The result is empty when
///   no reliable sample covers the cell — the caller must treat that as not-safe.
std::vector<DepthSample> reliableSamples(
  const BathymetryStore & store, const gggs::CellIndex & cell, double max_uncertainty);

/// @brief Is there ANY data under @p cell, anywhere in its region? (safety gate)
///
/// Quality-blind existence probe: `true` when some cell of some layer that bears
/// on @p cell holds data (a non-NaN depth), regardless of its uncertainty.
///
/// **Region-aware, and that is the point** (`uma-ADR-0013` D8). Where a layer
/// holds data FINER than @p cell, every covered native cell is probed — not the
/// single one under the query cell's centre. This is the query a costmap uses to
/// decide *unsurveyed* (leave NO_INFORMATION, or write LETHAL under a
/// closed-basin policy) versus *surveyed but unusable* (conservative LETHAL);
/// deciding that from the centre alone would let one no-data native cell — a
/// gated-drop hole, a between-lines gap, an absent fine tile beside a present
/// one — declare the whole query cell unsurveyed and drop a shoal sitting in any
/// of the other covered cells. `bestSource` must NOT be used for this: it is a
/// point lookup (see its `@warning`).
///
/// Cheaper than the safety queries despite covering the same ground: the walk
/// stops at the first cell holding data, so a covered query cell costs one cell,
/// and only a genuinely empty one pays the full fan-out over the tiles that
/// exist. It is deliberately kept separate from `reliableSamples` rather than
/// folded into it, because the caller needs the two answers distinguished:
/// "no data at all" and "data whose σ is unusable" get different costs (ADR-0002
/// §D7 / review M1).
bool hasAnyData(const BathymetryStore & store, const gggs::CellIndex & cell);

/// @brief Visit every GGGS cell overlapping the geographic box, with its best source.
///
/// Iterates the cells covered by the lat/lon box [@p minimum, @p maximum] at the
/// store's **default** level and invokes @p visitor with each `CellIndex` and
/// its `bestSource` result (`std::nullopt` = unknown cell). The per-cell
/// resolution is still multi-level (the returned `DepthSample.level` may differ
/// from the iteration level); only the iteration granularity is the default
/// level. Region form of `bestSource`; work is bounded by the requested box, so
/// callers control cost.
void forEachCellBestSource(
  const BathymetryStore & store,
  const geographic_msgs::msg::GeoPoint & minimum,
  const geographic_msgs::msg::GeoPoint & maximum,
  const std::function<void(const gggs::CellIndex &,
  const std::optional<DepthSample> &)> & visitor);

// NOTE (#221): `forEachChangedCell` (epoch differencing) was removed with the
// per-day epoch dimension — with one fused surface per layer it had no
// meaningful semantics. A revisit-and-compare change-detection workflow is
// deferred; see ADR-0002 Amendment A1 supersession note.

}  // namespace marine_bathymetry_store

#endif  // MARINE_BATHYMETRY_STORE__QUERY_HPP_
