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

#include "marine_sidescan_mosaic/projection.hpp"

#include <array>
#include <cmath>

#include "geodesy/ecef.h"
#include "geodesy/geodesics.h"

namespace marine_sidescan_mosaic
{

namespace
{
// 3x3 row-major matrix multiply (a · b).
using Mat3 = std::array<double, 9>;
Mat3 matmul(const Mat3 & a, const Mat3 & b)
{
  Mat3 r{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      double s = 0.0;
      for (int k = 0; k < 3; ++k) {
        s += a[i * 3 + k] * b[k * 3 + j];
      }
      r[i * 3 + j] = s;
    }
  }
  return r;
}
// Geodetic origin + the body→NED rotation for an ECEF pose. `valid` is false for a
// near-zero quaternion (caller falls back to a default orientation).
struct PoseNed
{
  double lat_deg = 0.0, lon_deg = 0.0, alt_m = 0.0;
  Mat3 r_body_ned{};
  bool valid = false;
};

PoseNed poseToNed(
  double tx, double ty, double tz,
  double qx, double qy, double qz, double qw)
{
  PoseNed out;
  // Position: ECEF → geodetic via geodesy (ADR-0002 §D8), not hand-rolled.
  const auto geo = geodesy::toMsg(geodesy::ECEFPoint(tx, ty, tz));
  out.lat_deg = geo.latitude;
  out.lon_deg = geo.longitude;
  out.alt_m = geo.altitude;

  const double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  if (norm < 1e-12) {
    return out;   // valid stays false.
  }
  qx /= norm; qy /= norm; qz /= norm; qw /= norm;

  // body → ECEF (p_parent = R p_child), matching tf2.
  const Mat3 r_body_ecef = {
    1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw),
    2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw),
    2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)};

  const double lat = out.lat_deg * M_PI / 180.0;
  const double lon = out.lon_deg * M_PI / 180.0;
  const double sla = std::sin(lat), cla = std::cos(lat);
  const double slo = std::sin(lon), clo = std::cos(lon);

  // ECEF → ENU (v_enu = R v_ecef).
  const Mat3 r_ecef_enu = {
    -slo, clo, 0.0,
    -sla * clo, -sla * slo, cla,
    cla * clo, cla * slo, sla};
  // ENU → NED.
  const Mat3 r_enu_ned = {
    0.0, 1.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 0.0, -1.0};

  out.r_body_ned = matmul(r_enu_ned, matmul(r_ecef_enu, r_body_ecef));
  out.valid = true;
  return out;
}
}  // namespace

GeoHeading ecefPoseToGeoHeading(
  double tx, double ty, double tz,
  double qx, double qy, double qz, double qw)
{
  const PoseNed p = poseToNed(tx, ty, tz, qx, qy, qz, qw);
  GeoHeading out;
  out.latitude_deg = p.lat_deg;
  out.longitude_deg = p.lon_deg;
  out.altitude_m = p.alt_m;
  // Body +X axis (column 0 of body→NED): the aerospace ZYX yaw.
  out.heading_rad = p.valid ?
    std::atan2(p.r_body_ned[1 * 3 + 0], p.r_body_ned[0 * 3 + 0]) :
    0.0;
  return out;
}

GeoBeam ecefPoseToGeoBeam(
  double tx, double ty, double tz,
  double qx, double qy, double qz, double qw)
{
  const PoseNed p = poseToNed(tx, ty, tz, qx, qy, qz, qw);
  GeoBeam out;
  out.latitude_deg = p.lat_deg;
  out.longitude_deg = p.lon_deg;
  out.altitude_m = p.alt_m;
  out.valid = p.valid;
  if (p.valid) {
    // +Z (range/beam) axis = column 2 of body→NED, as (N, E, D).
    const double zN = p.r_body_ned[0 * 3 + 2];
    const double zE = p.r_body_ned[1 * 3 + 2];
    const double zD = p.r_body_ned[2 * 3 + 2];
    // Assumes the beam +Z has a non-degenerate horizontal projection (it never
    // points straight down/up on a sidescan): for zN≈zE≈0 the azimuth atan2(0,0)
    // would be ill-defined.  Not guarded here as that geometry can't occur abeam.
    out.azimuth_rad = std::atan2(zE, zN);
    out.depression_rad = std::atan2(zD, std::hypot(zN, zE));
    // Body +X axis (column 0 of body→NED): the vessel ground-track azimuth, same
    // yaw as ecefPoseToGeoHeading — the along-track splat axis (#208).
    out.heading_rad = std::atan2(p.r_body_ned[1 * 3 + 0], p.r_body_ned[0 * 3 + 0]);
  }
  return out;
}

gggs::CellIndex projectSample(
  const geographic_msgs::msg::GeoPoint & origin,
  double azimuth_rad, double ground_range, const gggs::Level & level)
{
  const auto position = geodesy::wgs84::direct(origin, azimuth_rad, ground_range);
  // Resolve the grid from the sample's own position (no edge clamp).
  return level.cellIndex(position);
}

}  // namespace marine_sidescan_mosaic
