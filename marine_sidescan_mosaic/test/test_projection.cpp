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

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <set>

#include "geodesy/ecef.h"
#include "geographic_msgs/msg/geo_point.hpp"
#include "marine_autonomy/gggs.h"
#include "marine_backscatter/quality.hpp"
#include "marine_sidescan_mosaic/projection.hpp"

namespace msm = marine_sidescan_mosaic;

namespace
{
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

Mat3 transpose(const Mat3 & m)
{
  return Mat3{m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]};
}

// Quaternion (x,y,z,w) from a proper rotation matrix (row-major), p_parent = R p_child.
std::array<double, 4> quatFromMatrix(const Mat3 & m)
{
  const double t = m[0] + m[4] + m[8];
  double x, y, z, w;
  if (t > 0.0) {
    double s = 0.5 / std::sqrt(t + 1.0);
    w = 0.25 / s;
    x = (m[7] - m[5]) * s;
    y = (m[2] - m[6]) * s;
    z = (m[3] - m[1]) * s;
  } else if (m[0] > m[4] && m[0] > m[8]) {
    double s = 2.0 * std::sqrt(1.0 + m[0] - m[4] - m[8]);
    w = (m[7] - m[5]) / s;
    x = 0.25 * s;
    y = (m[1] + m[3]) / s;
    z = (m[2] + m[6]) / s;
  } else if (m[4] > m[8]) {
    double s = 2.0 * std::sqrt(1.0 + m[4] - m[0] - m[8]);
    w = (m[2] - m[6]) / s;
    x = (m[1] + m[3]) / s;
    y = 0.25 * s;
    z = (m[5] + m[7]) / s;
  } else {
    double s = 2.0 * std::sqrt(1.0 + m[8] - m[0] - m[4]);
    w = (m[3] - m[1]) / s;
    x = (m[2] + m[6]) / s;
    y = (m[5] + m[7]) / s;
    z = 0.25 * s;
  }
  return {x, y, z, w};
}

// Build the body->ECEF quaternion for a vessel at (lat,lon) with the given level
// heading (radians, CW from north), so ecefPoseToGeoHeading should recover it.
std::array<double, 4> bodyEcefQuatForHeading(double lat_deg, double lon_deg, double heading)
{
  const double lat = lat_deg * M_PI / 180.0;
  const double lon = lon_deg * M_PI / 180.0;
  const double sla = std::sin(lat), cla = std::cos(lat);
  const double slo = std::sin(lon), clo = std::cos(lon);
  const Mat3 r_ecef_enu = {
    -slo, clo, 0.0, -sla * clo, -sla * slo, cla, cla * clo, cla * slo, sla};
  const Mat3 r_enu_ned = {0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0};
  const Mat3 r_ecef_ned = matmul(r_enu_ned, r_ecef_enu);
  // body->NED: yaw `heading` about Down (body x at azimuth heading).
  const double ch = std::cos(heading), sh = std::sin(heading);
  const Mat3 r_body_ned = {ch, -sh, 0.0, sh, ch, 0.0, 0.0, 0.0, 1.0};
  const Mat3 r_body_ecef = matmul(transpose(r_ecef_ned), r_body_ned);
  return quatFromMatrix(r_body_ecef);
}

// body->ECEF quaternion for an arbitrary body->NED rotation at (lat,lon).
std::array<double, 4> bodyEcefQuatForBodyNed(
  double lat_deg, double lon_deg, const Mat3 & r_body_ned)
{
  const double lat = lat_deg * M_PI / 180.0;
  const double lon = lon_deg * M_PI / 180.0;
  const double sla = std::sin(lat), cla = std::cos(lat);
  const double slo = std::sin(lon), clo = std::cos(lon);
  const Mat3 r_ecef_enu = {
    -slo, clo, 0.0, -sla * clo, -sla * slo, cla, cla * clo, cla * slo, sla};
  const Mat3 r_enu_ned = {0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0};
  const Mat3 r_ecef_ned = matmul(r_enu_ned, r_ecef_enu);
  return quatFromMatrix(matmul(transpose(r_ecef_ned), r_body_ned));
}

// Sensor body->NED for a vessel at (yaw,pitch,roll) carrying a sidescan whose +Z
// (range/beam) axis points abeam-and-down. Built as the standard aerospace Z-Y-X
// body->NED DCM composed with the fixed mount that maps the sensor frame
// (x=forward, y=up, z=abeam) onto the ship frame (x=fwd, y=stbd, z=down). The look
// @p side selects which beam the +Z points along: starboard (+Z to ship-starboard)
// or port (+Z to ship-port, the starboard mount mirrored about the forward axis).
// At zero attitude the starboard mount reduces to (x=N, y=Up, z=E) — beam due east,
// level; port gives beam due west.
Mat3 shipSensorBodyNed(double yaw, double pitch, double roll, msm::Side side = msm::Side::Starboard)
{
  const double cps = std::cos(yaw), sps = std::sin(yaw);
  const double cth = std::cos(pitch), sth = std::sin(pitch);
  const double cph = std::cos(roll), sph = std::sin(roll);
  // Ship body->NED, standard Z-Y-X (yaw about Down, then pitch, then roll).
  const Mat3 r_ship_ned = {
    cth * cps, sph * sth * cps - cph * sps, cph * sth * cps + sph * sps,
    cth * sps, sph * sth * sps + cph * cps, cph * sth * sps - sph * cps,
    -sth, sph * cth, cph * cth};
  // Fixed mount: sensor axes expressed in the ship frame (cols: x=fwd, y=up=-down,
  // z=abeam).  Starboard points +Z to ship-starboard (the level "beam due east"
  // matrix above); port mirrors it about forward so +Z points to ship-port.
  const Mat3 r_mount_stbd{1, 0, 0, 0, 0, 1, 0, -1, 0};
  const Mat3 r_mount_port{1, 0, 0, 0, 0, -1, 0, 1, 0};
  const Mat3 r_mount = side == msm::Side::Starboard ? r_mount_stbd : r_mount_port;
  return matmul(r_ship_ned, r_mount);
}

geographic_msgs::msg::GeoPoint geoPoint(double lat, double lon, double alt = 0.0)
{
  geographic_msgs::msg::GeoPoint p;
  p.latitude = lat;
  p.longitude = lon;
  p.altitude = alt;
  return p;
}
}  // namespace

TEST(Projection, SlantGroundAzimuth)
{
  // sample_rate 10 kHz, sound speed 1500 m/s, near-field gate 100.
  EXPECT_DOUBLE_EQ(msm::slantRange(0, 100, 1500.0, 10000.0), 100.0 * 1500.0 / 20000.0);
  EXPECT_DOUBLE_EQ(msm::slantRange(900, 100, 1500.0, 10000.0), 1000.0 * 1500.0 / 20000.0);
  EXPECT_NEAR(msm::groundRange(15.0, 10.0), std::sqrt(125.0), 1e-9);
  EXPECT_DOUBLE_EQ(msm::groundRange(5.0, 10.0), 0.0);   // slant within altitude
  EXPECT_DOUBLE_EQ(msm::acrossTrackAzimuth(0.0, msm::Side::Starboard), M_PI / 2.0);
  EXPECT_DOUBLE_EQ(msm::acrossTrackAzimuth(0.0, msm::Side::Port), -M_PI / 2.0);
}

TEST(Projection, EcefPositionRoundTrip)
{
  geodesy::ECEFPoint e;
  geodesy::fromMsg(geoPoint(43.07, -71.42, 5.0), e);
  const auto gh = msm::ecefPoseToGeoHeading(e.x, e.y, e.z, 0.0, 0.0, 0.0, 1.0);
  EXPECT_NEAR(gh.latitude_deg, 43.07, 1e-6);
  EXPECT_NEAR(gh.longitude_deg, -71.42, 1e-6);
  EXPECT_NEAR(gh.altitude_m, 5.0, 1e-3);
}

TEST(Projection, HeadingRecoveredFromPose)
{
  geodesy::ECEFPoint e;
  geodesy::fromMsg(geoPoint(43.07, -71.42, 0.0), e);
  for (double h_deg : {0.0, 90.0, 200.0, 315.0}) {
    const double h = h_deg * M_PI / 180.0;
    const auto q = bodyEcefQuatForHeading(43.07, -71.42, h);
    const auto gh = msm::ecefPoseToGeoHeading(e.x, e.y, e.z, q[0], q[1], q[2], q[3]);
    // Compare on the circle (wrap-safe).
    EXPECT_NEAR(std::cos(gh.heading_rad), std::cos(h), 1e-6) << "heading " << h_deg;
    EXPECT_NEAR(std::sin(gh.heading_rad), std::sin(h), 1e-6) << "heading " << h_deg;
  }
}

TEST(Projection, BeamAzimuthDepressionFromFullAttitude)
{
  geodesy::ECEFPoint e;
  geodesy::fromMsg(geoPoint(43.07, -71.42, 0.0), e);

  // Frame +Z (range axis) points due East, horizontal: column 2 of body->NED is
  // (N,E,D)=(0,1,0).  X=North, Y=Up, Z=East.
  const Mat3 east_horizontal = {1, 0, 0, 0, 0, 1, 0, -1, 0};
  const auto q = bodyEcefQuatForBodyNed(43.07, -71.42, east_horizontal);
  const auto gb = msm::ecefPoseToGeoBeam(e.x, e.y, e.z, q[0], q[1], q[2], q[3]);
  EXPECT_NEAR(gb.latitude_deg, 43.07, 1e-6);
  EXPECT_NEAR(std::cos(gb.azimuth_rad), 0.0, 1e-6);     // azimuth = +90 deg (east)
  EXPECT_NEAR(std::sin(gb.azimuth_rad), 1.0, 1e-6);
  EXPECT_NEAR(gb.depression_rad, 0.0, 1e-6);

  // Frame +Z points East and 30 deg down: column 2 = (0, cos30, sin30).
  const double c = std::cos(M_PI / 6.0), s = std::sin(M_PI / 6.0);
  const Mat3 east_down = {1, 0, 0, 0, s, c, 0, -c, s};   // X=N, Y=(0,s,-c), Z=(0,c,s)
  const auto q2 = bodyEcefQuatForBodyNed(43.07, -71.42, east_down);
  const auto gb2 = msm::ecefPoseToGeoBeam(e.x, e.y, e.z, q2[0], q2[1], q2[2], q2[3]);
  EXPECT_NEAR(std::sin(gb2.azimuth_rad), 1.0, 1e-6);     // still east
  EXPECT_NEAR(gb2.depression_rad, M_PI / 6.0, 1e-6);     // 30 deg depression
}

// Level vessel (no roll/pitch): the full-attitude beam azimuth must match the
// yaw-only heading + 90° (starboard) and sit on the horizon (zero depression).
// Regression guard: with both paths in parallel, a level platform must agree.
TEST(Projection, BeamVsHeadingLevel)
{
  geodesy::ECEFPoint e;
  geodesy::fromMsg(geoPoint(43.07, -71.42, 0.0), e);
  for (double h_deg : {0.0, 35.0, 200.0, 315.0}) {
    const double h = h_deg * M_PI / 180.0;
    const Mat3 r = shipSensorBodyNed(h, 0.0, 0.0);   // level: no roll, no pitch
    const auto q = bodyEcefQuatForBodyNed(43.07, -71.42, r);
    const auto gh = msm::ecefPoseToGeoHeading(e.x, e.y, e.z, q[0], q[1], q[2], q[3]);
    const auto gb = msm::ecefPoseToGeoBeam(e.x, e.y, e.z, q[0], q[1], q[2], q[3]);
    ASSERT_TRUE(gb.valid) << "heading " << h_deg;
    const double expected = msm::acrossTrackAzimuth(gh.heading_rad, msm::Side::Starboard);
    EXPECT_NEAR(std::cos(gb.azimuth_rad), std::cos(expected), 1e-6) << "heading " << h_deg;
    EXPECT_NEAR(std::sin(gb.azimuth_rad), std::sin(expected), 1e-6) << "heading " << h_deg;
    EXPECT_NEAR(gb.depression_rad, 0.0, 1e-6) << "heading " << h_deg;
  }
}

// Port channel mirror of BeamVsHeadingLevel: a level vessel's port-look beam
// azimuth must equal heading − 90° (the starboard case is heading + 90°), still on
// the horizon.  Pins the per-channel +Z look-side assumption for port, which the
// starboard-only beam tests left untested.
TEST(Projection, BeamVsHeadingLevelPort)
{
  geodesy::ECEFPoint e;
  geodesy::fromMsg(geoPoint(43.07, -71.42, 0.0), e);
  for (double h_deg : {0.0, 35.0, 200.0, 315.0}) {
    const double h = h_deg * M_PI / 180.0;
    const Mat3 r = shipSensorBodyNed(h, 0.0, 0.0, msm::Side::Port);   // level, port look
    const auto q = bodyEcefQuatForBodyNed(43.07, -71.42, r);
    const auto gh = msm::ecefPoseToGeoHeading(e.x, e.y, e.z, q[0], q[1], q[2], q[3]);
    const auto gb = msm::ecefPoseToGeoBeam(e.x, e.y, e.z, q[0], q[1], q[2], q[3]);
    ASSERT_TRUE(gb.valid) << "heading " << h_deg;
    const double expected = msm::acrossTrackAzimuth(gh.heading_rad, msm::Side::Port);
    EXPECT_NEAR(std::cos(gb.azimuth_rad), std::cos(expected), 1e-6) << "heading " << h_deg;
    EXPECT_NEAR(std::sin(gb.azimuth_rad), std::sin(expected), 1e-6) << "heading " << h_deg;
    EXPECT_NEAR(gb.depression_rad, 0.0, 1e-6) << "heading " << h_deg;
  }
}

// Pure roll about the forward axis tilts the abeam +Z boresight in the vertical
// plane: it changes the depression but NOT the across-track azimuth (the
// horizontal projection of +Z stays abeam).  Pins the real geometry so a future
// change can't mislabel roll as an azimuth effect.
TEST(Projection, RollChangesDepressionNotAzimuth)
{
  geodesy::ECEFPoint e;
  geodesy::fromMsg(geoPoint(43.07, -71.42, 0.0), e);
  const double h = 35.0 * M_PI / 180.0;
  const double roll = 25.0 * M_PI / 180.0;
  const Mat3 r = shipSensorBodyNed(h, 0.0, roll);    // pure roll, level in pitch
  const auto q = bodyEcefQuatForBodyNed(43.07, -71.42, r);
  const auto gh = msm::ecefPoseToGeoHeading(e.x, e.y, e.z, q[0], q[1], q[2], q[3]);
  const auto gb = msm::ecefPoseToGeoBeam(e.x, e.y, e.z, q[0], q[1], q[2], q[3]);
  ASSERT_TRUE(gb.valid);
  // Azimuth still equals heading + 90° — roll did not move it.
  const double expected = msm::acrossTrackAzimuth(gh.heading_rad, msm::Side::Starboard);
  EXPECT_NEAR(std::cos(gb.azimuth_rad), std::cos(expected), 1e-6);
  EXPECT_NEAR(std::sin(gb.azimuth_rad), std::sin(expected), 1e-6);
  // ...but the depression now equals the applied roll angle.
  EXPECT_NEAR(gb.depression_rad, roll, 1e-6);
}

// Combined roll + pitch tilts the forward axis out of the horizontal plane, so the
// beam's +Z horizontal projection genuinely rotates away from heading ± 90°.  This
// is exactly the case the yaw-only path mis-placed and Stage 2 corrects.
TEST(Projection, BeamAzimuthDivergesUnderCombinedAttitude)
{
  geodesy::ECEFPoint e;
  geodesy::fromMsg(geoPoint(43.07, -71.42, 0.0), e);
  const double h = 35.0 * M_PI / 180.0;
  const double pitch = 18.0 * M_PI / 180.0;
  const double roll = 22.0 * M_PI / 180.0;
  const Mat3 r = shipSensorBodyNed(h, pitch, roll);
  const auto q = bodyEcefQuatForBodyNed(43.07, -71.42, r);
  const auto gh = msm::ecefPoseToGeoHeading(e.x, e.y, e.z, q[0], q[1], q[2], q[3]);
  const auto gb = msm::ecefPoseToGeoBeam(e.x, e.y, e.z, q[0], q[1], q[2], q[3]);
  ASSERT_TRUE(gb.valid);
  // The yaw-only path would place the beam at heading + 90°; full attitude moves it.
  const double yaw_only = msm::acrossTrackAzimuth(gh.heading_rad, msm::Side::Starboard);
  const double diff = std::atan2(
    std::sin(gb.azimuth_rad - yaw_only), std::cos(gb.azimuth_rad - yaw_only));
  EXPECT_GT(std::abs(diff), 2.0 * M_PI / 180.0);   // diverges measurably (> 2°)
  EXPECT_GT(gb.depression_rad, 0.0);               // and the beam tilts below horizon
}

TEST(Projection, ProjectsEastward)
{
  const gggs::Level level(13);
  const auto origin = geoPoint(43.07, -71.42, 0.0);
  const auto cell = msm::projectSample(origin, M_PI / 2.0, 20.0, level);   // 20 m due east
  const auto pos = cell.position();
  EXPECT_GT(pos.longitude, origin.longitude);            // moved east
  EXPECT_NEAR(pos.latitude, origin.latitude, 1e-4);      // ~same latitude
}

TEST(Projection, SampleCrossesGridBoundary)
{
  // Place the origin just west of a grid's eastern edge, then project east past
  // it: the sample must land in the NEXT grid, not be clamped to this grid's edge.
  const gggs::Level level(13);
  const gggs::GridIndex grid = level.gridIndex(43.07, -71.42);
  const double east_edge = grid.westLongitude() + grid.longitudinalSpan();
  // ~0.5 m west of the edge (longitude degrees at ~43 N).
  const double half_m_deg = 0.5 / (111320.0 * std::cos(43.07 * M_PI / 180.0));
  const auto origin = geoPoint(43.07, east_edge - half_m_deg, 0.0);

  const gggs::GridIndex origin_grid = level.gridIndex(origin.latitude, origin.longitude);
  const auto cell = msm::projectSample(origin, M_PI / 2.0, 5.0, level);    // 5 m east
  EXPECT_NE(cell.grid(), origin_grid);
  EXPECT_EQ(cell.grid().column(), origin_grid.column() + 1);
}

// GeoBeam.heading_rad (#208 along-track splat axis) is the body +X (ground-track)
// yaw — the same value ecefPoseToGeoHeading returns, even under full attitude.
TEST(Projection, BeamHeadingMatchesGroundTrack)
{
  geodesy::ECEFPoint e;
  geodesy::fromMsg(geoPoint(43.07, -71.42, 0.0), e);
  for (double h_deg : {0.0, 35.0, 200.0, 315.0}) {
    const double h = h_deg * M_PI / 180.0;
    const Mat3 r = shipSensorBodyNed(h, 12.0 * M_PI / 180.0, 20.0 * M_PI / 180.0);
    const auto q = bodyEcefQuatForBodyNed(43.07, -71.42, r);
    const auto gh = msm::ecefPoseToGeoHeading(e.x, e.y, e.z, q[0], q[1], q[2], q[3]);
    const auto gb = msm::ecefPoseToGeoBeam(e.x, e.y, e.z, q[0], q[1], q[2], q[3]);
    ASSERT_TRUE(gb.valid) << "heading " << h_deg;
    EXPECT_NEAR(std::cos(gb.heading_rad), std::cos(gh.heading_rad), 1e-9) << "heading " << h_deg;
    EXPECT_NEAR(std::sin(gb.heading_rad), std::sin(gh.heading_rad), 1e-9) << "heading " << h_deg;
  }
}

TEST(Projection, FootprintAlongTrack)
{
  EXPECT_NEAR(msm::footprintAlongTrack(30.0, 0.00768), 0.2304, 1e-9);   // 30 m @ 0.44°
  EXPECT_DOUBLE_EQ(msm::footprintAlongTrack(0.0, 0.00768), 0.0);        // zero range
  EXPECT_DOUBLE_EQ(msm::footprintAlongTrack(30.0, 0.0), 0.0);           // zero beamwidth
}

// A footprint of a few cells must paint several distinct cells stepped along the
// ground track; a zero footprint collapses to the single point-deposit cell.
TEST(Projection, SplatCoverageSpansFootprint)
{
  const gggs::Level level(13);                 // ~0.11 m cells
  const auto origin = geoPoint(43.07, -71.42, 0.0);
  const double heading = 0.0;                  // ground track due north
  const double azimuth = M_PI / 2.0;           // beam due east
  const double ground = 20.0;

  auto collect = [&](double footprint_m) {
      std::set<gggs::CellIndex> cells;
      msm::splatAlongTrack(
        origin, heading, footprint_m, azimuth, ground, level,
        [&cells](const gggs::CellIndex & c) {cells.insert(c);});
      return cells;
    };

  // Zero footprint → exactly one cell, identical to the legacy point-deposit.
  const auto none = collect(0.0);
  EXPECT_EQ(none.size(), 1u);
  EXPECT_EQ(*none.begin(), msm::projectSample(origin, azimuth, ground, level));

  // Sub-cell footprint still collapses to one cell (n_steps = 1).
  EXPECT_EQ(collect(0.05).size(), 1u);

  // A ~0.3 m footprint (≈ 3 cells) paints several distinct cells.
  const auto wide = collect(0.3);
  EXPECT_GE(wide.size(), 2u);

  // An even step count is symmetric — it straddles the ping rather than
  // under-covering to one side. Use a *larger* even n_steps (8, via a 7.5-cell
  // footprint → ceil = 8) so the centroid check has teeth: a one-sided splat biases
  // the centroid by (n_steps-1)/2 = 3.5 cells, far outside the half-cell tolerance
  // below, whereas the n_steps=2 case could drift only ±0.5 cell and slip a full-cell
  // tolerance. For a north-bound track the spread is in latitude, so the mean latitude
  // of the splat cells must sit within half a cell of the origin.
  const double cell_m = level.cellSize();
  const auto even = collect(7.5 * cell_m);   // 7.5 cells → ceil = 8 steps (even)
  EXPECT_GE(even.size(), 2u);
  const double cell_deg = cell_m / 111320.0;   // ~m per deg latitude
  double lat_sum = 0.0;
  for (const auto & c : even) {
    lat_sum += c.position().latitude;
  }
  EXPECT_NEAR(lat_sum / static_cast<double>(even.size()), origin.latitude, 0.5 * cell_deg);
}

// The cells of a north-bound ground track must differ in latitude (the splat steps
// along heading, not across the beam) — pins the along-track orientation.
TEST(Projection, SplatStepsAlongHeading)
{
  const gggs::Level level(13);
  const auto origin = geoPoint(43.07, -71.42, 0.0);
  std::set<double> lats;
  std::set<double> lons;
  msm::splatAlongTrack(
    origin, 0.0 /*north*/, 0.5 /*m*/, M_PI / 2.0 /*east beam*/, 20.0, level,
    [&lats, &lons](const gggs::CellIndex & c) {
      lats.insert(c.position().latitude);
      lons.insert(c.position().longitude);
    });
  EXPECT_GE(lats.size(), 2u);   // stepped in latitude (north-south), as expected
  // ...and held ~constant in longitude: the splat steps run along heading (north),
  // not across the beam, so every deposit shares the same across-track (east)
  // placement. Bound the longitude spread to a cell or two — far tighter than the
  // multi-cell latitude spread above — so an accidental across-track step regresses.
  const double cell_deg = level.cellSize() / 111320.0;   // ~m per deg latitude
  const double lon_span = *lons.rbegin() - *lons.begin();
  EXPECT_LT(lon_span, 2.0 * cell_deg);
}

// ---------------------------------------------------------------------------
// DEM orthorectification (#297): correctedGroundRange.
//
// Every assertion is tolerance-based — the routine is an iterative
// floating-point computation, so bit-identity is not a property it has.
// ---------------------------------------------------------------------------

namespace
{
constexpr double kDemLat = 43.07, kDemLon = -71.42;

// A synthetic bottom that is a constant-gradient plane along the beam azimuth:
// ellipsoidal height `z0 + dz_dr * r`, where r is the ground distance from
// @p origin. Returned as the DepthLookup callback signature.
class SlopeBottom
{
public:
  SlopeBottom(const geographic_msgs::msg::GeoPoint & origin, double z0, double dz_dr)
  : origin_(origin), z0_(z0), dz_dr_(dz_dr) {}

  std::optional<double> operator()(double lat, double lon) const
  {
    geographic_msgs::msg::GeoPoint p;
    p.latitude = lat;
    p.longitude = lon;
    p.altitude = 0.0;
    const double r = geodesy::wgs84::inverse(origin_, p).distance;
    return z0_ + dz_dr_ * r;
  }

  // Ground range where the ray of slant range @p slant meets this plane, from
  // (1 + m²)r² − 2·v0·m·r + (v0² − R²) = 0 with v0 = sensor_height − z0,
  // m = dz_dr. The + root is the one the flat-seeded iteration converges to.
  double analyticRange(double slant, double sensor_height) const
  {
    const double v0 = sensor_height - z0_;
    const double m = dz_dr_;
    const double a = 1.0 + m * m;
    const double disc = v0 * v0 * m * m - a * (v0 * v0 - slant * slant);
    return (v0 * m + std::sqrt(disc)) / a;
  }

private:
  geographic_msgs::msg::GeoPoint origin_;
  double z0_, dz_dr_;
};
}  // namespace

// A constant-depth DEM must reproduce the flat formula exactly: the correction
// is a generalization of groundRange, not a different geometry.
TEST(Projection, CorrectedGroundRangeFlatBottom)
{
  const auto origin = geoPoint(kDemLat, kDemLon);
  const double slant = 50.0, sensor_height = 0.0, vertical = 10.0;
  const double seed = msm::groundRange(slant, 12.0);   // deliberately a wrong seed
  const auto result = msm::correctedGroundRange(
    slant, sensor_height, origin, M_PI / 2.0, seed,
    [vertical, sensor_height](double, double) {
      return std::optional<double>(sensor_height - vertical);
    });
  EXPECT_EQ(result.status, msm::DemCorrection::Status::kConverged);
  EXPECT_NEAR(result.ground_range, msm::groundRange(slant, vertical), 1e-9);
  EXPECT_NEAR(result.vertical_offset, vertical, 1e-9);
}

// On a constant-gradient bottom the iteration must reach the analytically known
// intersection — this is the mis-placement issue #297 exists to fix.
TEST(Projection, CorrectedGroundRangeSlope)
{
  const auto origin = geoPoint(kDemLat, kDemLon);
  const double slant = 50.0, sensor_height = 0.0;
  const SlopeBottom bottom(origin, -10.0, -0.2);   // deepens 0.2 m per metre of range
  const double seed = msm::groundRange(slant, 10.0);
  const auto result = msm::correctedGroundRange(
    slant, sensor_height, origin, M_PI / 2.0, seed, bottom);
  EXPECT_EQ(result.status, msm::DemCorrection::Status::kConverged);
  EXPECT_NEAR(result.ground_range, bottom.analyticRange(slant, sensor_height), 1e-3);
  // A deepening bottom pulls the sample IN from the flat placement; the flat
  // range is wrong by metres, which is tens of L13 cells.
  EXPECT_LT(result.ground_range, seed - 2.0);
  // The returned pair is consistent: r² + v² = R².
  EXPECT_NEAR(
    std::hypot(result.ground_range, result.vertical_offset), slant, 1e-6);
}

// No DEM data anywhere ⇒ the flat seed is returned untouched. This is the path
// every sample takes when --bathy-store is omitted, so it must be exact.
TEST(Projection, CorrectedGroundRangeNoDemFallsBackToFlat)
{
  const auto origin = geoPoint(kDemLat, kDemLon);
  const double seed = msm::groundRange(50.0, 10.0);
  const auto result = msm::correctedGroundRange(
    50.0, 0.0, origin, M_PI / 2.0, seed,
    [](double, double) {return std::optional<double>();});
  EXPECT_EQ(result.status, msm::DemCorrection::Status::kNoCoverage);
  EXPECT_DOUBLE_EQ(result.ground_range, seed);
  EXPECT_TRUE(std::isnan(result.vertical_offset));
}

// A DEM cell above the sensor (bad cell or bad pose) must not drive the iterate
// to zero and oscillate about the sensor's own nadir point.
TEST(Projection, CorrectedGroundRangeDegenerateBottomAboveSensor)
{
  const auto origin = geoPoint(kDemLat, kDemLon);
  const double seed = msm::groundRange(50.0, 10.0);
  const auto result = msm::correctedGroundRange(
    50.0, 0.0, origin, M_PI / 2.0, seed,
    [](double, double) {return std::optional<double>(5.0);});   // bottom ABOVE the sensor
  EXPECT_EQ(result.status, msm::DemCorrection::Status::kDegenerate);
  EXPECT_DOUBLE_EQ(result.ground_range, seed);
}

// A depth that puts the sample inside the nadir cone is the other degenerate
// branch — groundRange would clamp to 0 and the next probe would re-read the
// sensor's own nadir cell.
TEST(Projection, CorrectedGroundRangeDegenerateInsideNadirCone)
{
  const auto origin = geoPoint(kDemLat, kDemLon);
  const double seed = msm::groundRange(50.0, 10.0);
  const auto result = msm::correctedGroundRange(
    50.0, 0.0, origin, M_PI / 2.0, seed,
    [](double, double) {return std::optional<double>(-60.0);});   // v = 60 >= slant 50
  EXPECT_EQ(result.status, msm::DemCorrection::Status::kDegenerate);
  EXPECT_DOUBLE_EQ(result.ground_range, seed);
}

// A pathological DEM the iteration cannot settle on must terminate at the cap
// and return the FLAT value — a non-converged iterate's error is unbounded and
// is never emitted.
TEST(Projection, CorrectedGroundRangeCapsIterations)
{
  const auto origin = geoPoint(kDemLat, kDemLon);
  const double seed = msm::groundRange(50.0, 10.0);
  int calls = 0;
  const auto result = msm::correctedGroundRange(
    50.0, 0.0, origin, M_PI / 2.0, seed,
    [&calls](double, double) {
      ++calls;
      // Neither depth agrees with the seed's, so the very first step already
      // moves — and the two alternate far apart forever.
      return std::optional<double>((calls % 2) ? -20.0 : -45.0);
    });
  EXPECT_EQ(result.status, msm::DemCorrection::Status::kNotConverged);
  EXPECT_DOUBLE_EQ(result.ground_range, seed);
  EXPECT_TRUE(std::isnan(result.vertical_offset));
  EXPECT_EQ(calls, msm::kMaxDemIterations);   // bounded cost, no hang
}

// The correction's (vertical_offset, ground_range) pair is what feeds the
// best-source quality score. On a slope it must beat the flat pair — this is
// the wiring the tool relies on, with no marine_backscatter API change.
TEST(Projection, CorrectedGroundRangeGrazingPairFeedsQuality)
{
  const auto origin = geoPoint(kDemLat, kDemLon);
  const double slant = 50.0, sensor_height = 0.0, nadir_altitude = 10.0;
  const SlopeBottom bottom(origin, -10.0, -0.2);
  const double flat_ground = msm::groundRange(slant, nadir_altitude);
  const auto result = msm::correctedGroundRange(
    slant, sensor_height, origin, M_PI / 2.0, flat_ground, bottom);
  ASSERT_EQ(result.status, msm::DemCorrection::Status::kConverged);

  const std::uint16_t flat_quality =
    marine_backscatter::grazingQuality(nadir_altitude, flat_ground);
  const std::uint16_t dem_quality =
    marine_backscatter::grazingQuality(result.vertical_offset, result.ground_range);
  EXPECT_GT(dem_quality, flat_quality);
}
