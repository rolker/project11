#ifndef P11_UTILS_H
#define P11_UTILS_H

#include <tf2/utils.h>
#include "gz4d_geo.h"

namespace project11
{
  template <typename A> double quaternionToHeadingDegrees(const A& a)
  {
    return 90.0-180.0*tf2::getYaw(a)/M_PI;
  }

  template <typename A> double quaternionToHeadingRadians(const A& a)
  {
    return (M_PI/2.0)-tf2::getYaw(a);
  }

  inline double speedOverGround(const geometry_msgs::Vector3 & v)
  {
    tf2::Vector3 v3;
    tf2::fromMsg(v, v3);
    return v3.length();
  }
  
  typedef gz4d::GeoPointLatLongDegrees LatLongDegrees;
  typedef gz4d::GeoPointECEF ECEF;
  typedef gz4d::Point<double> Point;
  typedef gz4d::LocalENU ENUFrame;
  
  // angle types that do not wrap
  typedef gz4d::Angle<double, gz4d::pu::Degree, gz4d::rt::Unclamped> AngleDegrees;
  typedef gz4d::Angle<double, gz4d::pu::Radian, gz4d::rt::Unclamped> AngleRadians;
  
  // angle types that wrap at +/- half a circle
  typedef gz4d::Angle<double, gz4d::pu::Degree, gz4d::rt::ZeroCenteredPeriod> AngleDegreesZeroCentered;
  typedef gz4d::Angle<double, gz4d::pu::Radian, gz4d::rt::ZeroCenteredPeriod> AngleRadiansZeroCentered;
  
  // angle types that wrap at 0 and full circle
  typedef gz4d::Angle<double, gz4d::pu::Degree, gz4d::rt::PositivePeriod> AngleDegreesPositive;
  typedef gz4d::Angle<double, gz4d::pu::Radian, gz4d::rt::PositivePeriod> AngleRadiansPositive;
  
  typedef gz4d::geo::WGS84::Ellipsoid WGS84;
  
  template <typename A> void fromMsg(const A& a, LatLongDegrees &b)
  {
    b.latitude() = a.latitude;
    b.longitude() = a.longitude;
    b.altitude() = a.altitude;
  }

  template <typename B> void toMsg(const LatLongDegrees &a, B &b)
  {
    b.latitude = a.latitude();
    b.longitude = a.longitude();
    b.altitude = a.altitude();
  }

  template <typename A> void fromMsg(const A& a, Point &b)
  {
    b = Point(a.x, a.y, a.z);
  }

  template <typename B> void toMsg(const Point& a, B &b)
  {
    b.x = a[0];
    b.y = a[1];
    b.z = a[2];
  }
}

std::ostream& operator<< (std::ostream &out, const project11::LatLongDegrees &p);
std::ostream& operator<< (std::ostream &out, const project11::ECEF &p);
std::ostream& operator<< (std::ostream &out, const project11::AngleDegrees &p);
std::ostream& operator<< (std::ostream &out, const project11::AngleRadians &p);

#endif
