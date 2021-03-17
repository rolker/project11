#!/usr/bin/env python

import math

a =  6378137.0
b =  6356752.3142
f =  0.0033528106647474805  #1.0/298.257223563
w =  7292115e-11
e2 = 0.006694380004260814   # 1.0-( 6356752.3142* 6356752.3142)/(6378137.0*6378137.0)
ep2 = ((a*a)/(b*b))-1.0
E2 = a*a-b*b

def M(latitude):
  """
  Meridional radius of curvature.
  Returns radius of curvature in north-south direction.
  latitude: Latitude in radians.
  """
  return a*(1.0-e2)/pow((1.0-e2)*pow(math.sin(latitude),2.0),3.0/2.0);

def N(latitude):
  """
  Transverse radius of curvature.
  Returns radius of curvature in east-west direction.
  latitude: Latitude in radians.
  """
  return a/math.sqrt(1-e2*pow(math.sin(latitude),2.0))

def toECEF(latitude, longitude, altitude=0.0):
  """
  Returns ECEF coordinates as x,y,z in meters.
  latitude: Latitude in radians
  longitude: Longitude in radians
  altitude: Altitude in meters, defaults to 0
  """
  n = N(latitude)
  cos_lat = math.cos(latitude)
  return (n+altitude)*cos_lat*math.cos(longitude), (n+altitude)*cos_lat*math.sin(longitude), (n*(1.0-e2)+altitude)*math.sin(latitude)

def toECEFfromDegrees(latitude, longitude, altitude=0.0):
  """
  Returns ECEF coordinates as x,y,z in meters.
  latitude: Latitude in degrees
  longitude: Longitude in degrees
  altitude: Altitude in meters, defaults to 0
  """
  return toECEF(math.radians(latitude), math.radians(longitude), altitude)

def fromECEFtoLatLong(x, y, z):
  """
  Returns Lat/Long in radsians and altitude in meters.
  x,y,z : ECEF coordinates in meters
  """
  r = math.sqrt(x*x+y*y)
  F = 54*b*b*z*z
  G = r*r +(1.0-e2)*z*z-e2*E2
  C = (e2*e2*F*r*r)/(G*G*G)
  s = pow(1.0+C+math.sqrt(C*C+2*C),1.0/3.0)
  P = F/(3.0*pow((s+(1.0/s)+1.0),2.0)*G*G)
  Q = math.sqrt(1.0+2.0*e2*e2*P)
  r0 = (-(P*e2*r)/(1.0+Q))+math.sqrt((1.0/2.0)*a*a*(1.0+1.0/Q)-((P*(1-e2)*z*z)/(Q*(1.0+Q)))-(1.0/2.0)*P*r*r)
  U = math.sqrt(pow(r-e2*r0,2.0)+z*z)
  V = math.sqrt(pow(r-e2*r0,2.0)+(1.0-e2)*z*z)
  Z0 = b*b*z/(a*V)
  return math.atan((z+ep2*Z0)/r), math.atan2(y, x), U*(1.0-(b*b)/(a*V))
 
def fromECEFtoLatLongDegrees(x, y, z):
  """
  Returns Lat/Long in degrees and altitude in meters.
  x,y,z : ECEF coordinates in meters
  """
  ret = fromECEFtoLatLong(x, y, z)
  return math.degrees(ret[0]), math.degrees(ret[1]), ret[2]


if __name__ == "__main__":
  import sys
  lat = float(sys.argv[1])
  lon = float(sys.argv[2])
  print (lat,lon)
  ecef = toECEFfromDegrees(lat,lon)
  print (ecef)
  lldeg = fromECEFtoLatLongDegrees(ecef[0],ecef[1],ecef[2])
  print (lldeg)
