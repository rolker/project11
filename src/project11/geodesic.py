#!/usr/bin/env python3

__author__    = 'Roland Arsenault'

__doc__ ='''
Implements Vincenty's formula to calculate the forward (position1+azimuth and range to position2)
and inverse (position1, position2 to azimuth and range) solutions of geodesics on the WGS84 ellipsoid.

Vincenty, T. (April 1975a)."DIRECT AND INVERSE SOLUTIONS OF GEODESICS 0N THE ELLIPSOID WITH APPLICATION
OF NESTED EQUATIONS"
http://www.ngs.noaa.gov/PUBS_LIB/inverse.pdf
'''

import math

def direct(lon1,lat1,azimuth,distance):
    '''Calculates the postion P2 from azimuth and distance from P1 on the WGS84 ellipsoid.
    @param lon1,lat1: Position P1 in rads
    @param azimuth: Clockwise angle in rads relative to true north.
    @param distance: Distance in meters
    @return: Position P2 in rads (lon2,lat2)
    '''
    
    phi1 = lat1
    alpha1 = azimuth
    
    a = 6378137.0        # length of major axis of the ellipsoid (radius at equator in meters)
    b = 6356752.314245   # length of minor axis of the ellipsoid (radius at the poles)
    f = 1/298.257223563  # flattening of the ellipsoid

    epsilon = 1e-12

    #U is 'reduced latitude'
    tanU1 = (1.0-f)*math.tan(phi1)
    cosU1 = 1/math.sqrt(1+tanU1*tanU1)
    sinU1 = tanU1*cosU1

    cosAlpha1 = math.cos(alpha1)
    sinAlpha1 = math.sin(alpha1)

    sigma1 = math.atan2(tanU1, cosAlpha1) # angular distance on sphere from equator to P1 along geodesic
    sinAlpha = cosU1*sinAlpha1
    cos2Alpha = 1.0-sinAlpha*sinAlpha

    u2 = cos2Alpha*(a*a-b*b)/(b*b)

    k1 = (math.sqrt(1.0+u2)-1.0)/(math.sqrt(1.0+u2)+1.0)
    A = (1.0+k1*k1/4.0)/(1.0-k1)
    B = k1*(1.0-3.0*k1*k1/8.0)

    sigma = distance/(b*A)
    
    while True:
        cos2Sigmam = math.cos(2.0*sigma1+sigma)
        sinSigma = math.sin(sigma)
        cosSigma = math.cos(sigma)
        
        deltaSigma = B*sinSigma*(cos2Sigmam+.25*B*(cosSigma*(-1.0+2.0*cos2Sigmam*cos2Sigmam)-(B/6.0)*cos2Sigmam*(-3.0+4.0*sinSigma*sinSigma)*(-3.0+4.0*cos2Sigmam*cos2Sigmam)))
        last_sigma = sigma
        sigma = (distance/(b*A))+deltaSigma
        if abs(last_sigma-sigma) <= epsilon:
            break

    cos2Sigmam = math.cos(2.0*sigma1+sigma)
            
    phi2 = math.atan2(sinU1*math.cos(sigma)+cosU1*math.sin(sigma)*cosAlpha1,(1-f)*math.sqrt(sinAlpha*sinAlpha+pow(sinU1*math.sin(sigma)-cosU1*math.cos(sigma)*cosAlpha1,2)))
    l = math.atan2(math.sin(sigma)*sinAlpha1,cosU1*math.cos(sigma)-sinU1*math.sin(sigma)*cosAlpha1)
    C = (f/16.0)*cos2Alpha*(4.0+f*(4.0-3.0*cos2Alpha))
    L = l-(1.0-C)*f*sinAlpha*(sigma+C*math.sin(sigma)*(cos2Sigmam+C*math.cos(sigma)*(-1+2.0*cos2Sigmam*cos2Sigmam)))

    lat2 = phi2
    lon2 = lon1 + L
    return lon2,lat2

def inverse(lon1,lat1,lon2,lat2):
    '''Calculates the azimuth and distance from P1 to P2 on the WGS84 ellipsoid.
    @param lon1,lat1: Position P1 in rads
    @param lon2,lat2: Position P2 in rads
    @return: azimuth in rads, distance in meters
    '''

    if(lon1 == lon2 and lat1 == lat2): #short circuit if same points, avoids div by zero later
        return 0.0,0.0
    
    
    a = 6378137.0        # length of major axis of the ellipsoid (radius at equator in meters)
    b = 6356752.314245   # length of minor axis of the ellipsoid (radius at the poles)
    f = 1/298.257223563  # flattening of the ellipsoid
    
    epsilon = 1e-12
    
    phi1 = lat1
    phi2 = lat2
    
    L = lon2-lon1

    U1 = math.atan((1.0-f)*math.tan(phi1))
    U2 = math.atan((1.0-f)*math.tan(phi2))
    cosU1 = math.cos(U1)
    cosU2 = math.cos(U2)
    sinU1 = math.sin(U1)
    sinU2 = math.sin(U2)
    
    l = L
    last_l = None

    while True:
        cosl = math.cos(l)
        sinl = math.sin(l)
    
        sinSigma = math.sqrt(((cosU2*sinl)**2)+(cosU1*sinU2-sinU1*cosU2*cosl)**2)
        cosSigma = sinU1*sinU2+cosU1*cosU2*cosl
        sigma = math.atan2(sinSigma,cosSigma)
        try:
            sinAlpha = (cosU1*cosU2*sinl)/sinSigma
        except Exception as e:
            return 0,0

        cos2Alpha = 1-sinAlpha*sinAlpha
        if cos2Alpha == 0:
            cos2Sigmam = 0
        else:
            cos2Sigmam = cosSigma-((2.0*sinU1*sinU2)/cos2Alpha)

        if last_l is not None and abs(last_l - l) <= epsilon:
            break
        last_l = l
            
        C = (f/16.0)*cos2Alpha*(4.0+f*(4.0-3.0*cos2Alpha))
        l = L+(1.0-C)*f*sinAlpha*(sigma+C*sinSigma*(cos2Sigmam+C*cosSigma*(-1.0+2.0*cos2Sigmam**2)))

    u2 = cos2Alpha*(a*a-b*b)/(b*b)
    k1 = (math.sqrt(1.0+u2)-1.0)/(math.sqrt(1.0+u2)+1.0)
    A = (1.0+k1*k1/4.0)/(1.0-k1)
    B = k1*(1.0-3.0*k1*k1/8.0)
    deltaSigma = B*sinSigma*(cos2Sigmam+.25*B*(cosSigma*(-1.0+2.0*cos2Sigmam*cos2Sigmam)-(B/6.0)*cos2Sigmam*(-3.0+4.0*sinSigma*sinSigma)*(-3.0+4.0*cos2Sigmam*cos2Sigmam)))
    s = b*A*(sigma-deltaSigma)
    alpha1 = math.atan2(cosU2*sinl,cosU1*sinU2-sinU1*cosU2*cosl)

    azimuth = alpha1
    if azimuth < 0.0:
        azimuth += math.radians(360.0)

    return azimuth,s

