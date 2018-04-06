#pragma once

#include <boost/python.hpp>
using namespace boost::python;

// given, lla, az1 and distance (s), return (lat2, lon2) and modify
// *az2 (starting return heading.)  Lat, lon, and azimuth are in
// degrees, distance in meters.
tuple py_geo_direct_wgs84 ( double lat1, double lon1, double az1, double s );

// given lat1, lon1, lat2, lon2, calculate starting and ending
// az1, az2 and distance (s).  Lat, lon, and azimuth are in degrees.
// distance in meters
tuple py_geo_inverse_wgs84( double lat1, double lon1, double lat2, double lon2 );
