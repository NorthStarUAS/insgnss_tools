/*! \file nav_interface.h
 *	\brief Navigation filter interface header
 *
 *	\details This file declares the standard function prototypes for interfacing with the navigation filter.
 *	All navigation filters must include this file and instantiate the init_nav(), get_nav(), and close_nav() functions.
 *
 * \author University of Minnesota
 * \author Aerospace Engineering and Mechanics
 * \copyright Copyright 2011 Regents of the University of Minnesota. All rights reserved.
 *
 * $Id: nav_interface.h 757 2012-01-04 21:57:48Z murch $
 */

#ifndef NAV_INTERFACE_HXX_
#define NAV_INTERFACE_HXX_


struct IMUdata {
    double time;		// seconds
    double p, q, r;		// rad/sec
    double ax, ay, az;		// m/sec^2
    double hx, hy, hz;		// guass
};

struct GPSdata {
    double time;		// seconds
    double lat, lon, alt;	// deg, meter
    double vn, ve, vd;		// m/sec
    bool newData;
};

/// Define status message enum list
enum errdefs {
    got_invalid,		// No data received
    checksum_err,		// Checksum mismatch
    gps_nolock,			// No GPS lock
    data_valid,			// Data valid
    noPacketHeader,		// Some data received, but cannot find packet header
    incompletePacket,	// Packet header found, but complete packet not received
    TU_only,			// NAV filter, time update only
    gps_aided,			// NAV filter, GPS aided
};

/// Navigation Filter Data Structure
struct NAVdata {
    double time;             // [sec], timestamp of NAV filter
    double lat;		     // [rad], geodetic latitude estimate
    double lon;		     // [rad], geodetic longitude estimate
    double alt;		     // [m], altitude relative to WGS84 estimate
    double vn;		     // [m/sec], north velocity estimate
    double ve;		     // [m/sec], east velocity estimate
    double vd;		     // [m/sec], down velocity estimate
    double phi;		     // [rad], Euler roll angle estimate
    double the;		     // [rad], Euler pitch angle estimate
    double psi;		     // [rad], Euler yaw angle estimate
    double qw, qx, qy, qz;   // Quaternion estimate
    double abx, aby, abz;    // [m/sec^2], accelerometer bias estimate
    double gbx, gby, gbz;    // [rad/sec], rate gyro bias estimate
    double Pp0, Pp1, Pp2;    // [rad], covariance estimate for position
    double Pv0, Pv1, Pv2;    // [rad], covariance estimate for velocity
    double Pa0, Pa1, Pa2;    // [rad], covariance estimate for angles
    double Pabx, Paby, Pabz; // [rad], covariance estimate for accelerometer bias
    double Pgbx, Pgby, Pgbz; // [rad], covariance estimate for rate gyro bias
    enum errdefs err_type;   // NAV filter status
};

const double g = 9.814;
const double D2R = M_PI / 180.0;


#endif // NAV_INTERFACE_HXX_
