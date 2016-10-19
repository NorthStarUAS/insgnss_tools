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
 */

#ifndef NAV_STRUCTS_HXX
#define NAV_STRUCTS_HXX


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

/// Navigation filter data structure
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


struct NAVconfig {
    double sig_w_ax;		// m/s^2
    double sig_w_ay;
    double sig_w_az;
    double sig_w_gx;		// rad/s (0.1 deg/s)
    double sig_w_gy;
    double sig_w_gz;
    double sig_a_d;		// 5e-2*g
    double tau_a;
    double sig_g_d;		// 0.1 deg/s
    double tau_g;
    double sig_gps_p_ne;
    double sig_gps_p_d;
    double sig_gps_v_ne;
    double sig_gps_v_d;
    double sig_mag;
};

// The following constructs a python interface for this class and
// associated structures.

#include <boost/python.hpp>
using namespace boost::python;

BOOST_PYTHON_MODULE(libnav_structs)
{
    class_<IMUdata>("IMUdata")
	.def_readwrite("time", &IMUdata::time)
	.def_readwrite("p", &IMUdata::p)
	.def_readwrite("q", &IMUdata::q)
	.def_readwrite("r", &IMUdata::r)
 	.def_readwrite("ax", &IMUdata::ax)
 	.def_readwrite("ay", &IMUdata::ay)
 	.def_readwrite("az", &IMUdata::az)
 	.def_readwrite("hx", &IMUdata::hx)
 	.def_readwrite("hy", &IMUdata::hy)
 	.def_readwrite("hz", &IMUdata::hz)
    ;
    
    class_<GPSdata>("GPSdata")
	.def_readwrite("time", &GPSdata::time)
	.def_readwrite("lat", &GPSdata::lat)
	.def_readwrite("lon", &GPSdata::lon)
	.def_readwrite("alt", &GPSdata::alt)
	.def_readwrite("vn", &GPSdata::vn)
	.def_readwrite("ve", &GPSdata::ve)
	.def_readwrite("vd", &GPSdata::vd)
	.def_readwrite("newData", &GPSdata::newData)
    ;

    class_<NAVdata>("NAVdata")
	.def_readonly("time", &NAVdata::time)
	.def_readonly("lat", &NAVdata::lat)
	.def_readonly("lon", &NAVdata::lon)
	.def_readonly("alt", &NAVdata::alt)
	.def_readonly("vn", &NAVdata::vn)
	.def_readonly("ve", &NAVdata::ve)
	.def_readonly("vd", &NAVdata::vd)
	.def_readonly("phi", &NAVdata::phi)
	.def_readonly("the", &NAVdata::the)
	.def_readonly("psi", &NAVdata::psi)
	.def_readonly("qw", &NAVdata::qw)
	.def_readonly("qx", &NAVdata::qx)
	.def_readonly("qy", &NAVdata::qy)
	.def_readonly("qz", &NAVdata::qz)
	.def_readonly("abx", &NAVdata::abx)
	.def_readonly("aby", &NAVdata::aby)
	.def_readonly("abz", &NAVdata::abz)
	.def_readonly("gbx", &NAVdata::gbx)
	.def_readonly("gby", &NAVdata::gby)
	.def_readonly("gbz", &NAVdata::gbz)
	.def_readonly("Pp0", &NAVdata::Pp0)
	.def_readonly("Pp1", &NAVdata::Pp1)
	.def_readonly("Pp2", &NAVdata::Pp2)
	.def_readonly("Pv0", &NAVdata::Pv0)
	.def_readonly("Pv1", &NAVdata::Pv1)
	.def_readonly("Pv2", &NAVdata::Pv2)
	.def_readonly("Pa0", &NAVdata::Pa0)
	.def_readonly("Pa1", &NAVdata::Pa1)
	.def_readonly("Pa2", &NAVdata::Pa2)
	.def_readonly("Pabx", &NAVdata::Pabx)
	.def_readonly("Paby", &NAVdata::Paby)
	.def_readonly("Pabz", &NAVdata::Pabz)
	.def_readonly("Pgbx", &NAVdata::Pgbx)
	.def_readonly("Pgby", &NAVdata::Pgby)
	.def_readonly("Pgbz", &NAVdata::Pgbz)
    ;

    class_<NAVconfig>("NAVconfig")
	.def_readwrite("sig_w_ax", &NAVconfig::sig_w_ax)
	.def_readwrite("sig_w_ay", &NAVconfig::sig_w_ay)
	.def_readwrite("sig_w_az", &NAVconfig::sig_w_az)
	.def_readwrite("sig_w_gx", &NAVconfig::sig_w_gx)
	.def_readwrite("sig_w_gy", &NAVconfig::sig_w_gy)
	.def_readwrite("sig_w_gz", &NAVconfig::sig_w_gz)
	.def_readwrite("sig_a_d", &NAVconfig::sig_a_d)
	.def_readwrite("tau_a", &NAVconfig::tau_a)
	.def_readwrite("sig_g_d", &NAVconfig::sig_g_d)
	.def_readwrite("tau_g", &NAVconfig::tau_g)
	.def_readwrite("sig_gps_p_ne", &NAVconfig::sig_gps_p_ne)
	.def_readwrite("sig_gps_p_d", &NAVconfig::sig_gps_p_d)
	.def_readwrite("sig_gps_v_ne", &NAVconfig::sig_gps_v_ne)
	.def_readwrite("sig_gps_v_d", &NAVconfig::sig_gps_v_d)
	.def_readwrite("sig_mag", &NAVconfig::sig_mag)
    ;
}

#endif // NAV_STRUCTS_HXX
