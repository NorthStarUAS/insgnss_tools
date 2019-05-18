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

#pragma once

#ifdef HAVE_PYBIND11
  #include <pybind11/pybind11.h>
  namespace py = pybind11;
#endif

struct IMUdata {
    float time;                 // seconds
    float p, q, r;		// rad/sec
    float ax, ay, az;		// m/sec^2
    float hx, hy, hz;		// guass
    float temp;                 // C

#ifdef HAVE_PYBIND11
    py::dict as_dict() {
        py::dict result;
        result["time"] = time;
        result["p"] = p;
        result["q"] = q;
        result["r"] = r;
        result["ax"] = ax;
        result["ay"] = ay;
        result["az"] = az;
        result["hx"] = hx;
        result["hy"] = hy;
        result["hz"] = hz;
        result["temp"] = temp;
        return result;
    }
#endif
};

struct GPSdata {
    float time;                 // seconds
    double unix_sec;		// seconds in unix time reference
    double lat, lon;            // rad
    float alt;                  // meter
    float vn, ve, vd;		// m/sec
    int sats;
    bool newData;
    
#ifdef HAVE_PYBIND11
    py::dict as_dict() {
        py::dict result;
        result["time"] = time;
        result["unix_sec"] = unix_sec;
        result["lat"] = lat;
        result["lon"] = lon;
        result["alt"] = alt;
        result["vn"] = vn;
        result["ve"] = ve;
        result["vd"] = vd;
        result["sats"] = sats;
        return result;
    }
#endif
};

struct Airdata {
    float time;                 // seconds
    float static_press;         // mbar
    float diff_press;		// pa
    float temp;                 // degree C
    float airspeed;		// knots
    float altitude;		// meters

#ifdef HAVE_PYBIND11
    py::dict as_dict() {
        py::dict result;
        result["time"] = time;
        result["static_press"] = static_press;
        result["diff_press"] = diff_press;
        result["temp"] = temp;
        result["airspeed"] = airspeed;
        result["altitude"] = altitude;
        return result;
    }
#endif
};

/// Define status message enum list
enum errdefs {
    got_invalid,		// No data received
    checksum_err,		// Checksum mismatch
    gps_nolock,			// No GPS lock
    data_valid,			// Data valid
    noPacketHeader,		// Some data received, but cannot find packet header
    incompletePacket, // Packet header found, but complete packet not received
    TU_only,			// NAV filter, time update only
    gps_aided,			// NAV filter, GPS aided
};

/// Navigation filter data structure
struct NAVdata {
    float time;              // [sec], timestamp of NAV filter
    double lat;              // [rad], geodetic latitude estimate
    double lon;              // [rad], geodetic longitude estimate
    float alt;               // [m], altitude relative to WGS84 estimate
    float vn;                // [m/sec], north velocity estimate
    float ve;                // [m/sec], east velocity estimate
    float vd;                // [m/sec], down velocity estimate
    float phi;               // [rad], Euler roll angle estimate
    float the;               // [rad], Euler pitch angle estimate
    float psi;               // [rad], Euler yaw angle estimate
    float qw, qx, qy, qz;    // Quaternion estimate
    float abx, aby, abz;     // [m/sec^2], accelerometer bias estimate
    float gbx, gby, gbz;     // [rad/sec], rate gyro bias estimate
    float Pp0, Pp1, Pp2;     // [rad], covariance estimate for position
    float Pv0, Pv1, Pv2;     // [rad], covariance estimate for velocity
    float Pa0, Pa1, Pa2;     // [rad], covariance estimate for angles
    float Pabx, Paby, Pabz;  // [rad], covariance estimate for accelerometer bias
    float Pgbx, Pgby, Pgbz;  // [rad], covariance estimate for rate gyro bias
    enum errdefs err_type;   // NAV filter status
    
#ifdef HAVE_PYBIND11
    py::dict as_dict() {
        py::dict result;
        result["time"] = time;
        result["lat"] = lat;
        result["lon"] = lon;
        result["alt"] = alt;
        result["vn"] = vn;
        result["ve"] = ve;
        result["vd"] = vd;
        result["phi"] = phi;
        result["the"] = the;
        result["psi"] = psi;
        result["qw"] = qw;
        result["qx"] = qx;
        result["qy"] = qy;
        result["qz"] = qz;
        result["abx"] = abx;
        result["aby"] = aby;
        result["abz"] = abz;
        result["gbx"] = gbx;
        result["gby"] = gby;
        result["gbz"] = gbz;
        result["Pp0"] = Pp0;
        result["Pp1"] = Pp1;
        result["Pp2"] = Pp2;
        result["Pv0"] = Pv0;
        result["Pv1"] = Pv1;
        result["Pv2"] = Pv2;
        result["Pa0"] = Pa0;
        result["Pa1"] = Pa1;
        result["Pa2"] = Pa2;
        result["Pabx"] = Pabx;
        result["Paby"] = Paby;
        result["Pabz"] = Pabz;
        result["Pgbx"] = Pgbx;
        result["Pgby"] = Pgby;
        result["Pgbz"] = Pgbz;
        return result;
    }
#endif
};

struct NAVconfig {
    float sig_w_ax;		// m/s^2
    float sig_w_ay;
    float sig_w_az;
    float sig_w_gx;		// rad/s (0.1 deg/s)
    float sig_w_gy;
    float sig_w_gz;
    float sig_a_d;		// 5e-2*g
    float tau_a;
    float sig_g_d;		// 0.1 deg/s
    float tau_g;
    float sig_gps_p_ne;
    float sig_gps_p_d;
    float sig_gps_v_ne;
    float sig_gps_v_d;
    float sig_mag;
    
#ifdef HAVE_PYBIND11
    py::dict as_dict() {
        py::dict result;
        result["sig_w_ax"] = sig_w_ax;
        result["sig_w_ay"] = sig_w_ay;
        result["sig_w_az"] = sig_w_az;
        result["sig_w_gx"] = sig_w_gx;
        result["sig_w_gy"] = sig_w_gy;
        result["sig_w_gz"] = sig_w_gz;
        result["sig_a_d"] = sig_a_d;
        result["tau_a"] = tau_a;
        result["sig_g_d"] = sig_g_d;
        result["tau_g"] = tau_g;
        result["sig_gps_p_ne"] = sig_gps_p_ne;
        result["sig_gps_p_d"] = sig_gps_p_d;
        result["sig_gps_v_ne"] = sig_gps_v_ne;
        result["sig_gps_v_d"] = sig_gps_v_d;
        result["sig_mag"] = sig_mag;
        return result;
    }
#endif
};
