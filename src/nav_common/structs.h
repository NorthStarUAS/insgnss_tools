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

#include <eigen3/Eigen/Core>

struct IMUdata {
    float time;                 // seconds
    float p, q, r;		// rad/sec
    float ax, ay, az;		// m/sec^2
    float hx, hy, hz;		// guass
    float temp;                 // C
};

struct GPSdata {
    float time;                 // seconds
    double unix_sec;		// seconds in unix time reference
    double lat, lon;            // rad
    float alt;                  // meter
    float vn, ve, vd;		// m/sec
    int sats;
};

struct GNSS_measurement
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /// time in [s]
    float time;
    /// GNSS measurement data
    /// GNSS_measurements     GNSS measurement data:
    /// Column 1              Pseudo-range measurements [m]
    /// Column 2              Pseudo-range rate measurements [m/s]
    /// Columns 3-5           Satellite ECEF position [m]
    /// Columns 6-8           Satellite ECEF velocity [m/s]
    Eigen::MatrixXd gnss_measurement;


    /// Empty Constructor
    // GNSS_measurement()
    // {
    //     time = 0.0f;
    //     gnss_measurement = MatrixXd::Zero(1, 8);
    // }
    
    // Factory function - returned by value:
    //static GNSS_measurement create(float timeIn, const Eigen::MatrixXd &measurementIn){ 
    //    return GNSS_measurement(timeIn,measurementIn); }
    /// Constructor
    //GNSS_measurement(float, const Eigen::MatrixXd&);
    // /// Function
    // void from_dict(const GNSS_measurement &gnss)
    // {
    //     time = gnss.time;
    //     gnss_measurement = gnss.gnss_measurement;
    // }
    // Constructor
    GNSS_measurement(float timeIn, const Eigen::MatrixXd &measurementIn)
        : time(timeIn), gnss_measurement(measurementIn) {}
    // Function
    void from_dict(const float timeIn, const Eigen::MatrixXd &measurementIn)
    {
        time = timeIn;
        gnss_measurement = measurementIn;
    }
};

struct GNSS_raw_measurement
{
    double AODO;          // the age of data offset, in seconds.
    double Cic;           // Amplitude of the Cosine Harmonic Correction Term to the Angle of Inclination
    double Cis;           // Amplitude of the Sine Harmonic Correction Term to the Angle of Inclination
    double Crc;           // Amplitude of the Cosine Harmonic Correction Term to the Orbit Radius
    double Crs;           // Amplitude of the Sine Harmonic Correction Term to the Orbit Radius
    double Cuc;           // Amplitude of the Cosine Harmonic Correction Term to the Argument of Latitude
    double Cus;           // Amplitude of the Sine Harmonic Correction Term to the Argument of Latitude
    bool FIT;             // ?
    double IDOT;          // Rate of Inclination Angle
    int IODC;             // Issue of Data, Clock (10 Bits) [Units: N/A]
    int IODE;             // Issue of Data (8 Bits) [Units: N/A]
    int L2;               // Code on L2 (2 Bits) [Units: N/A]
    int L2P;              // L2 P Data Flag (1 Bit) [Units: Discrete]
    double M0;            // Mean Anomaly at Reference Time
    double Omega0;        // Longitude of Ascending Node of Orbit Plane at Weekly Epoch
    double Omegad;        // Rate of Right Ascension
    int TOW17;            // Time-of-week
    double Tgd;           // (8 Bits / Two's Complement with sign on MSB) [Units: Seconds]
    double WN;            // GPS Week Number (10 Bits) [units: Week]
    double af0;           // (22 Bits / Two's Complement with sign on MSB) [Units: Seconds]
    double af1;           // (16 Bits / Two's Complement with sign on MSB) [Units: Sec/Sec]
    double af2;           // (8 Bits / Two's Complement with sign on MSB) [Units: Sec/Sec^2]
    int constellation;    // type of constellation, e.g., 0=GPS, GLONASS, BEIDOU
    double deltan;        // Mean Motion Difference From Computed Value
    double doppler;       // Doppler shift measurement, note: by multiplying frequency, we get psedu-range rate
    double e;             // Eccentricity
    bool frame1;          // Validity of subframe 1
    bool frame2;          // Validity of subframe 2
    bool frame3;          // Validity of subframe 3
    int gnssid;           // Satellite ID number
    int hlth;             // Satellite Vehicle Health (6 Bits) [Units: Discretes]
    double i0;            // Inclination at Reference Time
    double omega;         // Argument of Perigee
    double pseudorange;   // pseudorange (m)
    double sqrtA;         // Square Root off the Semi-Major Axis
    double timestamp;     // current seconds
    double toc;           // (16 Bits) [Units: Seconds]
    double toe;           // Reference Time Ephemeris
    int ura;              // Satellite Vehicle Accuracy (4 Bits) [Units: N/A], binary
};

struct Airdata {
    float time;                 // seconds
    float static_press;         // mbar
    float diff_press;		// pa
    float temp;                 // degree C
    float airspeed;		// knots
    float altitude;		// meters
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
    float sig_mag;             // magnetometer (relative to a normalized mag vector component.)
    float sig_pseudorange;     // GPS pesudorange measurement noise std dev (m)
    float sig_pseudorangeRate; // GPS pesudorange rate measurement noise std dev (m/s)
};
