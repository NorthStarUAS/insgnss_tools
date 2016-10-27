/*! \file EKF_15state.c
 *	\brief 15 state EKF navigation filter
 *
 *	\details  15 state EKF navigation filter using loosely integrated INS/GPS architecture.
 * 	Time update is done after every IMU data acquisition and GPS measurement
 * 	update is done every time the new data flag in the GPS data packet is set. Designed by Adhika Lie.
 *	Attitude is parameterized using quaternions.
 *	Estimates IMU bias errors.
 *	\ingroup nav_fcns
 *
 * \author University of Minnesota
 * \author Aerospace Engineering and Mechanics
 * \copyright Copyright 2011 Regents of the University of Minnesota. All rights reserved.
 *
 */

#ifndef NAV_OPENLOOP_HXX
#define NAV_OPENLOOP_HXX


#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/LU>
using namespace Eigen;

#include "../nav_core/nav_structs.hxx"
#include "../nav_core/nav_functions.hxx"

// usefule constants
const double g = 9.814;
const double D2R = M_PI / 180.0;

class OpenLoop {

public:

    OpenLoop();
    ~OpenLoop() {}

    // main interface
    void init(double lat_deg, double lon_deg, double alt_m,
	      double vn_ms, double ve_ms, double vd_ms,
	      double phi_deg, double the_deg, double psi_deg);
    void set_pos(double lat_deg, double lon_deg, double alt_m);
    void set_vel(double vn_ms, double ve_ms, double vd_ms);
    void set_att(double phi_deg, double the_deg, double psi_deg);
    void set_gyro_calib(double gxb, double gyb, double gzb,
			double gxs, double gys, double gzs);
    void set_G(double x11, double x12, double x13,
	       double x21, double x22, double x23,
	       double x31, double x32, double x33);
    void set_accel_calib(double axb, double ayb, double azb,
			 double axs, double ays, double azs);
    NAVdata update(IMUdata imu /*, GPSdata gps*/);
    
private:

    double lat_rad, lon_rad, alt_m;   // location
    double vn_ms, ve_ms, vd_ms;	      // ned vel
    double phi_rad, the_rad, psi_rad; // euler attitude
    double gxb, gyb, gzb;	      // gyro biases
    double gxs, gys, gzs;	      // gyro scale factors
    double axb, ayb, azb;	      // accel biases
    double axs, ays, azs;	      // accel scale factor
    Matrix3d G;			      // g force gyro bias matrix
    
    Vector3d pos_ecef, pos_lla;
    Quaterniond ned2body, body2ned;
    Quaterniond ecef2ned, ned2ecef;
    Vector3d vel_ned, vel_ecef;
    Vector3d glocal_ned;
    double tprev;
    NAVdata nav;
};


#endif // NAV_OPENLOOP_HXX
