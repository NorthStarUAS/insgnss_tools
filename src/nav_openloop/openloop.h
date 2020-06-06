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

#pragma once

#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/LU>
using namespace Eigen;

#include "../nav_common/constants.h"
#include "../nav_common/structs.h"
#include "../nav_common/nav_functions.h"

class OpenLoop {

public:

    OpenLoop();
    ~OpenLoop() {}

    // main interface
    void init(double lat_deg, double lon_deg, float alt_m,
	      float vn_ms, float ve_ms, float vd_ms,
	      float phi_deg, float the_deg, float psi_deg);
    void init_by_nav(NAVdata);
    void set_pos(double lat_deg, double lon_deg, float alt_m);
    void set_vel(float vn_ms, float ve_ms, float vd_ms);
    void set_att(float phi_deg, float the_deg, float psi_deg);
    void set_gyro_calib(float gxb, float gyb, float gzb,
			float gxd, float gyd, float gzd);
    void set_accel_calib(float axb, float ayb, float azb,
			 float axd, float ayd, float azd);
    // void set_G(float x11, float x12, float x13,
    //            float x21, float x22, float x23,
    //            float x31, float x32, float x33);
    NAVdata update(IMUdata imu /*, GPSdata gps*/);
    
private:

    double lat_rad, lon_rad;          // position
    float alt_m;                      // altitude
    float vn_ms, ve_ms, vd_ms;	      // ned vel
    float phi_rad, the_rad, psi_rad; // euler attitude
    float gxb, gyb, gzb;	      // gyro biases
    //float gxs, gys, gzs;	      // gyro scale factors
    //float gxd, gyd, gzd;	      // gyro time variation
    float axb, ayb, azb;	      // accel biases
    //float axs, ays, azs;	      // accel scale factor
    //float axd, ayd, azd;	      // accel time variation
    //Matrix3d G;		      // g force gyro bias matrix
    
    Vector3d pos_ecef, pos_lla;
    Quaternionf ned2body, body2ned;
    Quaternionf ecef2ned, ned2ecef;
    Vector3f vel_ned, vel_ecef;
    Vector3f glocal_ned;
    float tstart;
    float tprev;
    NAVdata nav;
};
