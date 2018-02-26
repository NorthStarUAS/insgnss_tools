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

#ifndef NAV_15STATE_FLOAT_HXX
#define NAV_15STATE_FLOAT_HXX


#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/LU>
using namespace Eigen;

#include "../nav_common/structs.hxx"

// usefule constants
const float g = 9.814;
const float D2R = M_PI / 180.0;

// define some types for notational convenience and consistency
typedef Matrix<float,6,6> Matrix6f;
typedef Matrix<float,12,12> Matrix12f;
typedef Matrix<float,15,15> Matrix15f;
typedef Matrix<float,6,15> Matrix6x15f;
typedef Matrix<float,15,6> Matrix15x6f;
typedef Matrix<float,15,12> Matrix15x12f;
typedef Matrix<float,6,1> Vector6f;
typedef Matrix<float,15,1> Vector15f;

class EKF15_float {

public:

    EKF15_float() {
	default_config();
    }
    ~EKF15_float() {}

    // set/get error characteristics of navigation sensors
    void set_config(NAVconfig config);
    NAVconfig get_config();
    void default_config();

    // main interface
    NAVdata init(IMUdata imu, GPSdata gps);
    NAVdata update(IMUdata imu, GPSdata gps);
    
private:

    Matrix15f F, PHI, P, Qw, Q, ImKH, KRKt, I15 /* identity */;
    Matrix15x12f G;
    Matrix15x6f K;
    Vector15f x;
    Matrix12f Rw;
    Matrix6x15f H;
    Matrix6f R;
    Vector6f y;
    Matrix3f C_N2B, C_B2N, I3 /* identity */, temp33;
    Vector3d pos_ins_ecef, pos_gps, pos_gps_ecef;
    Vector3f grav, f_b, om_ib, pos_ins_ned, pos_gps_ned, dx, mag_ned;

    Quaternionf quat;
    float tprev;

    NAVconfig config;
    NAVdata nav;
};


#endif // NAV_15STATE_FLOAT_HXX
