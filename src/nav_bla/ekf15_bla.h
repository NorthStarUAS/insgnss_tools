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

#include "BasicLinearAlgebra.h"
using namespace BLA;

#include "../util/nav_structs.h"

// define some types for notational convenience and consistency
typedef Matrix<6, 1, float> Vector6f;
typedef Matrix<15, 1, float> Vector15f;

class EKF15_bla{

public:

    EKF15_bla() {
        default_config();
    }
    ~EKF15_bla() {}

    // set/get error characteristics of navigation sensors
    void set_config(NAVconfig _config);
    NAVconfig get_config();
    void default_config();

    // main interface
    void init(IMUdata imu, GPSdata gps);
    void time_update(IMUdata imu);
    void measurement_update(GPSdata gps);

    NAVdata get_nav();

private:

    // if these matrices are allocated on the stack and this is a problem for
    // small embedded systems, the calling layer could allocate this entire
    // class instance on the heap at init time.
    Matrix<15, 15, float> F, PHI, P, Qw, Q, ImKH, KRKt;
    Matrix<15, 15, float> I15 = Eye<15, 15, float>();
    Matrix<15, 12, float> G;
    Matrix<15, 6, float> K;
    Matrix<12, 12, float> Rw;
    Matrix<6, 15, float> H = Eye<6, 15, float>();
    Matrix<6, 6, float> R;
    Matrix<15, 1, float> x;
    Matrix<6, 1, float> y;
    Matrix3f C_N2B, C_B2N, temp33;
    Matrix3f I3 = Eye<3, 3, float>();
    Vector3f grav, f_b, om_ib, dx, mag_ned;
    Quaternionf quat;

    IMUdata imu_last;
    NAVconfig config;
    NAVdata nav;
};
