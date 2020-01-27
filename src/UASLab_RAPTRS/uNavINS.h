/*
Copyright (c) 2016 - 2020 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor and Chris Regan

Addapted from earlier version
Copyright 2011 Regents of the University of Minnesota. All rights reserved.
Original Author: Adhika Lie
*/

// Reference Frames and Coordinates from nav-functions()
// I - ECI (Earch Center Inertial): origin at Earth center
// E - ECEF (Earch Center Earth Fixed): origin at Earth center
// D - Geodetic: origin at Earth center, Uses earth ellisoid definition (example WGS84)
// G - Geocentric: origin at Earth center, Uses spheroid definition
// L - Local Level: origin at specified reference, [x- North, y- East, z- Down]
// B - Body: origin at Body CG, [x- Fwd, y- Starboard, z- Down]
//
// All units meters and radians

#pragma once

#include <stdint.h>
#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
using namespace Eigen;

#include "nav-functions.h"

class uNavINS {
  public:
    uNavINS() {};
    void Configure();
    void Initialize(Vector3f wMeas_rps, Vector3f aMeas_mps2, Vector3f magMeas, Vector3d pMeas_D_rrm, Vector3f vMeas_L_mps);
    bool Initialized() { return initialized_; } // returns whether the INS has been initialized
    void Update(uint64_t t_us, unsigned long timeWeek, Vector3f wMeas_rps, Vector3f aMeas_mps2, Vector3f magMeas, Vector3d pMeas_D_rrm, Vector3f vMeas_L_mps);

    // Set
    inline void Set_AccelNoise(float val) { aNoiseSigma_mps2 = val; }
    inline void Set_GyroNoise(float val) { wNoiseSigma_rps = val; }
    inline void Set_AccelMarkov(float val) { aMarkovSigma_mps2 = val; }
    inline void Set_AccelTau(float val) { aMarkovTau_s = val; }
    inline void Set_GyroMarkov(float val) { wMarkovSigma_rps = val; }
    inline void Set_GyroTau(float val) { wMarkovTau_s = val; }
    inline void Set_GpsPosNoiseNE(float val) { pNoiseSigma_NE_m = val; }
    inline void Set_GpsPosNoiseD(float val) { pNoiseSigma_D_m = val; }
    inline void Set_GpsVelNoiseNE(float val) { vNoiseSigma_NE_mps = val; }
    inline void Set_GpsVelNoiseD(float val) { vNoiseSigma_D_mps = val; }

    // Get
    Vector3f Get_AccelEst() { return aEst_mps2_; }
    Vector3f Get_AccelBias() { return aBias_mps2_; }
    Vector3f Get_RotRateEst() { return wEst_rps_; }
    Vector3f Get_RotRateBias() { return wBias_rps_; }
    Vector3f Get_OrientEst() { return euler_BL_rad_; }
    Vector3d Get_PosEst() { return pEst_D_rrm_; }
    Vector3f Get_VelEst() { return vEst_L_mps_; }
    float Get_Track() { return track_rad; }

  private:
    // initialized
    bool initialized_ = false;

    // timing
    uint64_t tPrev_us_;
    float dt_s_;
    unsigned long timeWeekPrev_;

    // Sensor variances (as standard deviation) and models (tau)
    float aNoiseSigma_mps2 = 0.05f; // Std dev of Accelerometer Wide Band Noise (m/s^2)
    float aMarkovSigma_mps2 = 0.01f; // Std dev of Accelerometer Markov Bias
    float aMarkovTau_s = 100.0f; // Correlation time or time constant

    float wNoiseSigma_rps = 0.00175f; // Std dev of gyro output noise (rad/s)
    float wMarkovSigma_rps = 0.00025; // Std dev of correlated gyro bias
    float wMarkovTau_s = 50.0f; // Correlati1on time or time constant

    float pNoiseSigma_NE_m = 3.0f; // GPS measurement noise std dev (m)
    float pNoiseSigma_D_m = 6.0f; // GPS measurement noise std dev (m)

    float vNoiseSigma_NE_mps = 0.5f; // GPS measurement noise std dev (m/s)
    float vNoiseSigma_D_mps = 1.0f; // GPS measurement noise std dev (m/s)

    // Initial set of covariance
    const float pErrSigma_Init_m = 10.0f; // Std dev of initial position error (m)
    const float vErrSigma_Init_mps = 1.0f; // Std dev of initial velocity error (m/s)
    const float attErrSigma_Init_rad = 0.34906f; // Std dev of initial attitude error (rad)
    const float hdgErrSigma_Init_rad = 3.14159f; // Std dev of initial Heading error (rad)
    const float aBiasSigma_Init_mps2 = 0.9810f; // Std dev of initial accel bias (m/s^2)
    const float wBiasSigma_Init_rps = 0.01745f; // Std dev of initial gyro bias (rad/s)

    const float G = 9.807f; // acceleration due to gravity
    const double EARTH_RADIUS = 6378137.0; // earth semi-major axis radius (m)

    // Identity matrices
    Matrix2f I2 = Matrix2f::Identity();
    Matrix3f I3 = Matrix3f::Identity();
    Matrix<float, 5, 5> I5 = Matrix<float, 5, 5>::Identity();
    Matrix<float, 6, 6> I6 = Matrix<float, 6, 6>::Identity();
    Matrix<float, 15, 15> I15 = Matrix<float, 15, 15>::Identity();

    Matrix<float,6,15> H_ = Matrix<float,6,15>::Zero(); // Observation matrix

    Matrix<float,6,6> R_ = Matrix<float,6,6>::Zero();// Covariance of the Observation Noise (associated with MeasUpdate())
    Matrix<float,12,12> Rw_ = Matrix<float,12,12>::Zero(); // Covariance of the Sensor Noise (associated with TimeUpdate())
    Matrix<float, 15, 15> P_ = Matrix<float, 15, 15>::Zero(); // Covariance Estimate

    Vector3f aBias_mps2_ = Vector3f::Zero(); // accelerometer bias
    Vector3f wBias_rps_ = Vector3f::Zero(); // rotation rate bias
    Vector3f euler_BL_rad_ = Vector3f::Zero(); // Euler Angles (3-2-1) [phi, theta, psi]
    Quaternionf quat_BL_ = Quaternionf(0.0,0.0, 0.0, 0.0); // Quaternion of B wrt L
    Vector3f aEst_mps2_ = Vector3f::Zero(); // Estimated Acceleration in Body
    Vector3f wEst_rps_ = Vector3f::Zero(); // Estimated Rotation Rate in Body
    Vector3f vEst_L_mps_ = Vector3f::Zero(); // Estimated Velocity in NED
    Vector3d pEst_D_rrm_ = Vector3d::Zero(); // Estimated Position in LLA (rad, rad, m)

    float track_rad = 0;

    // Methods
    void TimeUpdate(Vector3f wMeas_rps, Vector3f aMeas_mps2);
    void MeasUpdate(Vector3d pMeas_D_rrm, Vector3f vMeas_L_mps);

};
