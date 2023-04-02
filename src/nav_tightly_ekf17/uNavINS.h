/*
Copyright (c) 2016 - 2020 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details

Adapted for 17-state Tightly-coupled EKF: Kerry Sun

Adapted from prior versions
Copyright 2011 Regents of the University of Minnesota. All rights reserved.
Original Author: Adhika Lie, Gokhan Inalhan, Demoz Gebre, Jung Soon Jang

Reference Frames and Coordinates from nav-functions()
I - ECI (Earch Center Inertial): origin at Earth center
E - ECEF (Earch Center Earth Fixed): origin at Earth center
D - Geodetic: origin at Earth center, Uses earth ellisoid definition (example WGS84)
G - Geocentric: origin at Earth center, Uses spheroid definition
L - Local Level: origin at specified reference, [x- North, y- East, z- Down]
B - Body: origin at Body CG, [x- Fwd, y- Starboard, z- Down]

All units meters and radians
"Acceleration" is actually "specific gravity", ie. gravity is removed.
*/

#pragma once

#include <stdint.h>
#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/LU>
//#include <Eigen/Core>
//#include <Eigen/Dense>
//#include <Eigen/Geometry>
using namespace Eigen;

#include "nav-functions.h"
#include "../nav_common/structs.h"


class uNavINS {
  public:
    uNavINS() {};
    void Configure();
    // void Initialize(Vector3f wMeas_B_rps, Vector3f aMeas_B_mps2, Vector3f magMeas_B_uT,
    //                 Vector3d pMeas_D_rrm, Vector3f vMeas_L_mps);
    void Initialize(Vector3f wMeas_B_rps, Vector3f aMeas_B_mps2, Vector3f magMeas_B_uT,
                     MatrixXd &gnss_measurement);                
    bool Initialized() { return initialized_; } // returns whether the INS has been initialized
    // void Update(uint64_t t_us, unsigned long timeWeek, Vector3f wMeas_B_rps, Vector3f aMeas_B_mps2, Vector3f magMeas_B_uT, Vector3d pMeas_D_rrm, Vector3f vMeas_L_mps);
    void Update(uint64_t t_us, unsigned long timeWeek, Vector3f wMeas_B_rps, Vector3f aMeas_B_mps2, Vector3f magMeas_B_uT,  MatrixXd &gnss_measurement);
    // Set Configuration
    inline void Set_AccelSigma(float val) { aNoiseSigma_mps2 = val; }
    inline void Set_AccelMarkov(float val) { aMarkovSigma_mps2 = val; }
    inline void Set_AccelTau(float val) { aMarkovTau_s = val; }
    inline void Set_RotRateSigma(float val) { wNoiseSigma_rps = val; }
    inline void Set_RotRateMarkov(float val) { wMarkovSigma_rps = val; }
    inline void Set_RotRateTau(float val) { wMarkovTau_s = val; }
    inline void Set_PosSigmaNE(float val) { pNoiseSigma_NE_m = val; }
    inline void Set_PosSigmaD(float val) { pNoiseSigma_D_m = val; }
    inline void Set_VelSigmaNE(float val) { vNoiseSigma_NE_mps = val; }
    inline void Set_VelSigmaD(float val) { vNoiseSigma_D_mps = val; }
    inline void Set_PseudorangeSigma(float val){ pseudorangeNoiseSigma_m = val; } 
    inline void Set_PseudorangeRateSigma(float val){pseudorangeRateNoiseSigma_mps = val; } 

    // Set Initial Covariance 
    inline void Set_InitPosSigma(float val) { pErrSigma_Init_m = val; }
    inline void Set_InitVelSigma(float val) { vErrSigma_Init_mps = val; }
    inline void Set_InitOrientSigma(float val) { attErrSigma_Init_rad = val; }
    inline void Set_InitHeadingSigma(float val) { hdgErrSigma_Init_rad = val; }
    inline void Set_InitAccelBiasSigma(float val) { aBiasSigma_Init_mps2 = val; }
    inline void Set_InitRotRateBiasSigma(float val) { wBiasSigma_Init_rps = val; }
    inline void Set_InitClockBiasSigma(float val) { clockSigma_Init_m = val; }
    inline void Set_InitClockRateBiasSigma(float val) { clockrateSigma_Init_mps= val; }


    // Get Navigation Estimates
    inline Vector3f Get_AccelEst() { return aEst_B_mps2_; }
    inline Vector3f Get_AccelBias() { return aBias_mps2_; }
    inline Vector3f Get_RotRateEst() { return wEst_B_rps_; }
    inline Vector3f Get_RotRateBias() { return wBias_rps_; }
    inline Vector3f Get_OrientEst() { return euler_BL_rad_; }
    inline Vector3d Get_PosEst() { return pEst_D_rrm_; }
    inline Vector3f Get_VelEst() { return vEst_L_mps_; }
    inline float Get_Track() { return atan2f(vEst_L_mps_(1), vEst_L_mps_(0)); }

    // Get Covariance Estimates
    inline Vector3d Get_CovPos() { return P_.block(0,0,3,3).diagonal(); }
    inline Vector3d Get_CovVel() { return P_.block(3,3,3,3).diagonal(); }
    inline Vector3d Get_CovOrient() { return P_.block(6,6,3,3).diagonal(); }
    inline Vector3d Get_CovAccelBias() { return P_.block(9,9,3,3).diagonal(); }
    inline Vector3d Get_CovRotRateBias() { return P_.block(12,12,3,3).diagonal(); }

    // Get Innovation
    //inline Vector3f Get_InnovationPos() { return S_.block(0,0,3,3).diagonal(); }
    //inline Vector3f Get_InnovationVel() { return S_.block(3,3,3,3).diagonal(); }

  private:
    // Model Constants
    const float G = 9.807f; // Acceleration due to gravity
    const double EARTH_RADIUS = 6378137.0; // earth semi-major axis radius (m)

    // Initialize flag
    bool initialized_ = false;

    // Timing
    uint64_t tPrev_us_;
    float dt_s_;
    unsigned long timeWeekPrev_;

    // Sensor variances (as standard deviation) and models (tau)
    float aNoiseSigma_mps2 = 0.05f; // Std dev of Accelerometer Wide Band Noise (m/s^2)
    float aMarkovSigma_mps2 = 0.01f; // Std dev of Accelerometer Markov Bias
    float aMarkovTau_s = 100.0f; // Correlation time or time constant

    float wNoiseSigma_rps = 0.00175f; // Std dev of rotation rate output noise (rad/s)
    float wMarkovSigma_rps = 0.00025; // Std dev of correlated rotation rate bias
    float wMarkovTau_s = 50.0f; // Correlati1on time or time constant

    float pNoiseSigma_NE_m = 3.0f; // GPS measurement noise std dev (m)
    float pNoiseSigma_D_m = 6.0f; // GPS measurement noise std dev (m)

    float vNoiseSigma_NE_mps = 0.5f; // GPS measurement noise std dev (m/s)
    float vNoiseSigma_D_mps = 1.0f; // GPS measurement noise std dev (m/s)

    float pseudorangeNoiseSigma_m = 1.0f; // GPS pseudorange measurement noise std dev (m)
    float pseudorangeRateNoiseSigma_mps = 1.0f; // GPS pseudorange rate measurement noise std dev (m/s)

    // Initial set of covariance
    float pErrSigma_Init_m = 10.0f; // Std dev of initial position error (m)
    float vErrSigma_Init_mps = 1.0f; // Std dev of initial velocity error (m/s)
    float attErrSigma_Init_rad = 0.34906f; // Std dev of initial attitude (phi and theta) error (rad)
    float hdgErrSigma_Init_rad = 3.14159f; // Std dev of initial Heading (psi) error (rad)
    float aBiasSigma_Init_mps2 = 0.9810f; // Std dev of initial acceleration bias (m/s^2)
    float wBiasSigma_Init_rps = 0.01745f; // Std dev of initial rotation rate bias (rad/s)
    float clockSigma_Init_m  = 10.0f; // Std dev of initial receiver clock frequency drift (m)
    float clockrateSigma_Init_mps = 0.1f; //Std dev of initial receiver clock phase drift (m/s)   

    // Identity matrices
    const Matrix<float,2,2> I2 = Matrix<float,2,2>::Identity();
    const Matrix<float,3,3> I3 = Matrix<float,3,3>::Identity();
    const Matrix<float,5,5> I5 = Matrix<float,5,5>::Identity();
    const Matrix<float,15,15> I15 = Matrix<float,15,15>::Identity();
    const Matrix<float,17,17> I17 = Matrix<float,17,17>::Identity();

    // Kalman Matrices
    //Matrix<float,6,15> H_; // Observation matrix
    //Matrix<float,6,6> R_;// Covariance of the Observation Noise (associated with MeasUpdate())
    Matrix<double,14,14> Rw_; // Covariance of the Sensor Noise (associated with TimeUpdate())
    //Matrix<float,6,6> S_; // Innovation covariance
    Matrix<double,17,17> P_; // Covariance estimate

    // Global variables
    Vector3f aBias_mps2_; // acceleration bias
    Vector3f wBias_rps_; // rotation rate bias
    Vector3f euler_BL_rad_; // Euler angles - B wrt L (3-2-1) [phi, theta, psi]
    Quaternionf quat_BL_; // Quaternion of B wrt L
    Vector3f aEst_B_mps2_; // Estimated acceleration in Body
    Vector3f wEst_B_rps_; // Estimated rotation rate in Body
    Vector3f vEst_L_mps_; // Estimated velocity in NED  (m/s)
    Vector3d pEst_D_rrm_; // Estimated position in LLA (rad, rad, m)
    Vector3d pEst_E_m_;   // Estimated position in ECEF - frame (m, m, m)
    Vector3d vEst_E_mps_; // Estimated velocity in ECEF (m/s)
    double clockBias_m_ = 0.0; // clock bias
    double clockRateBias_mps_ = 0.0; // clock rate bias
    Vector3d omega_ie;  // Earth rate vector 
    Matrix3d Omega_ie;  // Earth rate rotation matrix 
    //GNSS_raw_measurement gnss_raw_measurement_; 
   
    // Methods
    void TimeUpdateNED();
    void TimeUpdateECEF();
    //void MeasUpdate(Vector3d pMeas_D_rrm, Vector3f vMeas_L_mps);
    void MeasUpdate17NED(MatrixXd &gnss_measurement);
    // void MeasSeqUpdate17(MatrixXd &gnss_measurement);
    void MeasUpdate17ECEF(MatrixXd &gnss_measurement);
    
};
