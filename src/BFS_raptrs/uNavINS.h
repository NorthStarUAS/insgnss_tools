/*
Updated to be a class, use Eigen, and compile as an Arduino library.
Added methods to get gyro and accel bias. Added initialization to
estimated angles rather than assuming IMU is level. Added method to get psi,
rather than just heading, and ground track.

Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

/*
Addapted from earlier version
Copyright 2011 Regents of the University of Minnesota. All rights reserved.
Original Author: Adhika Lie
*/

// XXX - accel and gyro bias not being updated.
// XXX - add set methods for sensor characteristics.
// XXX - is psi heading or track? I think heading.
// XXX - add outputs for filter covariance to measure convergence.
// XXX - incorporate magnetometers.

#pragma once

#include <stdint.h>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>

class uNavINS {
  public:
    inline void setSig_W_A(float sig_w_a) { SIG_W_A = sig_w_a; }
    inline void setSig_W_G(float sig_w_g) { SIG_W_G = sig_w_g; }
    inline void setSig_A_D(float sig_a_d) { SIG_A_D = sig_a_d; }
    inline void setTau_A(float tau_a) { TAU_A = tau_a; }
    inline void setSig_G_D(float sig_g_d) { SIG_G_D = sig_g_d; }
    inline void setTau_G(float tau_g) { TAU_G = tau_g; }
    inline void setSig_GPS_P_NE(float sig_gps_p_ne) { SIG_GPS_P_NE = sig_gps_p_ne; }
    inline void setSig_GPS_P_D(float sig_gps_p_d) { SIG_GPS_P_D = sig_gps_p_d; }
    inline void setSig_GPS_V_NE(float sig_gps_v_ne) { SIG_GPS_V_NE = sig_gps_v_ne; }
    inline void setSig_GPS_V_D(float sig_gps_v_d) { SIG_GPS_V_D = sig_gps_v_d; }
    void update(uint64_t time,unsigned long TOW,double vn,double ve,double vd,double lat,double lon,double alt,float p,float q,float r,float ax,float ay,float az,float hx,float hy,float hz);
    bool initialized();
    float getPitch_rad();
    float getRoll_rad();
    float getYaw_rad();
    float getHeading_rad();
    double getLatitude_rad();
    double getLongitude_rad();
    double getAltitude_m();
    double getVelNorth_ms();
    double getVelEast_ms();
    double getVelDown_ms();
    float getGroundTrack_rad();
    float getGyroBiasX_rads();
    float getGyroBiasY_rads();
    float getGyroBiasZ_rads();
    float getAccelBiasX_mss();
    float getAccelBiasY_mss();
    float getAccelBiasZ_mss();
  private:
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // error characteristics of navigation parameters
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Std dev of Accelerometer Wide Band Noise (m/s^2)
    float SIG_W_A = 0.05f;
    // Std dev of gyro output noise (rad/s)
    float SIG_W_G = 0.00175f;
    // Std dev of Accelerometer Markov Bias
    float SIG_A_D = 0.01f;
    // Correlation time or time constant
    float TAU_A = 100.0f;
    // Std dev of correlated gyro bias
    float SIG_G_D = 0.00025;
    // Correlati1on time or time constant
    float TAU_G = 50.0f;
    // GPS measurement noise std dev (m)
    float SIG_GPS_P_NE = 3.0f;
    float SIG_GPS_P_D = 6.0f;
    // GPS measurement noise std dev (m/s)
    float SIG_GPS_V_NE = 0.5f;
    float SIG_GPS_V_D = 1.0f;
    // Initial set of covariance
    const float P_P_INIT = 10.0f;
    const float P_V_INIT = 1.0f;
    const float P_A_INIT = 0.34906f;
    const float P_HDG_INIT = 3.14159f;
    const float P_AB_INIT = 0.9810f;
    const float P_GB_INIT = 0.01745f;
    // acceleration due to gravity
    const float G = 9.807f;
    // major eccentricity squared
    const double ECC2 = 0.0066943799901;
    // earth semi-major axis radius (m)
    const double EARTH_RADIUS = 6378137.0;
    // initialized
    bool initialized_ = false;
    // timing
    uint64_t _tprev;
    float _dt;
    unsigned long previousTOW;
    // estimated attitude
    float phi, theta, psi, heading;
    // initial heading angle
    float psi_initial;
    // estimated NED velocity
    double vn_ins, ve_ins, vd_ins;
    // estimated location
    double lat_ins, lon_ins, alt_ins;
    // magnetic heading corrected for roll and pitch angle
    float Bxc, Byc;
    // accelerometer bias
    float abx = 0.0, aby = 0.0, abz = 0.0;
    // gyro bias
    float gbx = 0.0, gby = 0.0, gbz = 0.0;
    // earth radius at location
    double Re, Rn, denom;
    // State matrix
    Eigen::Matrix<float,15,15> Fs = Eigen::Matrix<float,15,15>::Identity();
    // State transition matrix
    Eigen::Matrix<float,15,15> PHI = Eigen::Matrix<float,15,15>::Zero();
    // Covariance matrix
    Eigen::Matrix<float,15,15> P = Eigen::Matrix<float,15,15>::Zero();
    // For process noise transformation
    Eigen::Matrix<float,15,12> Gs = Eigen::Matrix<float,15,12>::Zero();
    Eigen::Matrix<float,12,12> Rw = Eigen::Matrix<float,12,12>::Zero();
    // Process noise matrix
    Eigen::Matrix<float,15,15> Q = Eigen::Matrix<float,15,15>::Zero();
    // Gravity model
    Eigen::Matrix<float,3,1> grav = Eigen::Matrix<float,3,1>::Zero();
    // Rotation rate
    Eigen::Matrix<float,3,1> om_ib = Eigen::Matrix<float,3,1>::Zero();
    // Specific force
    Eigen::Matrix<float,3,1> f_b = Eigen::Matrix<float,3,1>::Zero();
    // DCM
    Eigen::Matrix<float,3,3> C_N2B = Eigen::Matrix<float,3,3>::Zero();
    // DCM transpose
    Eigen::Matrix<float,3,3> C_B2N = Eigen::Matrix<float,3,3>::Zero();
    // Temporary to get dxdt
    Eigen::Matrix<float,3,1> dx = Eigen::Matrix<float,3,1>::Zero();
    Eigen::Matrix<double,3,1> dxd = Eigen::Matrix<double,3,1>::Zero();
    // NED velocity INS
    Eigen::Matrix<double,3,1> V_ins = Eigen::Matrix<double,3,1>::Zero();
    // LLA INS
    Eigen::Matrix<double,3,1> lla_ins = Eigen::Matrix<double,3,1>::Zero();
    // NED velocity GPS
    Eigen::Matrix<double,3,1> V_gps = Eigen::Matrix<double,3,1>::Zero();
    // LLA GPS
    Eigen::Matrix<double,3,1> lla_gps = Eigen::Matrix<double,3,1>::Zero();
    // Position ECEF INS
    Eigen::Matrix<double,3,1> pos_ecef_ins = Eigen::Matrix<double,3,1>::Zero();
    // Position NED INS
    Eigen::Matrix<double,3,1> pos_ned_ins = Eigen::Matrix<double,3,1>::Zero();
    // Position ECEF GPS
    Eigen::Matrix<double,3,1> pos_ecef_gps = Eigen::Matrix<double,3,1>::Zero();
    // Position NED GPS
    Eigen::Matrix<double,3,1> pos_ned_gps = Eigen::Matrix<double,3,1>::Zero();
    // Quat
    Eigen::Matrix<float,4,1> quat = Eigen::Matrix<float,4,1>::Zero();
    // dquat
    Eigen::Matrix<float,4,1> dq = Eigen::Matrix<float,4,1>::Zero();
    // difference between GPS and INS
    Eigen::Matrix<float,6,1> y = Eigen::Matrix<float,6,1>::Zero();
    // GPS measurement noise
    Eigen::Matrix<float,6,6> R = Eigen::Matrix<float,6,6>::Zero();
    Eigen::Matrix<float,15,1> x = Eigen::Matrix<float,15,1>::Zero();
    // Kalman Gain
    Eigen::Matrix<float,15,6> K = Eigen::Matrix<float,15,6>::Zero();
    Eigen::Matrix<float,6,15> H = Eigen::Matrix<float,6,15>::Zero();
    // skew symmetric
    Eigen::Matrix<float,3,3> sk(Eigen::Matrix<float,3,1> w);
    // lla rate
    Eigen::Matrix<double,3,1> llarate(Eigen::Matrix<double,3,1> V,Eigen::Matrix<double,3,1> lla);
    // lla to ecef
    Eigen::Matrix<double,3,1> lla2ecef(Eigen::Matrix<double,3,1> lla);
    // ecef to ned
    Eigen::Matrix<double,3,1> ecef2ned(Eigen::Matrix<double,3,1> ecef,Eigen::Matrix<double,3,1> pos_ref);
    // quaternion to dcm
    Eigen::Matrix<float,3,3> quat2dcm(Eigen::Matrix<float,4,1> q);
    // quaternion multiplication
    Eigen::Matrix<float,4,1> qmult(Eigen::Matrix<float,4,1> p, Eigen::Matrix<float,4,1> q);
    // maps angle to +/- 180
    float constrainAngle180(float dta);
    // maps angle to 0-360
    float constrainAngle360(float dta);
};
