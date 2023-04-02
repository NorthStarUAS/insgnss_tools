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
#include <iostream>
#include "uNavINS.h"
using namespace std; 

void uNavINS::Configure()
{
  // Covariance of the Process Noise (associated with TimeUpdate())
  Rw_.setZero();
  Rw_.block(0, 0, 3, 3) = (aNoiseSigma_mps2 * aNoiseSigma_mps2) * I3.cast<double>();
  Rw_.block(3, 3, 3, 3) = (wNoiseSigma_rps * wNoiseSigma_rps) * I3.cast<double>();
  Rw_.block(6, 6, 3, 3) = 2.0f * (aMarkovSigma_mps2 * aMarkovSigma_mps2) / aMarkovTau_s * I3.cast<double>();
  Rw_.block(9, 9, 3, 3) = 2.0f * (wMarkovSigma_rps * wMarkovSigma_rps) / wMarkovTau_s * I3.cast<double>();
  Rw_(12,12) = 1.0f * (double)pseudorangeNoiseSigma_m * (double)pseudorangeNoiseSigma_m; 
  Rw_(13,13) = 1.0f * (double)pseudorangeRateNoiseSigma_mps * (double)pseudorangeRateNoiseSigma_mps;


  // Initial Covariance Estimate (P)
  P_.setZero();
  P_.block(0, 0, 3, 3) = (pErrSigma_Init_m * pErrSigma_Init_m) * I3.cast<double>();
  P_.block(3, 3, 3, 3) = (vErrSigma_Init_mps * vErrSigma_Init_mps) * I3.cast<double>();
  P_.block(6, 6, 2, 2) = (attErrSigma_Init_rad * attErrSigma_Init_rad) * I2.cast<double>();
  P_(8, 8) = ((double)hdgErrSigma_Init_rad * (double)hdgErrSigma_Init_rad);
  P_.block(9, 9, 3, 3) = (aBiasSigma_Init_mps2 * aBiasSigma_Init_mps2) * I3.cast<double>();
  P_.block(12, 12, 3, 3) = (wBiasSigma_Init_rps * wBiasSigma_Init_rps) * I3.cast<double>();
  P_(15, 15) = (double)clockSigma_Init_m * (double)clockSigma_Init_m;
  P_(16, 16) = (double)clockrateSigma_Init_mps *  (double)clockrateSigma_Init_mps;
}

void uNavINS::Initialize(Vector3f wMeas_B_rps, Vector3f aMeas_B_mps2, Vector3f magMeas_B_uT,
                          MatrixXd &gnss_measurement)
{
  // Initialize some matrices
  // Earth rotation vector and matrix 
  omega_ie(0) = 0.0;
  omega_ie(1) = 0.0;
  omega_ie(2) = OMEGA_DOT_EARTH;
  Omega_ie = Skew(omega_ie); 


  // Initialize Position and Velocity (E), GNSS clock offset/phase shift
  pEst_E_m_.setZero();
  vEst_E_mps_.setZero();
  GNSS_LS_pos_vel(gnss_measurement, pEst_E_m_, vEst_E_mps_, clockBias_m_, clockRateBias_mps_);
  // Initialize pEst_D_rrm_ and vEst_L_mps_
  cout << "\n Initialization using LS:" << endl;
  cout.precision(17);

  // pEst_E_m_(0) = 1.094224849500685e6; 
  // pEst_E_m_(1) = -4.355558745064588e6;
  // pEst_E_m_(2) =  4.513941214495038e6;
  // vEst_E_mps_(0) = -10.085547370687761;
  // vEst_E_mps_(1) =  -3.933205369001530;
  // vEst_E_mps_(2) =  -1.231888248259365;
  // clockBias_m_ = 1.000076402016978e+04;
  // clockRateBias_mps_ = 99.974764203471182;
  cout << "pEst_E_m_:" << pEst_E_m_.transpose() << endl;
  cout << "vEst_E_mps_:" << vEst_E_mps_.transpose() << endl; // questionable vEst, check GNSS_LS_pos_vel calc later 
  cout << "clockBias_m: " << clockBias_m_ << endl;
  cout << "clockRateBias_mps " <<  clockRateBias_mps_ << endl; 
  
  VectorXd pos_vel_ned = pv_ECEF_to_NED(pEst_E_m_, vEst_E_mps_);
  pEst_D_rrm_ = pos_vel_ned.segment(0, 3); // Position in LLA (rad, rad, m)
  vEst_L_mps_ = (pos_vel_ned.segment(3, 3)).cast<float>(); // Velocity in NED
  cout << "pEst_D_rrm_: " << pEst_D_rrm_.transpose() << endl; 
  cout << "vEst_L_mps_: " << vEst_L_mps_.transpose() << endl; 

  // Initialize sensor biases
  wBias_rps_ = wMeas_B_rps;
  aBias_mps2_.setZero();
  
  // hard coded biases at 10 sec time stamp for test4.mat
  // wBias_rps_(0) =  -0.202723255829531e-10;
  // wBias_rps_(1) =  -0.144987636871472e-10;
  // wBias_rps_(2) =  -0.000378539688904e-10;
  // aBias_mps2_(0) = 0.372511816993541e-4; 
  // aBias_mps2_(1) = -0.716534650969418e-4; 
  // aBias_mps2_(2) = -0.097868994227939e-4; 
  
  // New Specific forces and Rotation Rate
  aEst_B_mps2_ = aMeas_B_mps2 - aBias_mps2_;
  wEst_B_rps_ = wMeas_B_rps - wBias_rps_;
  
  // Initial attitude, roll and pitch
  Vector3f aEst_B_nd = aEst_B_mps2_ / aEst_B_mps2_.norm(); // Normalize to remove the 1g sensitivity
  euler_BL_rad_(1) = asinf(aEst_B_nd(0));
  euler_BL_rad_(0) = -asinf(aEst_B_nd(1) / cosf(euler_BL_rad_(1)));
  
  // Magnetic horizontal Y (right side) and X (forward) corrections due to roll and pitch angle
  Vector3f magMeas_B_nd = magMeas_B_uT / magMeas_B_uT.norm();
  float magY = magMeas_B_nd(1) * cosf(euler_BL_rad_(0)) - magMeas_B_nd(2) * sinf(euler_BL_rad_(0));
  float magX = magMeas_B_nd(0) * cosf(euler_BL_rad_(1)) + (magMeas_B_nd(1) * sinf(euler_BL_rad_(0)) + magMeas_B_nd(2) * cosf(euler_BL_rad_(0))) * sinf(euler_BL_rad_(1));

  // Estimate initial heading
  euler_BL_rad_(2) = -atan2f(magY, magX);

  // Euler to quaternion
  // euler_BL_rad_(0) =   0.0086;
  // euler_BL_rad_(1) =   0.0128;
  // euler_BL_rad_(2) =  -1.7157;
  quat_BL_ = Euler2Quat(euler_BL_rad_);
  cout << "Initial atti: " << euler_BL_rad_(0)*180/3.1415 << " " << euler_BL_rad_(1)*180/3.1415  << " " << euler_BL_rad_(2)*180/3.1415 << endl; 
  cout << "\n";
  // set initialized flag
  initialized_ = true;
}

//void uNavINS::Update(uint64_t t_us, unsigned long timeWeek, Vector3f wMeas_B_rps, Vector3f aMeas_B_mps2, Vector3f ////magMeas_B_uT, Vector3d pMeas_D_rrm, Vector3f vMeas_L_mps)
void uNavINS::Update(uint64_t t_us, unsigned long timeWeek, 
                     Vector3f wMeas_B_rps, Vector3f aMeas_B_mps2, 
                     Vector3f magMeas_B_uT, MatrixXd &gnss_measurement)
{
  // change in time
  dt_s_ = ((float)(t_us - tPrev_us_)) / 1e6;
  tPrev_us_ = t_us;

  // Catch large dt
  if (dt_s_ > 0.1)
  {
    dt_s_ = 0.1;
  }
  
  // A-priori accel and rotation rate estimate
  aEst_B_mps2_ = aMeas_B_mps2 - aBias_mps2_;
  wEst_B_rps_ = wMeas_B_rps - wBias_rps_;

  // Kalman Time Update (Prediction)
  TimeUpdateNED();
  //TimeUpdateECEF();
  

  // Gps measurement update, if TOW increased
  if ((timeWeek - timeWeekPrev_) > 0)
  {
    timeWeekPrev_ = timeWeek;

    // Process raw GNSS measurements (from ephemeris to a no_meas by 8 matrix )
    // MatrixXd gnss_measurement;
    // gnss_measurement = getGNSSmeasurement(gnss_raw_measurement, timeWeek);

    // Kalman Measurement Update
    // cout << "Time (Meas): " << timeWeek/100 << " s" << endl;
    MeasUpdate17NED(gnss_measurement);
    // MeasUpdate17ECEF(gnss_measurement);
    // MeasSeqUpdate17(gnss_measurement);

    // Post-priori accel and rotation rate estimate, biases updated in MeasUpdate()
    aEst_B_mps2_ = aMeas_B_mps2 - aBias_mps2_;
    wEst_B_rps_ = wMeas_B_rps - wBias_rps_;
  
  }

}



// Process raw gnss measurement
// MatrixXd uNavINS::getGNSSmeasurement(GNSS_raw_measurement gnss_raw_measurement, unsigned long timeWeek)
// {
//   // pre-allocate matrix for the processed gnss measurement
//   //int no_sat= gnss_raw_measurement.rows();
//   int no_sat= 10;
//   MatrixXd gnss_measurement(no_sat, 8);
//   VectorXd pos_vel_Sat_ecef(6);

//   float time = (float)timeWeek;
//   for (int i = 0; i < no_sat; ++i)
//   {
//     uint32_t TOW = 0; //TOW = gnss_raw_measurement(i,whateverTow is)
//     uint8_t L2 = 0;
//     uint16_t week_No = 0;
//     uint8_t L2_Flag = 0;
//     uint8_t SV_Acc = 0;
//     uint8_t SV_Hlth = 0;
//     double T_GD = 0;
//     uint16_t IODC = 0;
//     double t_OC = 0;
//     int8_t a_f2 = 0;
//     double a_f1 = 0;
//     double a_f0 = 0;
//     uint8_t IODE = 0;
//     double C_rs = 0;
//     double delta_n = 0;
//     double M_0 = 0;
//     double C_uc = 0;
//     double ecc = 0;
//     double C_us = 0;
//     double sqrt_A = 0;
//     double t_OE = 0;
//     double C_ic = 0;
//     double Omega_0 = 0;
//     double C_is = 0;
//     double i_0 = 0;
//     double C_rc = 0;
//     double omega = 0;
//     double Omega_dot = 0;
//     double IDOT = 0;
//     double doppler = 0;

//     pos_vel_Sat_ecef = EphemerisData2Satecef(time, TOW, L2, week_No, L2_Flag,
//                                              SV_Acc, SV_Hlth, T_GD, IODC, t_OC, a_f2,
//                                              a_f1, a_f0, IODE, C_rs, delta_n,
//                                              M_0, C_uc, ecc, C_us, sqrt_A,
//                                              t_OE, C_ic, Omega_0, C_is,
//                                              i_0, C_rc, omega, Omega_dot, IDOT);

//     //    pseudorange, peudorange rate
//     double pseudorange;
//     double lambda = 2*c / (1575.42e6); // L1
//     //double lambda = 2*c / (1227.60e6);            // L2
//     double pseudorange_rate = lambda * doppler; //
//     // record data in the row
//     gnss_measurement(i, 0) = pseudorange; //gnss_raw_measurement(i,1);
//     gnss_measurement(i, 1) = 0;           //-2000;
//     gnss_measurement(i, 2) = pos_vel_Sat_ecef[0];
//     gnss_measurement(i, 3) = pos_vel_Sat_ecef[1];
//     gnss_measurement(i, 4) = pos_vel_Sat_ecef[2];
//     gnss_measurement(i, 5) = pos_vel_Sat_ecef[3];
//     gnss_measurement(i, 6) = pos_vel_Sat_ecef[4];
//     gnss_measurement(i, 7) = pos_vel_Sat_ecef[5];
//   }

//   return gnss_measurement;
// }

void uNavINS::TimeUpdateNED()
{
  cout << "\n ----------------Time Update -------------" << endl; 
  // Attitude Update
  Quaternionf dQuat_BL = Quaternionf(1.0, 0.5f * wEst_B_rps_(0) * dt_s_, 0.5f * wEst_B_rps_(1) * dt_s_, 0.5f * wEst_B_rps_(2) * dt_s_);
  quat_BL_ = (quat_BL_ * dQuat_BL).normalized();

  // Avoid quaternion flips sign
  if (quat_BL_.w() < 0)
  {
    quat_BL_ = Quaternionf(-quat_BL_.w(), -quat_BL_.x(), -quat_BL_.y(), -quat_BL_.z());
  }
  euler_BL_rad_ = Quat2Euler(quat_BL_);
  cout << "Time Update(atti): " << euler_BL_rad_(0)*180/3.1415 << " " << euler_BL_rad_(1)*180/3.1415  << " " << euler_BL_rad_(2)*180/3.1415 << endl; 

  // Compute DCM (Body to/from NED) Transformations from Quaternion
  Matrix3f T_L2B = Quat2DCM(quat_BL_);
  Matrix3f T_B2L = T_L2B.transpose();

 
  // cout << "Before Time Update(vel_L): " << vEst_L_mps_(0) <<", "<< vEst_L_mps_(1) <<", "<< vEst_L_mps_(2) << endl; 
  cout << "Before Time  Update(pos_D): " << pEst_D_rrm_(0) <<", "<< pEst_D_rrm_(1) <<", "<< pEst_D_rrm_(2) << endl; 

  // Velocity Update
  Vector3f aGrav_mps2 = Vector3f(0.0, 0.0, G);
  vEst_L_mps_ += dt_s_ * (T_B2L * aEst_B_mps2_ + aGrav_mps2);
  // cout << "Time Update(vel_L): " << vEst_L_mps_(0) <<", "<< vEst_L_mps_(1) <<", "<< vEst_L_mps_(2) << endl; 
  // Position Update
  Vector3f pDot_D = L2D_Rate(vEst_L_mps_, pEst_D_rrm_);
  pEst_D_rrm_ += (dt_s_ * pDot_D).cast<double>();
  // cout << "Time Update(pos_D): " << pEst_D_rrm_(0) <<", "<< pEst_D_rrm_(1) <<", "<< pEst_D_rrm_(2) << endl; 

  // Clock Offset Update
  clockBias_m_ += clockRateBias_mps_ * dt_s_; 
 
  // Assemble the Jacobian (state update matrix) - NED
  Vector3d wEst_B_rps_d = wEst_B_rps_.cast<double>();
  Vector3d aEst_B_mps2_d = aEst_B_mps2_.cast<double>();
  Matrix<double, 17, 17> Fs;
  Fs.setZero();
  Fs.block(0, 3, 3, 3) = I3.cast<double>();
  Fs(5, 2) = -2.0f * G / EARTH_RADIUS;
  Fs.block(3, 6, 3, 3) = -2.0f*T_B2L.cast<double>() * Skew(aEst_B_mps2_d);
  Fs.block(3, 9, 3, 3) = -T_B2L.cast<double>();
  Fs.block(6, 6, 3, 3) = -Skew(wEst_B_rps_d);
  Fs.block(6, 12, 3, 3) = -0.5f*I3.cast<double>();
  Fs.block(9, 9, 3, 3) =   -1.0f / aMarkovTau_s * I3.cast<double>();   // ... Accel Markov Bias
  Fs.block(12, 12, 3, 3) = -1.0f / wMarkovTau_s * I3.cast<double>(); // ... Rotation Rate Markov Bias
  Fs(15,16)  = 1.0f;

  // State Transition Matrix Eq. (14.50) - first order approx 
  Matrix<double, 17, 17> PHI = I17.cast<double>() + Fs * dt_s_;

  // Process Noise Covariance (Discrete approximation)
  Matrix<double, 17, 14> Gs;
  Gs.setZero();
  Gs.block(3, 0, 3, 3) =     -T_B2L.cast<double>();
  Gs.block(6, 3, 3, 3) =   -0.5f*I3.cast<double>();
  Gs.block(9, 6, 3, 3) =         I3.cast<double>();
  Gs.block(12, 9, 3, 3) =        I3.cast<double>();
  Gs.block(15, 12, 2, 2) =       I2.cast<double>();
  
  // Discrete Process Noise
  Matrix<double, 17, 17> Q;
  Q.setZero();
  // Q.block(3, 3, 3, 3) =  (aNoiseSigma_mps2 * aNoiseSigma_mps2) * I3.cast<double>()*(double)dt_s_; 
  // Q.block(6, 6, 3, 3) = (wNoiseSigma_rps * wNoiseSigma_rps) * I3.cast<double>()*(double)dt_s_; 
  // Q.block(9, 9, 3, 3) = 2.0f * (aMarkovSigma_mps2 * aMarkovSigma_mps2) / aMarkovTau_s * I3.cast<double>()*(double)dt_s_; 
  // Q.block(12, 12, 3, 3) = 2.0f * (wMarkovSigma_rps * wMarkovSigma_rps) / wMarkovTau_s * I3.cast<double>()*(double)dt_s_; 
  // Q(15,15) = 1.0f * (double)pseudorangeNoiseSigma_m * (double)pseudorangeNoiseSigma_m*(double)dt_s_; 
  // Q(16,16) = 1.0f * (double)pseudorangeRateNoiseSigma_mps * (double)pseudorangeRateNoiseSigma_mps*(double)dt_s_; 
  Q = PHI * dt_s_ * Gs * Rw_ * Gs.transpose();
  Q = 0.5f * (Q + Q.transpose());
  // cout << "Q: " << (Q.diagonal()).transpose() << endl;
  // Covariance Time Update
  // P_ = PHI * (P_ + 0.5*Q) * PHI.transpose() + 0.5*Q;
  P_ = PHI * P_ * PHI.transpose() + Q;
  P_ = 0.5 * (P_ + P_.transpose());
}

// Measurement Update (NED)
void uNavINS::MeasUpdate17NED(MatrixXd &gnss_measurement)
{
  // 
  cout << "\n -----------This is EKF17 (NED) Meas. Update: --------" << endl; 
  // Determine number of available satellite
  int no_sat = gnss_measurement.rows();
  // Range Estimate
  MatrixXd rangeEst_m_(no_sat, 1);
  // Range rate Estimate
  MatrixXd rangeRateEst_mps_(no_sat, 1);
  // Range Error
  MatrixXd rangeErr_m(no_sat, 1);
  // Range rate Error
  MatrixXd rangeRateErr_mps(no_sat, 1);
  // Allocate measurement matrix
  MatrixXd H_(2 * no_sat, 17);
  H_.setZero();
  // Allocate measurement noise matrix
  MatrixXd R_(2 * no_sat, 2 * no_sat);
  R_.setIdentity();
  // Allocate space for Rew and Rns 
  double Rew, Rns;
  EarthRad(pEst_D_rrm_(0), &Rew, &Rns);

  // Convert position and velocity to E frame
  pEst_E_m_ = D2E(pEst_D_rrm_); 
  Vector3f vEst_E_mps_f = L2E(vEst_L_mps_, (pEst_D_rrm_).cast<float>());
  vEst_E_mps_ = vEst_E_mps_f.cast<double>();
  // cout << "(correct) pEst_E_m_: " << pEst_E_m_ << endl; 
  // cout << "(correct) vEst_E_m_: " << vEst_E_mps_  << endl;  
  // loop measurement
   for (int i = 0; i < no_sat; ++i)
  {
    //cout << i  << "th satellite: \n"; 
    // predict approx range
    Vector3d delta_r;
    Vector3d pMeas_E_m_(gnss_measurement(i, 2), gnss_measurement(i, 3), gnss_measurement(i, 4));
    Vector3d vMeas_E_mps_(gnss_measurement(i, 5), gnss_measurement(i, 6), gnss_measurement(i, 7));
    
    
    delta_r = pMeas_E_m_ - pEst_E_m_;
    double approx_range =  delta_r.norm();
    // cout << "delta_r: " << delta_r(0) << " , " << delta_r(1) << ", " << delta_r(2) << endl;
    // cout << "approx_range: " << approx_range  << endl; 
    // Calculate frame rotation during signal transit time using (Grove2nd:8.36)
    Matrix3d T_E2I; 
    T_E2I.setZero();
    T_E2I(0, 0) = 1;
    T_E2I(0, 1) =  OMEGA_DOT_EARTH * approx_range / c;
    T_E2I(0, 2) = 0;
    T_E2I(1, 0) = -OMEGA_DOT_EARTH * approx_range / c;
    T_E2I(1, 1) = 1;
    T_E2I(1, 2) = 0;
    T_E2I(2, 0) = 0;
    T_E2I(2, 1) = 0;
    T_E2I(2, 2) = 1;

    // Predict pseudo-range using (Grove2nd:9.165)
    delta_r = T_E2I* pMeas_E_m_ - pEst_E_m_;
    double range =  delta_r.norm();

    // cout << "delta_r: " << delta_r(0) << " , " << delta_r(1) << ", " << delta_r(2) << endl;
    // cout << "range: " << range  << endl;

    rangeEst_m_(i, 0) = range + clockBias_m_;
    
    // Formulate range innovation
    rangeErr_m(i, 0) = gnss_measurement(i, 0) - rangeEst_m_(i, 0);
    // cout << "gnss_measurement(i, 0): " << gnss_measurement(i, 0) << endl;
    // cout << "rangeEst_m_(i, 0): " << rangeEst_m_(i, 0) << endl;
    // cout << "rangeErr_m(i, 0): " << rangeErr_m(i, 0) << endl; 
    // Predict line of sight in ECI frame
    Vector3d u_as_E = delta_r / range;
    //cout << "u_as_E:" << u_as_E(0) << " ," << u_as_E(1) << ", " << u_as_E(2) << endl; 
    //cout << "pEst_D_rrm_: " << pEst_D_rrm_(0) << ", " << pEst_D_rrm_(1) << ", " << pEst_D_rrm_(2) << endl;
    Vector3d u_as_L = E2L(u_as_E, pEst_D_rrm_);
    //cout << "u_as_L:" << u_as_L(0) << " ," << u_as_L(1) << ", " << u_as_L(2) << endl; 

    // Predict pseudo-range rate using (9.165)
    double range_rate = u_as_E.transpose() * (T_E2I * (vMeas_E_mps_ + Omega_ie * pMeas_E_m_) 
                       - (vEst_E_mps_  + Omega_ie * pEst_E_m_));
    
    //cout << "u_as_E: " << u_as_E.transpose() << endl; 
    //cout << "T_E2I: " << T_E2I.cast<float>() << endl; 
    //cout << "pMeas_E_m_: " << pMeas_E_m_.transpose() << ", vMeas_E_mps_ " << vMeas_E_mps_.transpose() << endl;
    rangeRateEst_mps_(i, 0) = range_rate  + clockRateBias_mps_;

    // Formulate range-rate innovation
    rangeRateErr_mps(i, 0) = gnss_measurement(i, 1) - rangeRateEst_mps_(i, 0);
    // cout << "rangeRate: " << range_rate << endl;
    // cout << "rangeRateEst_mps_(i, 0): " << rangeRateEst_mps_(i, 0) << endl;
    // cout << "rangeRate Meas: " << gnss_measurement(i, 1) << endl; 
    // cout << endl; 
   
     // set H in L frame - Eq.(9.166) 
    H_(i, 0) =  -u_as_L(0);
    H_(i, 1) =  -u_as_L(1);
    H_(i, 2) =  -u_as_L(2);
    H_(i, 15) = 1;
    H_(i + no_sat, 3) =  -u_as_L(0);
    H_(i + no_sat, 4) =  -u_as_L(1);
    H_(i + no_sat, 5) =  -u_as_L(2);
    H_(i + no_sat, 16) = 1;
  
    // set R
    R_(i, i) = (double)pseudorangeNoiseSigma_m * (double)pseudorangeNoiseSigma_m;
    R_(i + no_sat, i + no_sat) = (double)pseudorangeRateNoiseSigma_mps * (double)pseudorangeRateNoiseSigma_mps;
  }

  // Create measurement Y, as Error between Measures and Outputs
  VectorXd y(2 * no_sat, 1);
  y.setZero();
  y.segment(0, no_sat) = rangeErr_m;
  y.segment(no_sat, no_sat) = rangeRateErr_mps;
  // cout <<"y: "<< y << endl; 
  // Innovation covariance
  MatrixXd S_(2 * no_sat, 2 * no_sat);
  S_.setZero();
  S_ = H_ * P_ * H_.transpose() + R_;
  // cout << "S: " << S_ << endl;
  // cout << "P_minus: " << P_ << endl;
  // Kalman gain
  MatrixXd K(17, 2 * no_sat);
  K.setZero();
  K = P_ * H_.transpose() * S_.inverse();
  // cout << "K: " << K << endl;
  // Covariance update, P = (I + K * H) * P * (I + K * H)' + K * R * K'
  MatrixXd I_KH(17, 17); 
  I_KH.setZero();
  I_KH = I17.cast<double>() - K * H_; // temp
  P_ = I_KH * P_ * I_KH.transpose() + K * R_ * K.transpose();

  // State update, x = K * y
  VectorXd x(17, 1);
  x.setZero();
  x = K * y;
  // cout <<"x: "<< x << endl; 
  // Pull apart x terms to update the Position, velocity, orientation, and sensor biases
  Vector3d pDeltaEst_D =  x.segment(0, 3);    // Position Deltas in ECEF
  Vector3d vDeltaEst_L =  x.segment(3, 3);    // Velocity Deltas in ECEF
  Vector3d quatDelta   =  x.segment(6, 3);    // Quaternion Delta
  Vector3d aBiasDelta  =  x.segment(9, 3);    // Accel Bias Deltas
  Vector3d wBiasDelta  =  x.segment(12, 3);   // Rotation Rate Bias Deltas
  double clockBiasDelta =           x(15,0);   // Clock Offset Deltas
  double clockBiasRateDelta =       x(16,0);   // Clock Phase Deltas
  // Position update
  pEst_D_rrm_(2) += -pDeltaEst_D(2);
  pEst_D_rrm_(0) +=  pDeltaEst_D(0) / (Rew + pEst_D_rrm_(2));
  pEst_D_rrm_(1) +=  pDeltaEst_D(1) / (Rns + pEst_D_rrm_(2)) / cos(pEst_D_rrm_(0));
  
  // Velocity update
  vEst_L_mps_    += vDeltaEst_L.cast<float>();
  
  // Attitude correction
  Quaternionf dQuat_BL = Quaternionf(1.0, quatDelta(0), quatDelta(1), quatDelta(2));
  quat_BL_ = (quat_BL_ * dQuat_BL).normalized();
  // cout << "Meas. Update(atti): " << quat_BL_.w() << " " << quat_BL_.x() << " " << quat_BL_.y() << " " << quat_BL_.z()  << endl; 
  
   //Grove Euler method 
  // Matrix3f T_L2B = Quat2DCM(quat_BL_);
  // Matrix3f T_B2L = T_L2B.transpose();
   
  // float cos_lat = cos(pEst_D_rrm_(0));
  // float sin_lat = sin(pEst_D_rrm_(0));
  // float cos_lon = cos(pEst_D_rrm_(1));
  // float sin_lon = sin(pEst_D_rrm_(1));

  // Matrix3f T_E2L;                
  // T_E2L(0,0) = -sin_lat*cos_lon;	T_E2L(0,1) = -sin_lat*sin_lon;	T_E2L(0,2) =  cos_lat;
  // T_E2L(1,0) =         -sin_lon;	T_E2L(1,1) =          cos_lon;	T_E2L(1,2) =        0;
  // T_E2L(2,0) = -cos_lat*cos_lon;	T_E2L(2,1) = -cos_lat*sin_lon;	T_E2L(2,2) = -sin_lat;
  
  // Matrix3f T_B2E = T_E2L.transpose()*T_B2L;
  // Vector3f quatDelta_f = quatDelta.cast<float>();
  // T_B2E = (I3 - Skew(quatDelta_f))*T_B2E; 
  // T_B2L =  T_E2L * T_B2E;
  // euler_BL_rad_  = DCM2Eul(T_B2L.transpose());
  // quat_BL_ = Euler2Quat(euler_BL_rad_);
  
  // Euler angles from quaternion
  euler_BL_rad_ = Quat2Euler(quat_BL_);
  cout << "Meas. Update(atti): " << euler_BL_rad_(0)*180/3.1415 << " " << euler_BL_rad_(1)*180/3.1415  << " " << euler_BL_rad_(2)*180/3.1415 << endl; 
  cout << "Meas. Update(vel): " << vEst_L_mps_(0) <<", "<< vEst_L_mps_(1) <<", "<< vEst_L_mps_(2) << endl; 
  cout << "Meas. Update(pos): " << pEst_D_rrm_(0) <<", "<< pEst_D_rrm_(1) <<", "<< pEst_D_rrm_(2) << endl; 
  // Update biases from states
  aBias_mps2_ += aBiasDelta.cast<float>();
  wBias_rps_  += wBiasDelta.cast<float>();
  clockBias_m_ += clockBiasDelta;           // clock bias
  clockRateBias_mps_ += clockBiasRateDelta; // clock rate bias (drift)
  cout << "clockBias(Meas Update): " << clockBias_m_ << endl;
  cout << "clockDrift(meas update): " << clockRateBias_mps_ << endl;
}

void uNavINS::TimeUpdateECEF()
{
  cout << "\n ----------------Time Update -------------" << endl; 
  // Attitude Update
  Quaternionf dQuat_BL = Quaternionf(1.0, 0.5f * wEst_B_rps_(0) * dt_s_, 0.5f * wEst_B_rps_(1) * dt_s_, 0.5f * wEst_B_rps_(2) * dt_s_);
  quat_BL_ = (quat_BL_ * dQuat_BL).normalized();

  // Avoid quaternion flips sign
  if (quat_BL_.w() < 0)
  {
    quat_BL_ = Quaternionf(-quat_BL_.w(), -quat_BL_.x(), -quat_BL_.y(), -quat_BL_.z());
  }
  euler_BL_rad_ = Quat2Euler(quat_BL_);
  cout << "Time Update(atti): " << euler_BL_rad_(0)*180/3.1415 << " " << euler_BL_rad_(1)*180/3.1415  << " " << euler_BL_rad_(2)*180/3.1415 << endl; 

  // Compute DCM (Body to/from NED) Transformations from Quaternion
  Matrix3f T_L2B = Quat2DCM(quat_BL_);
  Matrix3f T_B2L = T_L2B.transpose();
   
  float cos_lat = cos(pEst_D_rrm_(0));
  float sin_lat = sin(pEst_D_rrm_(0));
  float cos_lon = cos(pEst_D_rrm_(1));
  float sin_lon = sin(pEst_D_rrm_(1));

  Matrix3f T_E2L;                
  T_E2L(0,0) = -sin_lat*cos_lon;	T_E2L(0,1) = -sin_lat*sin_lon;	T_E2L(0,2) =  cos_lat;
  T_E2L(1,0) =         -sin_lon;	T_E2L(1,1) =          cos_lon;	T_E2L(1,2) =        0;
  T_E2L(2,0) = -cos_lat*cos_lon;	T_E2L(2,1) = -cos_lat*sin_lon;	T_E2L(2,2) = -sin_lat;
  
  Matrix3f T_B2E = T_E2L.transpose()*T_B2L;
  // Determine the Earth rotation over the update interval
  double alpha_ie = OMEGA_DOT_EARTH * (double)dt_s_;
  Vector3d Alpha_ie;
  Alpha_ie(0) = 0;
  Alpha_ie(1) = 0;
  Alpha_ie(2) = alpha_ie; 
  Matrix3d T_EARTH; 
  T_EARTH(0,0) = cos(alpha_ie);	T_EARTH(0,1) = sin(alpha_ie); T_EARTH(0,2) =    0;
  T_EARTH(1,0) =-sin(alpha_ie);	T_EARTH(1,1) = cos(alpha_ie);	T_EARTH(1,2) =    0;
  T_EARTH(2,0) =             0;	T_EARTH(2,1) = 0;	            T_EARTH(2,2) =    1;
  // Calculate attitude increment, magnitude, and skew-symmetric matrix
  Vector3d alpha_ib_b = wEst_B_rps_.cast<double>() * (double)dt_s_; 
  double mag_alpha = sqrt(alpha_ib_b.transpose()*alpha_ib_b);
  Matrix3d Alpha_ib_b = Skew(alpha_ib_b);
  // Obtain coordinate transformation matrix from the new attitude w.r.t. an
  // inertial frame to the old using Rodrigues' formula
  Matrix3d C_new_old;
  if (mag_alpha > 1e-8)
  {
      C_new_old = I3.cast<double>() + sin(mag_alpha)/mag_alpha*Alpha_ib_b + 
                            (1-cos(mag_alpha))/(mag_alpha*mag_alpha)*Alpha_ib_b*Alpha_ib_b; 
  }
  else
  {
      C_new_old = I3.cast<double>() + Alpha_ib_b;
  }
  // Attitude Update
  T_B2E = T_EARTH.cast<float>()*T_B2E*C_new_old.cast<float>();
  // T_B2L =  T_E2L * T_B2E;
  // euler_BL_rad_  = DCM2Eul(T_B2L.transpose());
  // cout << "Time Update using new code(atti): " << euler_BL_rad_(0)*180/3.1415 << " " << euler_BL_rad_(1)*180/3.1415  << " " << euler_BL_rad_(2)*180/3.1415 << endl; 
  // SPECIFIC FORCE FRAME TRANSFORMATION
  // Calculate the average body-to-ECEF-frame coordinate transformation
  // matrix over the update interval using (5.84) and (5.85)
  Matrix3d ave_C_b_e;
  if (mag_alpha > 1e-8)
  {
      ave_C_b_e =  T_B2E.cast<double>()*(I3.cast<double>() + (1-cos(mag_alpha))/(mag_alpha*mag_alpha)*Alpha_ib_b +
                               (1-sin(mag_alpha)/mag_alpha)/(mag_alpha*mag_alpha)*Alpha_ib_b*Alpha_ib_b)
                                   - 0.5*Skew(Alpha_ie)*T_B2E.cast<double>();
  }
  else
  {
      ave_C_b_e = T_B2E.cast<double>() - 0.5*Skew(Alpha_ie)*T_B2E.cast<double>();
  }
  //cout << "ave_C_b_e: " << ave_C_b_e << endl;
  // Velocity Update
  Vector3d aGrav_E_mps2 = Gravity_ECEF(pEst_E_m_); 
  //cout << "aGrav_E_mps2: " << aGrav_E_mps2 << endl;
  Vector3d vEst_E_prev_mps_ = vEst_E_mps_;
  //cout << "aEst_B_mps2_: " << aEst_B_mps2_ << endl;
  cout << "Before Time Update(vel_E): " << vEst_E_mps_(0) <<", "<< vEst_E_mps_(1) <<", "<< vEst_E_mps_(2) << endl; 

  vEst_E_mps_ +=  dt_s_ * (ave_C_b_e*aEst_B_mps2_.cast<double>() + aGrav_E_mps2-2.0f*Omega_ie*vEst_E_mps_); 
 

  cout << "Time Update(vel_E): " << vEst_E_mps_(0) <<", "<< vEst_E_mps_(1) <<", "<< vEst_E_mps_(2) << endl; 
  // Position Update 
  cout << "Before Time Update(pos_E): " << pEst_E_m_(0) <<", "<< pEst_E_m_(1) <<", "<< pEst_E_m_(2) << endl; 
  pEst_E_m_  += (vEst_E_mps_ +  vEst_E_prev_mps_)* (double)dt_s_ *0.5; 
  cout << "Time Update(pos_E): " << pEst_E_m_(0) <<", "<< pEst_E_m_(1) <<", "<< pEst_E_m_(2) << endl; 
  // Convert E to D and L 
  VectorXd pos_vel_ned = pv_ECEF_to_NED(pEst_E_m_, vEst_E_mps_);
  pEst_D_rrm_ = pos_vel_ned.segment(0, 3); // Position in LLA (rad, rad, m)
  vEst_L_mps_ = (pos_vel_ned.segment(3, 3)).cast<float>(); // Velocity in NED
  // cout << "New Time Update(vel_L): " << vEst_L_mps_(0) <<", "<< vEst_L_mps_(1) <<", "<< vEst_L_mps_(2) << endl; 
  cout << "Time Update(pos_D): " << pEst_D_rrm_(0) <<", "<< pEst_D_rrm_(1) <<", "<< pEst_D_rrm_(2) << endl; 
   
    
  // Velocity Update
  // Vector3f aGrav_mps2 = Vector3f(0.0, 0.0, G);
  // vEst_L_mps_ += dt_s_ * (T_B2L * aEst_B_mps2_ + aGrav_mps2);
  // cout << "Time Update(vel_L): " << vEst_L_mps_(0) <<", "<< vEst_L_mps_(1) <<", "<< vEst_L_mps_(2) << endl; 
  // Position Update
  // Vector3f pDot_D = L2D_Rate(vEst_L_mps_, pEst_D_rrm_);
  // pEst_D_rrm_ += (dt_s_ * pDot_D).cast<double>();
  // cout << "Time Update(pos_D): " << pEst_D_rrm_(0) <<", "<< pEst_D_rrm_(1) <<", "<< pEst_D_rrm_(2) << endl; 

  // Clock Offset Update
  clockBias_m_ += clockRateBias_mps_ * dt_s_; 
 
  // Assemble the Jacobian (state update matrix) - NED
  // Matrix<float, 17, 17> Fs;
  // Fs.setZero();
  // Fs.block(0, 3, 3, 3) = I3;
  // Fs(5, 2) = -2.0f * G / EARTH_RADIUS;
  // Fs.block(3, 6, 3, 3) = -2.0f * T_B2L * Skew(aEst_B_mps2_);
  // Fs.block(3, 9, 3, 3) = -T_B2L;
  // Fs.block(6, 6, 3, 3) = -Skew(wEst_B_rps_);
  // Fs.block(6, 12, 3, 3) = -0.5f * I3;
  // Fs.block(9, 9, 3, 3) = -1.0f / aMarkovTau_s * I3;   // ... Accel Markov Bias
  // Fs.block(12, 12, 3, 3) = -1.0f / wMarkovTau_s * I3; // ... Rotation Rate Markov Bias
  // Fs(16,15) = 1.0f;

  // State Transition Matrix Eq. (14.50) - first order approx 
  // Matrix<float, 17, 17> PHI = I17 + Fs * dt_s_;
  // cout << "aEst_B_mps2_: " << aEst_B_mps2_ << endl; 
  // cout << "T_B2E: " << T_B2E << endl; 

  // Matrix<double, 17, 17> PHI;
  // PHI.setIdentity();  
  // PHI.block(0, 3, 3, 3) = I3.cast<double>()*dt_s_;
  // Vector3d aEst_E_mps2_ = T_B2E.cast<double>()*aEst_B_mps2_.cast<double>();
  // double geocentric_radius = EARTH_RADIUS/sqrt(1-pow(Eccentricity*sin(pEst_D_rrm_(0)),2))* 
  // sqrt(pow(cos(pEst_D_rrm_(0)),2) + pow(1 - pow(Eccentricity,2),2)*pow(sin(pEst_D_rrm_(0)),2));
  // PHI.block(3, 0, 3, 3) = - dt_s_ * 2.0f* Gravity_ECEF(pEst_E_m_)/
  //                         geocentric_radius*pEst_E_m_.transpose()/sqrt(pEst_E_m_.transpose()*pEst_E_m_);
  // // double temp = Gravity_ECEF(pEst_E_m_);
  // // PHI(5, 2) = - dt_s_ * 2.0f* temp(2)/
  // //                         geocentric_radius*pEst_E_m_(2)/sqrt(pEst_E_m_(2)*pEst_E_m_(2));
  // PHI.block(3, 3, 3, 3)   = I3.cast<double>() -  2*Omega_ie*(double)dt_s_;
  // PHI.block(3, 6, 3, 3)   = -2*dt_s_ * Skew(aEst_E_mps2_);
  // PHI.block(3, 9, 3, 3)   = -T_B2E.cast<double>() * dt_s_;
  // PHI.block(6, 6, 3, 3)   = I3.cast<double>() - Omega_ie*(double)dt_s_; 
  // PHI.block(6, 12, 3, 3)  = -0.5f*I3.cast<double>() * dt_s_;
  // PHI.block(9, 9, 3, 3)   = I3.cast<double>() - 1.0f / aMarkovTau_s * I3.cast<double>()* dt_s_;   // ... Accel Markov Bias
  // PHI.block(12, 12, 3, 3) = I3.cast<double>() - 1.0f / wMarkovTau_s * I3.cast<double>()* dt_s_; // ... Rotation Rate Markov Bias
  // PHI(15,16)              = (double)dt_s_;

  // Euler version (working!)
  Matrix<double, 17, 17> PHI;
  PHI.setIdentity();  
  PHI.block(0, 3, 3, 3) = I3.cast<double>()*dt_s_;
  Vector3d aEst_E_mps2_ = T_B2E.cast<double>()*aEst_B_mps2_.cast<double>();
  double geocentric_radius = EARTH_RADIUS/sqrt(1-pow(Eccentricity*sin(pEst_D_rrm_(0)),2))* 
  sqrt(pow(cos(pEst_D_rrm_(0)),2) + pow(1 - pow(Eccentricity,2),2)*pow(sin(pEst_D_rrm_(0)),2));
  PHI.block(3, 0, 3, 3) = - dt_s_ * 2.0f* Gravity_ECEF(pEst_E_m_)/
                          geocentric_radius*pEst_E_m_.transpose()/sqrt(pEst_E_m_.transpose()*pEst_E_m_);
  PHI.block(3, 3, 3, 3)   = I3.cast<double>() -  2*Omega_ie*(double)dt_s_;
  PHI.block(3, 6, 3, 3)   = -dt_s_ * Skew(aEst_E_mps2_);
  PHI.block(3, 9, 3, 3)   = T_B2E.cast<double>() * dt_s_;
  PHI.block(6, 6, 3, 3)   = I3.cast<double>() - Omega_ie*(double)dt_s_; 
  PHI.block(6, 12, 3, 3)  = T_B2E.cast<double>() * dt_s_;
  PHI.block(9, 9, 3, 3)   = I3.cast<double>() - 1.0f / aMarkovTau_s * I3.cast<double>()* dt_s_;   // ... Accel Markov Bias
  PHI.block(12, 12, 3, 3) = I3.cast<double>() - 1.0f / wMarkovTau_s * I3.cast<double>()* dt_s_; // ... Rotation Rate Markov Bias
  PHI(15,16)              = (double)dt_s_;
  // cout << "PHI" << PHI << endl;

  // cout << "PHI(1:3,:) " << PHI.block(0, 0, 3, 17) << endl;
  // cout << "PHI(4:6,:) " << PHI.block(3, 0, 3, 12) << endl;
  // cout << "PHI(7:9,:) " << PHI.block(6, 0, 3, 15) << endl;
  // Process Noise Covariance (Discrete approximation)
  Matrix<double, 17, 14> Gs;
  Gs.setZero();
  Gs.block(3, 0, 3, 3) =   T_B2E.cast<double>();
  Gs.block(6, 3, 3, 3) = - T_B2E.cast<double>();
  Gs.block(9, 6, 3, 3) = I3.cast<double>();
  Gs.block(12, 9, 3, 3) = I3.cast<double>();
  Gs.block(15, 12, 2, 2) = I2.cast<double>();
  
  // Discrete Process Noise
  Matrix<double, 17, 17> Q;
  Q.setZero();
  // Q.block(3, 3, 3, 3) =  (aNoiseSigma_mps2 * aNoiseSigma_mps2) * I3.cast<double>()*(double)dt_s_; 
  // Q.block(6, 6, 3, 3) = (wNoiseSigma_rps * wNoiseSigma_rps) * I3.cast<double>()*(double)dt_s_; 
  // Q.block(9, 9, 3, 3) = 2.0f * (aMarkovSigma_mps2 * aMarkovSigma_mps2) / aMarkovTau_s * I3.cast<double>()*(double)dt_s_; 
  // Q.block(12, 12, 3, 3) = 2.0f * (wMarkovSigma_rps * wMarkovSigma_rps) / wMarkovTau_s * I3.cast<double>()*(double)dt_s_; 
  // Q(15,15) = 1.0f * (double)pseudorangeNoiseSigma_m * (double)pseudorangeNoiseSigma_m*(double)dt_s_; 
  // Q(16,16) = 1.0f * (double)pseudorangeRateNoiseSigma_mps * (double)pseudorangeRateNoiseSigma_mps*(double)dt_s_; 
  Q = PHI * dt_s_ * Gs * Rw_ * Gs.transpose();
  Q = 0.5f * (Q + Q.transpose());
  // cout << "Q: " << (Q.diagonal()).transpose() << endl;
  // Covariance Time Update
  //P_ = PHI * (P_ + 0.5*Q) * PHI.transpose() + 0.5*Q;
  P_ = PHI * P_ * PHI.transpose() + Q;
  P_ = 0.5 * (P_ + P_.transpose());
}

// Measurement Update (ECEF)
void uNavINS::MeasUpdate17ECEF(MatrixXd &gnss_measurement)
{
  // 
  cout << "\n -----------This is EKF17 (ECEF) Meas. Update: --------" << endl; 
  // Determine number of available satellite
  int no_sat = gnss_measurement.rows();
  // Range Estimate
  MatrixXd rangeEst_m_(no_sat, 1);
  // Range rate Estimate
  MatrixXd rangeRateEst_mps_(no_sat, 1);
  // Range Error
  MatrixXd rangeErr_m(no_sat, 1);
  // Range rate Error
  MatrixXd rangeRateErr_mps(no_sat, 1);
  // Allocate measurement matrix
  MatrixXd H_(2 * no_sat, 17);
  H_.setZero();
  // Allocate measurement noise matrix
  MatrixXd R_(2 * no_sat, 2 * no_sat);
  R_.setIdentity();
  // Allocate space for Rew and Rns 
  double Rew, Rns;
  EarthRad(pEst_D_rrm_(0), &Rew, &Rns);
  // debug
  // pEst_E_m_(0) = 1.094089993171103e6; 
  // pEst_E_m_(1) = -4.355608265999388e6;
  // pEst_E_m_(2) =  4.513925561991856e6;
  // vEst_E_mps_(0) = -14.195782274285234;
  // vEst_E_mps_(1) =  -5.278798999721270;
  // vEst_E_mps_(2) =  -1.503412538563618;
  // clockBias_m_ = 1.100028956763155e+04;
  // clockRateBias_mps_ = 1.000040111980258e+02;
  cout << "pEst_E_m_: " << pEst_E_m_.transpose() << ", vEst_E_mps_" << vEst_E_mps_.transpose() << endl;
  // cout << "gnss: " << gnss_measurement << endl; 
  // loop measurement
  for (int i = 0; i < no_sat; ++i)
  {
    //cout << i  << "th satellite: \n"; 
    // predict approx range
    Vector3d delta_r;
    Vector3d pMeas_E_m_(gnss_measurement(i, 2), gnss_measurement(i, 3), gnss_measurement(i, 4));
    Vector3d vMeas_E_mps_(gnss_measurement(i, 5), gnss_measurement(i, 6), gnss_measurement(i, 7));
    delta_r = pMeas_E_m_ - pEst_E_m_;
    double approx_range =  delta_r.norm();
    // cout << "delta_r: " << delta_r(0) << " , " << delta_r(1) << ", " << delta_r(2) << endl;
    // cout << "approx_range: " << approx_range  << endl; 
    // Calculate frame rotation during signal transit time using (Grove2nd:8.36)
    Matrix3d T_E2I; 
    T_E2I.setZero();
    T_E2I(0, 0) = 1;
    T_E2I(0, 1) =  OMEGA_DOT_EARTH * approx_range / c;
    T_E2I(0, 2) = 0;
    T_E2I(1, 0) = -OMEGA_DOT_EARTH * approx_range / c;
    T_E2I(1, 1) = 1;
    T_E2I(1, 2) = 0;
    T_E2I(2, 0) = 0;
    T_E2I(2, 1) = 0;
    T_E2I(2, 2) = 1;

    // Predict pseudo-range using (Grove2nd:9.165)
    delta_r = T_E2I* pMeas_E_m_ - pEst_E_m_;
    double range =  delta_r.norm();

    // cout << "delta_r: " << delta_r(0) << " , " << delta_r(1) << ", " << delta_r(2) << endl;
    // cout << "range: " << range  << endl;

    rangeEst_m_(i, 0) = range + clockBias_m_;
    
    // Formulate range innovation
    rangeErr_m(i, 0) = gnss_measurement(i, 0) - rangeEst_m_(i, 0);
    // cout << "gnss_measurement(i, 0): " << gnss_measurement(i, 0) << endl;
    // cout << "rangeEst_m_(i, 0): " << rangeEst_m_(i, 0) << endl;
    // cout << "rangeErr_m(i, 0): " << rangeErr_m(i, 0) << endl; 
    // Predict line of sight in ECI frame
    Vector3d u_as_E = delta_r / range;
    //cout << "u_as_E:" << u_as_E(0) << " ," << u_as_E(1) << ", " << u_as_E(2) << endl; 
    //cout << "pEst_D_rrm_: " << pEst_D_rrm_(0) << ", " << pEst_D_rrm_(1) << ", " << pEst_D_rrm_(2) << endl;
    //Matrix<double, 3, 1> u_as_L = E2L(u_as_E.cast<double>(), pEst_D_rrm_);
    //cout << "u_as_L:" << u_as_L(0) << " ," << u_as_L(1) << ", " << u_as_L(2) << endl; 

    // Predict pseudo-range rate using (9.165)
    double range_rate = u_as_E.transpose() * (T_E2I * (vMeas_E_mps_ + Omega_ie * pMeas_E_m_) 
                       - (vEst_E_mps_  + Omega_ie * pEst_E_m_));
    
    //cout << "u_as_E: " << u_as_E.transpose() << endl; 
    //cout << "T_E2I: " << T_E2I.cast<float>() << endl; 
    //cout << "pMeas_E_m_: " << pMeas_E_m_.transpose() << ", vMeas_E_mps_ " << vMeas_E_mps_.transpose() << endl;
    rangeRateEst_mps_(i, 0) = range_rate  + clockRateBias_mps_;

    // Formulate range-rate innovation
    rangeRateErr_mps(i, 0) = gnss_measurement(i, 1) - rangeRateEst_mps_(i, 0);
    // cout << "rangeRate: " << range_rate << endl;
    // cout << "rangeRateEst_mps_(i, 0): " << rangeRateEst_mps_(i, 0) << endl;
    // cout << "rangeRate Meas: " << gnss_measurement(i, 1) << endl; 
    // cout << endl; 
    // set H in E frame 
    H_(i, 0) = u_as_E(0);
    H_(i, 1) = u_as_E(1);
    H_(i, 2) = u_as_E(2);
    H_(i, 15) = 1;
    H_(i + no_sat, 3) = u_as_E(0);
    H_(i + no_sat, 4) = u_as_E(1);
    H_(i + no_sat, 5) = u_as_E(2);
    H_(i + no_sat, 16) = 1;
    // set R
    R_(i, i) = (double)pseudorangeNoiseSigma_m * (double)pseudorangeNoiseSigma_m;
    R_(i + no_sat, i + no_sat) = (double)pseudorangeRateNoiseSigma_mps * (double)pseudorangeRateNoiseSigma_mps;
  }
  // cout <<"H: "<< H_ << endl; 
  // cout <<"R: "<< R_ << endl; 
  // Create measurement Y, as Error between Measures and Outputs
  VectorXd y(2 * no_sat, 1);
  y.setZero();
  y.segment(0, no_sat) = rangeErr_m;
  y.segment(no_sat, no_sat) = rangeRateErr_mps;
  // cout <<"y: "<< y << endl; 
  // Innovation covariance
  MatrixXd S_(2 * no_sat, 2 * no_sat);
  S_.setZero();
  S_ = H_ * P_ * H_.transpose() + R_;
  // cout << "S: " << S_ << endl;
  // cout << "P_minus: " << P_ << endl;
  // Kalman gain
  MatrixXd K(17, 2 * no_sat);
  K.setZero();
  K = P_ * H_.transpose() * S_.inverse();
  // cout << "K: " << K << endl;
  // Covariance update, P = (I + K * H) * P * (I + K * H)' + K * R * K'
  MatrixXd I_KH(17, 17); 
  I_KH.setZero();
  I_KH = I17.cast<double>() - K * H_; // temp
  P_ = I_KH * P_ * I_KH.transpose() + K * R_ * K.transpose();

  // State update, x = K * y
  VectorXd x(17, 1);
  x.setZero();
  x = K * y;
  //cout <<"x: "<< x << endl; 
  // Pull apart x terms to update the Position, velocity, orientation, and sensor biases
  Vector3d pDeltaEst_E =  x.segment(0, 3);    // Position Deltas in ECEF
  Vector3d vDeltaEst_E =  x.segment(3, 3);    // Velocity Deltas in ECEF
  Vector3d quatDelta   =  x.segment(6, 3);    // Quaternion Delta
  Vector3d aBiasDelta  =  x.segment(9, 3);    // Accel Bias Deltas
  Vector3d wBiasDelta  =  x.segment(12, 3);   // Rotation Rate Bias Deltas
  double clockBiasDelta =           x(15,0);   // Clock Offset Deltas
  double clockBiasRateDelta =       x(16,0);   // Clock Phase Deltas
  
  // Position update
  pEst_E_m_  -= pDeltaEst_E;
 
  cout << "Meas. Update(pos_E): " << pEst_E_m_(0) <<", "<< pEst_E_m_(1) <<", "<< pEst_E_m_(2) << endl; 
  
  // Velocity update
  vEst_E_mps_ -= vDeltaEst_E;
  cout << "Meas. Update(vel_E): " << vEst_E_mps_(0) <<", "<< vEst_E_mps_(1) <<", "<< vEst_E_mps_(2) << endl; 
  // convert E to D and L 
  VectorXd pos_vel_ned = pv_ECEF_to_NED(pEst_E_m_, vEst_E_mps_);
  pEst_D_rrm_ = pos_vel_ned.segment(0, 3); // Position in LLA (rad, rad, m)
  vEst_L_mps_ = (pos_vel_ned.segment(3, 3)).cast<float>(); // Velocity in NED
  cout << "Meas. Update(pos_D): " << pEst_D_rrm_(0) <<", "<<pEst_D_rrm_(1) <<", "<< pEst_D_rrm_(2) << endl; 
  cout << "Meas. Update(vel_L): " << vEst_L_mps_(0) <<", "<< vEst_L_mps_(1) <<", "<< vEst_L_mps_(2) << endl; 
  
  // Attitude correction
  // Quaternionf dQuat_BL = Quaternionf(1.0, (float)quatDelta(0), (float)quatDelta(1), (float)quatDelta(2));
  // quat_BL_ = (quat_BL_ * dQuat_BL).normalized();  
  //  // Avoid quaternion flips sign
  // if (quat_BL_.w() < 0)
  // {
  //   quat_BL_ = Quaternionf(-quat_BL_.w(), -quat_BL_.x(), -quat_BL_.y(), -quat_BL_.z());
  // }
  //Grove Euler method 
  Matrix3f T_L2B = Quat2DCM(quat_BL_);
  Matrix3f T_B2L = T_L2B.transpose();
   
  float cos_lat = cos(pEst_D_rrm_(0));
  float sin_lat = sin(pEst_D_rrm_(0));
  float cos_lon = cos(pEst_D_rrm_(1));
  float sin_lon = sin(pEst_D_rrm_(1));

  Matrix3f T_E2L;                
  T_E2L(0,0) = -sin_lat*cos_lon;	T_E2L(0,1) = -sin_lat*sin_lon;	T_E2L(0,2) =  cos_lat;
  T_E2L(1,0) =         -sin_lon;	T_E2L(1,1) =          cos_lon;	T_E2L(1,2) =        0;
  T_E2L(2,0) = -cos_lat*cos_lon;	T_E2L(2,1) = -cos_lat*sin_lon;	T_E2L(2,2) = -sin_lat;
  
  Matrix3f T_B2E = T_E2L.transpose()*T_B2L;
  Vector3f quatDelta_f = quatDelta.cast<float>();
  T_B2E = (I3 - Skew(quatDelta_f))*T_B2E; 
  T_B2L =  T_E2L * T_B2E;
  euler_BL_rad_  = DCM2Eul(T_B2L.transpose());
  quat_BL_ = Euler2Quat(euler_BL_rad_);


  // Euler angles from quaternion
  euler_BL_rad_ = Quat2Euler(quat_BL_);
  cout << "Meas. Update(atti): " << euler_BL_rad_(0)*180/3.1415 << " " << euler_BL_rad_(1)*180/3.1415  << " " << euler_BL_rad_(2)*180/3.1415 << endl; 
 
  // Update biases from states
  aBias_mps2_ +=  aBiasDelta.cast<float>();
  wBias_rps_  +=  wBiasDelta.cast<float>();
  // cout << "aBias_mps2_: " << aBias_mps2_.transpose() << endl;
  // cout << "wBias_rps_: " << wBias_rps_.transpose() << endl;
  clockBias_m_ += clockBiasDelta;           // clock bias
  clockRateBias_mps_ += clockBiasRateDelta; // clock rate bias (drift)
  // cout << "clockBias(Meas Update): " << clockBias_m_ << endl;
  // cout << "clockDrift(meas update): " << clockRateBias_mps_ << endl;
}


// Sequential Measurement Update (NED)
// void uNavINS::MeasSeqUpdate17(MatrixXd &gnss_measurement)
// {
//   // 
//   cout << "this is EKF17 Seq. filter:" << endl; 
//   // determine no of satellite
//   int no_sat = gnss_measurement.rows();
//   MatrixXf gnss_measurement_f(no_sat,8);
//   gnss_measurement_f.setZero();
//   gnss_measurement_f =gnss_measurement.cast<float>();
//   cout << gnss_measurement_f << endl; 

//   // Range Estimate
//   float rangeEst_m_;
//   // Range rate Estimate
//   float rangeRateEst_mps_;
//   // Range Error
//   float rangeErr_m;
//   // Range rate Error
//   float rangeRateErr_mps;
//   // Allocate measurement matrix
//   MatrixXf H_(2, 17);
//   H_.setZero();
//   // Allocate measurement noise matrix
//   MatrixXf R_(2, 2);
//   R_.setIdentity();
//   // set R
//   R_(0, 0) = pseudorangeNoiseSigma_m * pseudorangeNoiseSigma_m;
//   R_(1, 1) = pseudorangeRateNoiseSigma_mps * pseudorangeRateNoiseSigma_mps;
//   // Allocate space for Rew and Rns 
//   double Rew, Rns;
//   EarthRad(pEst_D_rrm_(0), &Rew, &Rns);
//   // Estimate Position and Velocity in E frame 
//   pEst_E_m_ = D2E(pEst_D_rrm_).cast<float>();
//   vEst_E_mps_ = L2E(vEst_L_mps_, (pEst_D_rrm_).cast<float>());

//   cout << "esti_p (before meas update): " << pEst_D_rrm_ << endl;
//   cout << "esti_v (before meas update): " << vEst_L_mps_ << endl;
//   // Earth rotation vector and matrix 
//   omega_ie(0) = 0.0;
//   omega_ie(1) = 0.0;
//   omega_ie(2) = OMEGA_DOT_EARTH;
//   Omega_ie = Skew(omega_ie);  
  

//   // Create measurement Y, as Error between Measures and Outputs
//   VectorXf y(2, 1);
//   y.setZero();

//   // Innovation covariance
//   MatrixXf S_(2 , 2);
//   S_.setZero();

//   // Kalman gain
//   MatrixXf K(17, 2);
//   K.setZero();

//   MatrixXf I_KH(17, 17); 
//   I_KH.setZero();
  

//   // State update, x = K * y
//   VectorXf x(17, 1);
//   x.setZero();
  
//   cout << "estimate (ECEF) position: " << pEst_E_m_ << endl; 
//   cout << "estimate (ECEF) velocity: " << vEst_E_mps_ << endl; 
//   // loop measurement
//   for (int i = 0; i < no_sat; ++i)
//   {
//     // predict approx range
//     Vector3f delta_r;
//     Vector3f pMeas_E_m_(gnss_measurement_f(i, 2), gnss_measurement_f(i, 3), gnss_measurement_f(i, 4));
//     Vector3f vMeas_E_mps_(gnss_measurement_f(i, 5), gnss_measurement_f(i, 6), gnss_measurement_f(i, 7));
//     cout << pMeas_E_m_ << endl;
    
//     delta_r = pMeas_E_m_ - pEst_E_m_;
//     float approx_range =  delta_r.norm();

//     // Calculate frame rotation during signal transit time using (Grove2nd:8.36)
//     Matrix<double, 3, 3> T_E2I; 
//     T_E2I.setZero();
//     T_E2I(0, 0) = 1;
//     T_E2I(0, 1) =  OMEGA_DOT_EARTH * approx_range / c;
//     T_E2I(0, 2) = 0;
//     T_E2I(1, 0) = -OMEGA_DOT_EARTH * approx_range / c;
//     T_E2I(1, 1) = 1;
//     T_E2I(1, 2) = 0;
//     T_E2I(2, 0) = 0;
//     T_E2I(2, 1) = 0;
//     T_E2I(2, 2) = 1;

//     // Predict pseudo-range using (Grove2nd:9.165)
//     delta_r = T_E2I.cast<float>() * pMeas_E_m_ - pEst_E_m_;
//     float range =  delta_r.norm();
//     rangeEst_m_ = range + clockBias_m_;
//     cout << "rangeEst at:" << i <<"th satellite: " << rangeEst_m_ << endl; 
//     // Formulate range innovation
//     rangeErr_m = gnss_measurement_f(i, 0) - rangeEst_m_;
//     cout << "rangeErrEst at:" << i <<"th satellite: " << rangeErr_m  << endl; 
//     // Predict line of sight in ECI frame
//     Matrix<float, 3, 1> u_as_E = delta_r / range;
//     Matrix<double, 3, 1> u_as_L = E2L(u_as_E.cast<double>(), pEst_D_rrm_);
//     // Predict pseudo-range rate using (9.165)
//     float range_rate = u_as_E.transpose() * (T_E2I.cast<float>() * (vMeas_E_mps_ + Omega_ie.cast<float>() * pMeas_E_m_) 
//                        - (vEst_E_mps_ + Omega_ie.cast<float>() * pEst_E_m_));
//     rangeRateEst_mps_ = range_rate  + clockRateBias_mps_;
//     cout << "rangeRateEst at:" << i <<"th satellite: " << rangeRateEst_mps_ << endl; 
//     // Formulate range-rate innovation
//     rangeRateErr_mps = gnss_measurement_f(i, 1) - rangeRateEst_mps_;
//     cout << "rangeRateErrEst at:" << i <<"th satellite: " << rangeRateErr_mps   << endl; 
//     // set H in L frame - Eq.(14.128) 
//     H_(0, 6) = ((float)Rns+ (float)pEst_D_rrm_(2))*u_as_L(0);
//     H_(0, 7) = ((float)Rew+ (float)pEst_D_rrm_(2))*cos((float)pEst_D_rrm_(0))*u_as_L(1);
//     H_(0, 8) = - (float)u_as_L(2);
//     H_(0, 15) = 1;
//     H_(1, 3) = ((float)Rns+ (float)pEst_D_rrm_(2))*u_as_L(0);
//     H_(1, 4) = ((float)Rew+ (float)pEst_D_rrm_(2))*cos((float)pEst_D_rrm_(0))*u_as_L(1);
//     H_(1, 5) = -(float)u_as_L(2);
//     H_(1, 16) = 1;

//     y(0) = rangeErr_m;
//     y(1) = rangeRateErr_mps;
//     S_ = H_ * P_ * H_.transpose() + R_;
//     K = P_ * H_.transpose() * S_.inverse();
//     // Covariance update, P = (I + K * H) * P * (I + K * H)' + K * R * K'
//     I_KH = I17 - K * H_; // temp
//     P_ = I_KH * P_ * I_KH.transpose() + K * R_ * K.transpose();
//     x = K * y;
//   }

//   cout << "delta_x: " << x << endl;

//   // Pull apart x terms to update the Position, velocity, orientation, and sensor biases
//   Vector3f pDeltaEst_D = x.segment(0, 3); // Position Deltas in LLA
//   Vector3f vDeltaEst_L = x.segment(3, 3); // Velocity Deltas in NED
//   Vector3f quatDelta = x.segment(6, 3);   // Quaternion Delta
//   Vector3f aBiasDelta = x.segment(9, 3);  // Accel Bias Deltas
//   Vector3f wBiasDelta = x.segment(12, 3); // Rotation Rate Bias Deltas
//   float clockBiasDelta = x(15,0); // Clock Offset Deltas
//   float clockBiasRateDelta = x(16,0);  // Clock Phase Deltas
  
//   // Position update
//   pEst_D_rrm_(2) += -pDeltaEst_D(2);
//   pEst_D_rrm_(0) += pDeltaEst_D(0) / (Rew + pEst_D_rrm_(2));
//   pEst_D_rrm_(1) += pDeltaEst_D(1) / (Rns + pEst_D_rrm_(2)) / cos(pEst_D_rrm_(0));

//   // Velocity update
//   vEst_L_mps_ += vDeltaEst_L;
  
//   cout << "esti_p (after meas update): " << pEst_D_rrm_ << endl;
//   cout << "esti_v (after meas update): " << vEst_L_mps_ << endl;
//   // Attitude correction
//   Quaternionf dQuat_BL = Quaternionf(1.0, quatDelta(0), quatDelta(1), quatDelta(2));
//   quat_BL_ = (quat_BL_ * dQuat_BL).normalized();
//   cout << "quat (after meas update): " << quat_BL_.w() << " " << quat_BL_.x() << " " << quat_BL_.y() << " " << quat_BL_.z()  << endl; 
//   // Update biases from states
//   aBias_mps2_ += aBiasDelta;
//   wBias_rps_ += wBiasDelta;
//   clockBias_m_ += clockBiasDelta;           // clock bias
//   clockRateBias_mps_ += clockBiasRateDelta; // clock rate bias
// }
