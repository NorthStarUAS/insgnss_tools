/*
Copyright (c) 2016 - 2020 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Adapted for RAPTRS: Brian Taylor and Chris Regan

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

#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/LU>
//#include <Eigen/Core>
//#include <Eigen/Dense>
//#include <Eigen/Geometry>
using namespace Eigen;
#include "../nav_common/structs.h"

// Constants
const double EarthRadius = 6378137.0;        // earth semi-major axis radius (m)
const double ECC2 = 0.0066943799901;         // major eccentricity squared

// Constants for tightly coupled EKF
const double MU = 3.986005e14; //m^3 / sec^2
const double OMEGA_DOT_EARTH = 7.2921151467e-5; // rad/sec
const double c = 299792458; // Speed of light in m/s
const double J2 = 1.082627e-3; // WGS84 Earth's second gravitational constant

// Constants that are no longer used
// const double EarthRate = 0.00007292115;      // rotation rate of earth (rad/sec)
const double Eccentricity = 0.0818191908426; // major eccentricity of earth ellipsoid
// const double Flattening = 0.0033528106650;   // flattening of the ellipsoid
// const double Gravity0 = 9.7803730;           // zeroth coefficient for gravity model
// const double Gravity1 = 0.0052891;           // first coefficient for the gravity model
// const double Gravity2 = 0.0000059;           // second coefficient for the gravity model
// const double GravityNom = 9.81;              // nominal gravity
// const double Schuler2 = 1.533421593170545E-06; // Schuler Frequency (rad/sec) Squared

Vector3d L2D_Rate(Vector3d v_L, Vector3d pRef_D);
Vector3f L2D_Rate(Vector3f v_L, Vector3d pRef_D);

Vector3d NavRate(Vector3d v_L, Vector3d pRef_D);
Vector3f NavRate(Vector3f v_L, Vector3d pRef_D);

Vector3d D2E(Vector3d p_D);

Vector3d E2D(Vector3d p_E);

Vector3d E2L(Vector3d p_E, Vector3d pRef_D);
Matrix3d TransE2L(Vector3d pRef_D);
Quaterniond E2L_Quat(Vector3d pRef_D);

Matrix3d Skew(Vector3d w);
Matrix3f Skew(Vector3f w);

Vector3d Quat2Euler(Quaterniond quat);
Vector3f Quat2Euler(Quaternionf quat);

Quaterniond Euler2Quat(Vector3d euler);
Quaternionf Euler2Quat(Vector3f euler);

Matrix3d Quat2DCM(Quaterniond quat);
Matrix3f Quat2DCM(Quaternionf quat);

void EarthRad(double lat, double *Rew, double *Rns);

double WrapToPi(double a);
float WrapToPi(float a);
double WrapTo2Pi(double a);
float WrapTo2Pi(float a);


// EphemerisData (subframe1,2,3) to Satellite ecef x, y, z in meter, vx, vy, vz in m/s
VectorXd EphemerisData2Satecef(float t,
                               uint32_t TOW, uint8_t L2, uint16_t week_No, uint8_t L2_Flag, uint8_t SV_Acc, uint8_t SV_Hlth,
                               double T_GD, uint16_t IODC, double t_OC, int8_t a_f2, double a_f1, double a_f0,
                               uint8_t IODE, double C_rs, double delta_n, double M_0, double C_uc, double ecc, double C_us,
                               double sqrt_A, double t_OE, double C_ic, double Omega_0, double C_is, double i_0, double C_rc,
                               double omega, double Omega_dot, double IDOT);

// Compute direction cosine matric C from a Euler vector 
// eul = [yaw,pitch,roll]. (i.e., 3-2-1 rotation convention)
Matrix3f Eul2DCM(Vector3f eul); // this might need to change to double 

// Convert DCM to Euler 
Vector3f DCM2Eul(Matrix3f T_B2L);

// ned 2 ecef centered at the coordinate given by lla
Vector3f L2E(Vector3f p_L, Vector3f pRef_D); // this might need to change to double 

// compute vehicle's postion, velocity in E frame and clock offset (m) and drift (m/s) 
void GNSS_LS_pos_vel(MatrixXd &gnss_measurement,
                     Vector3d &pEst_E_m_, Vector3d &vEst_E_mps_, double &clockBias_m_, double &clockRateBias_mps_);

// process raw measurement to give a 8 x 1 matrix (range, range rate, x,y,z,vx,vy,vz)
VectorXd EphemerisData2PosVelClock(GNSS_raw_measurement gnss_raw_measurement);

// Converts Cartesian  to curvilinear position and velocity resolving axes from ECEF to NED
VectorXd pv_ECEF_to_NED(Vector3d &pEst_E_m_, Vector3d &vEst_E_mps_);

// Gravitation_ECI - Calculates  acceleration due to gravity resolved about ECEF-frame
Vector3d Gravity_ECEF(Vector3d &pEst_E_m_);
