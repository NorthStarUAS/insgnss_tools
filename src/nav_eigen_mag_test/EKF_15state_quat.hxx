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

#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/LU>
using namespace Eigen;

// define some types for notational convenience and consistency
typedef Matrix<double,9,9> Matrix9d;
typedef Matrix<double,12,12> Matrix12d;
typedef Matrix<double,15,15> Matrix15d;
typedef Matrix<double,9,15> Matrix9x15d;
typedef Matrix<double,15,9> Matrix15x9d;
typedef Matrix<double,15,12> Matrix15x12d;
typedef Matrix<double,9,1> Vector9d;
typedef Matrix<double,15,1> Vector15d;

class EKF {

public:

    EKF() {
	config();		// start with default values
    }
    ~EKF() {}

    // set error characteristics of navigation parameters
    void config(double SIG_W_AX = 0.05,	// m/s^2
		double SIG_W_AY = 0.05,
		double SIG_W_AZ = 0.05,
		double SIG_W_GX = 0.00175, // rad/s (0.1 deg/s)
		double SIG_W_GY = 0.00175,
		double SIG_W_GZ = 0.00175,
		double SIG_A_D  = 0.1,	 // 5e-2*g
		double TAU_A    = 100.0,
		double SIG_G_D  = 0.00873, // 0.1 deg/s
		double TAU_G    = 50.0,
		double SIG_GPS_P_NE = 3.0,
		double SIG_GPS_P_D  = 5.0,
		double SIG_GPS_V_NE = 0.5,
		double SIG_GPS_V_D  = 1.0,
		double SIG_MAG      = 0.2);
    NAVdata init(IMUdata imu, GPSdata gps);
    NAVdata update(IMUdata imu, GPSdata gps);
    
private:

    double SIG_W_AX;		// m/s^2
    double SIG_W_AY;
    double SIG_W_AZ;
    double SIG_W_GX;		// rad/s (0.1 deg/s)
    double SIG_W_GY;
    double SIG_W_GZ;
    double SIG_A_D;		// 5e-2*g
    double TAU_A;
    double SIG_G_D;		// 0.1 deg/s
    double TAU_G;
    double SIG_GPS_P_NE;
    double SIG_GPS_P_D;
    double SIG_GPS_V_NE;
    double SIG_GPS_V_D;
    double SIG_MAG;

    Matrix15d F, PHI, P, Qw, Q, ImKH, KRKt, I15 /* identity */;
    Matrix15x12d G;
    Matrix15x9d K;
    Vector15d x;
    Matrix12d Rw;
    Matrix9x15d H;
    Matrix9d R;
    Vector9d y;
    Matrix3d C_N2B, C_B2N, I3 /* identity */, temp33;
    Vector3d grav, f_b, om_ib, nr, pos_ins_ecef, pos_ins_ned, pos_gps, pos_gps_ecef, pos_gps_ned, dx, mag_ned;

    Quaterniond quat; // fixme, make state persist here, not in nav
    double denom, Re, Rn;
    double tprev;

    NAVdata nav;
};
