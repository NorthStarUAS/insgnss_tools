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
typedef Matrix<double,9,9>   Matrix9d;
typedef Matrix<double,12,12> Matrix12d;
typedef Matrix<double,15,15> Matrix15d;
typedef Matrix<double,9,15>  Matrix9x15d;
typedef Matrix<double,15,9>  Matrix15x9d;
typedef Matrix<double,15,12> Matrix15x12d;
typedef Matrix<double,9,1>   Vector9d;
typedef Matrix<double,15,1>  Vector15d;

class EKF {

public:

    EKF():
	// default values appropriate for typical hobby/diy sensors
	sig_w_ax(0.05),	   // m/s^2
	sig_w_ay(0.05),
	sig_w_az(0.05),
	sig_w_gx(0.00175), // rad/s (0.1 deg/s)
	sig_w_gy(0.00175),
	sig_w_gz(0.00175),
	sig_a_d(0.1),      // 5e-2*g
	tau_a(100.0),
	sig_g_d(0.00873),  // 0.1 deg/s
	tau_g(50.0),
	sig_gps_p_ne(3.0),
	sig_gps_p_d(5.0),
	sig_gps_v_ne(0.5),
	sig_gps_v_d(1.0),
	sig_mag(0.2)
    {
    }
    ~EKF() {}

    // set error characteristics of navigation parameters
    void config(double sig_w_ax = 0.05,	   // m/s^2
		double sig_w_ay = 0.05,
		double sig_w_az = 0.05,
		double sig_w_gx = 0.00175, // rad/s (0.1 deg/s)
		double sig_w_gy = 0.00175,
		double sig_w_gz = 0.00175,
		double sig_a_d  = 0.1,     // 5e-2*g
		double tau_a    = 100.0,
		double sig_g_d  = 0.00873, // 0.1 deg/s
		double tau_g    = 50.0,
		double sig_gps_p_ne = 3.0,
		double sig_gps_p_d  = 5.0,
		double sig_gps_v_ne = 0.5,
		double sig_gps_v_d  = 1.0,
		double sig_mag      = 0.2);
    NAVdata init(IMUdata imu, GPSdata gps);
    NAVdata update(IMUdata imu, GPSdata gps);
    
private:

    double sig_w_ax;		// m/s^2
    double sig_w_ay;
    double sig_w_az;
    double sig_w_gx;		// rad/s (0.1 deg/s)
    double sig_w_gy;
    double sig_w_gz;
    double sig_a_d;		// 5e-2*g
    double tau_a;
    double sig_g_d;		// 0.1 deg/s
    double tau_g;
    double sig_gps_p_ne;
    double sig_gps_p_d;
    double sig_gps_v_ne;
    double sig_gps_v_d;
    double sig_mag;

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
