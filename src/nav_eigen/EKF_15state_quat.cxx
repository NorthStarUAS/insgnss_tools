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
 * $Id: EKF_15state_quat.c 911 2012-10-08 15:00:59Z lie $
 */

#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/LU>
using namespace Eigen;

#include <iostream>
using std::cout;
using std::endl;
#include <stdio.h>

#include "nav_functions.hxx"
#include "nav_interface.hxx"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//error characteristics of navigation parameters
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
const double SIG_W_AX = 1.0;	 // 1 m/s^2
const double SIG_W_AY = 1.0;
const double SIG_W_AZ = 1.0;
const double SIG_W_GX = 0.00524; // 0.3 deg/s
const double SIG_W_GY = 0.00524;
const double SIG_W_GZ = 0.00524;
const double SIG_A_D  = 0.1;	 // 5e-2*g
const double TAU_A    = 100.0;
const double SIG_G_D  = 0.00873; // 0.1 deg/s
const double TAU_G    = 50.0;

const double SIG_GPS_P_NE = 3.0;
const double SIG_GPS_P_D  = 5.0;
const double SIG_GPS_V    = 0.5;

const double P_P_INIT = 10.0;
const double P_V_INIT = 1.0;
const double P_A_INIT = 0.34906;   // 20 deg
const double P_HDG_INIT = 3.14159; // 180 deg
const double P_AB_INIT = 0.9810;   // 0.5*g
const double P_GB_INIT = 0.01745;  //5 deg/s

const double Rew = 6.359058719353925e+006; // earth radius
const double Rns = 6.386034030458164e+006; // earth radius

// define some types for notational convenience and consistency
typedef Matrix<double,6,6> Matrix6d;
typedef Matrix<double,12,12> Matrix12d;
typedef Matrix<double,15,15> Matrix15d;
typedef Matrix<double,6,15> Matrix6x15d;
typedef Matrix<double,15,6> Matrix15x6d;
typedef Matrix<double,15,12> Matrix15x12d;
typedef Matrix<double,6,1> Vector6f;
typedef Matrix<double,15,1> Vector15f;

static Matrix15d F, PHI, P, Qw, Q, ImKH, KRKt, I15 /* identity */;
static Matrix15x12d G;
static Matrix15x6d K;
static Vector15f x;
static Matrix12d Rw;
static Matrix6x15d H;
static Matrix6d R;
static Vector6f y;
static Matrix3d C_N2B, C_B2N, I3 /* identity */, temp33;
static Vector3d grav, f_b, om_ib, nr, pos_ins_ecef, pos_ins_ned, pos_gps, pos_gps_ecef, pos_gps_ned, dx;

static Quaterniond quat; // fixme, make state persist here, not in nav
static double denom, Re, Rn;
static double tprev;

static NAVdata nav;

////////// BRT I think there are some several identity and sparse matrices, so probably some optimization still left there
////////// BRT Seems like a lot of the transforms could be more efficiently done with just a matrix or vector multiply
////////// BRT Probably could do a lot of block operations with F, i.e. F.block(j,k) = C_B2N, etc
////////// BRT A lot of these multi line equations with temp matrices can be compressed
	
NAVdata init_nav(IMUdata imu, GPSdata gps) {
    I15.setIdentity();
    I3.setIdentity();

    // Assemble the matrices
    // .... gravity, g
    grav(2) = g;
	
    // ... H
    H.topLeftCorner(6,6).setIdentity();

    // first order correlation + white noise, tau = time constant for correlation
    // gain on white noise plus gain on correlation
    // Rw small - trust time update, Rw more - lean on measurement update
    // split between accels and gyros and / or noise and correlation
    // ... Rw
    Rw(0,0) = SIG_W_AX*SIG_W_AX;	Rw(1,1) = SIG_W_AY*SIG_W_AY;	      Rw(2,2) = SIG_W_AZ*SIG_W_AZ; //1 sigma on noise
    Rw(3,3) = SIG_W_GX*SIG_W_GX;	Rw(4,4) = SIG_W_GY*SIG_W_GY;	      Rw(5,5) = SIG_W_GZ*SIG_W_GZ;
    Rw(6,6) = 2*SIG_A_D*SIG_A_D/TAU_A;	Rw(7,7) = 2*SIG_A_D*SIG_A_D/TAU_A;    Rw(8,8) = 2*SIG_A_D*SIG_A_D/TAU_A;
    Rw(9,9) = 2*SIG_G_D*SIG_G_D/TAU_G;	Rw(10,10) = 2*SIG_G_D*SIG_G_D/TAU_G;  Rw(11,11) = 2*SIG_G_D*SIG_G_D/TAU_G;
	
    // ... P (initial)
    P(0,0) = P_P_INIT*P_P_INIT; 	P(1,1) = P_P_INIT*P_P_INIT; 	      P(2,2) = P_P_INIT*P_P_INIT;
    P(3,3) = P_V_INIT*P_V_INIT; 	P(4,4) = P_V_INIT*P_V_INIT; 	      P(5,5) = P_V_INIT*P_V_INIT;
    P(6,6) = P_A_INIT*P_A_INIT; 	P(7,7) = P_A_INIT*P_A_INIT; 	      P(8,8) = P_HDG_INIT*P_HDG_INIT;
    P(9,9) = P_AB_INIT*P_AB_INIT; 	P(10,10) = P_AB_INIT*P_AB_INIT;       P(11,11) = P_AB_INIT*P_AB_INIT;
    P(12,12) = P_GB_INIT*P_GB_INIT; 	P(13,13) = P_GB_INIT*P_GB_INIT;       P(14,14) = P_GB_INIT*P_GB_INIT;
	
    // ... update P in get_nav
    nav.Pp[0] = P(0,0);	                nav.Pp[1] = P(1,1);	              nav.Pp[2] = P(2,2);
    nav.Pv[0] = P(3,3);	                nav.Pv[1] = P(4,4);	              nav.Pv[2] = P(5,5);
    nav.Pa[0] = P(6,6);	                nav.Pa[1] = P(7,7);	              nav.Pa[2] = P(8,8);
	
    nav.Pab[0] = P(9,9);	        nav.Pab[1] = P(10,10);	              nav.Pab[2] = P(11,11);
    nav.Pgb[0] = P(12,12);	        nav.Pgb[1] = P(13,13);	              nav.Pgb[2] = P(14,14);
	
    // ... R
    R(0,0) = SIG_GPS_P_NE*SIG_GPS_P_NE;	R(1,1) = SIG_GPS_P_NE*SIG_GPS_P_NE;   R(2,2) = SIG_GPS_P_D*SIG_GPS_P_D;
    R(3,3) = SIG_GPS_V*SIG_GPS_V;	R(4,4) = SIG_GPS_V*SIG_GPS_V;	      R(5,5) = SIG_GPS_V*SIG_GPS_V;
	
    // .. then initialize states with GPS Data
    nav.lat = gps.lat*D2R;
    nav.lon = gps.lon*D2R;
    nav.alt = gps.alt;
	
    nav.vn = gps.vn;
    nav.ve = gps.ve;
    nav.vd = gps.vd;
	
    // ... and initialize states with IMU Data
    // theta from Ax, aircraft at rest
    nav.the = asin(imu.ax/g); 
    // phi from Ay, aircraft at rest
    nav.phi = asin(imu.ay/(g*cos(nav.the))); 
    nav.psi = 0.0;

    // fixme: for now match the reference implementation so we can
    // compare intermediate calculations.
    nav.the = 8*D2R;
    nav.phi = 0*D2R;
    nav.psi = 90.0*D2R;

    /*
      if((imu.hy) > 0){
      nav.psi = 90*D2R - atan(imu.hx / imu.hy);
      }
      else if((imu.hy) < 0){
      nav.psi = 270*D2R - atan(imu.hx / imu.hy) - 360*D2R;
      }
      else if((imu.hy) == 0){
      if((imu.hx) < 0){
      nav.psi = 180*D2R;
      }
      else{
      nav.psi = 0.0;
      }
      }*/
	
    quat = eul2quat(nav.phi, nav.the, nav.psi);
    nav.quat[0] = quat.w();
    nav.quat[1] = quat.x();
    nav.quat[2] = quat.y();
    nav.quat[3] = quat.z();
	
    nav.ab[0] = 0.0;
    nav.ab[1] = 0.0; 
    nav.ab[2] = 0.0;
	
    nav.gb[0] = imu.p;
    nav.gb[1] = imu.q;
    nav.gb[2] = imu.r;
	
    // Specific forces and Rotation Rate
    f_b(0) = imu.ax - nav.ab[0];
    f_b(1) = imu.ay - nav.ab[1];
    f_b(2) = imu.az - nav.ab[2];
	
    om_ib(0) = imu.p - nav.gb[0];
    om_ib(1) = imu.q - nav.gb[1];
    om_ib(2) = imu.r - nav.gb[2];
	
    // Time during initialization
    tprev = imu.time;
	
    //nav.init = 1;
    nav.err_type = data_valid;

    return nav;
}

// Main get_nav filter function
NAVdata get_nav(IMUdata imu, GPSdata gps) {
    // compute time-elapsed 'dt'
    // This compute the navigation state at the DAQ's Time Stamp
    double tnow = imu.time;
    double imu_dt = tnow - tprev;
    tprev = tnow;		

    // ==================  Time Update  ===================

    // AHRS Transformations
    C_N2B = quat2dcm(quat);
    C_B2N = C_N2B.transpose();
	
    // Attitude Update
    // ... Calculate Navigation Rate
    Vector3d vel_vec(nav.vn, nav.ve, nav.vd);
    Vector3d pos_vec(nav.lat, nav.lon, nav.alt);
	
    nr = navrate(vel_vec,pos_vec);  /* note: unused, llarate used instead */
	
    Quaterniond dq;
    dq = Quaterniond(1.0, 0.5*om_ib(0)*imu_dt, 0.5*om_ib(1)*imu_dt, 0.5*om_ib(2)*imu_dt);
    quat = (quat * dq).normalized();

    if (quat.w() < 0) {
        // Avoid quaternion flips sign
        quat = Quaterniond(-quat.w(), -quat.x(), -quat.y(), -quat.z());
    }
    
    Vector3d att_vec = quat2eul(quat);
    nav.phi = att_vec(0);
    nav.the = att_vec(1);
    nav.psi = att_vec(2);
	
    // Velocity Update
    dx = C_B2N * f_b;
    dx += grav;
	
    nav.vn += imu_dt*dx(0);
    nav.ve += imu_dt*dx(1);
    nav.vd += imu_dt*dx(2);
	
    // Position Update
    dx = llarate(vel_vec, pos_vec);
    nav.lat += imu_dt*dx(0);
    nav.lon += imu_dt*dx(1);
    nav.alt += imu_dt*dx(2);
	
    // JACOBIAN
    F.setZero();
    // ... pos2gs
    F(0,3) = 1.0; 	F(1,4) = 1.0; 	F(2,5) = 1.0;
    // ... gs2pos
    F(5,2) = -2 * g / EARTH_RADIUS;
	
    // ... gs2att
    temp33 = C_B2N * sk(f_b);
	
    F(3,6) = -2.0*temp33(0,0);  F(3,7) = -2.0*temp33(0,1);  F(3,8) = -2.0*temp33(0,2);
    F(4,6) = -2.0*temp33(1,0);  F(4,7) = -2.0*temp33(1,1);  F(4,8) = -2.0*temp33(1,2);
    F(5,6) = -2.0*temp33(2,0);  F(5,7) = -2.0*temp33(2,1);  F(5,8) = -2.0*temp33(2,2);
	
    // ... gs2acc
    F(3,9) = -C_B2N(0,0);  F(3,10) = -C_B2N(0,1);  F(3,11) = -C_B2N(0,2);
    F(4,9) = -C_B2N(1,0);  F(4,10) = -C_B2N(1,1);  F(4,11) = -C_B2N(1,2);
    F(5,9) = -C_B2N(2,0);  F(5,10) = -C_B2N(2,1);  F(5,11) = -C_B2N(2,2);
	
    // ... att2att
    temp33 = sk(om_ib);
    F(6,6) = -temp33(0,0);  F(6,7) = -temp33(0,1);  F(6,8) = -temp33(0,2);
    F(7,6) = -temp33(1,0);  F(7,7) = -temp33(1,1);  F(7,8) = -temp33(1,2);
    F(8,6) = -temp33(2,0);  F(8,7) = -temp33(2,1);  F(8,8) = -temp33(2,2);
	
    // ... att2gyr
    F(6,12) = -0.5;
    F(7,13) = -0.5;
    F(8,14) = -0.5;
	
    // ... Accel Markov Bias
    F(9,9) = -1.0/TAU_A;    F(10,10) = -1.0/TAU_A;  F(11,11) = -1.0/TAU_A;
    F(12,12) = -1.0/TAU_G;  F(13,13) = -1.0/TAU_G;  F(14,14) = -1.0/TAU_G;
	
    // State Transition Matrix: PHI = I15 + F*dt;
    PHI = I15 + F * imu_dt;
	
    // Process Noise
    G.setZero();
    G(3,0) = -C_B2N(0,0);   G(3,1) = -C_B2N(0,1);   G(3,2) = -C_B2N(0,2);
    G(4,0) = -C_B2N(1,0);   G(4,1) = -C_B2N(1,1);   G(4,2) = -C_B2N(1,2);
    G(5,0) = -C_B2N(2,0);   G(5,1) = -C_B2N(2,1);   G(5,2) = -C_B2N(2,2);
	
    G(6,3) = -0.5;
    G(7,4) = -0.5;
    G(8,5) = -0.5;
	
    G(9,6) = 1.0; 	    G(10,7) = 1.0; 	    G(11,8) = 1.0;
    G(12,9) = 1.0; 	    G(13,10) = 1.0; 	    G(14,11) = 1.0;

    // Discrete Process Noise
    Qw = G * Rw * G.transpose() * imu_dt;		// Qw = dt*G*Rw*G'
    Q = PHI * Qw;					// Q = (I+F*dt)*Qw
    Q = (Q + Q.transpose()) * 0.5;			// Q = 0.5*(Q+Q')
	
    // Covariance Time Update
    P = PHI * P * PHI.transpose() + Q;			// P = PHI*P*PHI' + Q
    P = (P + P.transpose()) * 0.5;			// P = 0.5*(P+P')
	
    nav.Pp[0] = P(0,0);     nav.Pp[1] = P(1,1);     nav.Pp[2] = P(2,2);
    nav.Pv[0] = P(3,3);     nav.Pv[1] = P(4,4);     nav.Pv[2] = P(5,5);
    nav.Pa[0] = P(6,6);     nav.Pa[1] = P(7,7);     nav.Pa[2] = P(8,8);
    nav.Pab[0] = P(9,9);    nav.Pab[1] = P(10,10);  nav.Pab[2] = P(11,11);
    nav.Pgb[0] = P(12,12);  nav.Pgb[1] = P(13,13);  nav.Pgb[2] = P(14,14);

    // ==================  DONE TU  ===================
	
    if ( gps.newData ) {
	// ==================  GPS Update  ===================
	gps.newData = 0; // Reset the flag
		
	// Position, converted to NED
	Vector3d pos_vec(nav.lat, nav.lon, nav.alt);
	pos_ins_ecef = lla2ecef(pos_vec);

	Vector3d pos_ref = pos_vec;
	pos_ref(2) = 0.0;
	pos_ins_ned = ecef2ned(pos_ins_ecef, pos_ref);
		
	pos_gps(0) = gps.lat*D2R;
	pos_gps(1) = gps.lon*D2R;
	pos_gps(2) = gps.alt;
		
	pos_gps_ecef = lla2ecef(pos_gps);
		
	pos_gps_ned = ecef2ned(pos_gps_ecef, pos_ref);

	// Create Measurement: y
	y(0) = pos_gps_ned(0) - pos_ins_ned(0);
	y(1) = pos_gps_ned(1) - pos_ins_ned(1);
	y(2) = pos_gps_ned(2) - pos_ins_ned(2);
		
	y(3) = gps.vn - nav.vn;
	y(4) = gps.ve - nav.ve;
	y(5) = gps.vd - nav.vd;
		
	// Kalman Gain
	// K = P*H'*inv(H*P*H'+R)
	K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
		
	// Covariance Update
	ImKH = I15 - K * H;	                // ImKH = I - K*H
		
	KRKt = K * R * K.transpose();		// KRKt = K*R*K'
		
	P = ImKH * P * ImKH.transpose() + KRKt;	// P = ImKH*P*ImKH' + KRKt
		
	nav.Pp[0] = P(0,0); 	nav.Pp[1] = P(1,1); 	nav.Pp[2] = P(2,2);
	nav.Pv[0] = P(3,3); 	nav.Pv[1] = P(4,4); 	nav.Pv[2] = P(5,5);
	nav.Pa[0] = P(6,6); 	nav.Pa[1] = P(7,7); 	nav.Pa[2] = P(8,8);
	nav.Pab[0] = P(9,9); 	nav.Pab[1] = P(10,10);  nav.Pab[2] = P(11,11);
	nav.Pgb[0] = P(12,12);  nav.Pgb[1] = P(13,13);  nav.Pgb[2] = P(14,14);
		
	// State Update
	x = K * y;
	denom = (1.0 - (ECC2 * sin(nav.lat) * sin(nav.lat)));
	denom = sqrt(denom*denom);

	Re = EARTH_RADIUS / sqrt(denom);
	Rn = EARTH_RADIUS * (1-ECC2) / denom*sqrt(denom);
	nav.alt = nav.alt - x(2);
	nav.lat = nav.lat + x(0)/(Re + nav.alt);
	nav.lon = nav.lon + x(1)/(Rn + nav.alt)/cos(nav.lat);
		
	nav.vn = nav.vn + x(3);
	nav.ve = nav.ve + x(4);
	nav.vd = nav.vd + x(5);
		
	// Attitude correction
	dq = Quaterniond(1.0, x(6), x(7), x(8));
	quat = (quat * dq).normalized();
		
	Vector3d att_vec = quat2eul(quat);
	nav.phi = att_vec(0);
	nav.the = att_vec(1);
	nav.psi = att_vec(2);
	
	nav.ab[0] += x(9);
	nav.ab[1] += x(10);
	nav.ab[2] += x(11);

	nav.gb[0] += x(12);
	nav.gb[1] += x(13);
	nav.gb[2] += x(14);
    }
	
    nav.quat[0] = quat.w();
    nav.quat[1] = quat.x();
    nav.quat[2] = quat.y();
    nav.quat[3] = quat.z();
	
    // Remove current estimated biases from rate gyro and accels
    imu.p -= nav.gb[0];
    imu.q -= nav.gb[1];
    imu.r -= nav.gb[2];
    imu.ax -= nav.ab[0];
    imu.ay -= nav.ab[1];
    imu.az -= nav.ab[2];

    // Get the new Specific forces and Rotation Rate,
    // use in the next time update
    f_b(0) = imu.ax;
    f_b(1) = imu.ay;
    f_b(2) = imu.az;

    om_ib(0) = imu.p;
    om_ib(1) = imu.q;
    om_ib(2) = imu.r;

    return nav;
}
