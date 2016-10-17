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

#include <iostream>
using std::cout;
using std::endl;
#include <stdio.h>

#include "nav_functions.hxx"
#include "nav_interface.hxx"

#include "../utils/coremag.h"

#include "EKF_15state_quat.hxx"

const double P_P_INIT = 10.0;
const double P_V_INIT = 1.0;
const double P_A_INIT = 0.34906;   // 20 deg
const double P_HDG_INIT = 3.14159; // 180 deg
const double P_AB_INIT = 0.9810;   // 0.5*g
const double P_GB_INIT = 0.01745;  //5 deg/s

const double Rew = 6.359058719353925e+006; // earth radius
const double Rns = 6.386034030458164e+006; // earth radius

////////// BRT I think there are some several identity and sparse matrices, so probably some optimization still left there
////////// BRT Seems like a lot of the transforms could be more efficiently done with just a matrix or vector multiply
////////// BRT Probably could do a lot of block operations with F, i.e. F.block(j,k) = C_B2N, etc
////////// BRT A lot of these multi line equations with temp matrices can be compressed

void EKF::config(double SIG_W_AX, double SIG_W_AY, double SIG_W_AZ,
		 double SIG_W_GX, double SIG_W_GY, double SIG_W_GZ,
		 double SIG_A_D, double TAU_A, double SIG_G_D, double TAU_G,	
		 double SIG_GPS_P_NE, double SIG_GPS_P_D,
		 double SIG_GPS_V_NE, double SIG_GPS_V_D,
		 double SIG_MAG)
{
    this->SIG_W_AX = SIG_W_AX;
    this->SIG_W_AY = SIG_W_AY;
    this->SIG_W_AZ = SIG_W_AZ;
    this->SIG_W_GX = SIG_W_GX;
    this->SIG_W_GY = SIG_W_GY;
    this->SIG_W_GZ = SIG_W_GZ;
    this->SIG_A_D = SIG_A_D;
    this->TAU_A = TAU_A;
    this->SIG_G_D = SIG_G_D;
    this->TAU_G = TAU_G;
    this->SIG_GPS_P_NE = SIG_GPS_P_NE;
    this->SIG_GPS_P_D = SIG_GPS_P_D;
    this->SIG_GPS_V_NE = SIG_GPS_V_NE;
    this->SIG_GPS_V_D = SIG_GPS_V_D;
    this->SIG_MAG = SIG_MAG;
}

NAVdata EKF::init(IMUdata imu, GPSdata gps) {
    I15.setIdentity();
    I3.setIdentity();

    // Assemble the matrices
    // .... gravity, g
    grav(2) = g;
	
    // ... H
    H.topLeftCorner(6,6).setIdentity();

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
	
     // ... R
    R(0,0) = SIG_GPS_P_NE*SIG_GPS_P_NE;	 R(1,1) = SIG_GPS_P_NE*SIG_GPS_P_NE;  R(2,2) = SIG_GPS_P_D*SIG_GPS_P_D;
    R(3,3) = SIG_GPS_V_NE*SIG_GPS_V_NE;	 R(4,4) = SIG_GPS_V_NE*SIG_GPS_V_NE;  R(5,5) = SIG_GPS_V_D*SIG_GPS_V_D;
    R(6,6) = SIG_MAG*SIG_MAG;            R(7,7) = SIG_MAG*SIG_MAG;            R(8,8) = SIG_MAG*SIG_MAG;
   
    // ... update P in get_nav
    nav.Pp0 = P(0,0);	  nav.Pp1 = P(1,1);	nav.Pp2 = P(2,2);
    nav.Pv0 = P(3,3);	  nav.Pv1 = P(4,4);	nav.Pv2 = P(5,5);
    nav.Pa0 = P(6,6);	  nav.Pa1 = P(7,7);	nav.Pa2 = P(8,8);
	
    nav.Pabx = P(9,9);	  nav.Paby = P(10,10);	nav.Pabz = P(11,11);
    nav.Pgbx = P(12,12);  nav.Pgby = P(13,13);  nav.Pgbz = P(14,14);
	
    // .. then initialize states with GPS Data
    nav.lat = gps.lat*D2R;
    nav.lon = gps.lon*D2R;
    nav.alt = gps.alt;
	
    nav.vn = gps.vn;
    nav.ve = gps.ve;
    nav.vd = gps.vd;
	
    // ideal magnetic vector
    long int jd = now_to_julian_days();
    double field[6];
    calc_magvar( nav.lat, nav.lon,
		 nav.alt / 1000.0, jd, field );
    mag_ned(0) = field[3];
    mag_ned(1) = field[4];
    mag_ned(2) = field[5];
    mag_ned.normalize();
    // cout << "Ideal mag vector (ned): " << mag_ned << endl;
    // // initial heading
    // double init_psi_rad = 90.0*D2R;
    // if ( fabs(mag_ned[0][0]) > 0.0001 || fabs(mag_ned[0][1]) > 0.0001 ) {
    // 	init_psi_rad = atan2(mag_ned[0][1], mag_ned[0][0]);
    // }

    // fixme: for now match the reference implementation so we can
    // compare intermediate calculations.
    // nav.the = 0*D2R;
    // nav.phi = 0*D2R;
    // nav.psi = 90.0*D2R;

    // ... and initialize states with IMU Data
    // theta from Ax, aircraft at rest
    nav.the = asin(imu.ax/g); 
    // phi from Ay, aircraft at rest
    nav.phi = asin(-imu.ay/(g*cos(nav.the))); 
    // psi from magnetometer
    if ( fabs(imu.hx) > 0.0001 && fabs(imu.hy) > 0.0001 ) {
	nav.psi = 90*D2R - atan2(imu.hy, imu.hx);
    }
	
    quat = eul2quat(nav.phi, nav.the, nav.psi);
    nav.qw = quat.w();
    nav.qx = quat.x();
    nav.qy = quat.y();
    nav.qz = quat.z();
	
    nav.abx = 0.0;
    nav.aby = 0.0; 
    nav.abz = 0.0;
	
    nav.gbx = imu.p;
    nav.gby = imu.q;
    nav.gbz = imu.r;
	
    // Specific forces and Rotation Rate
    f_b(0) = imu.ax - nav.abx;
    f_b(1) = imu.ay - nav.aby;
    f_b(2) = imu.az - nav.abz;
	
    om_ib(0) = imu.p - nav.gbx;
    om_ib(1) = imu.q - nav.gby;
    om_ib(2) = imu.r - nav.gbz;
	
    // Time during initialization
    tprev = imu.time;
	
    //nav.init = 1;
    nav.err_type = data_valid;

    return nav;
}

// Main get_nav filter function
NAVdata EKF::update(IMUdata imu, GPSdata gps) {
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
	
    nav.Pp0 = P(0,0);     nav.Pp1 = P(1,1);     nav.Pp2 = P(2,2);
    nav.Pv0 = P(3,3);     nav.Pv1 = P(4,4);     nav.Pv2 = P(5,5);
    nav.Pa0 = P(6,6);     nav.Pa1 = P(7,7);     nav.Pa2 = P(8,8);
    nav.Pabx = P(9,9);    nav.Paby = P(10,10);  nav.Pabz = P(11,11);
    nav.Pgbx = P(12,12);  nav.Pgby = P(13,13);  nav.Pgbz = P(14,14);

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

	// measured mag vector (body frame)
	Vector3d mag_sense;
	mag_sense(0) = imu.hx;
	mag_sense(1) = imu.hy;
	mag_sense(2) = imu.hz;
	mag_sense.normalize();
	
	Vector3d mag_error; // magnetometer measurement error
	bool mag_error_in_ned = false;
	if ( mag_error_in_ned ) {
	    // rotate measured mag vector into ned frame (then normalized)
	    Vector3d mag_sense_ned = C_B2N * mag_sense;
	    mag_sense_ned.normalize();
	    mag_error = mag_sense_ned - mag_ned;
	} else {
	    // rotate ideal mag vector into body frame (then normalized)
	    Vector3d mag_ideal = C_N2B * mag_ned;
	    mag_ideal.normalize();
	    mag_error = mag_sense - mag_ideal;
	    // cout << "mag_error:" << mag_error << endl;

	    // Matrix<double,3,3> tmp1 = C_N2B * sk(mag_ned);
	    Matrix3d tmp1 = sk(mag_sense) * 2.0;
	    for ( int j = 0; j < 3; j++ ) {
		for ( int i = 0; i < 3; i++ ) {
		    H(6+i,6+j) = tmp1(i,j);
		}
	    }
	}

	// Create Measurement: y
	y(0) = pos_gps_ned(0) - pos_ins_ned(0);
	y(1) = pos_gps_ned(1) - pos_ins_ned(1);
	y(2) = pos_gps_ned(2) - pos_ins_ned(2);
		
	y(3) = gps.vn - nav.vn;
	y(4) = gps.ve - nav.ve;
	y(5) = gps.vd - nav.vd;
		
	y(6) = mag_error(0);
	y(7) = mag_error(1);
	y(8) = mag_error(2);
	
	// Kalman Gain
	// K = P*H'*inv(H*P*H'+R)
	K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
		
	// Covariance Update
	ImKH = I15 - K * H;	                // ImKH = I - K*H
		
	KRKt = K * R * K.transpose();		// KRKt = K*R*K'
		
	P = ImKH * P * ImKH.transpose() + KRKt;	// P = ImKH*P*ImKH' + KRKt
		
	nav.Pp0 = P(0,0);     nav.Pp1 = P(1,1);     nav.Pp2 = P(2,2);
	nav.Pv0 = P(3,3);     nav.Pv1 = P(4,4);     nav.Pv2 = P(5,5);
	nav.Pa0 = P(6,6);     nav.Pa1 = P(7,7);     nav.Pa2 = P(8,8);
	nav.Pabx = P(9,9);    nav.Paby = P(10,10);  nav.Pabz = P(11,11);
	nav.Pgbx = P(12,12);  nav.Pgby = P(13,13);  nav.Pgbz = P(14,14);
		
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
	
	nav.abx += x(9);
	nav.aby += x(10);
	nav.abz += x(11);

	nav.gbx += x(12);
	nav.gby += x(13);
	nav.gbz += x(14);
    }
	
    nav.qw = quat.w();
    nav.qx = quat.x();
    nav.qy = quat.y();
    nav.qz = quat.z();
	
    // Remove current estimated biases from rate gyro and accels
    imu.p -= nav.gbx;
    imu.q -= nav.gby;
    imu.r -= nav.gbz;
    imu.ax -= nav.abx;
    imu.ay -= nav.aby;
    imu.az -= nav.abz;

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


#include <boost/python.hpp>
using namespace boost::python;

BOOST_PYTHON_MODULE(libnav_eigen_mag_test)
{
    class_<IMUdata>("IMUdata")
	.def_readwrite("time", &IMUdata::time)
	.def_readwrite("p", &IMUdata::p)
	.def_readwrite("q", &IMUdata::q)
	.def_readwrite("r", &IMUdata::r)
 	.def_readwrite("ax", &IMUdata::ax)
 	.def_readwrite("ay", &IMUdata::ay)
 	.def_readwrite("az", &IMUdata::az)
 	.def_readwrite("hx", &IMUdata::hx)
 	.def_readwrite("hy", &IMUdata::hy)
 	.def_readwrite("hz", &IMUdata::hz)
    ;
    
    class_<GPSdata>("GPSdata")
	.def_readwrite("time", &GPSdata::time)
	.def_readwrite("lat", &GPSdata::lat)
	.def_readwrite("lon", &GPSdata::lon)
	.def_readwrite("alt", &GPSdata::alt)
	.def_readwrite("vn", &GPSdata::vn)
	.def_readwrite("ve", &GPSdata::ve)
	.def_readwrite("vd", &GPSdata::vd)
	.def_readwrite("newData", &GPSdata::newData)
    ;

    class_<NAVdata>("NAVdata")
	.def_readwrite("time", &NAVdata::time)
	.def_readwrite("lat", &NAVdata::lat)
	.def_readwrite("lon", &NAVdata::lon)
	.def_readwrite("alt", &NAVdata::alt)
	.def_readwrite("vn", &NAVdata::vn)
	.def_readwrite("ve", &NAVdata::ve)
	.def_readwrite("vd", &NAVdata::vd)
	.def_readwrite("phi", &NAVdata::phi)
	.def_readwrite("the", &NAVdata::the)
	.def_readwrite("psi", &NAVdata::psi)
	.def_readwrite("qw", &NAVdata::qw)
	.def_readwrite("qx", &NAVdata::qx)
	.def_readwrite("qy", &NAVdata::qy)
	.def_readwrite("qz", &NAVdata::qz)
	.def_readwrite("abx", &NAVdata::abx)
	.def_readwrite("aby", &NAVdata::aby)
	.def_readwrite("abz", &NAVdata::abz)
	.def_readwrite("gbx", &NAVdata::gbx)
	.def_readwrite("gby", &NAVdata::gby)
	.def_readwrite("gbz", &NAVdata::gbz)
	.def_readwrite("Pp0", &NAVdata::Pp0)
	.def_readwrite("Pp1", &NAVdata::Pp1)
	.def_readwrite("Pp2", &NAVdata::Pp2)
	.def_readwrite("Pv0", &NAVdata::Pv0)
	.def_readwrite("Pv1", &NAVdata::Pv1)
	.def_readwrite("Pv2", &NAVdata::Pv2)
	.def_readwrite("Pa0", &NAVdata::Pa0)
	.def_readwrite("Pa1", &NAVdata::Pa1)
	.def_readwrite("Pa2", &NAVdata::Pa2)
	.def_readwrite("Pabx", &NAVdata::Pabx)
	.def_readwrite("Paby", &NAVdata::Paby)
	.def_readwrite("Pabz", &NAVdata::Pabz)
	.def_readwrite("Pgbx", &NAVdata::Pgbx)
	.def_readwrite("Pgby", &NAVdata::Pgby)
	.def_readwrite("Pgbz", &NAVdata::Pgbz)
    ;
    
    class_<EKF>("EKF")
        .def("init", &EKF::init)
        .def("update", &EKF::update)
    ;
}

