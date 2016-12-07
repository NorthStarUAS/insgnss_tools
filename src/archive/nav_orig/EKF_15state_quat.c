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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>


#include "../include/globaldefs.h"
#include "../utils/matrix.h"

#include "nav_functions.h"
#include "nav_interface.h"


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//error characteristics of navigation parameters
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define		SIG_W_AX	1		//1 m/s^2
#define		SIG_W_AY	1
#define		SIG_W_AZ	1
#define		SIG_W_GX	0.00524		//0.3 deg/s
#define		SIG_W_GY	0.00524
#define		SIG_W_GZ	0.00524
#define		SIG_A_D		0.1	    	//5e-2*g
#define		TAU_A		100
#define		SIG_G_D		0.00873		//0.1 deg/s
#define		TAU_G		50

#define		SIG_GPS_P_NE 3
#define		SIG_GPS_P_D  5
#define		SIG_GPS_V	 0.5

#define		P_P_INIT	10.0
#define		P_V_INIT	1.0
#define		P_A_INIT	0.34906		// 20 deg
#define		P_HDG_INIT	3.14159		//180 deg
#define		P_AB_INIT	0.9810		//0.5*g
#define		P_GB_INIT	0.01745		//5 deg/s

#define     Rew     		6.359058719353925e+006      //earth radius
#define     Rns     		6.386034030458164e+006      //earth radius


static MATRIX C_N2B, C_B2N;
static MATRIX F, PHI, P, G, K, H;
static MATRIX Rw, Q, Qw, R;
static MATRIX x, y, eul, Rbodtonav, grav, f_b, om_ib, nr;
static MATRIX pos_ref, pos_ins_ecef, pos_ins_ned;
static MATRIX pos_gps, pos_gps_ecef, pos_gps_ned;
static MATRIX I15, I3, ImKH, KRKt;
static MATRIX dx, a_temp31, b_temp31, temp33, atemp33, temp615, temp1515, temp66, atemp66, temp156, temp1512;
static double quat[4];

static double denom, Re, Rn;
static double tprev;

	
void init_nav(struct imu *imuData_ptr, struct gps *gpsData_ptr, struct nav *navData_ptr){
	/*++++++++++++++++++++++++++++++++++++++++++++++++
	 *matrix creation for navigation computation
	 *++++++++++++++++++++++++++++++++++++++++++++++++*/	
	F 			= mat_creat(15,15,ZERO_MATRIX);		// State Matrix
	PHI 		= mat_creat(15,15,ZERO_MATRIX);		// State transition Matrix
	
	P 			= mat_creat(15,15,ZERO_MATRIX);		// Covariance Matrix
	G			= mat_creat(15,12,ZERO_MATRIX);		// for Process Noise Transformation
	Rw 			= mat_creat(12,12,ZERO_MATRIX);
	Qw 			= mat_creat(15,15,ZERO_MATRIX);
	Q 			= mat_creat(15,15,ZERO_MATRIX);		// Process Noise Matrix
	R 			= mat_creat(6,6,ZERO_MATRIX);		// GPS Measurement Noise matrix
	
	x			= mat_creat(15,1,ZERO_MATRIX);
	y			= mat_creat(6,1,ZERO_MATRIX);		// GPS Measurement
	eul			= mat_creat(3,1,ZERO_MATRIX);
	Rbodtonav 	= mat_creat(3,3,ZERO_MATRIX);
	grav		= mat_creat(3,1,ZERO_MATRIX);		// Gravity Model
	f_b 		= mat_creat(3,1,ZERO_MATRIX);
	om_ib		= mat_creat(3,1,ZERO_MATRIX);
	nr			= mat_creat(3,1,ZERO_MATRIX);		// Nav Transport Rate
	
	K 			= mat_creat(15,6,ZERO_MATRIX);		// Kalman Gain
	H			= mat_creat(6,15,ZERO_MATRIX);
	
	C_N2B 		= mat_creat(3,3,ZERO_MATRIX);		// DCM
	C_B2N 		= mat_creat(3,3,ZERO_MATRIX);		// DCM Transpose
	
	pos_ref		= mat_creat(3,1,ZERO_MATRIX);
	pos_ins_ecef= mat_creat(3,1,ZERO_MATRIX);
	pos_ins_ned	= mat_creat(3,1,ZERO_MATRIX);
	
	pos_gps		= mat_creat(3,1,ZERO_MATRIX);
	pos_gps_ecef= mat_creat(3,1,ZERO_MATRIX);
	pos_gps_ned	= mat_creat(3,1,ZERO_MATRIX);
	
	I15			= mat_creat(15,15,UNIT_MATRIX);		// Identity
	I3			= mat_creat(3,3,UNIT_MATRIX);		// Identity
	ImKH		= mat_creat(15,15,ZERO_MATRIX);
	KRKt		= mat_creat(15,15,ZERO_MATRIX);
	
	dx 			= mat_creat(3,1,ZERO_MATRIX);		// Temporary to get dxdt
	a_temp31	= mat_creat(3,1,ZERO_MATRIX);		// Temporary
	b_temp31	= mat_creat(3,1,ZERO_MATRIX);		// Temporary
	temp33		= mat_creat(3,3,ZERO_MATRIX);		// Temporary
	atemp33		= mat_creat(3,3,ZERO_MATRIX);		// Temporary
	temp1515	= mat_creat(15,15,ZERO_MATRIX);
	temp615 	= mat_creat(6,15,ZERO_MATRIX);
	temp66		= mat_creat(6,6,ZERO_MATRIX);
	atemp66 	= mat_creat(6,6,ZERO_MATRIX);
	temp156		= mat_creat(15,6,ZERO_MATRIX);
	temp1512	= mat_creat(15,12,ZERO_MATRIX);
	
	// Assemble the matrices
	// .... gravity, g
	grav[2][0] = g;
	
	// ... H
	H[0][0] = 1.0; 	H[1][1] = 1.0; 	H[2][2] = 1.0;
	H[3][3] = 1.0; 	H[4][4] = 1.0; 	H[5][5] = 1.0;
	
	// ... Rw
	Rw[0][0] = SIG_W_AX*SIG_W_AX;		Rw[1][1] = SIG_W_AY*SIG_W_AY;			Rw[2][2] = SIG_W_AZ*SIG_W_AZ;
	Rw[3][3] = SIG_W_GX*SIG_W_GX;		Rw[4][4] = SIG_W_GY*SIG_W_GY;			Rw[5][5] = SIG_W_GZ*SIG_W_GZ;
	Rw[6][6] = 2*SIG_A_D*SIG_A_D/TAU_A;	Rw[7][7] = 2*SIG_A_D*SIG_A_D/TAU_A;		Rw[8][8] = 2*SIG_A_D*SIG_A_D/TAU_A;
	Rw[9][9] = 2*SIG_G_D*SIG_G_D/TAU_G;	Rw[10][10] = 2*SIG_G_D*SIG_G_D/TAU_G;	Rw[11][11] = 2*SIG_G_D*SIG_G_D/TAU_G;
	
	// ... P (initial)
	P[0][0] = P_P_INIT*P_P_INIT; 		P[1][1] = P_P_INIT*P_P_INIT; 		P[2][2] = P_P_INIT*P_P_INIT;
	P[3][3] = P_V_INIT*P_V_INIT; 		P[4][4] = P_V_INIT*P_V_INIT; 		P[5][5] = P_V_INIT*P_V_INIT;
	P[6][6] = P_A_INIT*P_A_INIT; 		P[7][7] = P_A_INIT*P_A_INIT; 		P[8][8] = P_HDG_INIT*P_HDG_INIT;
	P[9][9] = P_AB_INIT*P_AB_INIT; 		P[10][10] = P_AB_INIT*P_AB_INIT; 	P[11][11] = P_AB_INIT*P_AB_INIT;
	P[12][12] = P_GB_INIT*P_GB_INIT; 	P[13][13] = P_GB_INIT*P_GB_INIT; 	P[14][14] = P_GB_INIT*P_GB_INIT;
	
	// ... update P in get_nav
	navData_ptr->Pp[0] = P[0][0];	navData_ptr->Pp[1] = P[1][1];	navData_ptr->Pp[2] = P[2][2];
	navData_ptr->Pv[0] = P[3][3];	navData_ptr->Pv[1] = P[4][4];	navData_ptr->Pv[2] = P[5][5];
	navData_ptr->Pa[0] = P[6][6];	navData_ptr->Pa[1] = P[7][7];	navData_ptr->Pa[2] = P[8][8];
	
	navData_ptr->Pab[0] = P[9][9];	navData_ptr->Pab[1] = P[10][10];	navData_ptr->Pab[2] = P[11][11];
	navData_ptr->Pgb[0] = P[12][12];	navData_ptr->Pgb[1] = P[13][13];	navData_ptr->Pgb[2] = P[14][14];
	
	// ... R
	R[0][0] = SIG_GPS_P_NE*SIG_GPS_P_NE;	R[1][1] = SIG_GPS_P_NE*SIG_GPS_P_NE;	R[2][2] = SIG_GPS_P_D*SIG_GPS_P_D;
	R[3][3] = SIG_GPS_V*SIG_GPS_V;			R[4][4] = SIG_GPS_V*SIG_GPS_V;			R[5][5] = SIG_GPS_V*SIG_GPS_V;
	

	// .. then initialize states with GPS Data
	navData_ptr->lat = gpsData_ptr->lat*D2R;
	navData_ptr->lon = gpsData_ptr->lon*D2R;
	navData_ptr->alt = gpsData_ptr->alt;
	
	navData_ptr->vn = gpsData_ptr->vn;
	navData_ptr->ve = gpsData_ptr->ve;
	navData_ptr->vd = gpsData_ptr->vd;
	
	// ... and initialize states with IMU Data
	//navData_ptr->the = asin(imuData_ptr->ax/g); // theta from Ax, aircraft at rest
	//navData_ptr->phi = asin(-imuData_ptr->ay/(g*cos(navData_ptr->the))); // phi from Ay, aircraft at rest
	navData_ptr->the = 8*D2R;
	navData_ptr->phi = 0*D2R;
	navData_ptr->psi = 90.0*D2R;
	
	eul2quat(navData_ptr->quat,(navData_ptr->phi),(navData_ptr->the),(navData_ptr->psi));
	
	navData_ptr->ab[0] = 0.0;
	navData_ptr->ab[1] = 0.0; 
	navData_ptr->ab[2] = 0.0;
	
	navData_ptr->gb[0] = imuData_ptr->p;
	navData_ptr->gb[1] = imuData_ptr->q;
	navData_ptr->gb[2] = imuData_ptr->r;
	
	// Specific forces and Rotation Rate
	f_b[0][0] = imuData_ptr->ax - navData_ptr->ab[0];
	f_b[1][0] = imuData_ptr->ay - navData_ptr->ab[1];
	f_b[2][0] = imuData_ptr->az - navData_ptr->ab[2];
	
	om_ib[0][0] = imuData_ptr->p - navData_ptr->gb[0];
	om_ib[1][0] = imuData_ptr->q - navData_ptr->gb[1];
	om_ib[2][0] = imuData_ptr->r - navData_ptr->gb[2];
	
	// Time during initialization
	tprev = imuData_ptr->time;
	
	navData_ptr->err_type = data_valid;
	//send_status("NAV filter initialized");
	//fprintf(stderr,"NAV Data Initialization Completed\n");
}

// Main get_nav filter function
void get_nav(struct imu *imuData_ptr, struct gps *gpsData_ptr, struct nav *navData_ptr){
	double tnow, imu_dt;
	double dq[4], quat_new[4];

	// compute time-elapsed 'dt'
	// This compute the navigation state at the DAQ's Time Stamp
	tnow = imuData_ptr->time;
	imu_dt = tnow - tprev;
	tprev = tnow;	

	int debug = 0;
	if (debug) { printf("get_nav(): tnow = %.3f\n", tnow); }
	
	// ==================  Time Update  ===================
	// Temporary storage in Matrix form
	quat[0] = navData_ptr->quat[0];
	quat[1] = navData_ptr->quat[1];
	quat[2] = navData_ptr->quat[2];
	quat[3] = navData_ptr->quat[3];
	// printf("quat: %.4f %.4f %.4f %.4f\n", quat[0], quat[1], quat[2], quat[3]);
	
	a_temp31[0][0] = navData_ptr->vn; a_temp31[1][0] = navData_ptr->ve;
	a_temp31[2][0] = navData_ptr->vd;
	
	b_temp31[0][0] = navData_ptr->lat; b_temp31[1][0] = navData_ptr->lon;
	b_temp31[2][0] = navData_ptr->alt;
	
	// AHRS Transformations
	// printf("quat: %.4f %.4f %.4f %.4f\n", quat[0], quat[1], quat[2], quat[3]);
	C_N2B = quat2dcm(quat, C_N2B);
	C_B2N = mat_tran(C_N2B, C_B2N);
	// mat_dump(C_N2B);
	// mat_dump(C_B2N);

	if (debug) {
	    printf("quat: %.4f %.4f %.4f %.4f\n", quat[0], quat[1], quat[2], quat[3]);
	    printf("C_N2B:\n");
	    mat_dump(C_N2B);
	    printf("C_B2N:\n");
	    mat_dump(C_B2N);
	}
	
	// Attitude Update
	// ... Calculate Navigation Rate
	nr = navrate(a_temp31,b_temp31,nr);
	
	dq[0] = 1;
	dq[1] = 0.5*om_ib[0][0]*imu_dt;
	dq[2] = 0.5*om_ib[1][0]*imu_dt;
	dq[3] = 0.5*om_ib[2][0]*imu_dt;

	if (debug) {
	    printf("dq (gyro): %.4f %.4f %.4f %.4f\n", dq[0], dq[1], dq[2], dq[3]);
	}
	       
	qmult(quat,dq,quat_new);
	if (debug) {
	    printf("quat (pre normalized): %.4f %.4f %.4f %.4f\n", quat_new[0], quat_new[1], quat_new[2], quat_new[3]);
	}

	quat[0] = quat_new[0]/sqrt(quat_new[0]*quat_new[0] + quat_new[1]*quat_new[1] + quat_new[2]*quat_new[2] + quat_new[3]*quat_new[3]);
	quat[1] = quat_new[1]/sqrt(quat_new[0]*quat_new[0] + quat_new[1]*quat_new[1] + quat_new[2]*quat_new[2] + quat_new[3]*quat_new[3]);
	quat[2] = quat_new[2]/sqrt(quat_new[0]*quat_new[0] + quat_new[1]*quat_new[1] + quat_new[2]*quat_new[2] + quat_new[3]*quat_new[3]);
	quat[3] = quat_new[3]/sqrt(quat_new[0]*quat_new[0] + quat_new[1]*quat_new[1] + quat_new[2]*quat_new[2] + quat_new[3]*quat_new[3]);
	
    if(quat[0] < 0) {
        // Avoid quaternion flips sign
        quat[0] = -quat[0];
        quat[1] = -quat[1];
        quat[2] = -quat[2];
        quat[3] = -quat[3];
    }

    // printf("quat1: %.4f %.4f %.4f %.4f\n", quat[0], quat[1], quat[2], quat[3]);

	navData_ptr->quat[0] = quat[0];
	navData_ptr->quat[1] = quat[1];
	navData_ptr->quat[2] = quat[2];
	navData_ptr->quat[3] = quat[3];
	
	quat2eul(navData_ptr->quat,&(navData_ptr->phi),&(navData_ptr->the),&(navData_ptr->psi));
	
	// Velocity Update
	dx = mat_mul(C_B2N,f_b,dx);
	dx = mat_add(dx,grav,dx);
	// printf("dx (vel):\n"); mat_dump(dx);
	
	navData_ptr->vn += imu_dt*dx[0][0];
	navData_ptr->ve += imu_dt*dx[1][0];
	navData_ptr->vd += imu_dt*dx[2][0];
	
	// Position Update
	dx = llarate(a_temp31,b_temp31,dx);
	// printf("dx (pos):\n"); mat_dump(dx);
	navData_ptr->lat += imu_dt*dx[0][0];
	navData_ptr->lon += imu_dt*dx[1][0];
	navData_ptr->alt += imu_dt*dx[2][0];
	
	// JACOBIAN
	F = mat_fill(F, ZERO_MATRIX);
	// ... pos2gs
	F[0][3] = 1.0; 	F[1][4] = 1.0; 	F[2][5] = 1.0;
	// ... gs2pos
	F[5][2] = -2*g/EARTH_RADIUS;
	
	// ... gs2att
	// printf("f_b:\n"); mat_dump(f_b);
	temp33 = sk(f_b,temp33);
	//printf("temp33:\n"); mat_dump(temp33);
	atemp33 = mat_mul(C_B2N,temp33,atemp33);
	//printf("atemp33:\n"); mat_dump(atemp33);
	
	F[3][6] = -2.0*atemp33[0][0]; F[3][7] = -2.0*atemp33[0][1]; F[3][8] = -2.0*atemp33[0][2];
	F[4][6] = -2.0*atemp33[1][0]; F[4][7] = -2.0*atemp33[1][1]; F[4][8] = -2.0*atemp33[1][2];
	F[5][6] = -2.0*atemp33[2][0]; F[5][7] = -2.0*atemp33[2][1]; F[5][8] = -2.0*atemp33[2][2];
	
	// ... gs2acc
	F[3][9] = -C_B2N[0][0]; F[3][10] = -C_B2N[0][1]; F[3][11] = -C_B2N[0][2];
	F[4][9] = -C_B2N[1][0]; F[4][10] = -C_B2N[1][1]; F[4][11] = -C_B2N[1][2];
	F[5][9] = -C_B2N[2][0]; F[5][10] = -C_B2N[2][1]; F[5][11] = -C_B2N[2][2];
	
	// ... att2att
	temp33 = sk(om_ib,temp33);
	F[6][6] = -temp33[0][0]; F[6][7] = -temp33[0][1]; F[6][8] = -temp33[0][2];
	F[7][6] = -temp33[1][0]; F[7][7] = -temp33[1][1]; F[7][8] = -temp33[1][2];
	F[8][6] = -temp33[2][0]; F[8][7] = -temp33[2][1]; F[8][8] = -temp33[2][2];
	
	// ... att2gyr
	F[6][12] = -0.5;
	F[7][13] = -0.5;
	F[8][14] = -0.5;
	
	// ... Accel Markov Bias
	F[9][9] = -1.0/TAU_A; 	F[10][10] = -1.0/TAU_A;	F[11][11] = -1.0/TAU_A;
	F[12][12] = -1.0/TAU_G; F[13][13] = -1.0/TAU_G;	F[14][14] = -1.0/TAU_G;
	
	//fprintf(stderr,"Jacobian Created\n");
	
	// State Transition Matrix: PHI = I15 + F*dt;
	temp1515 = mat_scalMul(F,imu_dt,temp1515);
	PHI = mat_add(I15,temp1515,PHI);
	
	// Process Noise
	G = mat_fill(G, ZERO_MATRIX);
	G[3][0] = -C_B2N[0][0];	G[3][1] = -C_B2N[0][1]; G[3][2] = -C_B2N[0][2];
	G[4][0] = -C_B2N[1][0];	G[4][1] = -C_B2N[1][1]; G[4][2] = -C_B2N[1][2];
	G[5][0] = -C_B2N[2][0];	G[5][1] = -C_B2N[2][1]; G[5][2] = -C_B2N[2][2];
	
	G[6][3] = -0.5;
	G[7][4] = -0.5;
	G[8][5] = -0.5;
	
	G[9][6] = 1.0; 			G[10][7] = 1.0; 		G[11][8] = 1.0;
	G[12][9] = 1.0; 		G[13][10] = 1.0; 		G[14][11] = 1.0;

	//fprintf(stderr,"Process Noise Matrix G is created\n");
	// Discrete Process Noise
	temp1512 = mat_mul(G,Rw,temp1512);
	temp1515 = mat_transmul(temp1512,G,temp1515);	// Qw = G*Rw*G'
	Qw = mat_scalMul(temp1515,imu_dt,Qw);			// Qw = dt*G*Rw*G'
	Q = mat_mul(PHI,Qw,Q);						// Q = (I+F*dt)*Qw
	
	temp1515 = mat_tran(Q,temp1515);
	Q = mat_add(Q,temp1515,Q);
	Q = mat_scalMul(Q,0.5,Q);				// Q = 0.5*(Q+Q')
	//fprintf(stderr,"Discrete Process Noise is created\n");
	
	// Covariance Time Update
	temp1515 = mat_mul(PHI,P,temp1515);
	P = mat_transmul(temp1515,PHI,P); 		// P = PHI*P*PHI'
	P = mat_add(P,Q,P);						// P = PHI*P*PHI' + Q
	temp1515 = mat_tran(P, temp1515);
	P = mat_add(P,temp1515,P);
	P = mat_scalMul(P,0.5,P);				// P = 0.5*(P+P')
	//fprintf(stderr,"Covariance Updated through Time Update\n");
	
	navData_ptr->Pp[0] = P[0][0]; navData_ptr->Pp[1] = P[1][1]; navData_ptr->Pp[2] = P[2][2];
	navData_ptr->Pv[0] = P[3][3]; navData_ptr->Pv[1] = P[4][4]; navData_ptr->Pv[2] = P[5][5];
	navData_ptr->Pa[0] = P[6][6]; navData_ptr->Pa[1] = P[7][7]; navData_ptr->Pa[2] = P[8][8];
	navData_ptr->Pab[0] = P[9][9]; navData_ptr->Pab[1] = P[10][10]; navData_ptr->Pab[2] = P[11][11];
	navData_ptr->Pgb[0] = P[12][12]; navData_ptr->Pgb[1] = P[13][13]; navData_ptr->Pgb[2] = P[14][14];
	
	navData_ptr->err_type = TU_only;
	//fprintf(stderr,"Time Update Done\n");
	// ==================  DONE TU  ===================
	if(gpsData_ptr->newData){
	    if (debug) { printf("new gps data\n"); }
	    
		// ==================  GPS Update  ===================
		gpsData_ptr->newData = 0; // Reset the flag
		
		// Position, converted to NED
		a_temp31[0][0] = navData_ptr->lat;
		a_temp31[1][0] = navData_ptr->lon;
		a_temp31[2][0] = navData_ptr->alt;
		pos_ins_ecef = lla2ecef(a_temp31,pos_ins_ecef);
		// printf("pos_ins_ecef:\n"); mat_dump(pos_ins_ecef);

		a_temp31[2][0] = 0.0;
		//pos_ref = lla2ecef(a_temp31,pos_ref);
		pos_ref = mat_copy(a_temp31,pos_ref);
		pos_ins_ned = ecef2ned(pos_ins_ecef,pos_ins_ned,pos_ref);
		
		pos_gps[0][0] = gpsData_ptr->lat*D2R;
		pos_gps[1][0] = gpsData_ptr->lon*D2R;
		pos_gps[2][0] = gpsData_ptr->alt;
		
		pos_gps_ecef = lla2ecef(pos_gps,pos_gps_ecef);
		
		pos_gps_ned = ecef2ned(pos_gps_ecef,pos_gps_ned,pos_ref);
		
		// Create Measurement: y
		y[0][0] = pos_gps_ned[0][0] - pos_ins_ned[0][0];
		y[1][0] = pos_gps_ned[1][0] - pos_ins_ned[1][0];
		y[2][0] = pos_gps_ned[2][0] - pos_ins_ned[2][0];
		
		y[3][0] = gpsData_ptr->vn - navData_ptr->vn;
		y[4][0] = gpsData_ptr->ve - navData_ptr->ve;
		y[5][0] = gpsData_ptr->vd - navData_ptr->vd;
		// printf("y:\n"); mat_dump(y);
	
		//fprintf(stderr,"Measurement Matrix, y, created\n");
		
		// Kalman Gain
		temp615 = mat_mul(H,P,temp615);
		temp66 = mat_transmul(temp615,H,temp66);
		atemp66 = mat_add(temp66,R,atemp66);
		temp66 = mat_inv(atemp66,temp66); // temp66 = inv(H*P*H'+R)
		//fprintf(stderr,"inv(H*P*H'+R) Computed\n");
		
		temp156 = mat_transmul(P,H,temp156); // P*H'
		//fprintf(stderr,"P*H' Computed\n");
		K = mat_mul(temp156,temp66,K);	   // K = P*H'*inv(H*P*H'+R)
		//fprintf(stderr,"Kalman Gain Computed\n");
		
		// Covariance Update
		temp1515 = mat_mul(K,H,temp1515);
		ImKH = mat_sub(I15,temp1515,ImKH);	// ImKH = I - K*H
		
		temp615 = mat_transmul(R,K,temp615);
		KRKt = mat_mul(K,temp615,KRKt);		// KRKt = K*R*K'
		
		temp1515 = mat_transmul(P,ImKH,temp1515);
		P = mat_mul(ImKH,temp1515,P);		// ImKH*P*ImKH'
		temp1515 = mat_add(P,KRKt,temp1515);
		P = mat_copy(temp1515,P);			// P = ImKH*P*ImKH' + KRKt
		//fprintf(stderr,"Covariance Updated through GPS Update\n");
		
		navData_ptr->Pp[0] = P[0][0]; navData_ptr->Pp[1] = P[1][1]; navData_ptr->Pp[2] = P[2][2];
		navData_ptr->Pv[0] = P[3][3]; navData_ptr->Pv[1] = P[4][4]; navData_ptr->Pv[2] = P[5][5];
		navData_ptr->Pa[0] = P[6][6]; navData_ptr->Pa[1] = P[7][7]; navData_ptr->Pa[2] = P[8][8];
		navData_ptr->Pab[0] = P[9][9]; navData_ptr->Pab[1] = P[10][10]; navData_ptr->Pab[2] = P[11][11];
		navData_ptr->Pgb[0] = P[12][12]; navData_ptr->Pgb[1] = P[13][13]; navData_ptr->Pgb[2] = P[14][14];
		
		// State Update
		x = mat_mul(K,y,x);
		denom = (1.0 - (ECC2 * sin(navData_ptr->lat) * sin(navData_ptr->lat)));
		denom = sqrt(denom*denom);

		Re = EARTH_RADIUS / sqrt(denom);
		Rn = EARTH_RADIUS*(1-ECC2) / denom*sqrt(denom);
		navData_ptr->alt = navData_ptr->alt - x[2][0];
		navData_ptr->lat = navData_ptr->lat + x[0][0]/(Re + navData_ptr->alt);
		navData_ptr->lon = navData_ptr->lon + x[1][0]/(Rn + navData_ptr->alt)/cos(navData_ptr->lat);
		
		navData_ptr->vn = navData_ptr->vn + x[3][0];
		navData_ptr->ve = navData_ptr->ve + x[4][0];
		navData_ptr->vd = navData_ptr->vd + x[5][0];
		
		quat[0] = navData_ptr->quat[0];
		quat[1] = navData_ptr->quat[1];
		quat[2] = navData_ptr->quat[2];
		quat[3] = navData_ptr->quat[3];
		
		// Attitude correction
		dq[0] = 1.0;
		dq[1] = x[6][0];
		dq[2] = x[7][0];
		dq[3] = x[8][0];

		if (debug) {
		    printf("x[6-8] = %.4f %.4f %.4f\n", x[6][0], x[7][0], x[8][0]);
		    printf("dq (update): %.4f %.4f %.4f %.4f\n", dq[0], dq[1], dq[2], dq[3]);
		}
		
		qmult(quat,dq,quat_new);
		
		quat[0] = quat_new[0]/sqrt(quat_new[0]*quat_new[0] + quat_new[1]*quat_new[1] + quat_new[2]*quat_new[2] + quat_new[3]*quat_new[3]);
		quat[1] = quat_new[1]/sqrt(quat_new[0]*quat_new[0] + quat_new[1]*quat_new[1] + quat_new[2]*quat_new[2] + quat_new[3]*quat_new[3]);
		quat[2] = quat_new[2]/sqrt(quat_new[0]*quat_new[0] + quat_new[1]*quat_new[1] + quat_new[2]*quat_new[2] + quat_new[3]*quat_new[3]);
		quat[3] = quat_new[3]/sqrt(quat_new[0]*quat_new[0] + quat_new[1]*quat_new[1] + quat_new[2]*quat_new[2] + quat_new[3]*quat_new[3]);

		if (debug) {
		    printf("quat (update): %.4f %.4f %.4f %.4f\n", quat[0], quat[1], quat[2], quat[3]);
		}
		
		navData_ptr->quat[0] = quat[0];
		navData_ptr->quat[1] = quat[1];
		navData_ptr->quat[2] = quat[2];
		navData_ptr->quat[3] = quat[3];
		
		quat2eul(navData_ptr->quat,&(navData_ptr->phi),&(navData_ptr->the),&(navData_ptr->psi));
		
		navData_ptr->ab[0] = navData_ptr->ab[0] + x[9][0];
		navData_ptr->ab[1] = navData_ptr->ab[1] + x[10][0];
		navData_ptr->ab[2] = navData_ptr->ab[2] + x[11][0];
		
		navData_ptr->gb[0] = navData_ptr->gb[0] + x[12][0];
		navData_ptr->gb[1] = navData_ptr->gb[1] + x[13][0];
		navData_ptr->gb[2] = navData_ptr->gb[2] + x[14][0];
		
		navData_ptr->err_type = gps_aided;
		//fprintf(stderr,"Measurement Update Done\n");		
	}
	
	// Remove current estimated biases from rate gyro and accels
	imuData_ptr->p -= navData_ptr->gb[0];
	imuData_ptr->q -= navData_ptr->gb[1];
	imuData_ptr->r -= navData_ptr->gb[2];
	imuData_ptr->ax -= navData_ptr->ab[0];
	imuData_ptr->ay -= navData_ptr->ab[1];
	imuData_ptr->az -= navData_ptr->ab[2];

	// Get the new Specific forces and Rotation Rate,
	// use in the next time update
	f_b[0][0] = imuData_ptr->ax;
	f_b[1][0] = imuData_ptr->ay;
	f_b[2][0] = imuData_ptr->az;

	om_ib[0][0] = imuData_ptr->p;
	om_ib[1][0] = imuData_ptr->q;
	om_ib[2][0] = imuData_ptr->r;


}

void close_nav(void){
	//free memory space
	mat_free(F);
	mat_free(PHI);
	mat_free(P);
	mat_free(G);
	mat_free(Rw);
	mat_free(Q);
	mat_free(Qw);
	mat_free(R);
	mat_free(eul);
	mat_free(x);
	mat_free(y);
	mat_free(Rbodtonav);
	mat_free(grav);
	mat_free(f_b);
	mat_free(om_ib);
	mat_free(nr);
	mat_free(K);
	mat_free(H);
	mat_free(C_N2B);
	mat_free(C_B2N);
	mat_free(pos_ref);
	mat_free(pos_ins_ecef);
	mat_free(pos_ins_ned);
	mat_free(pos_gps);
	mat_free(pos_gps_ecef);
	mat_free(pos_gps_ned);
	mat_free(I15);
	mat_free(I3);
	mat_free(ImKH);
	mat_free(KRKt);
	mat_free(dx);
	mat_free(a_temp31);
	mat_free(b_temp31);
	mat_free(temp33);
	mat_free(atemp33);
	mat_free(temp1515);
	mat_free(temp615);
	mat_free(temp66);
	mat_free(atemp66);
	mat_free(temp156);
	mat_free(temp1512);
	
}
