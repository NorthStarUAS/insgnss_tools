/******************************************************************************
 * FILE: micronav_ahrs.c
 * DESCRIPTION: attitude heading reference system providing the attitude of
 *   	       the vehicle using an extended Kalman filter
 *
 *
 * REVISION: arrange routines to speed up the computational time. Slow down
 *           the Kalman filter update routine at 25Hz while the propagation
 *           is done at 50Hz.
 *
 * Original Author: 8/31/06 Jung Soon Jang
 *
 * University of Minnesota 
 * Aerospace Engineering and Mechanics 
 * Copyright 2011 Regents of the University of Minnesota. All rights reserved.
 *
 * $Id: micronav_ahrs.c 725 2011-11-23 21:29:55Z murch $
 ******************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <cyg/posix/pthread.h>
#include <cyg/kernel/kapi.h>
#include <cyg/cpuload/cpuload.h>

#include "../globaldefs.h"
#include "../extern_vars.h"
#include "../utils/matrix.h"
#include "../utils/misc.h"

#include "nav_functions.h"
#include "nav_interface.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//prototype definition
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
double 		wraparound(double dta);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//sensor characteristics
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*err covariance of accelerometers: users must change these values
  depending on the environment under the vehicle is in operation                      */
#define			var_az  0.962361        //(0.1*g)^2
#define			var_ax	0.962361
#define        	var_ay  0.962361
/*err covariance of magnetometer heading					      */
#define			var_psi 0.014924        //(7*d2r)^2

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//global variables
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
static MATRIX aP,aQ,aR,aK,Fsys,Hj,Iden;
static MATRIX tmp73,tmp33,tmp77,tmpr,Rinv,mat77;
static MATRIX Hpsi,Kpsi,tmp71;
static double tprev=0;
#ifdef HIL_SIM
	// Initial value for HIL sim: phi,theta,psi = [0,0.0517,PI/2], 321 rotation
static 	double xs[7]={0.7069,0.7069,0.01828,-0.01828,0,0,0}; // four quaternions, 3 rate gyro biases
#else
	// normal state inital value for flight build
static double xs[7]={1,0,0,0,0,0,0}; // four quaternions, 3 rate gyro biases
#endif

static short  i=0,magCheck=0;
static double pc,qc,rc;
static double norm_ahrs,Bxc,Byc,invR,cPHI,sPHI;
static double Adt,Hdt,tnow,h_ahrs[3]={0.,};
static double coeff1[3]={0,},temp[2]={0,}, h_b[3][1];

	// ======================== magnetometer calibration data ==============================
	/*
#ifdef FLIGHT_BUILD
	// new calibration data (uNAV#?, tested on 10=06=07)
	static float K_G[3][3] = {{0.53803532638536,   0.02134656564593,   0.00382173456905},
			{0.02134656564593,   0.53583969599251,  -0.01755859489163},
			{0.00382173456905,  -0.01755859489163,   0.52788394795182}};

	static float be[3]  = {0.06997459270763,   0.05852923762322,  -0.08405181072893};

#else
	static float  K_G[3][3] = {{1, 0, 0},
			{0, 1, 0},
			{0, 0, 1}};
	static float be[3] = {0, 0, 0};
#endif
	 */
	static float  K_G[3][3] =
			{{1, 0, 0},
			{0, 1, 0},
			{0, 0, 1}};
	static float be[3] = {0, 0, 0};
	// =====================================================================================
	
	
	
void init_nav(struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr)
{

	// initialization of err, measurement, and process cov. matrices
	aP = mat_creat(7,7,ZERO_MATRIX);
	aQ = mat_creat(7,7,ZERO_MATRIX);
	aR = mat_creat(3,3,ZERO_MATRIX);

	aP[0][0]=aP[1][1]=aP[2][2]=aP[3][3]=1.0e-1; aP[4][4]=aP[5][5]=aP[6][6]=1.0e-1;
	aQ[0][0]=aQ[1][1]=aQ[2][2]=aQ[3][3]=1.0e-7; aQ[4][4]=aQ[5][5]=aQ[6][6]=1.0e-11;
	aR[0][0]=aR[1][1]=aR[2][2]=var_ax;

	//initialization of gain matrix
	aK = mat_creat(7,3,ZERO_MATRIX);
	//initialization of state transition matrix
	Fsys = mat_creat(7,7,UNIT_MATRIX);
	//initialization of Identity matrix
	Iden = mat_creat(7,7,UNIT_MATRIX);
	//initialization of Jacobian matrix
	Hj   = mat_creat(3,7,ZERO_MATRIX);
	//initialization related to heading
	Hpsi  = mat_creat(1,7,ZERO_MATRIX);
	Kpsi  = mat_creat(7,1,ZERO_MATRIX);
	tmp71 = mat_creat(7,1,ZERO_MATRIX);
	//initialization of other matrice used in ahrs
	Rinv  = mat_creat(3,3,ZERO_MATRIX);
	tmp33 = mat_creat(3,3,ZERO_MATRIX);
	tmp73 = mat_creat(7,3,ZERO_MATRIX);
	tmp77 = mat_creat(7,7,ZERO_MATRIX);
	tmpr  = mat_creat(7,7,ZERO_MATRIX);
	mat77 = mat_creat(7,7,ZERO_MATRIX);

	// Initialize gyro bias with current values
	 xs[4] = sensorData_ptr->imuData_ptr->p;
	 xs[5] = sensorData_ptr->imuData_ptr->q;
	 xs[6] = sensorData_ptr->imuData_ptr->r;

	navData_ptr->err_type = data_valid;
	send_status("NAV filter initialized");

}

void get_nav(struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr)
{
	
//	// add back in biases
//	sensorData_ptr->imuData_ptr->p += navData_ptr->gb[0];
//	sensorData_ptr->imuData_ptr->q += navData_ptr->gb[1];
//	sensorData_ptr->imuData_ptr->r += navData_ptr->gb[2];
//	sensorData_ptr->imuData_ptr->ax += navData_ptr->ab[0];
//	sensorData_ptr->imuData_ptr->ay += navData_ptr->ab[1];
//	sensorData_ptr->imuData_ptr->az += navData_ptr->ab[2];
	
	// ====================== AHRS algorithm begins =======================================

	//snap the time interval, dt, of this routine
	tnow = get_Time();
	Adt   = tnow - tprev;
	tprev= tnow;
	if (Adt<=0) Adt = 0.020;

	Hdt = 0.5*Adt;

	/*assign new variables			*/
	pc = sensorData_ptr->imuData_ptr->p*Hdt;
	qc = sensorData_ptr->imuData_ptr->q*Hdt;
	rc = sensorData_ptr->imuData_ptr->r*Hdt;

	/*state transition matrix			*/
	Fsys[0][1] = -pc; Fsys[0][2] = -qc; Fsys[0][3] = -rc;
	Fsys[1][0] =  pc; Fsys[1][2] =  rc; Fsys[1][3] = -qc;
	Fsys[2][0] =  qc; Fsys[2][1] = -rc; Fsys[2][3] =  pc;
	Fsys[3][0] =  rc; Fsys[3][1] =  qc; Fsys[3][2] = -pc;

	Fsys[0][4] = xs[1]*Hdt;  Fsys[0][5] = xs[2]*Hdt;  Fsys[0][6] = xs[3]*Hdt;
	Fsys[1][4] =-xs[0]*Hdt;  Fsys[1][5] = xs[3]*Hdt;  Fsys[1][6] =-Fsys[0][5];
	Fsys[2][4] =-Fsys[1][5]; Fsys[2][5] = Fsys[1][4]; Fsys[2][6] = Fsys[0][4];
	Fsys[3][4] = Fsys[0][5]; Fsys[3][5] =-Fsys[0][4]; Fsys[3][6] = Fsys[1][4];


	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//Extended Kalman filter: prediction step
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//propagation of quaternion using gyro measurement at a given sampling interval dt                                   
	xs[0] += -pc*xs[1] - qc*xs[2] - rc*xs[3];
	xs[1] +=  pc*xs[0] - qc*xs[3] + rc*xs[2];
	xs[2] +=  pc*xs[3] + qc*xs[0] - rc*xs[1];
	xs[3] += -pc*xs[2] + qc*xs[1] + rc*xs[0];

	//error covriance propagation: P = Fsys*P*Fsys' + Q
	mat_mul(Fsys,aP,tmp77);
	mat_transmul(tmp77,Fsys,aP);
	for(i=0;i<7;i++) aP[i][i] += aQ[i][i];

	// Pitch and Roll Update

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//Extended Kalman filter: correction step for pitch and roll
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//nonlinear measurement equation of h_ahrs(x)
	h_ahrs[0]    = -g2*(xs[1]*xs[3]-xs[0]*xs[2]) + 0.05*(sensorData_ptr->adData_ptr->ias)*sensorData_ptr->imuData_ptr->q;
	h_ahrs[1]    = -g2*(xs[0]*xs[1]+xs[2]*xs[3]) + (sensorData_ptr->adData_ptr->ias)*sensorData_ptr->imuData_ptr->r - 0.05*(sensorData_ptr->adData_ptr->ias)*sensorData_ptr->imuData_ptr->p;
	h_ahrs[2]    = -g*(xs[0]*xs[0]-xs[1]*xs[1]-xs[2]*xs[2]+xs[3]*xs[3]) - (sensorData_ptr->adData_ptr->ias)*sensorData_ptr->imuData_ptr->q;

	//compute Jacobian matrix of h_ahrs(x)
	Hj[0][0] = g2*xs[2]; Hj[0][1] =-g2*xs[3]; Hj[0][2] = g2*xs[0]; Hj[0][3] = -g2*xs[1]; Hj[0][5] = -0.05*(sensorData_ptr->adData_ptr->ias);
	Hj[1][0] = Hj[0][3]; Hj[1][1] =-Hj[0][2]; Hj[1][2] = Hj[0][1]; Hj[1][3] = -Hj[0][0]; Hj[1][4] = 0.05*(sensorData_ptr->adData_ptr->ias); Hj[1][6] = -(sensorData_ptr->adData_ptr->ias);
	Hj[2][0] =-Hj[0][2]; Hj[2][1] =-Hj[0][3]; Hj[2][2] = Hj[0][0]; Hj[2][3] =  Hj[0][1]; Hj[2][5] = (sensorData_ptr->adData_ptr->ias);

	//gain matrix aK = aP*Hj'*(Hj*aP*Hj' + aR)^-1
	mat_transmul(aP,Hj,tmp73);
	mat_mul(Hj,tmp73,tmp33);
	for(i=0;i<3;i++) tmp33[i][i] += aR[i][i];
	mat_inv(tmp33,Rinv);
	mat_mul(tmp73,Rinv,aK);

	//state update
	for(i=0;i<7;i++)
	{
		xs[i] += aK[i][0]*(sensorData_ptr->imuData_ptr->ax - h_ahrs[0])
								+  aK[i][1]*(sensorData_ptr->imuData_ptr->ay - h_ahrs[1])
								+  aK[i][2]*(sensorData_ptr->imuData_ptr->az - h_ahrs[2]);
	}

	//error covariance matrix update aP = (I - aK*Hj)*aP
	mat_mul(aK,Hj,mat77);
	mat_sub(Iden,mat77,tmpr);
	mat_mul(tmpr,aP,tmp77);
	mat_copy(tmp77,aP);
	

	if(++magCheck==5) // Heading update at 10 Hz
	{
		magCheck = 0;
		//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		// second stage kalman filter update to estimate the heading angle
		//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

		// corrected body axes magnetic field strength vector
		// h_body = K_G*(h_measured-bias)
		h_b[0][0] = K_G[0][0]*(sensorData_ptr->imuData_ptr->hx - be[0]) + K_G[0][1]*(sensorData_ptr->imuData_ptr->hy - be[1]) + K_G[0][2]*(sensorData_ptr->imuData_ptr->hz - be[2]);
		h_b[1][0] = K_G[1][0]*(sensorData_ptr->imuData_ptr->hx - be[0]) + K_G[1][1]*(sensorData_ptr->imuData_ptr->hy - be[1]) + K_G[1][2]*(sensorData_ptr->imuData_ptr->hz - be[2]);
		h_b[2][0] = K_G[2][0]*(sensorData_ptr->imuData_ptr->hx - be[0]) + K_G[2][1]*(sensorData_ptr->imuData_ptr->hy - be[1]) + K_G[2][2]*(sensorData_ptr->imuData_ptr->hz - be[2]);

		//magnetic heading correction due to roll and pitch angle
		cPHI= cos(sensorData_ptr->imuData_ptr->phi);
		sPHI= sin(sensorData_ptr->imuData_ptr->phi);
		Bxc = h_b[0][0]*cos(sensorData_ptr->imuData_ptr->the) + (h_b[1][0]*sPHI + h_b[2][0]*cPHI)*sin(sensorData_ptr->imuData_ptr->the);
		Byc = h_b[1][0]*cPHI - h_b[2][0]*sPHI;

		//Jacobian
		coeff1[0]= 2*(xs[1]*xs[2]+xs[0]*xs[3]);
		coeff1[1]= 1 - 2*(xs[2]*xs[2]+xs[3]*xs[3]);
		coeff1[2]= 2/(coeff1[0]*coeff1[0]+coeff1[1]*coeff1[1]);

		temp[0] = coeff1[1]*coeff1[2];
		temp[1] = coeff1[0]*coeff1[2];

		Hpsi[0][0] = xs[3]*temp[0];
		Hpsi[0][1] = xs[2]*temp[0];
		Hpsi[0][2] = xs[1]*temp[0]+2*xs[2]*temp[1];
		Hpsi[0][3] = xs[0]*temp[0]+2*xs[3]*temp[1];

		//gain matrix Kpsi = aP*Hpsi'*(Hpsi*aP*Hpsi' + Rpsi)^-1
		mat_transmul(aP,Hpsi,tmp71);
		invR = 1/(Hpsi[0][0]*tmp71[0][0]+Hpsi[0][1]*tmp71[1][0]+Hpsi[0][2]*tmp71[2][0]+Hpsi[0][3]*tmp71[3][0]+var_psi);

		//state update
		sensorData_ptr->imuData_ptr->psi = atan2(coeff1[0],coeff1[1]);
		for(i=0;i<7;i++) {
			Kpsi[i][0] = invR*tmp71[i][0];
			xs[i] += Kpsi[i][0]*wraparound(atan2(-Byc,Bxc) - sensorData_ptr->imuData_ptr->psi);
		}

		//error covariance matrix update aP = (I - Kpsi*Hpsi)*aP
		mat_mul(Kpsi,Hpsi,mat77);
		mat_sub(Iden,mat77,tmpr);
		mat_mul(tmpr,aP,tmp77);
		mat_copy(tmp77,aP);
		
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//scaling of quertonian,||q||^2 = 1
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	norm_ahrs = 1.0/sqrt(xs[0]*xs[0]+xs[1]*xs[1]+xs[2]*xs[2]+xs[3]*xs[3]);
	for(i=0;i<4;i++) xs[i] = xs[i]*norm_ahrs;

	//obtain euler angles from quaternion
	navData_ptr->the = asin(-2*(xs[1]*xs[3]-xs[0]*xs[2]));
	navData_ptr->phi = atan2(2*(xs[0]*xs[1]+xs[2]*xs[3]),1-2*(xs[1]*xs[1]+xs[2]*xs[2]));
	navData_ptr->psi = atan2(2*(xs[1]*xs[2]+xs[0]*xs[3]),1-2*(xs[2]*xs[2]+xs[3]*xs[3]));
	
	// Store gyro biases
	navData_ptr->gb[0] = xs[4];
	navData_ptr->gb[1] = xs[5];
	navData_ptr->gb[2] = xs[6];
	
}


void close_nav(void){
	//free memory space
	mat_free(aP);
	mat_free(aQ);
	mat_free(aR);
	mat_free(aK);
	mat_free(Fsys);
	mat_free(Iden);
	mat_free(Hj);
	mat_free(Rinv);
	mat_free(tmp77);
	mat_free(tmp33);
	mat_free(tmp73);
	mat_free(tmpr);
	mat_free(mat77);
	mat_free(Kpsi);
	mat_free(Hpsi);
	mat_free(tmp71);
}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// wrap around for -180 and + 180
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
double wraparound(double dta)
{
	//bound heading angle between -180 and 180
	if(dta >  PI) dta -= PI2;
	if(dta < -PI) dta += PI2;
	return dta;
}
