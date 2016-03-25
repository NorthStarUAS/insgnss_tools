/*! \file nav_functions.c
 *	\brief Auxiliary functions for nav filter
 *
 *	\details
 *     Module:          Navfuncs.c
 *     Modified:        Adhika Lie (revamp all functions)
 * 						Gokhan Inalhan (remaining)
 *                      Demoz Gebre (first three functions)
 *                      Jung Soon Jang
 *
 *     Description:     navfunc.c contains the listing for all the
 *                      real-time inertial navigation software.
 *
 *		Note: all the functions here do not create memory without
 *			  clearing it.
 *	\ingroup nav_fcns
 *
 * \author University of Minnesota
 * \author Aerospace Engineering and Mechanics
 * \copyright Copyright 2011 Regents of the University of Minnesota. All rights reserved.
 *
 * $Id$
 */

/*     Include Pertinent Header Files */

#include <math.h>
#include "../utils/matrix.h"
#include "nav_functions.h"

/*=================================================================*/
MATRIX eul2dcm(MATRIX euler, MATRIX dcm)
{
	/* Function:     MATRIX eul2dcm(MATRIX euler, MATRIX dcm)
	 * ----------------------------------------------------------------
	 * This function creates the direction cosine matrix (DCM) that
	 * transforms a vector from navigation frame to the body frame given 
	 * a set of Euler Angle in the form of [phi theta psi] for a 3-2-1 
	 * rotation sequence
	 */
  double cPHI,sPHI,cTHE,sTHE,cPSI,sPSI;

  if (MatRow(euler) != 3 || MatCol(euler) != 1 || MatCol(dcm) != 3 || MatRow(dcm) != 3) {
	  printf("eul2dcm error: incompatible matrix size\n");
  }
  cPHI = cos(euler[0][0]); sPHI = sin(euler[0][0]);
  cTHE = cos(euler[1][0]); sTHE = sin(euler[1][0]);
  cPSI = cos(euler[2][0]); sPSI = sin(euler[2][0]);

  dcm[0][0] = cTHE*cPSI; 				dcm[0][1] = cTHE*sPSI; 					dcm[0][2] = -sTHE;
  dcm[1][0] = sPHI*sTHE*cPSI-cPHI*sPSI;	dcm[1][1] = sPHI*sTHE*sPSI+cPHI*cPSI;	dcm[1][2] = sPHI*cTHE;
  dcm[2][0] = cPHI*sTHE*cPSI+sPHI*sPSI;	dcm[2][1] = cPHI*sTHE*sPSI-sPHI*cPSI;	dcm[2][2] = cPHI*cTHE;

  return(dcm);
}

/*=================================================================*/
MATRIX dcm2eul(MATRIX euler, MATRIX dcm) 
{
	/*
	* Function:     MATRIX dcm2eul(MATRIX euler, MATRIX dcm)
	*-----------------------------------------------------------------
	* Convert *any* DCM into its Euler Angle equivalent. For navigatin, 
	* use DCM from NED to Body-fixed as input to get the conevntional 
	* euler angles.
	* The output argument 'euler' is a vector containing the
	* the three euler angles in radians given in [phi; theta; psi] format.
	* Modified: Adhika Lie, 09/13/2011.
	*/

	if (MatRow(euler) != 3 || MatCol(euler) != 1 || MatCol(dcm) != 3 || MatRow(dcm) != 3) {
		printf("dcm2eul error: incompatible matrix size\n");
	}

	euler[0][0] = atan2(dcm[1][2],dcm[2][2]);
	euler[1][0] = -asin(dcm[0][2]);
	euler[2][0] = atan2(dcm[0][1],dcm[0][0]);
	
	return euler;
}

MATRIX create_R(MATRIX e, MATRIX R)
{
	/* This function is used to create the transformation matrix to get
	 * phi_dot, the_dot and psi_dot from given pqr (body rate).
	 */
	double ph, th, ps;

	if (MatRow(e) != 3 || MatCol(e) != 1 || MatCol(R) != 3 || MatRow(R) != 3) {
		printf("create_R error: incompatible matrix size\n");
	}

	ph = e[0][0]; 
	th = e[1][0]; 
	ps = e[2][0];
	
	R[0][0] = 1.0;
	R[0][1] = sin(ph)*tan(th);
	R[0][2] = cos(ph)*tan(th);
	
	R[1][0] = 0.0;
	R[1][1] = cos(ph);
	R[1][2] = -sin(ph);
	
	R[2][0] = 0.0;
	R[2][1] = sin(ph)/cos(th);
	R[2][2] = cos(ph)/cos(th);
	
	return R;
}

MATRIX llarate(MATRIX V, MATRIX lla, MATRIX lla_dot) 
{
	/* This function calculates the rate of change of latitude, longitude,
	 * and altitude.
	 * Using WGS-84.
	 */
	double lat, h, Rew, Rns, denom;

	if (MatRow(lla) != 3 || MatCol(lla) != 1 || MatCol(V) != 1 || MatRow(V) != 3 || MatRow(lla_dot) != 3 || MatCol(lla_dot) != 1) {
		printf("llarate error: incompatible matrix size\n");
	}

	lat = lla[0][0]; h = lla[2][0];
	
	denom = (1.0 - (ECC2 * sin(lat) * sin(lat)));
	denom = sqrt(denom*denom);

	Rew = EARTH_RADIUS / sqrt(denom);
	Rns = EARTH_RADIUS*(1-ECC2) / denom*sqrt(denom);
	
	lla_dot[0][0] = V[0][0]/(Rns + h);
	lla_dot[1][0] = V[1][0]/((Rew + h)*cos(lat));
	lla_dot[2][0] = -V[2][0];
	
	return lla_dot;
}

MATRIX navrate(MATRIX V, MATRIX lla, MATRIX nr) 
{
	/* This function calculates the angular velocity of the NED frame, 
	 * also known as the navigation rate.
	 * Using WGS-84.
	 */
	double lat, h, Rew, Rns, denom;

	if (MatRow(lla) != 3 || MatCol(lla) != 1 || MatCol(V) != 1 || MatRow(V) != 3 || MatRow(nr) != 3 || MatCol(nr) != 1) {
		printf("navrate error: incompatible matrix size\n");
	}

	lat = lla[0][0]; 
	h = lla[2][0];
	
	denom = (1.0 - (ECC2 * sin(lat) * sin(lat)));
	denom = sqrt(denom*denom);

	Rew = EARTH_RADIUS / sqrt(denom);
	Rns = EARTH_RADIUS*(1-ECC2) / denom*sqrt(denom);
	
	nr[0][0] = V[1][0]/(Rew + h);
	nr[1][0] = -V[0][0]/(Rns + h);
	nr[2][0] = -V[1][0]*tan(lat)/(Rew + h);
	
	return nr;
}

MATRIX ecef2lla(MATRIX ecef, MATRIX lla)
{
	/* This function calculates the Latitude, Longitude and Altitude of a
	 * point located on earth given the ECEF Coordinate.
	 * Reference: Jekeli, C.,"Inertial Navigation Systems With Geodetic 
	         Applications", Walter de Gruyter, New York, 2001, pp. 24
	*/
	double x, y, z;
	double lat, lon, alt, p, err, denom, Rew;

	if (MatRow(ecef) != 3 || MatCol(ecef) != 1 || MatRow(lla) != 3 || MatCol(lla) != 1) {
		printf("ecef2lla error: incompatible matrix size\n");
	}

	x = ecef[0][0]; y = ecef[1][0]; z = ecef[2][0];
	lon = atan2(y,x);
	
	p = sqrt(x*x+y*y);

	lat = atan2(z,p*(1-ECC2));
	
	err = 1.0;
	while (fabs(err)>1e-14){
		denom = (1.0 - (ECC2 * sin(lat) * sin(lat)));
		denom = sqrt(denom*denom);

		Rew = EARTH_RADIUS / sqrt(denom);
		
		alt = p/cos(lat) - Rew;
		
		err = atan2(z*(1+ECC2*Rew*sin(lat)/z),p) - lat;
		lat = lat + err;
	}
	
	lla[0][0] = lat;
	lla[1][0] = lon;
	lla[2][0] = alt;
	
	return lla;
}

MATRIX lla2ecef(MATRIX lla, MATRIX ecef)
{  
	/* This function calculates the ECEF Coordinate given the Latitude,
	 * Longitude and Altitude.
	 */
	
	double Rew, alt, denom;
	double sinlat, coslat, coslon, sinlon;

	if (MatRow(ecef) != 3 || MatCol(ecef) != 1 || MatRow(lla) != 3 || MatCol(lla) != 1) {
		printf("lla2ecef error: incompatible matrix size\n");
	}

	sinlat = sin(lla[0][0]);
	coslat = cos(lla[0][0]);
	coslon = cos(lla[1][0]);
	sinlon = sin(lla[1][0]);
	alt = lla[2][0];

	denom = (1.0 - (ECC2 * sinlat * sinlat));
	denom = sqrt(denom*denom);

	Rew = EARTH_RADIUS / sqrt(denom);
  
	ecef[0][0] = (Rew + alt) * coslat * coslon;
	ecef[1][0] = (Rew + alt) * coslat * sinlon;
	ecef[2][0] = (Rew * (1.0 - ECC2) + alt) * sinlat;
	
	return ecef;
}

MATRIX ecef2ned(MATRIX ecef, MATRIX ned, MATRIX pos_ref)
{
	/* This function converts a vector in ecef to ned coordinate centered
	 * at ecef_ref.
	 */
	//MATRIX lla_ref = mat_creat(3,3,ZERO_MATRIX);
	double lat, lon;
	
	//lla_ref = ecef2lla(ecef_ref, lla_ref);
	//lat = lla_ref[0][0];
	//lon = lla_ref[1][0];

	if (MatRow(ecef) != 3 || MatCol(ecef) != 1 || MatRow(ned) != 3 || MatCol(ned) != 1 || MatRow(pos_ref) != 3 || MatCol(pos_ref) != 1) {
		printf("ecef2ned error: incompatible matrix size\n");
	}

	lat = pos_ref[0][0];
	lon = pos_ref[1][0];
	
	ned[2][0]=-cos(lat)*cos(lon)*ecef[0][0]-cos(lat)*sin(lon)*ecef[1][0]-sin(lat)*ecef[2][0];
	ned[1][0]=-sin(lon)*ecef[0][0] + cos(lon)*ecef[1][0];
	ned[0][0]=-sin(lat)*cos(lon)*ecef[0][0]-sin(lat)*sin(lon)*ecef[1][0]+cos(lat)*ecef[2][0];
	
	//mat_free(lla_ref);
	
	return ned;
}

MATRIX sk(MATRIX w, MATRIX C)
{
	/* This function gives a skew symmetric matrix from a given vector w
	 */
	
	if (MatRow(w) != 3 || MatCol(w) != 1 || MatRow(C) != 3 || MatCol(C) != 3) {
		printf("lla2ecef error: incompatible matrix size\n");
	}

	C[0][0] = 0.0;			C[0][1] = -w[2][0];		C[0][2] = w[1][0];
	C[1][0] = w[2][0];		C[1][1] = 0.0;			C[1][2] = -w[0][0];
	C[2][0] = -w[1][0];		C[2][1] = w[0][0];		C[2][2] = 0.0;
	
	return C;
}

MATRIX ortho(MATRIX C, MATRIX C_ortho)
{
	/*
	This function orthogonalizes a DCM by method presented in the paper
	Bar-Itzhack: "Orthogonalization Techniques of DCM" (1969, IEEE)
	Input:
		C: DCM, 3x3
	Output:
		C_ortho: Orthogonalized DCM, 3x3
	Programmer:    Adhika Lie
	Created:    	 May 10, 2011
	Last Modified: May 10, 2011
	*/
	
	if (MatRow(C) != 3 || MatCol(C) != 3 || MatRow(C_ortho) != 3 || MatCol(C_ortho) != 3) {
		printf("ortho error: incompatible matrix size\n");
	}

	MATRIX e = mat_creat(3,1,ONES_MATRIX);
	MATRIX w1 = mat_creat(3,1,ONES_MATRIX);
	MATRIX w1_p = mat_creat(3,1,ONES_MATRIX);
	MATRIX w1_n = mat_creat(3,1,ONES_MATRIX);
	MATRIX w2 = mat_creat(3,1,ONES_MATRIX);
	MATRIX w2_p = mat_creat(3,1,ONES_MATRIX);
	MATRIX w2_n = mat_creat(3,1,ONES_MATRIX);
	MATRIX w3 = mat_creat(3,1,ONES_MATRIX);
	MATRIX w3_p = mat_creat(3,1,ONES_MATRIX);
	MATRIX w3_n = mat_creat(3,1,ONES_MATRIX);
	double mag_w1, mag_w2, mag_w3;
	
	w1[0][0] = C[0][0]; 
	w1[1][0] = C[1][0];
	w1[2][0] = C[2][0];
	mag_w1 = norm(w1);
	mag_w1 = 1.0/mag_w1;
	w1 = mat_scalMul(w1,mag_w1,w1);
	
	w2[0][0] = C[0][1]; 
	w2[1][0] = C[1][1];
	w2[2][0] = C[2][1];
	mag_w2 = norm(w2);
	mag_w2 = 1.0/mag_w2;
	w2 = mat_scalMul(w2,mag_w2,w2);
	
	w3[0][0] = C[0][2]; 
	w3[1][0] = C[1][2];
	w3[2][0] = C[2][2];
	mag_w3 = norm(w3);
	mag_w3 = 1.0/mag_w3;
	w3 = mat_scalMul(w3,mag_w3,w3);
	
	while (norm(e) > 1e-15){
		w1_p = cross(w2,w3,w1_p);
		w2_p = cross(w3,w1,w2_p);
		w3_p = cross(w1,w2,w3_p);
		
		w1_n = mat_add(w1,w1_p,w1_n);
		w1_n = mat_scalMul(w1_n,0.5,w1_n);
		w1 = mat_scalMul(w1_n,1.0/norm(w1_n),w1);
		
		w2_n = mat_add(w2,w2_p,w2_n);
		w2_n = mat_scalMul(w2_n,0.5,w2_n);
		w2 = mat_scalMul(w2_n,1.0/norm(w2_n),w2);
		
		w3_n = mat_add(w3,w3_p,w3_n);
		w3_n = mat_scalMul(w3_n,0.5,w3_n);
		w3 = mat_scalMul(w3_n,1.0/norm(w3_n),w3);
		
		w1_p = cross(w2,w3,w1_p);
		w2_p = cross(w3,w1,w2_p);
		w3_p = cross(w1,w2,w3_p);
		
		w1_n = mat_sub(w1,w1_p,w1_n);
		e[0][0] = norm(w1_n);
		w2_n = mat_sub(w2,w2_p,w2_n);
		e[1][0] = norm(w2_n);
		w3_n = mat_sub(w3,w3_p,w3_n);
		e[2][0] = norm(w3_n);
	}
	
	C_ortho[0][0] = w1[0][0]; C_ortho[0][1] = w2[0][0];	C_ortho[0][2] = w3[0][0];
	C_ortho[1][0] = w1[1][0]; C_ortho[1][1] = w2[1][0];	C_ortho[1][2] = w3[1][0];
	C_ortho[2][0] = w1[2][0]; C_ortho[2][1] = w2[2][0];	C_ortho[2][2] = w3[2][0];
	
	return C_ortho;
}

double norm (MATRIX a) 
{
	int i;
	double mag = 0.0;
	
	for (i=0;i<3;i++) mag += a[i][0]*a[i][0];
	
	return sqrt(mag);
}

MATRIX cross (MATRIX a, MATRIX b, MATRIX c)
{
	// c = a x b;
	if (MatRow(a) != 3 || MatCol(a) != 1 || MatRow(b) != 3 || MatCol(b) != 1 || MatRow(c) != 3 || MatCol(c) != 1) {
		printf("cross error: incompatible matrix size\n");
	}
	c[0][0] = a[1][0]*b[2][0] - a[2][0]*b[1][0];
	c[1][0] = a[2][0]*b[0][0] - a[0][0]*b[2][0];
	c[2][0] = a[0][0]*b[1][0] - a[1][0]*b[0][0];
	return c;
}

/*=====================================================================*/
/*======================= QUATERNION FUNCTIONS ========================*/
/*=====================================================================*/
void qmult(double *p, double *q, double *r)
{
	/* Quaternion Multiplication: r = p x q
	 */
	int i;
	
	for(i=0;i<3;i++) r[i] = 0.0;
	
	r[0] = p[0]*q[0] - (p[1]*q[1] + p[2]*q[2] + p[3]*q[3]);
	r[1] = p[0]*q[1] + q[0]*p[1] + p[2]*q[3] - p[3]*q[2];
	r[2] = p[0]*q[2] + q[0]*p[2] + p[3]*q[1] - p[1]*q[3];
	r[3] = p[0]*q[3] + q[0]*p[3] + p[1]*q[2] - p[2]*q[1];
}

void quat2eul(double *q, double *phi, double *the, double *psi) {
	// Quaternion to Euler Angle
	double q0, q1, q2, q3;
	double m11, m12, m13, m23, m33;
	
	q0 = q[0];
	q1 = q[1];
	q2 = q[2];
	q3 = q[3];

	m11 = 2*q0*q0 +2*q1*q1 -1;
	m12 = 2*q1*q2 + 2*q0*q3;
	m13 = 2*q1*q3 - 2*q0*q2;
	m23 = 2*q2*q3 + 2*q0*q1;
	m33 = 2*q0*q0 + 2*q3*q3 - 1;
	
	*psi = atan2(m12,m11);
	*the = asin(-m13);
	*phi = atan2(m23,m33);
}

void eul2quat(double *q, double phi, double the, double psi) {
	phi = phi/2.0;
	the = the/2.0;
	psi = psi/2.0;
	
	q[0] = cos(psi)*cos(the)*cos(phi) + sin(psi)*sin(the)*sin(phi);  
	q[1] = cos(psi)*cos(the)*sin(phi) - sin(psi)*sin(the)*cos(phi);
	q[2] = cos(psi)*sin(the)*cos(phi) + sin(psi)*cos(the)*sin(phi);  
	q[3] = sin(psi)*cos(the)*cos(phi) - cos(psi)*sin(the)*sin(phi);
}

MATRIX quat2dcm(double *q, MATRIX C_N2B) {
	// Quaternion to C_N2B
	double q0, q1, q2, q3;
	q0 = q[0]; q1 = q[1]; q2 = q[2]; q3 = q[3];
	if (MatRow(C_N2B) != 3 || MatCol(C_N2B) != 3) {
		printf("quat2dcm error: incompatible matrix size\n");
	}
	C_N2B[0][0] = 2*q0*q0 - 1 + 2*q1*q1;
	C_N2B[1][1] = 2*q0*q0 - 1 + 2*q2*q2;
	C_N2B[2][2] = 2*q0*q0 - 1 + 2*q3*q3;
	
	C_N2B[0][1] = 2*q1*q2 + 2*q0*q3;
	C_N2B[0][2] = 2*q1*q3 - 2*q0*q2;
	
	C_N2B[1][0] = 2*q1*q2 - 2*q0*q3;
	C_N2B[1][2] = 2*q2*q3 + 2*q0*q1;
	
	C_N2B[2][0] = 2*q1*q3 + 2*q0*q2;
	C_N2B[2][1] = 2*q2*q3 - 2*q0*q1;
	
	return(C_N2B);
}

/*=====================================================================*/
/*=====================================================================*/
/*=====================================================================*/
/*=============================OLD FUNCTIONS==========================*/
/*=====================================================================*/
/*=====================================================================*/
/*=====================================================================*/
/*=====================================================================*/
/*=====================================================================*/
// dcm better be 3 x 3
MATRIX EulerToDcm(MATRIX euler, double decA, MATRIX dcm)
{
  MATRIX A,B;
  double cPHI,sPHI,cTHE,sTHE,cPSI,sPSI;

  if (MatRow(euler) != 3 || MatCol(euler) != 1 || MatRow(dcm) != 3 || MatCol(dcm) != 3) {
	  printf("EulerToDcm error: incompatible matrix size\n");
  }

  cPHI = cos(euler[2][0]); sPHI = sin(euler[2][0]);
  cTHE = cos(euler[1][0]); sTHE = sin(euler[1][0]);
  cPSI = cos(euler[0][0]); sPSI = sin(euler[0][0]);

  A = mat_creat(3,3,ZERO_MATRIX);
  B = mat_creat(3,3,UNDEFINED);

  A[0][0] = cos(decA); A[0][1] =-sin(decA);
  A[1][0] = sin(decA); A[1][1] = cos(decA);
  A[2][2] = 1;

  B[0][0] = cTHE*cPSI; B[0][1] = sPHI*sTHE*cPSI-cPHI*sPSI; B[0][2] = cPHI*sTHE*cPSI+sPHI*sPSI;
  B[1][0] = cTHE*sPSI; B[1][1] = sPHI*sTHE*sPSI+cPHI*cPSI; B[1][2] = cPHI*sTHE*sPSI-sPHI*cPSI;
  B[2][0] =-sTHE;      B[2][1] = sPHI*cTHE;                B[2][2] = cPHI*cTHE;

  mat_mul(A,B,dcm);

  mat_free(A);
  mat_free(B);

  return(dcm);

}




/*=================================================================*/

void EcefToLatLonAlt(MATRIX vector)
{

  int i;
  double x, y, z, q, p, sinlat, sinlat2;
  double a, b, d, radius, lat, alt, dummy;
  double E_WGS84, E2_WGS84, ONE_MIN_E2, A_WGS84;
  MATRIX lla;

  lla = mat_creat(3,1,ZERO_MATRIX);
  
  if (MatRow(vector) != 3 || MatCol(vector) != 1 ) {
	  printf("EcefToLatLonAlt error: incompatible matrix size\n");
  }

  E_WGS84 = ECCENTRICITY;   /* Earth ellipse ecc - unitless */
  E2_WGS84 = E_WGS84*E_WGS84;  /* Earth's ellipse ecc^2 - unitless */
  ONE_MIN_E2 = 1.0 - E2_WGS84;
  A_WGS84 = EARTH_RADIUS;         /* Earth's ellipse semi-major axis - meters */

  x = vector[0][0];
  y = vector[1][0];
  z = vector[2][0];

  lla[1][0] = atan2(y, x);           /*  Longitude  */

  p = sqrt((x * x) + (y * y));      /*  Latitude and Altitude  */

  if (p < 0.1)
  {
    p = 0.1;
  }

  q = z / p;
  alt = 0.0;
  lat = atan(q * (1.0 / ONE_MIN_E2));
  a = 1.0;
  i = 0;

  while ((a > 0.2) && (i < 20))
    {
        sinlat = sin(lat);
        sinlat2 = sinlat * sinlat;
        dummy =sqrt((1.0 - (E2_WGS84 * sinlat2))*(1.0 - (E2_WGS84 * sinlat2)));
        radius = A_WGS84 / sqrt(dummy);
        d = alt;
        alt = (p / cos(lat)) - radius;
        a = q * (radius + alt);
        b = (ONE_MIN_E2 * radius) + alt;
        lat = atan2(a, b);
        a = sqrt((alt - d)*(alt - d));
        i = i + 1;
    }

    lla[0][0] = lat;
    lla[2][0] = alt;

    for (i = 0; i < 3; i++)
      {
        vector[i][0] = lla[i][0];
      }

      mat_free(lla);

}


/*=================================================================*/

void EcefToEnu(MATRIX outputVector, MATRIX inputVector, MATRIX position)
{

  int i;
  double lat, lon;
  MATRIX C, ned, ref_position;
  MATRIX position_copy, delta_pos;

  if (MatRow(outputVector) != 3 || MatCol(outputVector) != 1 || MatRow(inputVector) != 3 || MatCol(inputVector) != 1 || MatRow(position) != 3 || MatCol(position) != 1) {
	  printf("EcefToEnu error: incompatible matrix size\n");
  }

  C = mat_creat(3,3,ZERO_MATRIX);
  ref_position = mat_creat(3,1,ZERO_MATRIX);
  delta_pos = mat_creat(3,1,ZERO_MATRIX);
  position_copy = mat_creat(MatRow(position),MatCol(position),ZERO_MATRIX);
  mat_copy(position, position_copy);

  lat = position[0][0];
  lon = position[1][0];

  LatLonAltToEcef(ref_position,position_copy);

  mat_sub(inputVector,ref_position,delta_pos);

  C[0][0] = -sin(lon);
  C[0][1] = cos(lon);
  C[0][2] = 0;

  C[1][0] = -sin(lat)*cos(lon);
  C[1][1] = -sin(lat)*sin(lon);
  C[1][2] = cos(lat);

  C[2][0] = cos(lat)*cos(lon);
  C[2][1] = cos(lat)*sin(lon);
  C[2][2] = sin(lat);

  ned = mat_creat(MatRow(C),MatCol(delta_pos),ZERO_MATRIX);
  mat_mul(C,delta_pos,ned);

  for (i = 0; i < 3; i++)
    {
      outputVector[i][0] = ned[i][0];
    }

  mat_free(ned);
  mat_free(C);
  mat_free(delta_pos);
  mat_free(ref_position);
  mat_free(position_copy);

}

/*=================================================================*/

void LatLonAltToEcef(MATRIX vector, MATRIX position)
{


  double Rn, alt, denom;
  double sinlat, coslat, coslon, sinlon;

  if (MatRow(vector) != 3 || MatCol(vector) != 1 || MatRow(position) != 3 || MatCol(position) != 1) {
	  printf("EcefToEnu error: incompatible matrix size\n");
  }

  sinlat = sin(position[0][0]);
  coslat = cos(position[0][0]);
  coslon = cos(position[1][0]);
  sinlon = sin(position[1][0]);
  alt = position[2][0];

  denom = (1.0 - (ECC2 * sinlat * sinlat));
  denom = sqrt(denom*denom);

  Rn = EARTH_RADIUS / sqrt(denom);

//  vector[0][0] = (Rn - alt) * coslat * coslon;
// vector[1][0] = (Rn - alt) * coslat * sinlon;
//  vector[2][0] = (Rn * (1.0 - ecc2) - alt) * sinlat;
  vector[0][0] = (Rn + alt) * coslat * coslon;
  vector[1][0] = (Rn + alt) * coslat * sinlon;
  vector[2][0] = (Rn * (1.0 - ECC2) + alt) * sinlat;
}

/*=================================================================*/

void nCltrans(MATRIX n_C_l, double magdec)
{
    double gamma;

    gamma=magdec;
    n_C_l[0][0]=-sin(gamma);
    n_C_l[0][1]=cos(gamma);
    n_C_l[0][2]=0;
    n_C_l[1][0]=cos(gamma);
    n_C_l[1][1]=sin(gamma);
    n_C_l[1][2]=0;
    n_C_l[2][0]=0;
    n_C_l[2][1]=0;
    n_C_l[2][2]=-1;

}


/*=================================================================*/

void eCntrans(MATRIX e_C_n, MATRIX LatLon)
{
    double lat,lon;

    lat=LatLon[0][0];
    lon=LatLon[1][0];

    e_C_n[0][0]=-sin(lon);
    e_C_n[0][1]=-cos(lon)*sin(lat);
    e_C_n[0][2]=cos(lon)*sin(lat);

    e_C_n[1][0]=cos(lon);
    e_C_n[1][1]=-sin(lat)*sin(lon);
    e_C_n[1][2]=sin(lon)*cos(lat);

    e_C_n[2][0]=0;
    e_C_n[2][1]=cos(lat);
    e_C_n[2][2]=sin(lat);
}


/*=====================================================================*/

void lCbtrans(MATRIX l_C_b, MATRIX YawPitchRoll)
{
    double psi,theta,phi;

    psi=YawPitchRoll[0][0];
    theta=YawPitchRoll[1][0];
    phi=YawPitchRoll[2][0];

    l_C_b[0][0]=cos(theta)*cos(psi);
    l_C_b[0][1]=-cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi);
    l_C_b[0][2]=sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi);

    l_C_b[1][0]=cos(theta)*sin(psi);
    l_C_b[1][1]=cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi);
    l_C_b[1][2]=-sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi);

    l_C_b[2][0]=-sin(theta);
    l_C_b[2][1]=sin(phi)*cos(theta);
    l_C_b[2][2]=cos(phi)*cos(theta);
}


