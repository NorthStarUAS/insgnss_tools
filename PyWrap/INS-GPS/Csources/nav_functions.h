/*! \file nav_functions.h
 *	\brief Auxiliary functions for nav filter header file
 *
 *	\details
 *     Module:          navfunc.h
 *     Modified:        Gokhan Inalhan (remaining) 
 *                      Demoz Gebre (first three functions)
 *                      Adhika Lie
 *                      Jung Soon Jang
 *     Description:     navfunc.h contains all the variable, 
 *                      constants and function prototypes that are 
 *                      used with the inertial navigation software.
 *	\ingroup nav_fcns
 *
 * \author University of Minnesota
 * \author Aerospace Engineering and Mechanics
 * \copyright Copyright 2011 Regents of the University of Minnesota. All rights reserved.
 *
 * $Id$
 */

//#include "matrix.h"
#ifndef NAV_FUNCTIONS_H_
#define NAV_FUNCTIONS_H_

/*     Define Constants   */

#define EARTH_RATE   0.00007292115   /* rotation rate of earth (rad/sec) */
#define EARTH_RADIUS 6378137.0       /* earth semi-major axis radius (m) */
#define ECCENTRICITY 0.0818191908426 /* major eccentricity of earth ellipsoid */
#define ECC2		 0.00669437999014 /* major eccentricity squared */
#define FLATTENING   0.0033528106650 /* flattening of the ellipsoid */
#define GRAVITY_0    9.7803730       /* zeroth coefficient for gravity model */
#define GRAVITY_1    0.0052891       /* first coefficient for the gravity model*/ 
#define GRAVITY_2    0.0000059       /* second coefficient for the gravity model*/
#define GRAVITY_NOM  9.81            /* nominal gravity */ 
#define SCHULER2     1.533421593170545E-06 /* Sculer Frequency (rad/sec) Squared */
//#define R2D          57.29577951308232     /* radians to degrees conversion factor */
//#define D2R          0.01745329251994      /* degrees to radians conversion factor */  
#define FT2M         0.3048                /* feet to meters conversion factor */
#define KTS2ms       0.5144                /* Knots to meters/sec conversion factor*/
//#define PI           3.14159265358979      /* pi */
#define MAG_DEC      0.270944862           /*magnetic declination of Stanford (rad): 15.15 degrees */
#define MM2M         0.001                 /*mm to m*/

/*---------------     Define Structures and Enumerated Types -------------*/
typedef enum {OFF, ON} toggle;


MATRIX eul2dcm(MATRIX euler, MATRIX dcm);

MATRIX dcm2eul(MATRIX euler, MATRIX dcm);

MATRIX create_R(MATRIX e, MATRIX R);

MATRIX llarate(MATRIX V, MATRIX lla, MATRIX lla_dot);

MATRIX navrate(MATRIX V, MATRIX lla, MATRIX nr);

MATRIX ecef2ned(MATRIX ecef, MATRIX ned, MATRIX ecef_ref);

MATRIX ecef2lla(MATRIX ecef, MATRIX lla);

MATRIX lla2ecef(MATRIX lla, MATRIX ecef);

MATRIX sk(MATRIX w, MATRIX C);

MATRIX ortho(MATRIX C, MATRIX C_ortho);

double norm (MATRIX a);

MATRIX cross (MATRIX a, MATRIX b, MATRIX c);

void qmult(double *p, double *q, double *r);

void quat2eul(double *q, double *phi, double *the, double *psi);

void eul2quat(double *q, double phi, double the, double psi);

MATRIX quat2dcm(double *q, MATRIX C_N2B);
/*=====================================================================*/
/*=====================================================================*/
/*=====================================================================*/
/*=============================OLD FUNCTIONS==========================*/
/*=====================================================================*/
/*=====================================================================*/
/*=====================================================================*/
/*=====================================================================*/
/*=====================================================================*/
/*
 * Function:     MATRIX EulerToDcm(MATRIX euler)
 *----------------------------------------------------------------------
 * Computer the direction cosine matrix that transforms a vector in
 * a reference axis system at time k to a reference axis system
 * at time k+1.  The input argument 'euler' is a vector containing the
 * the three euler angles in radians.  The order of the angles is assumed
 * to be yaw, pitch and roll (i.e., 3-2-1 rotation convention).
 */
MATRIX EulerToDcm(MATRIX euler, double dipA, MATRIX dcm);

/* Function    void EcefToEnu(MATRIX outputVector, MATRIX inputVector,
 *                              MATRIX position);
 *-------------------------------------------------------------
 * Converts the vector given in ECEF coordinates to a vector in 
 * ENU (East, North, Up) coordinates centered at the location
 * given in position (in lattitude, longitude, altitude);
 */ 
void EcefToEnu(MATRIX outputVector, MATRIX inputVector, MATRIX position);


/* Function    void EcefToLatLonAlt(MATRIX vector);
 *-----------------------------------------------------
 * Converts a position vector given in ECEF coordinates
 * into latitude, longitude and altitude.
 */
void EcefToLatLonAlt(MATRIX vector);

/* Function void LatLonAltToEcef(MATRIX vector, MATRIX position );
 *--------------------------------------------------------------
 * Converts a position vector given in lattitude, longitude and 
 * altitude to a vector in ECEF coordinates.
 */
void LatLonAltToEcef(MATRIX vector, MATRIX position);

/* Function void nCltrans(MATRIX n_C_l, double magdec)
 *--------------------------------------------------------------
 * Creates the transformation matrix from IMU level earth frame 
 * to instantenous Navigation ENU frame
 */
void nCltrans(MATRIX n_C_l, double magdec);

/* Function void eCntrans(MATRIX e_C_n, MATRIX LatLon)
 *--------------------------------------------------------------
 * Creates the transformation matrix from Navigation ENU frame
 * to ECEF frame through LatLon matrix=[Lat Lon]' (rad)
 */
void eCntrans(MATRIX e_C_n, MATRIX LatLon);

/* void lCbtrans(MATRIX l_C_b, MATRIX YawPitchRoll)
 *---------------------------------------------------------------
 * Creates the transformation matrix from B(b:body) to ELF(l:earth level) frame
 * input: yaw-psi(rad) pitch-theta(rad) roll-phi(rad) inverse of 3-2-1 Euler transformation
 */
void lCbtrans(MATRIX l_C_b, MATRIX YawPitchRoll); 
#endif












