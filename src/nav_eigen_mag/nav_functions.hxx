/*! \file nav_functions.h
 *	\brief Auxiliary functions for nav filter header file
 *
 *	\details
 *     Module:          navfunc.h
 *     Modified:        Brian Taylor (convert to eigen3)
 *						Gokhan Inalhan (remaining) 
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
 * $Id: nav_functions.h 922 2012-10-17 19:14:09Z joh07594 $
 */

//#include "matrix.h"
#ifndef NAV_FUNCTIONS_H_
#define NAV_FUNCTIONS_H_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
using namespace Eigen;

/*     Define Constants   */

#define EARTH_RATE   0.00007292115   		/* rotation rate of earth (rad/sec) */
#define EARTH_RADIUS 6378137         		/* earth semi-major axis radius (m) */
#define ECCENTRICITY 0.0818191908426 		/* major eccentricity of earth ellipsoid */
#define ECC2	     0.0066943799901 		/* major eccentricity squared */
#define FLATTENING   0.0033528106650 		/* flattening of the ellipsoid */
#define GRAVITY_0    9.7803730       		/* zeroth coefficient for gravity model */
#define GRAVITY_1    0.0052891       		/* first coefficient for the gravity model */ 
#define GRAVITY_2    0.0000059       		/* second coefficient for the gravity model */
#define GRAVITY_NOM  9.81            		/* nominal gravity */ 
#define SCHULER2     1.533421593170545E-06 	/* Sculer Frequency (rad/sec) Squared */
#define FT2M         0.3048                	/* feet to meters conversion factor */
#define KTS2ms       0.5144                	/* Knots to meters/sec conversion factor */
#define MAG_DEC      0.270944862           	/* magnetic declination of Stanford (rad): 15.15 degrees */
#define MM2M         0.001                 	/* mm to m */

/*---------------     Define Structures and Enumerated Types -------------*/

Matrix<double,3,1> llarate(Matrix<double,3,1> V, Matrix<double,3,1> lla);

Matrix<double,3,1> navrate(Matrix<double,3,1> V, Matrix<double,3,1> lla);

Matrix<double,3,1> ecef2ned(Matrix<double,3,1> ecef, Matrix<double,3,1> pos_ref);

Matrix<double,3,1> lla2ecef(Matrix<double,3,1> lla);

Matrix<double,3,3> sk(Matrix<double,3,1> w);

void qmult(double *p, double *q, double *r);

Matrix<double,3,1> quat2eul(Quaterniond q);

Quaterniond eul2quat(double phi, double the, double psi);

Matrix<double,3,3> quat2dcm(Quaterniond q);

#endif
