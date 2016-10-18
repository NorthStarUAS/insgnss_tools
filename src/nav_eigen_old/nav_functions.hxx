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
#ifndef NAV_FUNCTIONS_HXX_
#define NAV_FUNCTIONS_HXX_

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

Vector3d llarate(Vector3d V, Vector3d lla);

Vector3d navrate(Vector3d V, Vector3d lla);

Vector3d ecef2ned(Vector3d ecef, Vector3d pos_ref);

Vector3d lla2ecef(Vector3d lla);

Matrix3d sk(Vector3d w);

Vector3d quat2eul(Quaterniond q);

Quaterniond eul2quat(double phi, double the, double psi);

Matrix3d quat2dcm(Quaterniond q);

#endif	// NAV_FUNCTIONS_HXX
