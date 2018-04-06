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

#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
using namespace Eigen;

/*     Define Constants   */

#define EARTH_RATE   0.00007292115   		/* rotation rate of earth (rad/sec) */
#define EARTH_RADIUS 6378137.0         		/* earth semi-major axis radius (m) */
#define ECCENTRICITY 0.0818191908426 		/* major eccentricity of earth ellipsoid */
#define ECC2	     0.0066943799901 		/* major eccentricity squared */
#define FLATTENING   0.0033528106650 		/* flattening of the ellipsoid */
#define GRAVITY_0    9.7803730       		/* zeroth coefficient for gravity model */
#define GRAVITY_1    0.0052891       		/* first coefficient for the gravity model */ 
#define GRAVITY_2    0.0000059       		/* second coefficient for the gravity model */
#define GRAVITY_NOM  9.81            		/* nominal gravity */ 
#define SCHULER2     1.533421593170545E-06 	/* Schuler Frequency (rad/sec) Squared */
#define FT2M         0.3048                	/* feet to meters conversion factor */
#define KTS2ms       0.5144                	/* Knots to meters/sec conversion factor */
#define MAG_DEC      0.270944862           	/* magnetic declination of Stanford (rad): 15.15 degrees */
#define MM2M         0.001                 	/* mm to m */

/* for ecef2lla */
#define _SQUASH    0.9966471893352525192801545
#define E2         fabs(1 - _SQUASH*_SQUASH)


// This function calculates the rate of change of latitude, longitude,
// and altitude using WGS-84.
Vector3d llarated(Vector3d V, Vector3d lla);

// This function calculates the angular velocity of the NED frame,
// also known as the navigation rate using WGS-84.
Vector3d navrated(Vector3d V, Vector3d lla);

// This function calculates the ECEF Coordinate given the
// Latitude, Longitude and Altitude.
Vector3d lla2ecef(Vector3d lla);

// This function calculates the Latitude, Longitude and Altitude given
// the ECEF Coordinates.
Vector3d ecef2lla( Vector3d ecef_pos );
    
// This function converts a vector in ecef to ned coordinate centered
// at pos_ref.
Vector3d ecef2nedd(Vector3d ecef, Vector3d pos_ref);

// Return a quaternion rotation from the earth centered to the
// simulation usual horizontal local frame from given longitude and
// latitude.  The horizontal local frame used in simulations is the
// frame with x-axis pointing north, the y-axis pointing eastwards and
// the z axis pointing downwards.  (Returns the ecef2ned
// transformation as a quaternion.)
Quaterniond lla2quat(double lon_rad, double lat_rad);

// This function gives a skew symmetric matrix from a given vector w
Matrix3d skd(Vector3d w);

// Quaternion to euler angle: returns phi, the, psi as a vector
Vector3d quat2euld(Quaterniond q);

// Computes a quaternion from the given euler angles
Quaterniond eul2quatd(double phi, double the, double psi);

// Quaternion to C_N2B
Matrix3d quat2dcmd(Quaterniond q);
