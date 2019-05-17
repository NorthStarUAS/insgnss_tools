#pragma once

#include <math.h>

#define EARTH_RATE   0.00007292115   		/* rotation rate of earth (rad/sec) */
#define EARTH_RADIUS 6378137.0         		/* earth semi-major axis radius (m) */
#define ECCENTRICITY 0.0818191908426 		/* major eccentricity of earth ellipsoid */
// #define ECC2	     0.0066943799901 		/* major eccentricity squared */
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

// useful constants
const float g = 9.814;

/* additional constants */
const double D2R = M_PI / 180.0; // degrees to radians
const double R2D = 180.0 / M_PI; // radians to degrees
const double F2M = 0.3048;       // feets to meters
const double M2F = 1.0 / F2M;    // meters to feets
