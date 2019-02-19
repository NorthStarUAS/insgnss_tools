/*! \file nav_functions.c
 *	\brief Auxiliary functions for nav filter
 *
 *	\details
 *     Module:          Navfuncs.c
 *     Modified:        Brian Taylor (convert to eigen3)
 *						Adhika Lie (revamp all functions)
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
 * $Id: nav_functions.c 922 2012-10-17 19:14:09Z joh07594 $
 */

/*     Include Pertinent Header Files */

#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
using namespace Eigen;

#include "nav_functions_double.hxx"


// This function calculates the rate of change of latitude, longitude,
// and altitude using WGS-84.
Vector3d llarated(Vector3d V, Vector3d lla) {
    double lat = lla(0,0);
    double h = lla(2,0);
	
    double denom = fabs(1.0 - (ECC2 * sin(lat) * sin(lat)));
    double sqrt_denom = sqrt(denom);
    
    double Rew = EARTH_RADIUS / sqrt_denom;
    double Rns = EARTH_RADIUS*(1-ECC2) / (denom*sqrt_denom);
	
    Vector3d lla_dot;
    lla_dot(0,0) = V(0,0)/(Rns + h);
    lla_dot(1,0) = V(1,0)/((Rew + h)*cos(lat));
    lla_dot(2,0) = -V(2,0);
	
    return lla_dot;
}

// This function calculates the angular velocity of the NED frame,
// also known as the navigation rate using WGS-84.
Vector3d navrated(Vector3d V, Vector3d lla) {
    double lat = lla(0,0);
    double h = lla(2,0);
	
    double denom = fabs(1.0 - (ECC2 * sin(lat) * sin(lat)));
    double sqrt_denom = sqrt(denom);
    
    double Rew = EARTH_RADIUS / sqrt_denom;
    double Rns = EARTH_RADIUS*(1-ECC2) / (denom*sqrt_denom);
	
    Vector3d nr;
    nr(0,0) = V(1,0)/(Rew + h);
    nr(1,0) = -V(0,0)/(Rns + h);
    nr(2,0) = -V(1,0)*tan(lat)/(Rew + h);
	
    return nr;
}

// This function calculates the ECEF Coordinate given the
// Latitude, Longitude and Altitude.
Vector3d lla2ecef(Vector3d lla) {
    double sinlat = sin(lla(0,0));
    double coslat = cos(lla(0,0));
    double coslon = cos(lla(1,0));
    double sinlon = sin(lla(1,0));
    double alt = lla(2,0);

    double denom = fabs(1.0 - (ECC2 * sinlat * sinlat));

    double Rew = EARTH_RADIUS / sqrt(denom);
  
    Vector3d ecef;
    ecef(0,0) = (Rew + alt) * coslat * coslon;
    ecef(1,0) = (Rew + alt) * coslat * sinlon;
    ecef(2,0) = (Rew * (1.0 - ECC2) + alt) * sinlat;
	
    return ecef;
}

// This function calculates the Latitude, Longitude and Altitude given
// the ECEF Coordinates.
Vector3d ecef2lla( Vector3d ecef_pos ) {
    const double ra2 = 1.0/(EARTH_RADIUS*EARTH_RADIUS);
    const double e2 = E2;
    const double e4 = E2*E2;
    
    // according to
    // H. Vermeille,
    // Direct transformation from geocentric to geodetic ccordinates,
    // Journal of Geodesy (2002) 76:451-454
    Vector3d lla;
    double X = ecef_pos(0);
    double Y = ecef_pos(1);
    double Z = ecef_pos(2);
    double XXpYY = X*X+Y*Y;
    if( XXpYY + Z*Z < 25 ) {
	// This function fails near the geocenter region, so catch
	// that special case here.  Define the innermost sphere of
	// small radius as earth center and return the coordinates
	// 0/0/-EQURAD. It may be any other place on geoide's surface,
	// the Northpole, Hawaii or Wentorf. This one was easy to code
	// ;-)
	lla(0) = 0.0;
	lla(1) = 0.0;
	lla(2) = -EARTH_RADIUS;
	return lla;
    }
    
    double sqrtXXpYY = sqrt(XXpYY);
    double p = XXpYY*ra2;
    double q = Z*Z*(1-e2)*ra2;
    double r = 1/6.0*(p+q-e4);
    double s = e4*p*q/(4*r*r*r);
    /* 
       s*(2+s) is negative for s = [-2..0]
       slightly negative values for s due to floating point rounding errors
       cause nan for sqrt(s*(2+s))
       We can probably clamp the resulting parable to positive numbers
    */
    if( s >= -2.0 && s <= 0.0 )
	s = 0.0;
    double t = pow(1+s+sqrt(s*(2+s)), 1/3.0);
    double u = r*(1+t+1/t);
    double v = sqrt(u*u+e4*q);
    double w = e2*(u+v-q)/(2*v);
    double k = sqrt(u+v+w*w)-w;
    double D = k*sqrtXXpYY/(k+e2);
    lla(1) = 2*atan2(Y, X+sqrtXXpYY);
    double sqrtDDpZZ = sqrt(D*D+Z*Z);
    lla(0) = 2*atan2(Z, D+sqrtDDpZZ);
    lla(2) = (k+e2-1)*sqrtDDpZZ/k;
    return lla;
}

// This function converts a vector in ecef to ned coordinate centered
// at pos_ref.
Vector3d ecef2nedd(Vector3d ecef, Vector3d pos_ref) {
    double lat = pos_ref(0,0);
    double lon = pos_ref(1,0);
    double sin_lat = sin(lat);
    double sin_lon = sin(lon);
    double cos_lat = cos(lat);
    double cos_lon = cos(lon);
    
    Vector3d ned;
    ned(2,0) = -cos_lat*cos_lon*ecef(0,0) - cos_lat*sin_lon*ecef(1,0) - sin_lat*ecef(2,0);
    ned(1,0) = -sin_lon*ecef(0,0) + cos_lon*ecef(1,0);
    ned(0,0) = -sin_lat*cos_lon*ecef(0,0) - sin_lat*sin_lon*ecef(1,0) + cos_lat*ecef(2,0);
	
    return ned;
}

// Return a quaternion rotation from the earth centered to the
// simulation usual horizontal local frame from given longitude and
// latitude.  The horizontal local frame used in simulations is the
// frame with x-axis pointing north, the y-axis pointing eastwards and
// the z axis pointing downwards.  (Returns the ecef2ned
// transformation as a quaternion.)
Quaterniond lla2quatd(double lon_rad, double lat_rad) {
    Quaterniond q;
    double zd2 = 0.5*lon_rad;
    double yd2 = -0.25*M_PI - 0.5*lat_rad;
    double Szd2 = sin(zd2);
    double Syd2 = sin(yd2);
    double Czd2 = cos(zd2);
    double Cyd2 = cos(yd2);
    q.w() = Czd2*Cyd2;
    q.x() = -Szd2*Syd2;
    q.y() = Czd2*Syd2;
    q.z() = Szd2*Cyd2;
    return q;
}

// This function gives a skew symmetric matrix from a given vector w
Matrix3d skd(Vector3d w) {
    Matrix3d C;

    C(0,0) = 0.0;	C(0,1) = -w(2,0);	C(0,2) = w(1,0);
    C(1,0) = w(2,0);	C(1,1) = 0.0;		C(1,2) = -w(0,0);
    C(2,0) = -w(1,0);	C(2,1) = w(0,0);	C(2,2) = 0.0;
	
    return C;
}

// Quaternion to euler angle: returns phi, the, psi as a vector
Vector3d quat2euld(Quaterniond q) {
    double q0, q1, q2, q3;
    double m11, m12, m13, m23, m33;
	
    q0 = q.w();
    q1 = q.x();
    q2 = q.y();
    q3 = q.z();

    m11 = 2*(q0*q0 + q1*q1) - 1;
    m12 = 2*(q1*q2 + q0*q3);
    m13 = 2*(q1*q3 - q0*q2);
    m23 = 2*(q2*q3 + q0*q1);
    m33 = 2*(q0*q0 + q3*q3) - 1;
    
    Vector3d result;
    result(2) = atan2(m12,m11);
    result(1) = asin(-m13);
    result(0) = atan2(m23,m33);

    return result;
}

// Computes a quaternion from the given euler angles
Quaterniond eul2quatd(double phi, double the, double psi) {
    double sin_psi = sin(psi*0.5);
    double cos_psi = cos(psi*0.5);
    double sin_the = sin(the*0.5);
    double cos_the = cos(the*0.5);
    double sin_phi = sin(phi*0.5);
    double cos_phi = cos(phi*0.5);

    Quaterniond q;
    q.w() = cos_psi*cos_the*cos_phi + sin_psi*sin_the*sin_phi;  
    q.x() = cos_psi*cos_the*sin_phi - sin_psi*sin_the*cos_phi;
    q.y() = cos_psi*sin_the*cos_phi + sin_psi*cos_the*sin_phi;  
    q.z() = sin_psi*cos_the*cos_phi - cos_psi*sin_the*sin_phi;

    return q;
}

// Quaternion to C_N2B
Matrix3d quat2dcmd(Quaterniond q) {
    double q0, q1, q2, q3;
    Matrix3d C_N2B;

    q0 = q.w(); q1 = q.x(); q2 = q.y(); q3 = q.z();

    C_N2B(0,0) = 2*(q0*q0 + q1*q1) - 1;
    C_N2B(1,1) = 2*(q0*q0 + q2*q2) - 1;
    C_N2B(2,2) = 2*(q0*q0 + q3*q3) - 1;
	
    C_N2B(0,1) = 2*(q1*q2 + q0*q3);
    C_N2B(0,2) = 2*(q1*q3 - q0*q2);
	
    C_N2B(1,0) = 2*(q1*q2 - q0*q3);
    C_N2B(1,2) = 2*(q2*q3 + q0*q1);
	
    C_N2B(2,0) = 2*(q1*q3 + q0*q2);
    C_N2B(2,1) = 2*(q2*q3 - q0*q1);
	
    return C_N2B;
}
