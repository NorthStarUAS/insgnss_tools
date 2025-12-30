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

#include "BasicLinearAlgebra.h"
using namespace BLA;

typedef Matrix<3, 1, float> Vector3f;
typedef Matrix<3, 1, double> Vector3d;
typedef Matrix<3, 3, float> Matrix3f;

class Quaternionf {
private:
    Matrix<4, 1, float> q;
public:
    Quaternionf() {
        q(0,0) = 0.0f;
        q(1,0) = 0.0f;
        q(2,0) = 0.0f;
        q(3,0) = 1.0f;
    }
    Quaternionf(float w, float x, float y, float z) {
        q(0,0) = x;
        q(1,0) = y;
        q(2,0) = z;
        q(3,0) = w;
    }
    float w() const { return q(3,0); }
    float x() const { return q(0,0); }
    float y() const { return q(1,0); }
    float z() const { return q(2,0); }
    float& w() { return q(3,0); }
    float& x() { return q(0,0); }
    float& y() { return q(1,0); }
    float& z() { return q(2,0); }
    inline Quaternionf operator*(const Quaternionf& rhs){
        return Quaternionf(
            w()*rhs.w() - x()*rhs.x() - y()*rhs.y() - z()*rhs.z(),
            w()*rhs.x() + x()*rhs.w() + y()*rhs.z() - z()*rhs.y(),
            w()*rhs.y() - x()*rhs.z() + y()*rhs.w() + z()*rhs.x(),
            w()*rhs.z() + x()*rhs.y() - y()*rhs.x() + z()*rhs.w()
        );
    }
    Quaternionf normalized() {
        float norm = sqrt(w()*w() + x()*x() + y()*y() + z()*z());
        return Quaternionf(w()/norm, x()/norm, y()/norm, z()/norm);
    }
};

// #if defined(ARDUINO)
// # include <math.h>
// # include <eigen.h>
// # include <Eigen/Geometry>
// #else
// # include "eigen3/Eigen/Core"
// # include "eigen3/Eigen/Geometry"
// #endif
// using namespace Eigen;

// Constants
const double EarthRadius = 6378137.0;        // earth semi-major axis radius (m)
const double ECC2 = 0.0066943799901;         // major eccentricity squared

// Constants that are no longer used
// const double EarthRate = 0.00007292115;      // rotation rate of earth (rad/sec)
// const double Eccentricity = 0.0818191908426; // major eccentricity of earth ellipsoid
// const double Flattening = 0.0033528106650;   // flattening of the ellipsoid
// const double Gravity0 = 9.7803730;           // zeroth coefficient for gravity model
// const double Gravity1 = 0.0052891;           // first coefficient for the gravity model
// const double Gravity2 = 0.0000059;           // second coefficient for the gravity model
// const double GravityNom = 9.81;              // nominal gravity
// const double Schuler2 = 1.533421593170545E-06; // Schuler Frequency (rad/sec) Squared

// This function calculates the rate of change of latitude, longitude,
// and altitude using WGS-84.
Vector3f llarate(Vector3f V, Vector3d lla);

// This function calculates the angular velocity of the NED frame,
// also known as the navigation rate using WGS-84.
Vector3d navrate(Vector3d V, Vector3d lla);

// This function calculates the ECEF Coordinate given the
// Latitude, Longitude and Altitude.
Vector3d lla2ecef(Vector3d lla);

// This function calculates the Latitude, Longitude and Altitude given
// the ECEF Coordinates.
Vector3d ecef2lla( Vector3d ecef_pos );

// This function converts a vector in ecef to ned coordinate centered
// at pos_ref.
Vector3f ecef2ned(Vector3d ecef, Vector3d pos_ref);

// Return a quaternion rotation from the earth centered to the
// simulation usual horizontal local frame from given longitude and
// latitude.  The horizontal local frame used in simulations is the
// frame with x-axis pointing north, the y-axis pointing eastwards and
// the z axis pointing downwards.  (Returns the ecef2ned
// transformation as a quaternion.)
Quaternionf lla2quat(double lon_rad, double lat_rad);

// This function gives a skew symmetric matrix from a given vector w
Matrix3f sk(Vector3f w);

// Quaternion to euler angle: returns phi, the, psi as a vector
Vector3f quat2eul(Quaternionf q);

// Computes a quaternion from the given euler angles
Quaternionf eul2quat(float phi, float the, float psi);

// Quaternion to C_N2B
Matrix3f quat2dcm(Quaternionf q);