/*
Copyright (c) 2016 - 2020 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Adapted for RAPTRS: Brian Taylor and Chris Regan

Adapted from prior versions
Copyright 2011 Regents of the University of Minnesota. All rights reserved.
Original Author: Adhika Lie, Gokhan Inalhan, Demoz Gebre, Jung Soon Jang

Reference Frames and Coordinates from nav-functions()
I - ECI (Earch Center Inertial): origin at Earth center
E - ECEF (Earch Center Earth Fixed): origin at Earth center
D - Geodetic: origin at Earth center, Uses earth ellisoid definition (example WGS84)
G - Geocentric: origin at Earth center, Uses spheroid definition
L - Local Level: origin at specified reference, [x- North, y- East, z- Down]
B - Body: origin at Body CG, [x- Fwd, y- Starboard, z- Down]

All units meters and radians
"Acceleration" is actually "specific gravity", ie. gravity is removed.
*/
#include <iostream>
using namespace std; 
#include "nav-functions.h"
#include "../nav_common/structs.h"

// Calculate the rate of change of latitude, longitude,
// and altitude using the velocity in NED coordinates and WGS-84.
Vector3d L2D_Rate(Vector3d v_L, Vector3d pRef_D) {
    double Rew, Rns;
    EarthRad(pRef_D(0), &Rew, &Rns);

    Vector3d pDot_D;
    pDot_D(0) = v_L(0) / (Rns + pRef_D(2)); // latDot = vNorth / (Rns + alt)
    pDot_D(1) = v_L(1) / ((Rew + pRef_D(2)) * cos(pRef_D(0))); // lonDot = vEast / ((Rns + alt)*cos(lat))
    pDot_D(2) = -v_L(2);

    return pDot_D;
}
Vector3f L2D_Rate(Vector3f v_L, Vector3d pRef_D) {
    double Rew, Rns;
    EarthRad(pRef_D(0), &Rew, &Rns);

    Vector3f pDot_D;
    pDot_D(0) = v_L(0) / (Rns + pRef_D(2));
    pDot_D(1) = v_L(1) / ((Rew + pRef_D(2)) * cos(pRef_D(0)));
    pDot_D(2) = -v_L(2);

    return pDot_D;
}

// Calculate the angular velocity of the NED frame,
// also known as the navigation rate using WGS-84.
// Rotation rate of the NED frame wrt Geodetic, in NED coordinates.
Vector3d NavRate(Vector3d v_L, Vector3d pRef_D) {
    double Rew, Rns;
    EarthRad(pRef_D(0), &Rew, &Rns);

    Vector3d w_L;
    w_L(0) = v_L(1) / (Rew + pRef_D(2)); // rotation rate about North = vEast / (Rew + alt)
    w_L(1) = -v_L(0) / (Rns + pRef_D(2)); // rotation rate about East = -vNorth / (Rns + alt)
    w_L(2) = -v_L(1) * tan(pRef_D(0)) / (Rew + pRef_D(2)); // rotation rate about Down

    return w_L;
}
Vector3f NavRate(Vector3f v_L, Vector3d pRef_D) {
    double Rew, Rns;
    EarthRad(pRef_D(0), &Rew, &Rns);

    Vector3f w_LD_L;
    w_LD_L(0) = v_L(1) / (Rew + pRef_D(2));
    w_LD_L(1) = -v_L(0) / (Rns + pRef_D(2));
    w_LD_L(2) = -v_L(1) * tan(pRef_D(0)) / (Rew + pRef_D(2));

    return w_LD_L;
}

// Calculates the ECEF Coordinates given the
// Latitude, Longitude and Altitude.
Vector3d D2E(Vector3d p_D) {
    double sinlat = sin(p_D(0));
    double coslat = cos(p_D(0));
    double coslon = cos(p_D(1));
    double sinlon = sin(p_D(1));
    double alt = p_D(2);

    double denom = fabs(1.0 - (ECC2 * sinlat * sinlat));

    double Rew = EarthRadius / sqrt(denom);

    Vector3d p_E;
    p_E(0) = (Rew + alt) * coslat * coslon;
    p_E(1) = (Rew + alt) * coslat * sinlon;
    p_E(2) = (Rew * (1.0 - ECC2) + alt) * sinlat;

    return p_E;
}


// Calculate the Latitude, Longitude and Altitude given
// the ECEF Coordinates.
Vector3d E2D( Vector3d p_E ) {
    const double Squash = 0.9966471893352525192801545;
    const double ra2 = 1.0 / (EarthRadius * EarthRadius);
    const double e2 = fabs(1 - Squash * Squash);
    const double e4 = e2 * e2;

    // according to
    // H. Vermeille,
    // Direct transformation from geocentric to geodetic ccordinates,
    // Journal of Geodesy (2002) 76:451-454
    Vector3d p_D;
    double X = p_E(0);
    double Y = p_E(1);
    double Z = p_E(2);

    double XXpYY = X*X+Y*Y;
    if( XXpYY + Z*Z < 25 ) {
    	// This function fails near the geocenter region, so catch
    	// that special case here.  Define the innermost sphere of
    	// small radius as earth center and return the coordinates
    	// 0/0/-EQURAD. It may be any other place on geoide's surface,
    	// the Northpole, Hawaii or Wentorf. This one was easy to code
    	// ;-)
    	p_D(0) = 0.0;
    	p_D(1) = 0.0;
    	p_D(2) = -EarthRadius;
    	return p_D;
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
    if( s >= -2.0 && s <= 0.0 ) {
      s = 0.0;
    }

    double t = pow(1+s+sqrt(s*(2+s)), 1/3.0);
    double u = r*(1+t+1/t);
    double v = sqrt(u*u+e4*q);
    double w = e2*(u+v-q)/(2*v);
    double k = sqrt(u+v+w*w)-w;
    double D = k*sqrtXXpYY/(k+e2);
    double sqrtDDpZZ = sqrt(D*D+Z*Z);

    p_D(1) = 2*atan2(Y, X+sqrtXXpYY);
    p_D(0) = 2*atan2(Z, D+sqrtDDpZZ);
    p_D(2) = (k+e2-1)*sqrtDDpZZ/k;

    return p_D;
}

// Transform a vector in ECEF coord to NED coord centered at pRef.
Vector3d E2L(Vector3d p_E, Vector3d pRef_D) {
  Vector3d p_L = TransE2L(pRef_D) * p_E;

  return p_L;
}

// Calculate the ECEF to NED coordinate transform DCM
Matrix3d TransE2L(Vector3d pRef_D) {
  Matrix3d T_E2L;
  T_E2L = AngleAxisd(pRef_D(0) - 3*M_PI/2, Vector3d::UnitY())
    * AngleAxisd(-pRef_D(1), Vector3d::UnitZ());

  // T_E2L(0,0) = -sin(pRef_D(0))*cos(pRef_D(1)); T_E2L(0,1) = -sin(pRef_D(0))*sin(pRef_D(1)); T_E2L(0,2) = cos(pRef_D(0));
  // T_E2L(1,0) = -sin(pRef_D(1));                T_E2L(1,1) = cos(pRef_D(1));                 T_E2L(1,2) = 0.0f;
  // T_E2L(2,0) = -cos(pRef_D(0))*cos(pRef_D(1)); T_E2L(2,1) = -cos(pRef_D(0))*sin(pRef_D(1)); T_E2L(2,2) = -sin(pRef_D(0));
  return T_E2L;
}


// Calculate the ECEF to NED coordinate transform quaternion
Quaterniond E2L_Quat(Vector3d pRef_D) {
  double zd2 = 0.5 * pRef_D(1);
  double yd2 = -0.25 * M_PI - 0.5 * pRef_D(0);

  Quaterniond quat;
  quat.w() = cos(zd2) * cos(yd2);
  quat.x() = -sin(zd2) * sin(yd2);
  quat.y() = cos(zd2) * sin(yd2);
  quat.z() = sin(zd2) * cos(yd2);

  return quat;
}

// Skew symmetric matrix from a given vector w
Matrix3d Skew(Vector3d w) {
  Matrix3d C;

  C(0,0) =  0.0;	C(0,1) = -w(2);	C(0,2) =  w(1);
  C(1,0) =  w(2);	C(1,1) =  0.0;	C(1,2) = -w(0);
  C(2,0) = -w(1);	C(2,1) =  w(0);	C(2,2) =  0.0;

  return C;
}
Matrix3f Skew(Vector3f w) {
  Matrix3f C;

  C(0,0) =  0.0;	C(0,1) = -w(2);	C(0,2) =  w(1);
  C(1,0) =  w(2);	C(1,1) =  0.0;	C(1,2) = -w(0);
  C(2,0) = -w(1);	C(2,1) =  w(0);	C(2,2) =  0.0;

  return C;
}

// Quaternion to Euler angles (3-2-1)
Vector3d Quat2Euler(Quaterniond quat) {
  double m11 = 2*(quat.w()*quat.w() + quat.x()*quat.x()) - 1;
  double m12 = 2*(quat.x()*quat.y() + quat.w()*quat.z());
  double m13 = 2*(quat.x()*quat.z() - quat.w()*quat.y());
  double m23 = 2*(quat.y()*quat.z() + quat.w()*quat.x());
  double m33 = 2*(quat.w()*quat.w() + quat.z()*quat.z()) - 1;

  Vector3d euler;
  euler(2) = atan2(m12, m11);
  euler(1) = asin(-m13);
  euler(0) = atan2(m23, m33);

  return euler;
}
Vector3f Quat2Euler(Quaternionf quat) {
  float m11 = 2*(quat.w()*quat.w() + quat.x()*quat.x()) - 1;
  float m12 = 2*(quat.x()*quat.y() + quat.w()*quat.z());
  float m13 = 2*(quat.x()*quat.z() - quat.w()*quat.y());
  float m23 = 2*(quat.y()*quat.z() + quat.w()*quat.x());
  float m33 = 2*(quat.w()*quat.w() + quat.z()*quat.z()) - 1;

  Vector3f euler;
  euler(2) = atan2(m12, m11);
  euler(1) = asin(-m13);
  euler(0) = atan2(m23, m33);

  return euler;
}

// Quaternion to DCM
Matrix3d Quat2DCM(Quaterniond quat) {
  Matrix3d T;

  T(0,0) = 2*(quat.w()*quat.w() + quat.x()*quat.x()) - 1;
  T(1,1) = 2*(quat.w()*quat.w() + quat.y()*quat.y()) - 1;
  T(2,2) = 2*(quat.w()*quat.w() + quat.z()*quat.z()) - 1;

  T(0,1) = 2*(quat.x()*quat.y() + quat.w()*quat.z());
  T(0,2) = 2*(quat.x()*quat.z() - quat.w()*quat.y());

  T(1,0) = 2*(quat.x()*quat.y() - quat.w()*quat.z());
  T(1,2) = 2*(quat.y()*quat.z() + quat.w()*quat.x());

  T(2,0) = 2*(quat.x()*quat.z() + quat.w()*quat.y());
  T(2,1) = 2*(quat.y()*quat.z() - quat.w()*quat.x());

  return T;
}
Matrix3f Quat2DCM(Quaternionf quat) {
  Matrix3f T;

  T(0,0) = 2.0f*(quat.w()*quat.w() + quat.x()*quat.x()) - 1.0f;
  T(1,1) = 2.0f*(quat.w()*quat.w() + quat.y()*quat.y()) - 1.0f;
  T(2,2) = 2.0f*(quat.w()*quat.w() + quat.z()*quat.z()) - 1.0f;

  T(0,1) = 2.0f*(quat.x()*quat.y() + quat.w()*quat.z());
  T(0,2) = 2.0f*(quat.x()*quat.z() - quat.w()*quat.y());

  T(1,0) = 2.0f*(quat.x()*quat.y() - quat.w()*quat.z());
  T(1,2) = 2.0f*(quat.y()*quat.z() + quat.w()*quat.x());

  T(2,0) = 2.0f*(quat.x()*quat.z() + quat.w()*quat.y());
  T(2,1) = 2.0f*(quat.y()*quat.z() - quat.w()*quat.x());

  return T;
}

// euler angles (3-2-1) to quaternion
Quaterniond Euler2Quat(Vector3d euler) {
  Quaterniond quat;

  quat.w() = cos(euler(2) / 2.0f) * cos(euler(1) / 2.0f) * cos(euler(0) / 2.0f) + sin(euler(2) / 2.0f) * sin(euler(1) / 2.0f) * sin(euler(0) / 2.0f);
  quat.x() = cos(euler(2) / 2.0f) * cos(euler(1) / 2.0f) * sin(euler(0) / 2.0f) - sin(euler(2) / 2.0f) * sin(euler(1) / 2.0f) * cos(euler(0) / 2.0f);
  quat.y() = cos(euler(2) / 2.0f) * sin(euler(1) / 2.0f) * cos(euler(0) / 2.0f) + sin(euler(2) / 2.0f) * cos(euler(1) / 2.0f) * sin(euler(0) / 2.0f);
  quat.z() = sin(euler(2) / 2.0f) * cos(euler(1) / 2.0f) * cos(euler(0) / 2.0f) - cos(euler(2) / 2.0f) * sin(euler(1) / 2.0f) * sin(euler(0) / 2.0f);

  return quat;
}
Quaternionf Euler2Quat(Vector3f euler) {
  Quaternionf quat;

  quat.w() = cosf(euler(2) / 2.0f) * cosf(euler(1) / 2.0f) * cosf(euler(0) / 2.0f) + sinf(euler(2) / 2.0f) * sinf(euler(1) / 2.0f) * sinf(euler(0) / 2.0f);
  quat.x() = cosf(euler(2) / 2.0f) * cosf(euler(1) / 2.0f) * sinf(euler(0) / 2.0f) - sinf(euler(2) / 2.0f) * sinf(euler(1) / 2.0f) * cosf(euler(0) / 2.0f);
  quat.y() = cosf(euler(2) / 2.0f) * sinf(euler(1) / 2.0f) * cosf(euler(0) / 2.0f) + sinf(euler(2) / 2.0f) * cosf(euler(1) / 2.0f) * sinf(euler(0) / 2.0f);
  quat.z() = sinf(euler(2) / 2.0f) * cosf(euler(1) / 2.0f) * cosf(euler(0) / 2.0f) - cosf(euler(2) / 2.0f) * sinf(euler(1) / 2.0f) * sinf(euler(0) / 2.0f);

  return quat;
}

// Earth Radius Updates
void EarthRad(double lat, double *Rew, double *Rns) {
  double denom = fabs(1.0 - (ECC2 * sin(lat) * sin(lat)));
  double sqrt_denom = sqrt(denom);

  (*Rew) = EarthRadius / sqrt_denom; // Transverse (along East-West)
  (*Rns) = EarthRadius * (1 - ECC2) / (denom * sqrt_denom); // Merdian (along North-South)
}


// bound angle between -pi and pi
double WrapToPi(double a) {
  if(a >  M_PI) a -= (M_PI * 2.0f);
  if(a < -M_PI) a += (M_PI * 2.0f);
  return a;
}
float WrapToPi(float a) {
  if(a >  M_PI) a -= (M_PI * 2.0f);
  if(a < -M_PI) a += (M_PI * 2.0f);
  return a;
}

// bound angle between 0 and 2*pi
double WrapTo2Pi(double a){
  a = fmod(a, 2.0f * M_PI);
  if (a < 0)
    a += 2.0f * M_PI;
  return a;
}
float WrapTo2Pi(float a){
  a = fmod(a, 2.0f * M_PI);
  if (a < 0)
    a += 2.0f * M_PI;
  return a;
}


// EphemerisData (subframe1,2,3) to Satellite ecef x, y, z in meter, vx, vy, vz in m/s
VectorXd EphemerisData2Satecef(float t,
                               uint32_t TOW, uint8_t L2, uint16_t week_No, uint8_t L2_Flag, uint8_t SV_Acc, uint8_t SV_Hlth,
                               double T_GD, uint16_t IODC, double t_OC, int8_t a_f2, double a_f1, double a_f0,
                               uint8_t IODE, double C_rs, double delta_n, double M_0, double C_uc, double ecc, double C_us,
                               double sqrt_A, double t_OE, double C_ic, double Omega_0, double C_is, double i_0, double C_rc,
                               double omega, double Omega_dot, double IDOT)
{
    // All the equations are based ON: https://www.gps.gov/technical/icwg/IS-GPS-200H.pdf
    // pg. 104-105, Also Grove  p335-338

    // Process subframe 1,2,3 information
    double A_semiMajorAxis;        // Semi-major axis
    double n_0_computedMeanMotion; // Computed mean motion
    double n_correctedMeanMotion;  // Corrected mean motion
    double e_eccentricity;         // Eccentricity
    //double phi_k_argumentOfLattitude;   // Argument of latitude
    double M_0_trueAnomalyAtRef;
    double omega0_longitudeofAscendingNodeofOrbitPlane;
    double omega_argumentOfPerigee;
    double omegaDot_argumentOfPerigee;
    double i_0_inclinationAtRef;
    double iDot_rateOfInclination;

    A_semiMajorAxis = pow(sqrt_A, 2);
    n_0_computedMeanMotion = sqrt(MU / pow(A_semiMajorAxis, 3));
    n_correctedMeanMotion = n_0_computedMeanMotion + delta_n;
    e_eccentricity = ecc;
    M_0_trueAnomalyAtRef = M_0;
    omega0_longitudeofAscendingNodeofOrbitPlane = Omega_0;
    omega_argumentOfPerigee = omega;
    omegaDot_argumentOfPerigee = Omega_dot;
    i_0_inclinationAtRef = i_0;
    iDot_rateOfInclination = IDOT;

    // Compute the time from the ephemeris reference epoch
    double t_k_timeFromReferenceEpoch = t - t_OE;
    // Correct that time for end-of-week crossovers
    if (t_k_timeFromReferenceEpoch > 302400)
    {
        t_k_timeFromReferenceEpoch -= 604800;
    }
    if (t_k_timeFromReferenceEpoch < -302400)
    {
        t_k_timeFromReferenceEpoch += 604800;
    }

    // Compute the mean anomaly
    double M_k_meanAnomaly = M_0_trueAnomalyAtRef + n_correctedMeanMotion * t_k_timeFromReferenceEpoch;

    // Below, we iteratively solve for E_k_eccentricAnomaly using Newton-Raphson method
    double solutionError = 1000000.;
    double E_k_eccentricAnomaly = 1.;
    double currentDerivative = 0;
    int iterationCount = 0;

    solutionError = (E_k_eccentricAnomaly -
                     (e_eccentricity * sin(E_k_eccentricAnomaly)) -
                     M_k_meanAnomaly);

    while ((fabs(solutionError) > 1.0e-6) &&
           iterationCount < 1000)
    {
        currentDerivative = (1.0 - (e_eccentricity * cos(E_k_eccentricAnomaly)));
        E_k_eccentricAnomaly = E_k_eccentricAnomaly - solutionError / currentDerivative;

        solutionError = (E_k_eccentricAnomaly -
                         (e_eccentricity * sin(E_k_eccentricAnomaly)) -
                         M_k_meanAnomaly);
        iterationCount += 1;
        //   if (VERBOSE)
        //   {
        //     std::cout<< "Iteration #: " << iterationCount << " Error: " << solutionError << std::endl;
        //   }
    }
    double cos_E_k = cos(E_k_eccentricAnomaly);
    double sin_E_k = sin(E_k_eccentricAnomaly);
    double nu_k_trueAnomaly = atan2(
        (sqrt(1.0 - pow(e_eccentricity, 2)) * sin_E_k) /
            (1.0 - (e_eccentricity * cos_E_k)),
        (cos_E_k - e_eccentricity) /
            (1.0 - e_eccentricity * cos_E_k));

    double phi_k_argumentOfLatitude = nu_k_trueAnomaly + omega_argumentOfPerigee;

    // Compute the corrective 2nd order terms
    double sin2PhiK = sin(2.0 * phi_k_argumentOfLatitude);
    double cos2PhiK = cos(2.0 * phi_k_argumentOfLatitude);

    double deltaU_argumentOfLatCorrection = (C_us * sin2PhiK) + (C_uc * cos2PhiK);
    double deltaR_radiusCorrection = (C_rs * sin2PhiK) + (C_rc * cos2PhiK);
    double deltaI_inclinationCorrection = (C_is * sin2PhiK) + (C_ic * cos2PhiK);

    // Now compute the updated corrected orbital elements
    double u_argumentOfLat = phi_k_argumentOfLatitude + deltaU_argumentOfLatCorrection;
    double r_radius = (A_semiMajorAxis * (1.0 - (e_eccentricity * cos_E_k))) + deltaR_radiusCorrection;
    double i_inclination =
        i_0_inclinationAtRef +
        (iDot_rateOfInclination * t_k_timeFromReferenceEpoch) +
        deltaI_inclinationCorrection;

    // Compute the satellite position within the orbital plane
    double xPositionOrbitalPlane = r_radius * cos(u_argumentOfLat);
    double yPositionOrbitalPlane = r_radius * sin(u_argumentOfLat);
    double omegaK_longitudeAscendingNode =
        omega0_longitudeofAscendingNodeofOrbitPlane +
        ((omegaDot_argumentOfPerigee - OMEGA_DOT_EARTH) * t_k_timeFromReferenceEpoch) -
        (OMEGA_DOT_EARTH * t_OE);

    double sinOmegaK = sin(omegaK_longitudeAscendingNode);
    double cosOmegaK = cos(omegaK_longitudeAscendingNode);

    double sinIK = sin(i_inclination);
    double cosIK = cos(i_inclination);
    // Earth-fixed coordinates:
    double x = (xPositionOrbitalPlane * cosOmegaK) - (yPositionOrbitalPlane * cosIK * sinOmegaK);
    double y = (xPositionOrbitalPlane * sinOmegaK) + (yPositionOrbitalPlane * cosIK * cosOmegaK);
    double z = (yPositionOrbitalPlane * sinIK);


    
    // ECEF velocity calculation:
    double E_dot_k_eccentricAnomaly = n_correctedMeanMotion/(1.0 - (e_eccentricity * cos_E_k)); // Eq.(8.21)
    double phi_dot_k_argumentOfLatitude = sin(nu_k_trueAnomaly)/sin_E_k*E_dot_k_eccentricAnomaly; // Eq.(8.22)
    double r_dot_o_os = (A_semiMajorAxis*e_eccentricity*sin_E_k)*E_dot_k_eccentricAnomaly +
                        2*((C_rs * cos2PhiK) - (C_rc * sin2PhiK))*phi_dot_k_argumentOfLatitude; // Eq.(8.23a)
    double u_dot_o_os = (1+2*C_us * cos2PhiK - 2*C_uc * sin2PhiK)*phi_dot_k_argumentOfLatitude; // Eq.(8.23b)                    
     
    double x_dot_o_os = r_dot_o_os*cos(u_argumentOfLat) - r_radius*u_dot_o_os*sin(u_argumentOfLat); // Eq.(8.24a)
    double y_dot_o_os = r_dot_o_os*sin(u_argumentOfLat) + r_radius*u_dot_o_os*cos(u_argumentOfLat); // Eq.(8.24b)

    double omega_dot_K_longitudeAscendingNode = omegaDot_argumentOfPerigee - OMEGA_DOT_EARTH; // Eq. (8.25)
    double i_dot_inclination = iDot_rateOfInclination + 2*((C_is * cos2PhiK) - (C_ic * sin2PhiK))*phi_dot_k_argumentOfLatitude; // Eq. (8.26)
    
    // Eq. (8.27)
    double vx = (x_dot_o_os*cosOmegaK - y_dot_o_os*cosIK*sinOmegaK + i_dot_inclination*yPositionOrbitalPlane*sinIK*sinOmegaK) -
                omega_dot_K_longitudeAscendingNode*(xPositionOrbitalPlane*sinOmegaK + yPositionOrbitalPlane*cosIK*cosOmegaK);
    double vy = (x_dot_o_os*sinOmegaK + y_dot_o_os*cosIK*cosOmegaK - i_dot_inclination*yPositionOrbitalPlane*sinIK*cosOmegaK) -
                omega_dot_K_longitudeAscendingNode*(-xPositionOrbitalPlane*cosOmegaK +yPositionOrbitalPlane*cosIK*sinOmegaK);
    double vz = (y_dot_o_os*sinIK + i_dot_inclination*yPositionOrbitalPlane*cosIK);
    
    VectorXd pos_vel_Sat_ecef(6);
    pos_vel_Sat_ecef << x, y, z, vx, vy, vz;
    // cout << x << y << y << endl;


    return pos_vel_Sat_ecef;
}


// Compute direction cosine matric C from a Euler vector 
// eul = [yaw,pitch,roll]. (i.e., 3-2-1 rotation convention)
Matrix3f Eul2DCM(Vector3f eul)
{
    Matrix3f C, C1, C2, C3;
    double ps=eul(0);
    double th=eul(1); 
    double ph=eul(2);
     
    C1(0, 0) = 1;       C1(0, 1) = 0;         C1(0, 2) = 0;
    C1(1, 0) = 0;       C1(1, 1) = cos(ph);   C1(1, 2) = sin(ph);
    C1(2, 0) = 0;       C1(2, 1) = -sin(ph);  C1(2, 2) = cos(ph);
   
    C2(0, 0) = cos(th);       C2(0, 1) = 0;   C2(0, 2) = -sin(th);
    C2(1, 0) = 0;             C2(1, 1) = 1;   C2(1, 2) = 0;
    C2(2, 0) = sin(th);       C2(2, 1) = 0;   C2(2, 2) = cos(th);
    
    C3(0, 0) = cos(ps);   C3(0, 1) = sin(ps);   C3(0, 2) = 0;
    C3(1, 0) = -sin(ps);  C3(1, 1) = cos(ps);   C3(1, 2) = 0;
    C3(2, 0) = 0;         C3(2, 1) = 0;         C3(2, 2) = 1;
   
    C=C1*C2*C3;
    return C;
}

// Convert DCM to Euler 
Vector3f DCM2Eul(Matrix3f T_B2L)
{
   Vector3f eul; 
   eul(0) = atan2f(T_B2L(1,2), T_B2L(2,2));
   eul(1) = -asinf(T_B2L(0,2));
   eul(2) = atan2f(T_B2L(0,1),T_B2L(0,0));
   return eul; 
}

// ned to ecef centered at the coordinate given by lla
Vector3f L2E(Vector3f p_L, Vector3f pRef_D)
{
    double lat =  pRef_D(0, 0);
    double lon =  pRef_D(1, 0);
    double pitch = abs(lat) + M_PI/2;
    Vector3f eul;
    if (lat >=0)
    {
        eul(0,0) = lon; 
        eul(1,0) = -pitch;
        eul(2,0) = 0;
    }
    else
    {
        eul(0,0) = lon; 
        eul(1,0) = pitch;
        eul(2,0) = 0;
    }

    Matrix<float,3,3> C_ecef2ned = Eul2DCM(eul);
    Vector3f ecef  = C_ecef2ned.transpose()*p_L;


    return ecef;
}

// compute vehicle's postion, velocity in E frame and clock offset (m) and drift (m/s) 
void GNSS_LS_pos_vel(MatrixXd &gnss_measurement,
                     Vector3d &pEst_E_m_, Vector3d &vEst_E_mps_, double &clockBias_m_, double &clockRateBias_mps_)
{
    /*
   GNSS_LS_position_velocity - Calculates position, velocity, clock offset, 
   and clock drift using unweighted iterated least squares. Separate
   calculations are implemented for position and clock offset and for
   velocity and clock drift

    % Inputs:
    %   GNSS_measurements     GNSS measurement data:
    %     Column 0              Pseudo-range measurements (m)
    %     Column 1              Pseudo-range rate measurements (m/s)
    %     Columns 2-4           Satellite ECEF position (m)
    %     Columns 5-7           Satellite ECEF velocity (m/s)
    %   no_GNSS_meas          Number of satellites for which measurements are
    %                         supplied
    %   pEst_E_m_        prior predicted ECEF user position (m)
    %   vEst_E_mps_      prior predicted ECEF user velocity (m/s)
    %
    % Outputs:

        output is a 8 by 1 matrix containing the following: 

    %   est_r_ea_e            estimated ECEF user position (m)
    %   est_v_ea_e            estimated ECEF user velocity (m/s)
    %   est_clock             estimated receiver clock offset (m) and drift (m/s)
   */

    int no_sat = gnss_measurement.rows();
    cout << "no_sats: " << no_sat << endl;
    cout << "pEst_E_m_:" << pEst_E_m_.transpose() << endl;
    cout << "vEst_E_mps_:" << vEst_E_mps_.transpose() << endl; // questionable vEst, check GNSS_LS_pos_vel calc later 
    cout << "clockBias_m: " << clockBias_m_ << endl;
    cout << "clockRateBias_mps " <<  clockRateBias_mps_ << endl;
  
    // Position and Clock OFFSET
    VectorXd x_pred(4, 1);
    x_pred.setZero();
    x_pred.segment(0, 3) = pEst_E_m_;
    x_pred(3) = 0;

    // allocate space for estimation
    VectorXd delta_r(3, 1);
    delta_r.setZero();
    double approx_range = 0;
    double range = 0;
    double range_rate = 0;
    VectorXd pred_meas(no_sat, 1);
    pred_meas.setZero();
    MatrixXd H(no_sat, 4);
    H.setZero();
    Matrix<double, 3, 3> T_E2I;
    T_E2I.Identity();
    VectorXd u_as_E(3, 1);
    u_as_E.setZero();

    // output
    Matrix<double, 4, 1> x_est_1;
    x_est_1.setZero();
    Matrix<double, 4, 1> x_est_2;
    x_est_2.setZero();
    VectorXd output(8, 1);
    output.setZero();

    // set the flag for the while-loop
    double test_convergence = 1;
    while (test_convergence > 0.0001)
    {
        cout << "test_convergence: " << test_convergence << endl;
        
        for (int j = 0; j < no_sat; j++)
        {
            Vector3d x_temp;
            x_temp(0) = gnss_measurement(j, 2);
            x_temp(1) = gnss_measurement(j, 3);
            x_temp(2) = gnss_measurement(j, 4);

            delta_r = x_temp - x_pred.segment(0, 3);
            approx_range = delta_r.norm();

            // Calculate frame rotation during signal transit time using (Grove2nd:8.36)
            T_E2I(0, 0) = 1;
            T_E2I(0, 1) = OMEGA_DOT_EARTH * approx_range / c;
            T_E2I(0, 2) = 0;
            T_E2I(1, 0) = -OMEGA_DOT_EARTH * approx_range / c;
            T_E2I(1, 1) = 1;
            T_E2I(1, 2) = 0;
            T_E2I(2, 0) = 0;
            T_E2I(2, 1) = 0;
            T_E2I(2, 2) = 1;

            delta_r = T_E2I.cast<double>() * x_temp - x_pred.segment(0, 3);
            range = delta_r.norm();
            pred_meas(j, 0) = range + x_pred(3);
            u_as_E = delta_r / range;
            H(j, 0) = -u_as_E(0);
            H(j, 1) = -u_as_E(1);
            H(j, 2) = -u_as_E(2);
            H(j, 3) = 1;
        }
        // least sqaure method
        x_est_1 = x_pred + (H.transpose() * H).inverse() * H.transpose() * (gnss_measurement.col(0) - pred_meas);
        test_convergence = (x_est_1 - x_pred).norm();
        x_pred = x_est_1;
        cout << "x_pred: " << x_pred << endl;
    }
    
    // Earth rotation vector and matrix
    Vector3d omega_ie; // Earth rate vector
    Matrix3d Omega_ie; // Earth rate rotation matrix
    omega_ie(0) = 0.0;
    omega_ie(1) = 0.0;
    omega_ie(2) = OMEGA_DOT_EARTH;
    Omega_ie = Skew(omega_ie);

    // save the estimated postion and clock offset
    // output(0) = x_est_1(0);
    // output(1) = x_est_1(1);
    // output(2) = x_est_1(2);
    // output(6) = x_est_1(3); // record clock offset
    pEst_E_m_(0) = x_est_1(0);
    pEst_E_m_(1) = x_est_1(1);
    pEst_E_m_(2) = x_est_1(2);
    clockBias_m_ = (float)x_est_1(3); // record clock offset

    x_pred.setZero();
    pred_meas.setZero();
    x_pred.segment(0, 3) = vEst_E_mps_;
    x_pred(3) = 0;
    test_convergence = 1;
    u_as_E.setZero();
    
    while (test_convergence > 0.0001)
    {
        for (int j = 0; j < no_sat; j++)
        {
            Vector3d p_temp, v_temp;
            p_temp(0) = gnss_measurement(j, 2);
            p_temp(1) = gnss_measurement(j, 3);
            p_temp(2) = gnss_measurement(j, 4);
            v_temp(0) = gnss_measurement(j, 5);
            v_temp(1) = gnss_measurement(j, 6);
            v_temp(2) = gnss_measurement(j, 7);

            delta_r = p_temp - x_est_1.segment(0, 3);
            approx_range = delta_r.norm();

            // Calculate frame rotation during signal transit time using (Grove2nd:8.36)
            T_E2I(0, 0) = 1;
            T_E2I(0, 1) = OMEGA_DOT_EARTH * approx_range / c;
            T_E2I(0, 2) = 0;
            T_E2I(1, 0) = -OMEGA_DOT_EARTH * approx_range / c;
            T_E2I(1, 1) = 1;
            T_E2I(1, 2) = 0;
            T_E2I(2, 0) = 0;
            T_E2I(2, 1) = 0;
            T_E2I(2, 2) = 1;

            delta_r = T_E2I.cast<double>() * p_temp - x_est_1.segment(0, 3);
            range = delta_r.norm();
            u_as_E = delta_r / range;
     
            // Predict pseudo-range rate using (9.165)
            range_rate = u_as_E.transpose() * (T_E2I.cast<double>() * (v_temp + Omega_ie.cast<double>() * p_temp) -
                                               (x_pred.segment(0, 3) + Omega_ie.cast<double>() * x_est_1.segment(0, 3)));
       
            pred_meas(j, 0) = range_rate + x_pred(3);  
    
            H(j, 0) = -u_as_E(0);
            H(j, 1) = -u_as_E(1);
            H(j, 2) = -u_as_E(2);
            H(j, 3) = 1;
        }

        x_est_2 = x_pred + (H.transpose() * H).inverse() * H.transpose() * (gnss_measurement.col(1) - pred_meas);
        test_convergence = (x_est_2 - x_pred).norm();
        x_pred = x_est_2;
    }
    
    // save the estimated postion and clock offset
    // output(3) = x_est_2(0);
    // output(4) = x_est_2(1);
    // output(5) = x_est_2(2);
    // output(7) = x_est_2(3);
    vEst_E_mps_(0) = x_est_2(0);
    vEst_E_mps_(1) = x_est_2(1);
    vEst_E_mps_(2) = x_est_2(2);
    clockRateBias_mps_ = (float)x_est_2(3);
}

// process raw measurement to give a 8 x 1 matrix (range, range rate, x,y,z,vx,vy,vz)
VectorXd EphemerisData2PosVelClock(GNSS_raw_measurement gnss_raw_measurement)
{
    // All the equations are based ON: https://www.gps.gov/technical/icwg/IS-GPS-200H.pdf
    // pg. 104-105, Also Grove  p335-338

    // Process subframe 1,2,3 information
    double A_semiMajorAxis = pow(gnss_raw_measurement.sqrtA, 2);        // Semi-major axis
    double n_0_computedMeanMotion = sqrt(MU / pow(A_semiMajorAxis, 3)); // Computed mean motion
    double n_correctedMeanMotion = n_0_computedMeanMotion + gnss_raw_measurement.deltan;// Corrected mean motion
    double e_eccentricity = gnss_raw_measurement.e; // Eccentricity
    //double phi_k_argumentOfLattitude;   // Argument of latitude
    double M_0_trueAnomalyAtRef = gnss_raw_measurement.M0;
    double omega0_longitudeofAscendingNodeofOrbitPlane = gnss_raw_measurement.Omega0;
    double omega_argumentOfPerigee = gnss_raw_measurement.omega;
    double omegaDot_argumentOfPerigee = gnss_raw_measurement.Omegad;
    double i_0_inclinationAtRef = gnss_raw_measurement.i0;
    double iDot_rateOfInclination = gnss_raw_measurement.IDOT;

    double t_OE = gnss_raw_measurement.toe;
    // 2nd harmonic terms
    double C_us = gnss_raw_measurement.Cus;
    double C_uc = gnss_raw_measurement.Cuc;
    double C_rs = gnss_raw_measurement.Crs;
    double C_rc = gnss_raw_measurement.Crc;
    double C_is = gnss_raw_measurement.Cis;
    double C_ic = gnss_raw_measurement.Cic;

    // Compute the time from the ephemeris reference epoch
    double t_k_timeFromReferenceEpoch = gnss_raw_measurement.timestamp - t_OE;
    // Correct that time for end-of-week crossovers
    if (t_k_timeFromReferenceEpoch > 302400)
    {
        t_k_timeFromReferenceEpoch -= 604800;
    }
    if (t_k_timeFromReferenceEpoch < -302400)
    {
        t_k_timeFromReferenceEpoch += 604800;
    }

    // Compute the mean anomaly
    double M_k_meanAnomaly = M_0_trueAnomalyAtRef + n_correctedMeanMotion * t_k_timeFromReferenceEpoch;

    // Below, we iteratively solve for E_k_eccentricAnomaly using Newton-Raphson method
    double solutionError = 1000000.;
    double E_k_eccentricAnomaly = 1.;
    double currentDerivative = 0;
    int iterationCount = 0;

    solutionError = (E_k_eccentricAnomaly -
                     (e_eccentricity * sin(E_k_eccentricAnomaly)) -
                     M_k_meanAnomaly);

    while ((fabs(solutionError) > 1.0e-6) &&
           iterationCount < 1000)
    {
        currentDerivative = (1.0 - (e_eccentricity * cos(E_k_eccentricAnomaly)));
        E_k_eccentricAnomaly = E_k_eccentricAnomaly - solutionError / currentDerivative;

        solutionError = (E_k_eccentricAnomaly -
                         (e_eccentricity * sin(E_k_eccentricAnomaly)) -
                         M_k_meanAnomaly);
        iterationCount += 1;
        //   if (VERBOSE)
        //   {
        //     std::cout<< "Iteration #: " << iterationCount << " Error: " << solutionError << std::endl;
        //   }
    }
    double cos_E_k = cos(E_k_eccentricAnomaly);
    double sin_E_k = sin(E_k_eccentricAnomaly);
    double nu_k_trueAnomaly = atan2(
        (sqrt(1.0 - pow(e_eccentricity, 2)) * sin_E_k) /
            (1.0 - (e_eccentricity * cos_E_k)),
        (cos_E_k - e_eccentricity) /
            (1.0 - e_eccentricity * cos_E_k));

    double phi_k_argumentOfLatitude = nu_k_trueAnomaly + omega_argumentOfPerigee;

    // Compute the corrective 2nd order terms
    double sin2PhiK = sin(2.0 * phi_k_argumentOfLatitude);
    double cos2PhiK = cos(2.0 * phi_k_argumentOfLatitude);

    double deltaU_argumentOfLatCorrection = (C_us * sin2PhiK) + (C_uc * cos2PhiK);
    double deltaR_radiusCorrection = (C_rs * sin2PhiK) + (C_rc * cos2PhiK);
    double deltaI_inclinationCorrection = (C_is * sin2PhiK) + (C_ic * cos2PhiK);

    // Now compute the updated corrected orbital elements
    double u_argumentOfLat = phi_k_argumentOfLatitude + deltaU_argumentOfLatCorrection;
    double r_radius = (A_semiMajorAxis * (1.0 - (e_eccentricity * cos_E_k))) + deltaR_radiusCorrection;
    double i_inclination =
        i_0_inclinationAtRef +
        (iDot_rateOfInclination * t_k_timeFromReferenceEpoch) +
        deltaI_inclinationCorrection;

    // Compute the satellite position within the orbital plane
    double xPositionOrbitalPlane = r_radius * cos(u_argumentOfLat);
    double yPositionOrbitalPlane = r_radius * sin(u_argumentOfLat);
    double omegaK_longitudeAscendingNode =
        omega0_longitudeofAscendingNodeofOrbitPlane +
        ((omegaDot_argumentOfPerigee - OMEGA_DOT_EARTH) * t_k_timeFromReferenceEpoch) -
        (OMEGA_DOT_EARTH * t_OE);

    double sinOmegaK = sin(omegaK_longitudeAscendingNode);
    double cosOmegaK = cos(omegaK_longitudeAscendingNode);

    double sinIK = sin(i_inclination);
    double cosIK = cos(i_inclination);
    // Earth-fixed coordinates:
    double x = (xPositionOrbitalPlane * cosOmegaK) - (yPositionOrbitalPlane * cosIK * sinOmegaK);
    double y = (xPositionOrbitalPlane * sinOmegaK) + (yPositionOrbitalPlane * cosIK * cosOmegaK);
    double z = (yPositionOrbitalPlane * sinIK);

    // ECEF velocity calculation:
    double E_dot_k_eccentricAnomaly = n_correctedMeanMotion / (1.0 - (e_eccentricity * cos_E_k));     // Eq.(8.21)
    double phi_dot_k_argumentOfLatitude = sin(nu_k_trueAnomaly) / sin_E_k * E_dot_k_eccentricAnomaly; // Eq.(8.22)
    double r_dot_o_os = (A_semiMajorAxis * e_eccentricity * sin_E_k) * E_dot_k_eccentricAnomaly +
                        2 * ((C_rs * cos2PhiK) - (C_rc * sin2PhiK)) * phi_dot_k_argumentOfLatitude;     // Eq.(8.23a)
    double u_dot_o_os = (1 + 2 * C_us * cos2PhiK - 2 * C_uc * sin2PhiK) * phi_dot_k_argumentOfLatitude; // Eq.(8.23b)

    double x_dot_o_os = r_dot_o_os * cos(u_argumentOfLat) - r_radius * u_dot_o_os * sin(u_argumentOfLat); // Eq.(8.24a)
    double y_dot_o_os = r_dot_o_os * sin(u_argumentOfLat) + r_radius * u_dot_o_os * cos(u_argumentOfLat); // Eq.(8.24b)

    double omega_dot_K_longitudeAscendingNode = omegaDot_argumentOfPerigee - OMEGA_DOT_EARTH;                                       // Eq. (8.25)
    double i_dot_inclination = iDot_rateOfInclination + 2 * ((C_is * cos2PhiK) - (C_ic * sin2PhiK)) * phi_dot_k_argumentOfLatitude; // Eq. (8.26)

    // Eq. (8.27)
    double vx = (x_dot_o_os * cosOmegaK - y_dot_o_os * cosIK * sinOmegaK + i_dot_inclination * yPositionOrbitalPlane * sinIK * sinOmegaK) -
                omega_dot_K_longitudeAscendingNode * (xPositionOrbitalPlane * sinOmegaK + yPositionOrbitalPlane * cosIK * cosOmegaK);
    double vy = (x_dot_o_os * sinOmegaK + y_dot_o_os * cosIK * cosOmegaK - i_dot_inclination * yPositionOrbitalPlane * sinIK * cosOmegaK) -
                omega_dot_K_longitudeAscendingNode * (-xPositionOrbitalPlane * cosOmegaK + yPositionOrbitalPlane * cosIK * sinOmegaK);
    double vz = (y_dot_o_os * sinIK + i_dot_inclination * yPositionOrbitalPlane * cosIK);

    VectorXd pos_vel_ecef_clock(8);
    double lambda = 2*c / (1575.4282e6);  // L1 according ublox8
    // https://www.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_%28UBX-13003221%29.pdf
    double PseudorangeRate = lambda * gnss_raw_measurement.doppler;
    pos_vel_ecef_clock << gnss_raw_measurement.pseudorange, PseudorangeRate, x, y, z, vx, vy, vz;
    // cout << x << y << y << endl;

    return pos_vel_ecef_clock;
}

VectorXd pv_ECEF_to_NED(Vector3d &pEst_E_m_, Vector3d &vEst_E_mps_)
{
  VectorXd pos_vel_ned(6);
  Vector3d pEst_D_rrm_;
  Vector3d vEst_L_mps_;
  // long
  pEst_D_rrm_(1) = atan2(pEst_E_m_(1),pEst_E_m_(0));
  double k1 = sqrt(1-Eccentricity*Eccentricity)*abs(pEst_E_m_(2));
  double k2 = Eccentricity*Eccentricity*EarthRadius;
  double beta = sqrt(pEst_E_m_(0)*pEst_E_m_(0) + pEst_E_m_(1)*pEst_E_m_(1));
  double E = (k1-k2)/beta;
  double F = (k1+k2)/beta;
  double P = 4/3.0f*(E*F + 1);
  double Q = 2.0f*(E*E - F*F);
  double D = P*P*P + Q*Q;
  double V = pow(sqrt(D)-Q,1/3.0f) - pow(sqrt(D)+Q,1/3.0f);
  double G = 0.5f*(sqrt(E*E+V)+E);
  double T = sqrt(G*G +(F-V*G)/(2.0f*G-E)) - G;
  // cout << "K1" << k1 << endl; 
  // cout << "k2" << k2 << endl; 
  // cout << "beta" << beta << endl;
  // cout << "E" << E << endl;
  // cout << "F " << F << endl; 
  // cout << "P " << P << endl; 
  // cout << "Q " << Q << endl;
  // cout << " V " << V << endl; 
  // cout << "G " << G << endl;
  // cout << "T: " << T << endl; 

  // lat
  pEst_D_rrm_(0) = copysign(1,pEst_E_m_(2))*atan((1-T*T)/(2*T*sqrt(1-Eccentricity*Eccentricity)));
  
  // altitude
  //cout << "beta: " << beta << ",T: " << T << ", pEst_E_m_(2): " << pEst_E_m_(2) << ", pEst_D_rrm_(0)" << pEst_D_rrm_(0) << endl; 
  pEst_D_rrm_(2) = (beta-EarthRadius*T)*cos(pEst_D_rrm_(0))+
                  (pEst_E_m_(2) - copysign(1,pEst_E_m_(2))*EarthRadius*sqrt(1-Eccentricity*Eccentricity))*sin(pEst_D_rrm_(0));
  double cos_lat = cos(pEst_D_rrm_(0));
  double sin_lat = sin(pEst_D_rrm_(0));
  double cos_lon = cos(pEst_D_rrm_(1));
  double sin_lon = sin(pEst_D_rrm_(1));

  Matrix3d T_E2N;                
  T_E2N(0,0) = -sin_lat*cos_lon;	T_E2N(0,1) = -sin_lat*sin_lon;	T_E2N(0,2) =  cos_lat;
  T_E2N(1,0) =         -sin_lon;	T_E2N(1,1) =          cos_lon;	T_E2N(1,2) =        0;
  T_E2N(2,0) = -cos_lat*cos_lon;	T_E2N(2,1) = -cos_lat*sin_lon;	T_E2N(2,2) = -sin_lat;
  
  vEst_L_mps_ = T_E2N*vEst_E_mps_;

  pos_vel_ned.segment(0,3) =  pEst_D_rrm_;
  pos_vel_ned.segment(3,3) =  vEst_L_mps_;

  return pos_vel_ned; 

}
// Gravitation_ECI - Calculates  acceleration due to gravity resolved about ECEF-frame
Vector3d Gravity_ECEF(Vector3d &pEst_E_m_)
{
   Vector3d aGrav_E_mps2;
   double  mag_r = sqrt((pEst_E_m_.transpose())*pEst_E_m_);
   if (mag_r == 0)
   {
     aGrav_E_mps2(0) = 0; aGrav_E_mps2(1) = 0; aGrav_E_mps2(2) = 0; 
   }
   else
   {
     double z_scale = 5.0f * (pEst_E_m_(2)/mag_r)*(pEst_E_m_(2)/mag_r);
     Vector3d gamma;
     gamma(0) = -MU/pow(mag_r,3)*(pEst_E_m_(0) + 1.5f*J2*pow(EarthRadius/mag_r,2)*(1-z_scale)*pEst_E_m_(0));
     gamma(1) = -MU/pow(mag_r,3)*(pEst_E_m_(1) + 1.5f*J2*pow(EarthRadius/mag_r,2)*(1-z_scale)*pEst_E_m_(1));
     gamma(2) = -MU/pow(mag_r,3)*(pEst_E_m_(2) + 1.5f*J2*pow(EarthRadius/mag_r,2)*(3-z_scale)*pEst_E_m_(2));
     aGrav_E_mps2(0) = gamma(0) + pow(OMEGA_DOT_EARTH,2)*pEst_E_m_(0); 
     aGrav_E_mps2(1) = gamma(1) + pow(OMEGA_DOT_EARTH,2)*pEst_E_m_(1); 
     aGrav_E_mps2(2) = gamma(2);
   }

  return aGrav_E_mps2;  

}
