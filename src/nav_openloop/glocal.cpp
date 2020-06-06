// Compute local gravity as a function of latitude (radians) and
// altitude (m).  Returns the local gravity vector in ned coordinates.
// In other words, it computes the magnitude of the of local gravity
// at the wgs84 reference elliposid using the Somigliana model and
// makes corrections for altitude. 

#include <math.h>

#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
using namespace Eigen;

#include "glocal.h"


static double f = 1.0 / 298.257223563; // WGS-84 Flattening.
static double e = sqrt( f * (2 - f) ); // Eccentricity.
static double omega_ie = 7.292115e-5;  // WGS-84 Earth rate (rad/s).
static double R_0 = 6378137;           // WGS-84 equatorial radius (m).
static double R_P = R_0 * (1 - f);	   // Polar radius (m).
static double mu_E = 3.986004418e14;   // WGS-84 Earth's gravitational
static double g_equator = 9.7803253359; // Normal Equatorial Gravity
static double g_n_const = 0.00193185265241; // Normal Gravity Constant


Vector3f local_gravity( double lat_rad, float alt_m ) {
    double sin_lat = sin(lat_rad);
    double e_sin_lat = e * sin_lat;
    double g_0 = ( g_equator / (sqrt(1.0 - e_sin_lat*e_sin_lat)) )
	* ( 1.0 + g_n_const*sin_lat*sin_lat);

    double omega_ie_R_0 = omega_ie * R_0;
    double alt_m_R_0 = alt_m / R_0;
    double k = 1.0 - (2.0 * alt_m / R_0)
	* ( 1.0 + f + ( omega_ie_R_0*omega_ie_R_0 ) * (R_P / mu_E) )
        + 3.0 * alt_m_R_0*alt_m_R_0;

    Vector3f glocal(0.0, 0.0, k * g_0);

    return glocal;
}
