#include <limits>

#include <boost/python.hpp>
using namespace boost::python;

#include <math.h>

#include "wgs84.hxx"

// These are hard numbers from the WGS84 standard.  DON'T MODIFY
// unless you want to change the datum.
static const double EQURAD = 6378137.0;
static const double iFLATTENING = 298.257223563;

static double d2r = M_PI / 180.0;
static double r2d = 180.0 / M_PI;


////////////////////////////////////////////////////////////////////////
//
// Direct and inverse distance functions 
//
// Proceedings of the 7th International Symposium on Geodetic
// Computations, 1985
//
// "The Nested Coefficient Method for Accurate Solutions of Direct and
// Inverse Geodetic Problems With Any Length"
//
// Zhang Xue-Lian
// pp 747-763
//
// modified for FlightGear to use WGS84 only -- Norman Vine
// modified for AuraUAS by Curtis Olson

static inline double M0( double e2 ) {
    //double e4 = e2*e2;
    return M_PI*0.5*(1.0 - e2*( 1.0/4.0 + e2*( 3.0/64.0 + 
					       e2*(5.0/256.0) )));
}

// given, lat1, lon1, az1 and distance (s), calculate lat2, lon2
// and az2.  Lat, lon, and azimuth are in degrees.  distance in meters
static int _geo_direct_wgs_84 ( double lat1, double lon1, double az1,
                        double s, double *lat2, double *lon2,
                        double *az2 )
{
    double a = EQURAD, rf = iFLATTENING;
    double testv = 1.0E-10;
    double f = ( rf > 0.0 ? 1.0/rf : 0.0 );
    double b = a*(1.0-f);
    double e2 = f*(2.0-f);
    double phi1 = lat1*d2r, lam1 = lon1*d2r;
    double sinphi1 = sin(phi1), cosphi1 = cos(phi1);
    double azm1 = az1*d2r;
    double sinaz1 = sin(azm1), cosaz1 = cos(azm1);
	
	
    if( fabs(s) < 0.01 ) {	// distance < centimeter => congruency
	*lat2 = lat1;
	*lon2 = lon1;
	*az2 = 180.0 + az1;
	if( *az2 > 360.0 ) *az2 -= 360.0;
	return 0;
    } else if( fabs(cosphi1) > std::numeric_limits<int>::min() ) {
    	// non-polar origin
	// u1 is reduced latitude
	double tanu1 = sqrt(1.0-e2)*sinphi1/cosphi1;
	double sig1 = atan2(tanu1,cosaz1);
	double cosu1 = 1.0/sqrt( 1.0 + tanu1*tanu1 ), sinu1 = tanu1*cosu1;
	double sinaz =  cosu1*sinaz1, cos2saz = 1.0-sinaz*sinaz;
	double us = cos2saz*e2/(1.0-e2);

	// Terms
	double	ta = 1.0+us*(4096.0+us*(-768.0+us*(320.0-175.0*us)))/16384.0,
	    tb = us*(256.0+us*(-128.0+us*(74.0-47.0*us)))/1024.0,
	    tc = 0;

	// FIRST ESTIMATE OF SIGMA (SIG)
	double first = s/(b*ta);  // !!
	double sig = first;
	double c2sigm, sinsig,cossig, temp,denom,rnumer, dlams, dlam;
	do {
	    c2sigm = cos(2.0*sig1+sig);
	    sinsig = sin(sig); cossig = cos(sig);
	    temp = sig;
	    sig = first + 
		tb*sinsig*(c2sigm+tb*(cossig*(-1.0+2.0*c2sigm*c2sigm) - 
				      tb*c2sigm*(-3.0+4.0*sinsig*sinsig)
				      *(-3.0+4.0*c2sigm*c2sigm)/6.0)
			   /4.0);
	} while( fabs(sig-temp) > testv);

	// LATITUDE OF POINT 2
	// DENOMINATOR IN 2 PARTS (TEMP ALSO USED LATER)
	temp = sinu1*sinsig-cosu1*cossig*cosaz1;
	denom = (1.0-f)*sqrt(sinaz*sinaz+temp*temp);

	// NUMERATOR
	rnumer = sinu1*cossig+cosu1*sinsig*cosaz1;
	*lat2 = atan2(rnumer,denom)*r2d;

	// DIFFERENCE IN LONGITUDE ON AUXILARY SPHERE (DLAMS )
	rnumer = sinsig*sinaz1;
	denom = cosu1*cossig-sinu1*sinsig*cosaz1;
	dlams = atan2(rnumer,denom);

	// TERM C
	tc = f*cos2saz*(4.0+f*(4.0-3.0*cos2saz))/16.0;

	// DIFFERENCE IN LONGITUDE
	dlam = dlams-(1.0-tc)*f*sinaz*(sig+tc*sinsig*
				       (c2sigm+
					tc*cossig*(-1.0+2.0*
						   c2sigm*c2sigm)));
	*lon2 = (lam1+dlam)*r2d;
	if (*lon2 > 180.0  ) *lon2 -= 360.0;
	if (*lon2 < -180.0 ) *lon2 += 360.0;

	// AZIMUTH - FROM NORTH
	*az2 = atan2(-sinaz,temp)*r2d;
	if ( fabs(*az2) < testv ) *az2 = 0.0;
	if( *az2 < 0.0) *az2 += 360.0;
	return 0;
    } else {			// phi1 == 90 degrees, polar origin
	double dM = a*M0(e2) - s;
	double paz = ( phi1 < 0.0 ? 180.0 : 0.0 );
        double zero = 0.0f;
	return _geo_direct_wgs_84( zero, lon1, paz, dM, lat2, lon2, az2 );
    } 
}

// given lat1, lon1, lat2, lon2, calculate starting and ending
// az1, az2 and distance (s).  Lat, lon, and azimuth are in degrees.
// distance in meters
static int _geo_inverse_wgs_84( double lat1, double lon1, double lat2,
				double lon2, double *az1, double *az2,
				double *s )
{
    double a = EQURAD, rf = iFLATTENING;
    int iter=0;
    double testv = 1.0E-10;
    double f = ( rf > 0.0 ? 1.0/rf : 0.0 );
    double b = a*(1.0-f);
    // double e2 = f*(2.0-f); // unused in this routine
    double phi1 = lat1*d2r, lam1 = lon1*d2r;
    double sinphi1 = sin(phi1), cosphi1 = cos(phi1);
    double phi2 = lat2*d2r, lam2 = lon2*d2r;
    double sinphi2 = sin(phi2), cosphi2 = cos(phi2);
	
    if( (fabs(lat1-lat2) < testv && 
	 ( fabs(lon1-lon2) < testv)) || (fabs(lat1-90.0) < testv ) )
    {	
	// TWO STATIONS ARE IDENTICAL : SET DISTANCE & AZIMUTHS TO ZERO */
	*az1 = 0.0; *az2 = 0.0; *s = 0.0;
	return 0;
    } else if(  fabs(cosphi1) < testv ) {
	// initial point is polar
	int k = _geo_inverse_wgs_84( lat2,lon2,lat1,lon1, az1,az2,s );
	k = k; // avoid compiler error since return result is unused
	b = *az1; *az1 = *az2; *az2 = b;
	return 0;
    } else if( fabs(cosphi2) < testv ) {
	// terminal point is polar
        double _lon1 = lon1 + 180.0f;
	int k = _geo_inverse_wgs_84( lat1, lon1, lat1, _lon1, 
				    az1, az2, s );
	k = k; // avoid compiler error since return result is unused
	*s /= 2.0;
	*az2 = *az1 + 180.0;
	if( *az2 > 360.0 ) *az2 -= 360.0; 
	return 0;
    } else if( (fabs( fabs(lon1-lon2) - 180 ) < testv) && 
	       (fabs(lat1+lat2) < testv) ) 
    {
	// Geodesic passes through the pole (antipodal)
	double s1,s2;
	_geo_inverse_wgs_84( lat1,lon1, lat1,lon2, az1,az2, &s1 );
	_geo_inverse_wgs_84( lat2,lon2, lat1,lon2, az1,az2, &s2 );
	*az2 = *az1;
	*s = s1 + s2;
	return 0;
    } else {
	// antipodal and polar points don't get here
	double dlam = lam2 - lam1, dlams = dlam;
	double sdlams,cdlams, sig,sinsig,cossig, sinaz,
	    cos2saz, c2sigm;
	double tc,temp, us,rnumer,denom, ta,tb;
	double cosu1,sinu1, sinu2,cosu2;

	// Reduced latitudes
	temp = (1.0-f)*sinphi1/cosphi1;
	cosu1 = 1.0/sqrt(1.0+temp*temp);
	sinu1 = temp*cosu1;
	temp = (1.0-f)*sinphi2/cosphi2;
	cosu2 = 1.0/sqrt(1.0+temp*temp);
	sinu2 = temp*cosu2;
    
	do {
	    sdlams = sin(dlams), cdlams = cos(dlams);
	    sinsig = sqrt(cosu2*cosu2*sdlams*sdlams+
			  (cosu1*sinu2-sinu1*cosu2*cdlams)*
			  (cosu1*sinu2-sinu1*cosu2*cdlams));
	    cossig = sinu1*sinu2+cosu1*cosu2*cdlams;
	    
	    sig = atan2(sinsig,cossig);
	    sinaz = cosu1*cosu2*sdlams/sinsig;
	    cos2saz = 1.0-sinaz*sinaz;
	    c2sigm = (sinu1 == 0.0 || sinu2 == 0.0 ? cossig : 
		      cossig-2.0*sinu1*sinu2/cos2saz);
	    tc = f*cos2saz*(4.0+f*(4.0-3.0*cos2saz))/16.0;
	    temp = dlams;
	    dlams = dlam+(1.0-tc)*f*sinaz*
		(sig+tc*sinsig*
		 (c2sigm+tc*cossig*(-1.0+2.0*c2sigm*c2sigm)));
	    if (fabs(dlams) > M_PI && iter++ > 50) {
		return iter;
	    }
	} while ( fabs(temp-dlams) > testv);

	us = cos2saz*(a*a-b*b)/(b*b); // !!
	// BACK AZIMUTH FROM NORTH
	rnumer = -(cosu1*sdlams);
	denom = sinu1*cosu2-cosu1*sinu2*cdlams;
	*az2 = atan2(rnumer,denom)*r2d;
	if( fabs(*az2) < testv ) *az2 = 0.0;
	if(*az2 < 0.0) *az2 += 360.0;

	// FORWARD AZIMUTH FROM NORTH
	rnumer = cosu2*sdlams;
	denom = cosu1*sinu2-sinu1*cosu2*cdlams;
	*az1 = atan2(rnumer,denom)*r2d;
	if( fabs(*az1) < testv ) *az1 = 0.0;
	if(*az1 < 0.0) *az1 += 360.0;

	// Terms a & b
	ta = 1.0+us*(4096.0+us*(-768.0+us*(320.0-175.0*us)))/
	    16384.0;
	tb = us*(256.0+us*(-128.0+us*(74.0-47.0*us)))/1024.0;

	// GEODETIC DISTANCE
	*s = b*ta*(sig-tb*sinsig*
		   (c2sigm+tb*(cossig*(-1.0+2.0*c2sigm*c2sigm)-tb*
			       c2sigm*(-3.0+4.0*sinsig*sinsig)*
			       (-3.0+4.0*c2sigm*c2sigm)/6.0)/
		    4.0));
	return 0;
    }
}

tuple py_geo_direct_wgs84(double lat1, double lon1, double az1, double s) {
    double lat2, lon2, az2;
    _geo_direct_wgs_84( lat1, lon1, az1, s, &lat2, &lon2, &az2 );
    return make_tuple(lat2, lon2, az2);
}

tuple py_geo_inverse_wgs84(double lat1, double lon1, double lat2, double lon2) {
    double az1, az2, s;
    _geo_inverse_wgs_84( lat1, lon1, lat2, lon2, &az1, &az2, &s );
    return make_tuple(az1, az2, s);
}


#ifdef HAVE_BOOST_PYTHON

// The following constructs a python interface for this class.

BOOST_PYTHON_MODULE(wgs84)
{
    def("geo_direct_wgs84", py_geo_direct_wgs84);
    def("geo_inverse_wgs84", py_geo_inverse_wgs84);
}

#endif // HAVE_BOOST_PYTHON
