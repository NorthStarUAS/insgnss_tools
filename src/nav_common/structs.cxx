// The following constructs a python interface for this class and
// associated structures.

#include "structs.hxx"
#include "wgs84.hxx"

#include <boost/python.hpp>
using namespace boost::python;

BOOST_PYTHON_MODULE(structs)
{
    class_<IMUdata>("IMUdata")
	.def_readwrite("time", &IMUdata::time)
	.def_readwrite("p", &IMUdata::p)
	.def_readwrite("q", &IMUdata::q)
	.def_readwrite("r", &IMUdata::r)
 	.def_readwrite("ax", &IMUdata::ax)
 	.def_readwrite("ay", &IMUdata::ay)
 	.def_readwrite("az", &IMUdata::az)
 	.def_readwrite("hx", &IMUdata::hx)
 	.def_readwrite("hy", &IMUdata::hy)
 	.def_readwrite("hz", &IMUdata::hz)
 	.def_readwrite("temp", &IMUdata::temp)
    ;
    
    class_<GPSdata>("GPSdata")
	.def_readwrite("time", &GPSdata::time)
	.def_readwrite("lat", &GPSdata::lat)
	.def_readwrite("lon", &GPSdata::lon)
	.def_readwrite("alt", &GPSdata::alt)
	.def_readwrite("vn", &GPSdata::vn)
	.def_readwrite("ve", &GPSdata::ve)
	.def_readwrite("vd", &GPSdata::vd)
	.def_readwrite("sats", &GPSdata::sats)
	.def_readwrite("newData", &GPSdata::newData)
    ;

    class_<Airdata>("Airdata")
	.def_readwrite("time", &Airdata::time)
	.def_readwrite("static_press", &Airdata::static_press)
	.def_readwrite("diff_press", &Airdata::diff_press)
	.def_readwrite("temp", &Airdata::temp)
	.def_readwrite("airspeed", &Airdata::airspeed)
	.def_readwrite("altitude", &Airdata::altitude)
    ;

    class_<NAVdata>("NAVdata")
	.def_readwrite("time", &NAVdata::time)
	.def_readwrite("lat", &NAVdata::lat)
	.def_readwrite("lon", &NAVdata::lon)
	.def_readwrite("alt", &NAVdata::alt)
	.def_readwrite("vn", &NAVdata::vn)
	.def_readwrite("ve", &NAVdata::ve)
	.def_readwrite("vd", &NAVdata::vd)
	.def_readwrite("phi", &NAVdata::phi)
	.def_readwrite("the", &NAVdata::the)
	.def_readwrite("psi", &NAVdata::psi)
	.def_readwrite("qw", &NAVdata::qw)
	.def_readwrite("qx", &NAVdata::qx)
	.def_readwrite("qy", &NAVdata::qy)
	.def_readwrite("qz", &NAVdata::qz)
	.def_readwrite("abx", &NAVdata::abx)
	.def_readwrite("aby", &NAVdata::aby)
	.def_readwrite("abz", &NAVdata::abz)
	.def_readwrite("gbx", &NAVdata::gbx)
	.def_readwrite("gby", &NAVdata::gby)
	.def_readwrite("gbz", &NAVdata::gbz)
	.def_readwrite("Pp0", &NAVdata::Pp0)
	.def_readwrite("Pp1", &NAVdata::Pp1)
	.def_readwrite("Pp2", &NAVdata::Pp2)
	.def_readwrite("Pv0", &NAVdata::Pv0)
	.def_readwrite("Pv1", &NAVdata::Pv1)
	.def_readwrite("Pv2", &NAVdata::Pv2)
	.def_readwrite("Pa0", &NAVdata::Pa0)
	.def_readwrite("Pa1", &NAVdata::Pa1)
	.def_readwrite("Pa2", &NAVdata::Pa2)
	.def_readwrite("Pabx", &NAVdata::Pabx)
	.def_readwrite("Paby", &NAVdata::Paby)
	.def_readwrite("Pabz", &NAVdata::Pabz)
	.def_readwrite("Pgbx", &NAVdata::Pgbx)
	.def_readwrite("Pgby", &NAVdata::Pgby)
	.def_readwrite("Pgbz", &NAVdata::Pgbz)
    ;

    class_<NAVconfig>("NAVconfig")
	.def_readwrite("sig_w_ax", &NAVconfig::sig_w_ax)
	.def_readwrite("sig_w_ay", &NAVconfig::sig_w_ay)
	.def_readwrite("sig_w_az", &NAVconfig::sig_w_az)
	.def_readwrite("sig_w_gx", &NAVconfig::sig_w_gx)
	.def_readwrite("sig_w_gy", &NAVconfig::sig_w_gy)
	.def_readwrite("sig_w_gz", &NAVconfig::sig_w_gz)
	.def_readwrite("sig_a_d", &NAVconfig::sig_a_d)
	.def_readwrite("tau_a", &NAVconfig::tau_a)
	.def_readwrite("sig_g_d", &NAVconfig::sig_g_d)
	.def_readwrite("tau_g", &NAVconfig::tau_g)
	.def_readwrite("sig_gps_p_ne", &NAVconfig::sig_gps_p_ne)
	.def_readwrite("sig_gps_p_d", &NAVconfig::sig_gps_p_d)
	.def_readwrite("sig_gps_v_ne", &NAVconfig::sig_gps_v_ne)
	.def_readwrite("sig_gps_v_d", &NAVconfig::sig_gps_v_d)
	.def_readwrite("sig_mag", &NAVconfig::sig_mag)
    ;
}
