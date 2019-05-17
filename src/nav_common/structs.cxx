// The following constructs a python interface for this class and
// associated structures.

#include "structs.hxx"

#ifdef HAVE_PYBIND11

#include <pybind11/pybind11.h>
namespace py = pybind11;

PYBIND11_MODULE(structs, m) {
    py::class_<IMUdata>(m, "IMUdata")
        .def(py::init<>())
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
 	.def_readwrite("temp", &IMUdata::temp);
    
    py::class_<GPSdata>(m, "GPSdata")
        .def(py::init<>())
	.def_readwrite("time", &GPSdata::time)
	.def_readwrite("lat", &GPSdata::lat)
	.def_readwrite("lon", &GPSdata::lon)
	.def_readwrite("alt", &GPSdata::alt)
	.def_readwrite("vn", &GPSdata::vn)
	.def_readwrite("ve", &GPSdata::ve)
	.def_readwrite("vd", &GPSdata::vd)
	.def_readwrite("sats", &GPSdata::sats)
	.def_readwrite("newData", &GPSdata::newData);
    
    py::class_<Airdata>(m, "Airdata")
        .def(py::init<>())
	.def_readwrite("time", &Airdata::time)
	.def_readwrite("static_press", &Airdata::static_press)
	.def_readwrite("diff_press", &Airdata::diff_press)
	.def_readwrite("temp", &Airdata::temp)
	.def_readwrite("airspeed", &Airdata::airspeed)
	.def_readwrite("altitude", &Airdata::altitude);

    py::class_<NAVdata>(m, "NAVdata")
        .def(py::init<>())
	.def_readonly("time", &NAVdata::time)
	.def_readonly("lat", &NAVdata::lat)
	.def_readonly("lon", &NAVdata::lon)
	.def_readonly("alt", &NAVdata::alt)
	.def_readonly("vn", &NAVdata::vn)
	.def_readonly("ve", &NAVdata::ve)
	.def_readonly("vd", &NAVdata::vd)
	.def_readonly("phi", &NAVdata::phi)
	.def_readonly("the", &NAVdata::the)
	.def_readonly("psi", &NAVdata::psi)
	.def_readonly("qw", &NAVdata::qw)
	.def_readonly("qx", &NAVdata::qx)
	.def_readonly("qy", &NAVdata::qy)
	.def_readonly("qz", &NAVdata::qz)
	.def_readonly("abx", &NAVdata::abx)
	.def_readonly("aby", &NAVdata::aby)
	.def_readonly("abz", &NAVdata::abz)
	.def_readonly("gbx", &NAVdata::gbx)
	.def_readonly("gby", &NAVdata::gby)
	.def_readonly("gbz", &NAVdata::gbz)
	.def_readonly("Pp0", &NAVdata::Pp0)
	.def_readonly("Pp1", &NAVdata::Pp1)
	.def_readonly("Pp2", &NAVdata::Pp2)
	.def_readonly("Pv0", &NAVdata::Pv0)
	.def_readonly("Pv1", &NAVdata::Pv1)
	.def_readonly("Pv2", &NAVdata::Pv2)
	.def_readonly("Pa0", &NAVdata::Pa0)
	.def_readonly("Pa1", &NAVdata::Pa1)
	.def_readonly("Pa2", &NAVdata::Pa2)
	.def_readonly("Pabx", &NAVdata::Pabx)
	.def_readonly("Paby", &NAVdata::Paby)
	.def_readonly("Pabz", &NAVdata::Pabz)
	.def_readonly("Pgbx", &NAVdata::Pgbx)
	.def_readonly("Pgby", &NAVdata::Pgby)
	.def_readonly("Pgbz", &NAVdata::Pgbz);

    py::class_<NAVconfig>(m, "NAVconfig")
        .def(py::init<>())
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
	.def_readwrite("sig_mag", &NAVconfig::sig_mag);
}

#endif
