// The following constructs a python interface for this class and
// associated structures.

#include "structs.h"

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
        .def_readwrite("temp", &IMUdata::temp)
        .def("as_dict",
             [](const IMUdata &imu) {
                 py::dict result;
                 result["time"] = imu.time;
                 result["p"] = imu.p;
                 result["q"] = imu.q;
                 result["r"] = imu.r;
                 result["ax"] = imu.ax;
                 result["ay"] = imu.ay;
                 result["az"] = imu.az;
                 result["hx"] = imu.hx;
                 result["hy"] = imu.hy;
                 result["hz"] = imu.hz;
                 result["temp"] = imu.temp;
                 return result;
             }
         )
        .def("from_dict",
             [](IMUdata &imu, const py::dict &d) {
                 imu.time = py::float_(d["time"]);
                 imu.p = py::float_(d["p"]);
                 imu.q = py::float_(d["q"]);
                 imu.r = py::float_(d["r"]);
                 imu.ax = py::float_(d["ax"]);
                 imu.ay = py::float_(d["ay"]);
                 imu.az = py::float_(d["az"]);
                 imu.hx = py::float_(d["hx"]);
                 imu.hy = py::float_(d["hy"]);
                 imu.hz = py::float_(d["hz"]);
                 imu.temp = py::float_(d["temp"]);
             }
         )
        ;

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
        .def("as_dict",
             [](const GPSdata &gps) {
                 py::dict result;
                 result["time"] = gps.time;
                 result["unix_sec"] = gps.unix_sec;
                 result["lat"] = gps.lat;
                 result["lon"] = gps.lon;
                 result["alt"] = gps.alt;
                 result["vn"] = gps.vn;
                 result["ve"] = gps.ve;
                 result["vd"] = gps.vd;
                 result["sats"] = gps.sats;
                 return result;
             }
         )
        .def("from_dict",
             [](GPSdata &gps, const py::dict &d) {
                 gps.time = py::float_(d["time"]);
                 gps.unix_sec = py::float_(d["unix_sec"]);
                 gps.lat = py::float_(d["lat"]);
                 gps.lon = py::float_(d["lon"]);
                 gps.alt = py::float_(d["alt"]);
                 gps.vn = py::float_(d["vn"]);
                 gps.ve = py::float_(d["ve"]);
                 gps.vd = py::float_(d["vd"]);
                 gps.sats = py::int_(d["sats"]);
              }
         )
        ;

    py::class_<Airdata>(m, "Airdata")
        .def(py::init<>())
        .def_readwrite("time", &Airdata::time)
        .def_readwrite("static_press", &Airdata::static_press)
        .def_readwrite("diff_press", &Airdata::diff_press)
        .def_readwrite("temp", &Airdata::temp)
        .def_readwrite("airspeed", &Airdata::airspeed)
        .def_readwrite("altitude", &Airdata::altitude)
        .def("as_dict",
             [](const Airdata &airdata) {
                 py::dict result;
                 result["time"] = airdata.time;
                 result["static_press"] = airdata.static_press;
                 result["diff_press"] = airdata.diff_press;
                 result["temp"] = airdata.temp;
                 result["airspeed"] = airdata.airspeed;
                 result["altitude"] = airdata.altitude;
                 return result;
             }
         )
        ;

    py::class_<NAVdata>(m, "NAVdata")
        .def(py::init<>())
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
        .def("as_dict",
             [](const NAVdata &nav) {
                 py::dict result;
                 result["time"] = nav.time;
                 result["lat"] = nav.lat;
                 result["lon"] = nav.lon;
                 result["alt"] = nav.alt;
                 result["vn"] = nav.vn;
                 result["ve"] = nav.ve;
                 result["vd"] = nav.vd;
                 result["phi"] = nav.phi;
                 result["the"] = nav.the;
                 result["psi"] = nav.psi;
                 result["qw"] = nav.qw;
                 result["qx"] = nav.qx;
                 result["qy"] = nav.qy;
                 result["qz"] = nav.qz;
                 result["abx"] = nav.abx;
                 result["aby"] = nav.aby;
                 result["abz"] = nav.abz;
                 result["gbx"] = nav.gbx;
                 result["gby"] = nav.gby;
                 result["gbz"] = nav.gbz;
                 result["Pp0"] = nav.Pp0;
                 result["Pp1"] = nav.Pp1;
                 result["Pp2"] = nav.Pp2;
                 result["Pv0"] = nav.Pv0;
                 result["Pv1"] = nav.Pv1;
                 result["Pv2"] = nav.Pv2;
                 result["Pa0"] = nav.Pa0;
                 result["Pa1"] = nav.Pa1;
                 result["Pa2"] = nav.Pa2;
                 result["Pabx"] = nav.Pabx;
                 result["Paby"] = nav.Paby;
                 result["Pabz"] = nav.Pabz;
                 result["Pgbx"] = nav.Pgbx;
                 result["Pgby"] = nav.Pgby;
                 result["Pgbz"] = nav.Pgbz;
                 return result;
             }
         )
        ;

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
        .def_readwrite("sig_mag", &NAVconfig::sig_mag)
        .def("as_dict",
             [](const NAVconfig &config) {
                 py::dict result;
                 result["sig_w_ax"] = config.sig_w_ax;
                 result["sig_w_ay"] = config.sig_w_ay;
                 result["sig_w_az"] = config.sig_w_az;
                 result["sig_w_gx"] = config.sig_w_gx;
                 result["sig_w_gy"] = config.sig_w_gy;
                 result["sig_w_gz"] = config.sig_w_gz;
                 result["sig_a_d"] = config.sig_a_d;
                 result["tau_a"] = config.tau_a;
                 result["sig_g_d"] = config.sig_g_d;
                 result["tau_g"] = config.tau_g;
                 result["sig_gps_p_ne"] = config.sig_gps_p_ne;
                 result["sig_gps_p_d"] = config.sig_gps_p_d;
                 result["sig_gps_v_ne"] = config.sig_gps_v_ne;
                 result["sig_gps_v_d"] = config.sig_gps_v_d;
                 result["sig_mag"] = config.sig_mag;
                 return result;
             }
         )
        .def("from_dict",
             [](NAVconfig &config, const py::dict &d) {
                 config.sig_w_ax = py::float_(d["sig_w_ax"]);
                 config.sig_w_ay = py::float_(d["sig_w_ay"]);
                 config.sig_w_az = py::float_(d["sig_w_az"]);
                 config.sig_w_gx = py::float_(d["sig_w_gx"]);
                 config.sig_w_gy = py::float_(d["sig_w_gy"]);
                 config.sig_w_gz = py::float_(d["sig_w_gz"]);
                 config.sig_a_d = py::float_(d["sig_a_d"]);
                 config.tau_a = py::float_(d["tau_a"]);
                 config.sig_g_d = py::float_(d["sig_g_d"]);
                 config.tau_g = py::float_(d["tau_g"]);
                 config.sig_gps_p_ne = py::float_(d["sig_gps_p_ne"]);
                 config.sig_gps_p_d = py::float_(d["sig_gps_p_d"]);
                 config.sig_gps_v_ne = py::float_(d["sig_gps_v_ne"]);
                 config.sig_gps_v_d = py::float_(d["sig_gps_v_d"]);
                 config.sig_mag = py::float_(d["sig_mag"]);
             }
         )
        ;
}

#endif
