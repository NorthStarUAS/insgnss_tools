// The following constructs a python interface for this class and
// associated structures.

#include "nav_structs.h"

#ifdef HAVE_PYBIND11

#include <pybind11/pybind11.h>
namespace py = pybind11;

PYBIND11_MODULE(nav_structs, m) {
    py::class_<IMUdata>(m, "IMUdata")
        .def(py::init<>())
        .def_readwrite("time_sec", &IMUdata::time_sec)
        .def_readwrite("p_rps", &IMUdata::p_rps)
        .def_readwrite("q_rps", &IMUdata::q_rps)
        .def_readwrite("r_rps", &IMUdata::r_rps)
        .def_readwrite("ax_mps2", &IMUdata::ax_mps2)
        .def_readwrite("ay_mps2", &IMUdata::ay_mps2)
        .def_readwrite("az_mps2", &IMUdata::az_mps2)
        .def_readwrite("hx", &IMUdata::hx)
        .def_readwrite("hy", &IMUdata::hy)
        .def_readwrite("hz", &IMUdata::hz)
        .def_readwrite("temp_C", &IMUdata::temp_C)
        .def("as_dict",
             [](const IMUdata &imu) {
                 py::dict result;
                 result["time_sec"] = imu.time_sec;
                 result["p_rps"] = imu.p_rps;
                 result["q_rps"] = imu.q_rps;
                 result["r_rps"] = imu.r_rps;
                 result["ax_mps2"] = imu.ax_mps2;
                 result["ay_mps2"] = imu.ay_mps2;
                 result["az_mps2"] = imu.az_mps2;
                 result["hx"] = imu.hx;
                 result["hy"] = imu.hy;
                 result["hz"] = imu.hz;
                 result["temp_C"] = imu.temp_C;
                 return result;
             }
         )
        .def("from_dict",
             [](IMUdata &imu, const py::dict &d) {
                 imu.time_sec = py::float_(d["time_sec"]);
                 imu.p_rps = py::float_(d["p_rps"]);
                 imu.q_rps = py::float_(d["q_rps"]);
                 imu.r_rps = py::float_(d["r_rps"]);
                 imu.ax_mps2 = py::float_(d["ax_mps2"]);
                 imu.ay_mps2 = py::float_(d["ay_mps2"]);
                 imu.az_mps2 = py::float_(d["az_mps2"]);
                 imu.hx = py::float_(d["hx"]);
                 imu.hy = py::float_(d["hy"]);
                 imu.hz = py::float_(d["hz"]);
                 imu.temp_C = py::float_(d["temp_C"]);
             }
         )
        ;

    py::class_<GPSdata>(m, "GPSdata")
        .def(py::init<>())
        .def_readwrite("time_sec", &GPSdata::time_sec)
        .def_readwrite("unix_sec", &GPSdata::unix_sec)
        .def_readwrite("latitude_deg", &GPSdata::latitude_deg)
        .def_readwrite("longitude_deg", &GPSdata::longitude_deg)
        .def_readwrite("altitude_m", &GPSdata::altitude_m)
        .def_readwrite("vn_mps", &GPSdata::vn_mps)
        .def_readwrite("ve_mps", &GPSdata::ve_mps)
        .def_readwrite("vd_mps", &GPSdata::vd_mps)
        .def_readwrite("sats", &GPSdata::sats)
        .def("as_dict",
             [](const GPSdata &gps) {
                 py::dict result;
                 result["time_sec"] = gps.time_sec;
                 result["unix_sec"] = gps.unix_sec;
                 result["latitude_deg"] = gps.latitude_deg;
                 result["longitude_deg"] = gps.longitude_deg;
                 result["altitude_m"] = gps.altitude_m;
                 result["vn_mps"] = gps.vn_mps;
                 result["ve_mps"] = gps.ve_mps;
                 result["vd_mps"] = gps.vd_mps;
                 result["num_sats"] = gps.sats;
                 return result;
             }
         )
        .def("from_dict",
             [](GPSdata &gps, const py::dict &d) {
                 gps.time_sec = py::float_(d["time_sec"]);
                 gps.unix_sec = py::float_(d["unix_sec"]);
                 gps.latitude_deg = py::float_(d["latitude_deg"]);
                 gps.longitude_deg = py::float_(d["longitude_deg"]);
                 gps.altitude_m = py::float_(d["altitude_m"]);
                 gps.vn_mps = py::float_(d["vn_mps"]);
                 gps.ve_mps = py::float_(d["ve_mps"]);
                 gps.vd_mps = py::float_(d["vd_mps"]);
                 gps.sats = py::int_(d["num_sats"]);
              }
         )
        ;

    py::class_<Airdata>(m, "Airdata")
        .def(py::init<>())
        .def_readwrite("time_sec", &Airdata::time_sec)
        .def_readwrite("static_press", &Airdata::static_press)
        .def_readwrite("diff_press", &Airdata::diff_press)
        .def_readwrite("temp", &Airdata::temp)
        .def_readwrite("airspeed", &Airdata::airspeed)
        .def_readwrite("altitude", &Airdata::altitude)
        .def("as_dict",
             [](const Airdata &airdata) {
                 py::dict result;
                 result["time_sec"] = airdata.time_sec;
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
        .def_readwrite("time_sec", &NAVdata::time_sec)
        .def_readwrite("latitude_deg", &NAVdata::latitude_deg)
        .def_readwrite("longitude_deg", &NAVdata::longitude_deg)
        .def_readwrite("altitude_m", &NAVdata::altitude_m)
        .def_readwrite("vn_mps", &NAVdata::vn_mps)
        .def_readwrite("ve_mps", &NAVdata::ve_mps)
        .def_readwrite("vd_mps", &NAVdata::vd_mps)
        .def_readwrite("phi_deg", &NAVdata::phi_deg)
        .def_readwrite("theta_deg", &NAVdata::theta_deg)
        .def_readwrite("psi_deg", &NAVdata::psi_deg)
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
                 result["time_sec"] = nav.time_sec;
                 result["latitude_deg"] = nav.latitude_deg;
                 result["longitude_deg"] = nav.longitude_deg;
                 result["altitude_m"] = nav.altitude_m;
                 result["vn_mps"] = nav.vn_mps;
                 result["ve_mps"] = nav.ve_mps;
                 result["vd_mps"] = nav.vd_mps;
                 result["phi_deg"] = nav.phi_deg;
                 result["theta_deg"] = nav.theta_deg;
                 result["psi_deg"] = nav.psi_deg;
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
