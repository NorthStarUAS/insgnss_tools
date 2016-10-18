/*! \file EKF_15state.c
 *	\brief 15 state EKF navigation filter
 *
 *	\details  15 state EKF navigation filter using loosely integrated INS/GPS architecture.
 * 	Time update is done after every IMU data acquisition and GPS measurement
 * 	update is done every time the new data flag in the GPS data packet is set. Designed by Adhika Lie.
 *	Attitude is parameterized using quaternions.
 *	Estimates IMU bias errors.
 *	\ingroup nav_fcns
 *
 * \author University of Minnesota
 * \author Aerospace Engineering and Mechanics
 * \copyright Copyright 2011 Regents of the University of Minnesota. All rights reserved.
 *
 */

#ifndef NAV_15STATE_HXX
#define NAV_15STATE_HXX


#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/LU>
using namespace Eigen;

#include "nav_structs.hxx"

// usefule constants
const double g = 9.814;
const double D2R = M_PI / 180.0;

// define some types for notational convenience and consistency
typedef Matrix<double,6,6> Matrix6d;
typedef Matrix<double,12,12> Matrix12d;
typedef Matrix<double,15,15> Matrix15d;
typedef Matrix<double,6,15> Matrix6x15d;
typedef Matrix<double,15,6> Matrix15x6d;
typedef Matrix<double,15,12> Matrix15x12d;
typedef Matrix<double,6,1> Vector6d;
typedef Matrix<double,15,1> Vector15d;

class EKF15 {

public:

    EKF15() {
	default_config();
    }
    ~EKF15() {}

    // set/get error characteristics of navigation sensors
    void set_config(NAVconfig config);
    NAVconfig get_config();
    void default_config();

    // main interface
    NAVdata init(IMUdata imu, GPSdata gps);
    NAVdata update(IMUdata imu, GPSdata gps);
    
private:

    Matrix15d F, PHI, P, Qw, Q, ImKH, KRKt, I15 /* identity */;
    Matrix15x12d G;
    Matrix15x6d K;
    Vector15d x;
    Matrix12d Rw;
    Matrix6x15d H;
    Matrix6d R;
    Vector6d y;
    Matrix3d C_N2B, C_B2N, I3 /* identity */, temp33;
    Vector3d grav, f_b, om_ib, nr, pos_ins_ecef, pos_ins_ned, pos_gps, pos_gps_ecef, pos_gps_ned, dx, mag_ned;

    Quaterniond quat;
    double denom, Re, Rn;
    double tprev;

    NAVconfig config;
    NAVdata nav;
};


// The following constructs a python interface for this class and
// associated structures.

#include <boost/python.hpp>
using namespace boost::python;

BOOST_PYTHON_MODULE(libnav_eigen)
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
    ;
    
    class_<GPSdata>("GPSdata")
	.def_readwrite("time", &GPSdata::time)
	.def_readwrite("lat", &GPSdata::lat)
	.def_readwrite("lon", &GPSdata::lon)
	.def_readwrite("alt", &GPSdata::alt)
	.def_readwrite("vn", &GPSdata::vn)
	.def_readwrite("ve", &GPSdata::ve)
	.def_readwrite("vd", &GPSdata::vd)
	.def_readwrite("newData", &GPSdata::newData)
    ;

    class_<NAVdata>("NAVdata")
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
	.def_readonly("Pgbz", &NAVdata::Pgbz)
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
	
    class_<EKF15>("EKF15")
        .def("set_config", &EKF15::set_config)
        .def("init", &EKF15::init)
        .def("update", &EKF15::update)
    ;
}

#endif // NAV_15STATE_HXX
