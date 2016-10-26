#include <iostream>
using std::cout;
using std::endl;

#include "../nav_core/nav_functions.hxx"

#include "glocal.hxx"
#include "openloop.hxx"

static Vector3d quat_transform(Quaterniond q, Vector3d v) {
    Quaterniond tmp1(0.0, v(0), v(1), v(2));
    //Quaterniond tmp2 = body2ned * tmp1 * ned2body;
    Quaterniond tmp2 = q * tmp1 * q.inverse();
    return tmp2.vec();
}

OpenLoop::OpenLoop() {
    tprev = 0.0;
    set_gyro_calib(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    set_accel_calib(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

void OpenLoop::init(double lat_rad, double lon_rad, double alt_m,
		    double vn_ms, double ve_ms, double vd_ms,
		    double phi_rad, double the_rad, double psi_rad)
{
    set_pos(lat_rad, lon_rad, alt_m);
    set_vel(vn_ms, ve_ms, vd_ms);
    set_att(phi_rad, the_rad, psi_rad);
}

void OpenLoop::set_pos(double lat_rad, double lon_rad, double alt_m) {
    this->lat_rad = lat_rad;
    this->lon_rad = lon_rad;
    this->alt_m = alt_m;
    pos_lla = Vector3d(lat_rad, lon_rad, alt_m);
    pos_ecef = lla2ecef(pos_lla);
    glocal_ned = local_gravity( lat_rad, alt_m );
    ecef2ned = lla2quat(lon_rad, lat_rad);
    Vector3d pos_lla1 = ecef2lla(pos_ecef);
    //printf("lla: %.8f %.8f %.2f\n", pos_lla1(0)/D2R, pos_lla1(1)/D2R, pos_lla1(2));
}

void OpenLoop::set_vel(double vn_ms, double ve_ms, double vd_ms) {
    this->vn_ms = vn_ms;
    this->ve_ms = ve_ms;
    this->vd_ms = vd_ms;
    vel_ned = Vector3d(vn_ms, ve_ms, vd_ms);
    //printf("set vel_ned: %.2f %.2f %.2f\n", vel_ned(0), vel_ned(1), vel_ned(2));
}

void OpenLoop::set_att(double phi_rad, double the_rad, double psi_rad) {
    this->phi_rad = phi_rad;
    this->the_rad = the_rad;
    this->psi_rad = psi_rad;
    ned2body = eul2quat(phi_rad, the_rad, psi_rad);
}

void OpenLoop::set_gyro_calib(double gxb, double gyb, double gzb,
			      double gxs, double gys, double gzs)
{
    this->gxb = gxb;
    this->gyb = gyb;
    this->gzb = gzb;
    this->gxs = gxs;
    this->gys = gys;
    this->gzs = gzs;
}

void OpenLoop::set_accel_calib(double axb, double ayb, double azb,
			       double axs, double ays, double azs)
{
    this->axb = axb;
    this->ayb = ayb;
    this->azb = azb;
    this->axs = axs;
    this->ays = ays;
    this->azs = azs;
}


NAVdata OpenLoop::update(IMUdata imu /*, GPSdata gps*/) {
    // compute time-elapsed 'dt'
    double tnow = imu.time;
    double dt = tnow - tprev;
    if ( fabs(dt) > 0.1 ) {
	// sanity check
	dt = 0.0;
    }
    //printf("dt = %.4f tnow = %.4f  tprev = %.4f\n", dt, tnow, tprev);
    tprev = tnow;

    // update attitude from gyro
    double p = (imu.p - gxb) * (1 + gxs);
    double q = (imu.q - gyb) * (1 + gys);
    double r = (imu.r - gzb) * (1 + gzs);
    Quaterniond rot_body = eul2quat(p*dt, q*dt, r*dt);
    ned2body *= rot_body;
    ned2body.normalize();
    body2ned = ned2body.inverse();
    Vector3d att_vec = quat2eul(ned2body);
    nav.phi = att_vec(0);
    nav.the = att_vec(1);
    nav.psi = att_vec(2);

    // rotate accelerometer vector into ned frame
    double ax = (imu.ax - axb) * (1 + axs);
    double ay = (imu.ay - ayb) * (1 + ays);
    double az = (imu.az - azb) * (1 + azs);
    Vector3d accel_ned = quat_transform(ned2body, Vector3d(ax, ay, az));

    // add the local gravity vector.
    accel_ned += glocal_ned;
    //printf("accel_ned: %.2f %.2f %.2f\n", accel_ned(0), accel_ned(1), accel_ned(2));

    // update the velocity vector
    vel_ned += accel_ned*dt;
    //printf("vel_ned: %.2f %.2f %.2f\n", vel_ned(0), vel_ned(1), vel_ned(2));
    nav.vn = vel_ned(0);
    nav.ve = vel_ned(1);
    nav.vd = vel_ned(2);
    // transform to ecef frame
    vel_ecef = quat_transform(ecef2ned, vel_ned);
    //printf("vel_ecef: %.2f %.2f %.2f\n", vel_ecef(0), vel_ecef(1), vel_ecef(2));
    
    // update the position
    pos_ecef += vel_ecef*dt;
    //printf("ecef: %.2f %.2f %.2f\n", pos_ecef(0), pos_ecef(1), pos_ecef(2));
    pos_lla = ecef2lla(pos_ecef);
    //printf("lla: %.8f %.8f %.2f\n", pos_lla(0)/D2R, pos_lla(1)/D2R, pos_lla(2));
    nav.lat = pos_lla(0);
    nav.lon = pos_lla(1);
    nav.alt = pos_lla(2);
    
    // update ecef2ned transform with just updated position
    ecef2ned = lla2quat(lon_rad, lat_rad);
    
    nav.time = imu.time;

    return nav;
}


// The following constructs a python interface for this class.

#include <boost/python.hpp>
using namespace boost::python;

BOOST_PYTHON_MODULE(libnav_openloop)
{
    class_<OpenLoop>("OpenLoop")
        .def("init", &OpenLoop::init)
        .def("set_pos", &OpenLoop::set_pos)
        .def("set_vel", &OpenLoop::set_vel)
        .def("set_att", &OpenLoop::set_att)
        .def("set_gyro_calib", &OpenLoop::set_gyro_calib)
        .def("set_accel_calib", &OpenLoop::set_accel_calib)
        .def("update", &OpenLoop::update)
    ;
}

