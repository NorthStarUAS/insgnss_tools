#include <iostream>
using std::cout;
using std::endl;

#include "../nav_core/nav_functions.hxx"

#include "glocal.hxx"
#include "openloop.hxx"

static Vector3d quat_transform1(Quaterniond q, Vector3d v) {
    Quaterniond tmp1(0.0, v(0), v(1), v(2));
    //Quaterniond tmp2 = body2ned * tmp1 * ned2body;
    Quaterniond tmp2 = q * tmp1 * q.inverse();
    return tmp2.vec();
}
static Vector3d quat_transform(Quaterniond q, Vector3d v) {
    //Quaterniond tmp1(0.0, v(0), v(1), v(2));
    //Quaterniond tmp2 = q * tmp1 * q.inverse();
    //return tmp2.vec();
    
    double r = 2/q.dot(q);
    Vector3d qimag = q.vec();
    double qr = q.w();
    return (r*qr*qr - 1)*v + (r*qimag.dot(v))*qimag - (r*qr)*qimag.cross(v);
}

OpenLoop::OpenLoop() {
    tprev = 0.0;
    set_gyro_calib(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    set_accel_calib(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    G.setZero();
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
    
    Vector3d t1 = quat_transform(ecef2ned, pos_ecef);
    // printf("up? %.2f %.2f %.2f\n", t1(0), t1(1), t1(2));
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

void OpenLoop::set_G(double x11, double x12, double x13,
		     double x21, double x22, double x23,
		     double x31, double x32, double x33)
{
    G << x11, x12, x13, x21, x22, x23, x31, x32, x33;
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

    // modeling the force/accel effects on the gyros seems
    // unproductive, so zero out Fb() for now.
    // Vector3d Fb(0, 0, 0);
    Vector3d Fb = G * Vector3d(imu.ax, imu.ay, imu.az);
    
    // update attitude from gyro
    double p = (1 + gxs) * imu.p - gxb - Fb(0);
    double q = (1 + gys) * imu.q - gyb - Fb(1);
    double r = (1 + gzs) * imu.r - gzb - Fb(2);
    Quaterniond rot_body = eul2quat(p*dt, q*dt, r*dt);
    ned2body *= rot_body;
    ned2body.normalize();
    body2ned = ned2body.inverse();
    body2ned.normalize();
    Vector3d att_vec = quat2eul(ned2body);
    nav.phi = att_vec(0);
    nav.the = att_vec(1);
    nav.psi = att_vec(2);

    // rotate accelerometer vector into ned frame
    double ax = (1 + axs) * imu.ax - axb;
    double ay = (1 + ays) * imu.ay - ayb;
    double az = (1 + azs) * imu.az - azb;
    Vector3d accel_ned = quat_transform(body2ned, Vector3d(ax, ay, az));

    // add the local gravity vector.
    accel_ned += glocal_ned;

    // update the velocity vector
    vel_ned += accel_ned*dt;
    nav.vn = vel_ned(0);
    nav.ve = vel_ned(1);
    nav.vd = vel_ned(2);
    // transform to ecef frame
    vel_ecef = quat_transform(ecef2ned.inverse(), vel_ned);
    
    // update the position
    pos_ecef += vel_ecef*dt;
    //printf("ecef: %.2f %.2f %.2f\n", pos_ecef(0), pos_ecef(1), pos_ecef(2));
    pos_lla = ecef2lla(pos_ecef);
    //printf("lla: %.8f %.8f %.2f\n", pos_lla(0)/D2R, pos_lla(1)/D2R, pos_lla(2));
    nav.lat = pos_lla(0);
    nav.lon = pos_lla(1);
    nav.alt = pos_lla(2);

    // populate the bias fields
    nav.gbx = gxb / (1 + gxs);
    nav.gby = gyb / (1 + gys);
    nav.gbz = gzb / (1 + gzs);
    nav.abx = axb / (1 + axs);
    nav.aby = ayb / (1 + ays);
    nav.abz = azb / (1 + azs);
	
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
        .def("set_G", &OpenLoop::set_G)
        .def("set_accel_calib", &OpenLoop::set_accel_calib)
        .def("update", &OpenLoop::update)
    ;
}

