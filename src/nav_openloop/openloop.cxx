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

void OpenLoop::init(double lat_rad, double lon_rad, double alt_m,
		    double vn_ms, double ve_ms, double vd_ms,
		    double phi_rad, double the_rad, double psi_rad)
{
    set_gyro_calib(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
    set_accel_calib(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
    
    this->lat_rad = lat_rad;
    this->lon_rad = lon_rad;
    this->alt_m = alt_m;
    this->vn_ms = vn_ms;
    this->ve_ms = ve_ms;
    this->vd_ms = vd_ms;
    this->phi_rad = phi_rad;
    this->the_rad = the_rad;
    this->psi_rad = psi_rad;

    printf("%.8f %.8f\n", lat_rad, lon_rad);
    pos_lla = Vector3d(lat_rad, lon_rad, alt_m);
    pos_ecef = lla2ecef(pos_lla);
    vel_ned = Vector3d(vn_ms, ve_ms, vd_ms);
    glocal_ned = local_gravity( lat_rad, alt_m );
    ned2body = eul2quat(phi_rad, the_rad, psi_rad);
    ecef2ned = lla2quat(lon_rad, lat_rad);
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
    tprev = tnow;

    // update attitude from gyro
    double p = (imu.p - gxb) * gxs;
    double q = (imu.q - gyb) * gys;
    double r = (imu.r - gzb) * gzs;
    Quaterniond rot_body = eul2quat(p*dt, q*dt, r*dt);
    ned2body *= rot_body;
    ned2body.normalize();
    body2ned = ned2body.inverse();
    Vector3d att_vec = quat2eul(ned2body);
    nav.phi = att_vec(0);
    nav.the = att_vec(1);
    nav.psi = att_vec(2);

    // rotate accelerometer vector into ned frame
    double ax = (imu.ax - axb) * axs;
    double ay = (imu.ay - ayb) * ays;
    double az = (imu.az - azb) * azs;
    Vector3d accel_ned = quat_transform(ned2body, Vector3d(ax, ay, az));

    // add the local gravity vector.
    accel_ned += glocal_ned;

    // update the velocity vector
    vel_ned += accel_ned*dt;
    nav.vn = vel_ned(0);
    nav.ve = vel_ned(1);
    nav.vd = vel_ned(2);
    // transform to ecef frame
    //vel_ecef = quat_transform(ecef2ned.inverse(), vel_ned);
    vel_ecef = quat_transform(ecef2ned, vel_ned);

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
