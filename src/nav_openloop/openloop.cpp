#include <iostream>
using std::cout;
using std::endl;

#include "../nav_common/nav_functions.h"

#include "glocal.h"
#include "openloop.h"

static Vector3f quat_transform1(Quaternionf q, Vector3f v) {
    Quaternionf tmp1(0.0, v(0), v(1), v(2));
    //Quaterniond tmp2 = body2ned * tmp1 * ned2body;
    Quaternionf tmp2 = q * tmp1 * q.inverse();
    return tmp2.vec();
}

static Vector3d quat_transformd(Quaterniond q, Vector3d v) {
    double r = 2/q.dot(q);
    Vector3d qimag = q.vec();
    double qr = q.w();
    return (r*qr*qr - 1)*v + (r*qimag.dot(v))*qimag - (r*qr)*qimag.cross(v);
}

static Vector3f quat_transformf(Quaternionf q, Vector3f v) {
    float r = 2/q.dot(q);
    Vector3f qimag = q.vec();
    float qr = q.w();
    return (r*qr*qr - 1)*v + (r*qimag.dot(v))*qimag - (r*qr)*qimag.cross(v);
}

OpenLoop::OpenLoop() {
    //tstart = -1.0;
    tprev = 0.0;
    //gxs = gys = gzs = axs = ays = azs = 0.0;
    //gxd = gyd = gzd = axd = ayd = azd = 0.0;
    set_gyro_calib(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    set_accel_calib(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    // G.setZero();
}

void OpenLoop::init(double lat_rad, double lon_rad, float alt_m,
		    float vn_ms, float ve_ms, float vd_ms,
		    float phi_rad, float the_rad, float psi_rad)
{
    set_pos(lat_rad, lon_rad, alt_m);
    set_vel(vn_ms, ve_ms, vd_ms);
    set_att(phi_rad, the_rad, psi_rad);
}

void OpenLoop::init_by_nav(NAVdata nav)
{
    this->nav = nav;
    set_pos(nav.lat, nav.lon, nav.alt);
    set_vel(nav.vn, nav.ve, nav.vd);
    set_att(nav.phi, nav.the, nav.psi);
    set_gyro_calib(nav.gbx, nav.gby, nav.gbz, 0.0, 0.0, 0.0);
    set_accel_calib(nav.abx, nav.aby, nav.abz, 0.0, 0.0, 0.0);
    tprev = nav.time;
}

void OpenLoop::set_pos(double lat_rad, double lon_rad, float alt_m) {
    this->lat_rad = lat_rad;
    this->lon_rad = lon_rad;
    this->alt_m = alt_m;
    pos_lla = Vector3d(lat_rad, lon_rad, alt_m);
    pos_ecef = lla2ecef(pos_lla);
    glocal_ned = local_gravity( lat_rad, alt_m );
    ecef2ned = lla2quat(lon_rad, lat_rad).cast<float>();
    // Vector3d pos_lla1 = ecef2lla(pos_ecef);
    //printf("lla: %.8f %.8f %.2f\n", pos_lla1(0)/D2R, pos_lla1(1)/D2R, pos_lla1(2));
    
    // Vector3d t1 = quat_transformd(ecef2ned, pos_ecef);
    // printf("up? %.2f %.2f %.2f\n", t1(0), t1(1), t1(2));
}

void OpenLoop::set_vel(float vn_ms, float ve_ms, float vd_ms) {
    this->vn_ms = vn_ms;
    this->ve_ms = ve_ms;
    this->vd_ms = vd_ms;
    vel_ned = Vector3f(vn_ms, ve_ms, vd_ms);
    //printf("set vel_ned: %.2f %.2f %.2f\n", vel_ned(0), vel_ned(1), vel_ned(2));
}

void OpenLoop::set_att(float phi_rad, float the_rad, float psi_rad) {
    this->phi_rad = phi_rad;
    this->the_rad = the_rad;
    this->psi_rad = psi_rad;
    ned2body = eul2quat(phi_rad, the_rad, psi_rad);
}

void OpenLoop::set_gyro_calib(float gxb, float gyb, float gzb,
			      float gxd, float gyd, float gzd)
{
    this->gxb = gxb;
    this->gyb = gyb;
    this->gzb = gzb;
    // this->gxs = gxs;
    // this->gys = gys;
    // this->gzs = gzs;
    // this->gxd = gxd;
    // this->gyd = gyd;
    // this->gzd = gzd;
}

void OpenLoop::set_accel_calib(float axb, float ayb, float azb,
			       float axd, float ayd, float azd)
{
    this->axb = axb;
    this->ayb = ayb;
    this->azb = azb;
    // this->axs = axs;
    // this->ays = ays;
    // this->azs = azs;
    // this->axd = axd;
    // this->ayd = ayd;
    // this->azd = azd;
}

// void OpenLoop::set_G(float x11, float x12, float x13,
// 		     float x21, float x22, float x23,
// 		     float x31, float x32, float x33)
// {
//     G << x11, x12, x13, x21, x22, x23, x31, x32, x33;
// }


NAVdata OpenLoop::update(IMUdata imu /*, GPSdata gps*/) {
    // compute time-elapsed 'dt'
    float tnow = imu.time;
    // if ( tstart < 0.0 ) {
    //	tstart = tnow;
    // }
    float dt = tnow - tprev;
    if ( fabs(dt) > 0.1 ) {
	// sanity check
	dt = 0.0;
    }
    //printf("dt = %.4f tnow = %.4f  tprev = %.4f\n", dt, tnow, tprev);
    tprev = tnow;
    // float elapsed = tnow - tstart;

    // modeling the force/accel effects on the gyros seems
    // unproductive, so zero out Fb() for now.
    // Vector3d Fb(0, 0, 0);
    // Vector3d Fb = G * Vector3d(imu.ax, imu.ay, imu.az);
    
    // update attitude from gyro
    float p = imu.p - gxb;
    float q = imu.q - gyb;
    float r = imu.r - gzb;
    //float p = (1 + gxs) * imu.p - (gxb + elapsed*gxd) - Fb(0);
    //float q = (1 + gys) * imu.q - (gyb + elapsed*gyd) - Fb(1);
    //float r = (1 + gzs) * imu.r - (gzb + elapsed*gzd) - Fb(2);
    Quaternionf rot_body = eul2quat(p*dt, q*dt, r*dt);
    ned2body *= rot_body;
    ned2body.normalize();
    body2ned = ned2body.inverse();
    body2ned.normalize();
    Vector3f att_vec = quat2eul(ned2body);
    nav.phi = att_vec(0);
    nav.the = att_vec(1);
    nav.psi = att_vec(2);

    // rotate accelerometer vector into ned frame
    float ax = imu.ax - axb;
    float ay = imu.ay - ayb;
    float az = imu.az - azb;
    //float ax = (1 + axs) * imu.ax - (axb + elapsed*axd);
    //float ay = (1 + ays) * imu.ay - (ayb + elapsed*ayd);
    //float az = (1 + azs) * imu.az - (azb + elapsed*azd);
    Vector3f accel_ned = quat_transformf(body2ned, Vector3f(ax, ay, az));

    // add the local gravity vector.
    accel_ned += glocal_ned;

    // update the velocity vector
    vel_ned += accel_ned*dt;
    nav.vn = vel_ned(0);
    nav.ve = vel_ned(1);
    nav.vd = vel_ned(2);
    // transform to ecef frame
    vel_ecef = quat_transformf(ecef2ned.inverse(), vel_ned);
    
    // update the position
    pos_ecef += vel_ecef.cast<double>() * dt;
    //printf("ecef: %.2f %.2f %.2f\n", pos_ecef(0), pos_ecef(1), pos_ecef(2));
    pos_lla = ecef2lla(pos_ecef);
    //printf("lla: %.8f %.8f %.2f\n", pos_lla(0), pos_lla(1), pos_lla(2));
    nav.lat = pos_lla(0);
    nav.lon = pos_lla(1);
    nav.alt = pos_lla(2);

    // populate the bias fields
    // nav.gbx = gxb + elapsed*gxd;
    // nav.gby = gyb + elapsed*gyd;
    // nav.gbz = gzb + elapsed*gzd;
    // nav.abx = axb + elapsed*axd;
    // nav.aby = ayb + elapsed*ayd;
    // nav.abz = azb + elapsed*azd;
	
    // update ecef2ned transform with just updated position
    ecef2ned = lla2quat(nav.lon, nav.lat).cast<float>();
    
    nav.time = imu.time;

    return nav;
}
