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

#include "nav_constants.h"
#include "nav_functions.h"
#include "ekf15.h"

const float P_P_INIT = 10.0;
const float P_V_INIT = 1.0;
const float P_A_INIT = 0.34906;   // 20 deg
const float P_HDG_INIT = 3.14159; // 180 deg
const float P_AB_INIT = 0.9810;   // 0.5*g
const float P_GB_INIT = 0.01745;  // 5 deg/s

const double Rew = 6.359058719353925e+006; // earth radius
const double Rns = 6.386034030458164e+006; // earth radius

// BRT: (1) I think there are some several identity and sparse
// matrices, so probably some optimization still left there.  (2)
// Seems like a lot of the transforms could be more efficiently done
// with just a matrix or vector multiply.  (3) Probably could do a lot
// of block operations with F, i.e. F.block(j,k) = C_B2N, etc.  (4) A
// lot of these multi line equations with temp matrices can be
// compressed.

void EKF15::set_config(NAVconfig _config) {
    config = _config;
}

NAVconfig EKF15::get_config() {
    return config;
}

void EKF15::default_config()
{
    config.sig_w_ax = 0.05;     // Std dev of Accelerometer Wide Band Noise (m/s^2)
    config.sig_w_ay = 0.05;
    config.sig_w_az = 0.05;
    config.sig_w_gx = 0.00175;  // Std dev of gyro output noise (rad/s)  (0.1 deg/s)
    config.sig_w_gy = 0.00175;
    config.sig_w_gz = 0.00175;
    config.sig_a_d  = 0.01;     // Std dev of Accelerometer Markov Bias
    config.tau_a    = 100.0;    // Correlation time or time constant of b_{ad}
    config.sig_g_d  = 0.00025;  // Std dev of correlated gyro bias (rad)
    config.tau_g    = 50.0;     // Correlation time or time constant of b_{gd}
    config.sig_gps_p_ne = 3.0;  // GPS measurement noise std dev (m)
    config.sig_gps_p_d  = 6.0;  // GPS measurement noise std dev (m)
    config.sig_gps_v_ne = 0.5;  // GPS measurement noise std dev (m/s)
    config.sig_gps_v_d  = 1.0;  // GPS measurement noise std dev (m/s)
    config.sig_mag      = 0.3;  // Magnetometer measurement noise std dev (normalized -1 to 1)
}

void EKF15::init(IMUdata imu, GPSdata gps) {
    F.resize(15,15);
    PHI.resize(15,15);
    P.resize(15,15);
    Qw.resize(15,15);
    Q.resize(15,15);
    ImKH.resize(15,15);
    KRKt.resize(15,15);
    I15.resize(15,15);

    G.resize(15,12);
    K.resize(15,6);

    Rw.resize(12,12);

    H.resize(6,15);

    R.resize(6,6);

    I15.setIdentity();
    I3.setIdentity();

    // Assemble the matrices
    // .... gravity, g
    grav = Vector3f(0.0, 0.0, g);

    // ... H
    H.setZero();
    H.topLeftCorner(6,6).setIdentity();

    // first order correlation + white noise, tau = time constant for correlation
    // gain on white noise plus gain on correlation
    // Rw small - trust time update, Rw more - lean on measurement update
    // split between accels and gyros and / or noise and correlation
    // ... Rw
    Rw.setZero();
    Rw(0,0) = config.sig_w_ax*config.sig_w_ax;	Rw(1,1) = config.sig_w_ay*config.sig_w_ay;	      Rw(2,2) = config.sig_w_az*config.sig_w_az; //1 sigma on noise
    Rw(3,3) = config.sig_w_gx*config.sig_w_gx;	Rw(4,4) = config.sig_w_gy*config.sig_w_gy;	      Rw(5,5) = config.sig_w_gz*config.sig_w_gz;
    Rw(6,6) = 2*config.sig_a_d*config.sig_a_d/config.tau_a;	Rw(7,7) = 2*config.sig_a_d*config.sig_a_d/config.tau_a;    Rw(8,8) = 2*config.sig_a_d*config.sig_a_d/config.tau_a;
    Rw(9,9) = 2*config.sig_g_d*config.sig_g_d/config.tau_g;	Rw(10,10) = 2*config.sig_g_d*config.sig_g_d/config.tau_g;  Rw(11,11) = 2*config.sig_g_d*config.sig_g_d/config.tau_g;

    // ... P (initial)
    P.setZero();
    P(0,0) = P_P_INIT*P_P_INIT; 	P(1,1) = P_P_INIT*P_P_INIT; 	      P(2,2) = P_P_INIT*P_P_INIT;
    P(3,3) = P_V_INIT*P_V_INIT; 	P(4,4) = P_V_INIT*P_V_INIT; 	      P(5,5) = P_V_INIT*P_V_INIT;
    P(6,6) = P_A_INIT*P_A_INIT; 	P(7,7) = P_A_INIT*P_A_INIT; 	      P(8,8) = P_HDG_INIT*P_HDG_INIT;
    P(9,9) = P_AB_INIT*P_AB_INIT; 	P(10,10) = P_AB_INIT*P_AB_INIT;       P(11,11) = P_AB_INIT*P_AB_INIT;
    P(12,12) = P_GB_INIT*P_GB_INIT; 	P(13,13) = P_GB_INIT*P_GB_INIT;       P(14,14) = P_GB_INIT*P_GB_INIT;

    // ... R
    R.setZero();
    R(0,0) = config.sig_gps_p_ne*config.sig_gps_p_ne;	 R(1,1) = config.sig_gps_p_ne*config.sig_gps_p_ne;  R(2,2) = config.sig_gps_p_d*config.sig_gps_p_d;
    R(3,3) = config.sig_gps_v_ne*config.sig_gps_v_ne;	 R(4,4) = config.sig_gps_v_ne*config.sig_gps_v_ne;  R(5,5) = config.sig_gps_v_d*config.sig_gps_v_d;

    // ... update P in get_nav
    nav.Pp0 = P(0,0);	  nav.Pp1 = P(1,1);	nav.Pp2 = P(2,2);
    nav.Pv0 = P(3,3);	  nav.Pv1 = P(4,4);	nav.Pv2 = P(5,5);
    nav.Pa0 = P(6,6);	  nav.Pa1 = P(7,7);	nav.Pa2 = P(8,8);

    nav.Pabx = P(9,9);	  nav.Paby = P(10,10);	nav.Pabz = P(11,11);
    nav.Pgbx = P(12,12);  nav.Pgby = P(13,13);  nav.Pgbz = P(14,14);

    // .. then initialize states with GPS Data
    nav.latitude_deg = gps.latitude_deg;
    nav.longitude_deg = gps.longitude_deg;
    nav.altitude_m = gps.altitude_m;

    nav.vn_mps = gps.vn_mps;
    nav.ve_mps = gps.ve_mps;
    nav.vd_mps = gps.vd_mps;

    // ... and initialize states with IMU Data, theta from Ax, aircraft
    // at rest
    float theta_rad = asin(imu.ax_mps2/g);
    nav.theta_deg = theta_rad * R2D;
    // phi from Ay, aircraft at rest
    float phi_rad = asin(imu.ay_mps2/(g*cos(theta_rad)));
    nav.phi_deg = phi_rad * R2D;

    // this is atan2(x, -y) because the aircraft body X,Y axis are
    // swapped with the cartesion axes from the top down perspective
    // nav.psi_rad = 90*D2R - atan2(imu.hx, -imu.hy);
    // printf("ekf: hx: %.2f hy: %.2f psi: %.2f\n", imu.hx, imu.hy, nav.psi_rad*R2D);
    // printf("atan2: %.2f\n", atan2(imu.hx, -imu.hy)*R2D);

    // tilt compensated heading
    float psi_rad = atan2(imu.hz*sin(phi_rad)-imu.hy*cos(phi_rad),imu.hx*cos(theta_rad)+imu.hy*sin(theta_rad)*sin(phi_rad)+imu.hz*sin(theta_rad)*cos(phi_rad));
    nav.psi_deg = psi_rad * R2D;
    // printf("tilt compensated psi: %.2f\n", nav.psi*R2D);

    quat = eul2quat(phi_rad, theta_rad, psi_rad);

    nav.abx = 0.0;
    nav.aby = 0.0;
    nav.abz = 0.0;

    // Assume IMU has made no attempt to zero its biases
    // nav.gbx = imu.p_rps;
    // nav.gby = imu.q_rps;
    // nav.gbz = imu.r_rps;

    // Assume IMU has already zero'd its biases
    nav.gbx = 0;
    nav.gby = 0;
    nav.gbz = 0;

    imu_last = imu;

    nav.time_sec = imu.time_sec;
    nav.err_type = data_valid;
}

// Main get_nav filter function
void EKF15::time_update(IMUdata imu) {
    // compute time-elapsed 'dt'
    // This computes the navigation state at the DAQ's Time Stamp
    float imu_dt = imu.time_sec - imu_last.time_sec;
    if ( imu_dt < 0.0 ) { imu_dt = 0.0; }
    if ( imu_dt > 0.1 ) { imu_dt = 0.1; }
    nav.time_sec = imu.time_sec;

    // ==================  Time Update  ===================

    // Attitude Update
    // ... Calculate Navigation Rate
    Vector3f vel_vec(nav.vn_mps, nav.ve_mps, nav.vd_mps);
    Vector3d pos_ref(nav.latitude_deg*D2R, nav.longitude_deg*D2R, nav.altitude_m);

    if ( false ) {
        // Get the new Specific forces and Rotation Rate from previous
        // frame (k) to use in this frame (k+1).  Rectangular
        // integration.
        f_b(0) = imu_last.ax_mps2 - nav.abx;
        f_b(1) = imu_last.ay_mps2 - nav.aby;
        f_b(2) = imu_last.az_mps2 - nav.abz;

        om_ib(0) = imu_last.p_rps - nav.gbx;
        om_ib(1) = imu_last.q_rps - nav.gby;
        om_ib(2) = imu_last.r_rps - nav.gbz;
    } else if ( false ) {
        // Combine the Specific forces and Rotation Rate from previous
        // frame (k) with current frame (k+1) to use in this frame
        // (k+1).  Trapazoidal integration.
        f_b(0) = 0.5 * (imu_last.ax_mps2 + imu.ax_mps2) - nav.abx;
        f_b(1) = 0.5 * (imu_last.ay_mps2 + imu.ay_mps2) - nav.aby;
        f_b(2) = 0.5 * (imu_last.az_mps2 + imu.az_mps2) - nav.abz;

        om_ib(0) = 0.5 * (imu_last.p_rps + imu.p_rps) - nav.gbx;
        om_ib(1) = 0.5 * (imu_last.q_rps + imu.q_rps) - nav.gby;
        om_ib(2) = 0.5 * (imu_last.r_rps + imu.r_rps) - nav.gbz;
    } else {
        // Chris says the first two ways are BS
        f_b(0) = imu.ax_mps2 - nav.abx;
        f_b(1) = imu.ay_mps2 - nav.aby;
        f_b(2) = imu.az_mps2 - nav.abz;

        om_ib(0) = imu.p_rps - nav.gbx;
        om_ib(1) = imu.q_rps - nav.gby;
        om_ib(2) = imu.r_rps - nav.gbz;
    }

    imu_last = imu;

    Quaternionf dq = Quaternionf(1.0, 0.5*om_ib(0)*imu_dt, 0.5*om_ib(1)*imu_dt, 0.5*om_ib(2)*imu_dt);
    quat = (quat * dq).normalized();

    if (quat.w() < 0) {
        // Avoid quaternion flips sign
        quat = Quaternionf(-quat.w(), -quat.x(), -quat.y(), -quat.z());
    }

    Vector3f att_vec = quat2eul(quat);
    nav.phi_deg = att_vec(0)*R2D;
    nav.theta_deg = att_vec(1)*R2D;
    nav.psi_deg = att_vec(2)*R2D;

    // AHRS Transformations
    C_N2B = quat2dcm(quat);
    C_B2N = C_N2B.transpose();

    // Velocity Update
    dx = C_B2N * f_b;
    dx += grav;

    nav.vn_mps += imu_dt*dx(0);
    nav.ve_mps += imu_dt*dx(1);
    nav.vd_mps += imu_dt*dx(2);

    // Position Update
    dx = llarate(vel_vec, pos_ref);
    nav.latitude_deg += imu_dt*dx(0)*R2D;
    nav.longitude_deg += imu_dt*dx(1)*R2D;
    nav.altitude_m += imu_dt*dx(2);

    // JACOBIAN
    F.setZero();
    // ... pos2gs
    F(0,3) = 1.0; 	F(1,4) = 1.0; 	F(2,5) = 1.0;
    // ... gs2pos
    F(5,2) = -2 * g / EarthRadius;

    // ... gs2att
    temp33 = C_B2N * sk(f_b);

    F(3,6) = -2.0*temp33(0,0);  F(3,7) = -2.0*temp33(0,1);  F(3,8) = -2.0*temp33(0,2);
    F(4,6) = -2.0*temp33(1,0);  F(4,7) = -2.0*temp33(1,1);  F(4,8) = -2.0*temp33(1,2);
    F(5,6) = -2.0*temp33(2,0);  F(5,7) = -2.0*temp33(2,1);  F(5,8) = -2.0*temp33(2,2);

    // ... gs2acc
    F(3,9) = -C_B2N(0,0);  F(3,10) = -C_B2N(0,1);  F(3,11) = -C_B2N(0,2);
    F(4,9) = -C_B2N(1,0);  F(4,10) = -C_B2N(1,1);  F(4,11) = -C_B2N(1,2);
    F(5,9) = -C_B2N(2,0);  F(5,10) = -C_B2N(2,1);  F(5,11) = -C_B2N(2,2);

    // ... att2att
    temp33 = sk(om_ib);
    F(6,6) = -temp33(0,0);  F(6,7) = -temp33(0,1);  F(6,8) = -temp33(0,2);
    F(7,6) = -temp33(1,0);  F(7,7) = -temp33(1,1);  F(7,8) = -temp33(1,2);
    F(8,6) = -temp33(2,0);  F(8,7) = -temp33(2,1);  F(8,8) = -temp33(2,2);

    // ... att2gyr
    F(6,12) = -0.5;
    F(7,13) = -0.5;
    F(8,14) = -0.5;

    // ... Accel Markov Bias
    F(9,9) = -1.0/config.tau_a;    F(10,10) = -1.0/config.tau_a;  F(11,11) = -1.0/config.tau_a;
    F(12,12) = -1.0/config.tau_g;  F(13,13) = -1.0/config.tau_g;  F(14,14) = -1.0/config.tau_g;

    // State Transition Matrix: PHI = I15 + F*dt;
    PHI = I15 + F * imu_dt;

    // Process Noise
    G.setZero();
    G(3,0) = -C_B2N(0,0);   G(3,1) = -C_B2N(0,1);   G(3,2) = -C_B2N(0,2);
    G(4,0) = -C_B2N(1,0);   G(4,1) = -C_B2N(1,1);   G(4,2) = -C_B2N(1,2);
    G(5,0) = -C_B2N(2,0);   G(5,1) = -C_B2N(2,1);   G(5,2) = -C_B2N(2,2);

    G(6,3) = -0.5;
    G(7,4) = -0.5;
    G(8,5) = -0.5;

    G(9,6) = 1.0; 	    G(10,7) = 1.0; 	    G(11,8) = 1.0;
    G(12,9) = 1.0; 	    G(13,10) = 1.0; 	    G(14,11) = 1.0;

    // Discrete Process Noise
    Qw = G * Rw * G.transpose() * imu_dt;		// Qw = dt*G*Rw*G'
    Q = PHI * Qw;					// Q = (I+F*dt)*Qw
    Q = (Q + Q.transpose()) * 0.5;			// Q = 0.5*(Q+Q')

    // Covariance Time Update
    P = PHI * P * PHI.transpose() + Q;			// P = PHI*P*PHI' + Q
    P = (P + P.transpose()) * 0.5;			// P = 0.5*(P+P')

    nav.Pp0 = P(0,0);     nav.Pp1 = P(1,1);     nav.Pp2 = P(2,2);
    nav.Pv0 = P(3,3);     nav.Pv1 = P(4,4);     nav.Pv2 = P(5,5);
    nav.Pa0 = P(6,6);     nav.Pa1 = P(7,7);     nav.Pa2 = P(8,8);
    nav.Pabx = P(9,9);    nav.Paby = P(10,10);  nav.Pabz = P(11,11);
    nav.Pgbx = P(12,12);  nav.Pgby = P(13,13);  nav.Pgbz = P(14,14);

    // ==================  DONE Time Update  ===================
}

void EKF15::measurement_update(GPSdata gps) {
    // ==================  GPS Update  ===================

    // Position, converted to NED
    Vector3d pos_ref(nav.latitude_deg*D2R, nav.longitude_deg*D2R, nav.altitude_m);
    Vector3d pos_ins_ecef = lla2ecef(pos_ref);

    Vector3d pos_gps(gps.latitude_deg*D2R, gps.longitude_deg*D2R, gps.altitude_m);
    Vector3d pos_gps_ecef = lla2ecef(pos_gps);

    Vector3d pos_error_ecef = pos_gps_ecef - pos_ins_ecef;

    Vector3f pos_error_ned = ecef2ned(pos_error_ecef, pos_ref);

    // Create Measurement: y
    y(0) = pos_error_ned(0);
    y(1) = pos_error_ned(1);
    y(2) = pos_error_ned(2);

    y(3) = gps.vn_mps - nav.vn_mps;
    y(4) = gps.ve_mps - nav.ve_mps;
    y(5) = gps.vd_mps - nav.vd_mps;

    // Kalman Gain
    // K = P*H'*inv(H*P*H'+R)
    K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

    // Covariance Update
    ImKH = I15 - K * H;	                // ImKH = I - K*H

    KRKt = K * R * K.transpose();		// KRKt = K*R*K'

    P = ImKH * P * ImKH.transpose() + KRKt;	// P = ImKH*P*ImKH' + KRKt

    nav.Pp0 = P(0,0);     nav.Pp1 = P(1,1);     nav.Pp2 = P(2,2);
    nav.Pv0 = P(3,3);     nav.Pv1 = P(4,4);     nav.Pv2 = P(5,5);
    nav.Pa0 = P(6,6);     nav.Pa1 = P(7,7);     nav.Pa2 = P(8,8);
    nav.Pabx = P(9,9);    nav.Paby = P(10,10);  nav.Pabz = P(11,11);
    nav.Pgbx = P(12,12);  nav.Pgby = P(13,13);  nav.Pgbz = P(14,14);

    // State Update
    x = K * y;
    double lat_rad = nav.latitude_deg*D2R;
    double lon_rad = nav.longitude_deg*D2R;
    double denom = fabs(1.0 - (ECC2 * sin(lat_rad) * sin(lat_rad)));
    double denom_sqrt = sqrt(denom);
    double Re = EarthRadius / denom_sqrt;
    double Rn = EarthRadius * (1-ECC2) * denom_sqrt / denom;
    nav.altitude_m = nav.altitude_m - x(2);
    nav.latitude_deg = (lat_rad + x(0)/(Re + nav.altitude_m))*R2D;
    nav.longitude_deg = (lon_rad + x(1)/(Rn + nav.altitude_m)/cos(lat_rad))*R2D;

    nav.vn_mps = nav.vn_mps + x(3);
    nav.ve_mps = nav.ve_mps + x(4);
    nav.vd_mps = nav.vd_mps + x(5);

    // Attitude correction
    Quaternionf dq = Quaternionf(1.0, x(6), x(7), x(8));
    quat = (quat * dq).normalized();

    Vector3f att_vec = quat2eul(quat);
    nav.phi_deg = att_vec(0)*R2D;
    nav.theta_deg = att_vec(1)*R2D;
    nav.psi_deg = att_vec(2)*R2D;

    nav.abx += x(9);
    nav.aby += x(10);
    nav.abz += x(11);

    nav.gbx += x(12);
    nav.gby += x(13);
    nav.gbz += x(14);
}


NAVdata EKF15::get_nav() {
    nav.qw = quat.w();
    nav.qx = quat.x();
    nav.qy = quat.y();
    nav.qz = quat.z();

    return nav;
}
