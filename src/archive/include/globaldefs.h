/*! \file globaldefs.h
 *	\brief Global definitions
 *
 *	\details This file is used to define macros and structures used in the program
 *
 * \author University of Minnesota
 * \author Aerospace Engineering and Mechanics
 * \copyright Copyright 2011 Regents of the University of Minnesota. All rights reserved.
 *
 * $Id: globaldefs_new_signals.h 854 2012-07-10 13:32:52Z joh07594 $
 */
#ifndef _UMN_GLOBALDEFS_H_
#define _UMN_GLOBALDEFS_H_

/* Constants */

#define D2R			0.017453292519943	// [rad] degrees to radians */
#define R2D			57.295779513082323	// [deg] radians to degrees */
#define PSI_TO_KPA  6.89475729  		// [KPa] PSI to KPa */
#define	g			9.814				// [m/sec^2] gravity */
#define g2      	19.62   			// [m/sec^2] 2*g */
#define PI      	3.14159265358979    // pi */
#define PI2     	6.28318530717958	// pi*2 */
#define half_pi		1.57079632679490	// pi/2 */
// *****************************************************************************

// Define status message enum list
enum   umn_errdefs	{
    got_invalid,      // No data received
    checksum_err,     // Checksum mismatch
    gps_nolock,	      // No GPS lock
    data_valid,	      // Data valid
    noPacketHeader,   // Some data received, but cannot find packet header
    incompletePacket, // Packet header found, but complete packet not received
    TU_only,	      // NAV filter, time update only
    gps_aided,	      // NAV filter, GPS aided
};

// IMU Data Structure
struct imu {
    double p;	// [rad/sec], body X axis angular rate (roll)
    double q;	// [rad/sec], body Y axis angular rate (pitch)
    double r;	// [rad/sec], body Z axis angular rate (yaw)
    double ax;	// [m/sec^2], body X axis acceleration
    double ay;	// [m/sec^2], body Y axis acceleration
    double az;	// [m/sec^2], body Z axis acceleration
    double hx;	// [Gauss], body X axis magnetic field
    double hy;	// [Gauss], body Y axis magnetic field
    double hz;	// [Gauss], body Z axis magnetic field
    //double phi; // [rad], Euler roll angle. Only used if IMU sensor reports attitude.
    //double the; // [rad], Euler pitch angle. Only used if IMU sensor reports attitude.
    //double psi; // [rad], Euler yaw angle. Only used if IMU sensor reports attitude.
    //float  T;	// [degC], temperature of IMU sensor
    //float  Vs;	// [Volts], supply voltage of IMU sensor
    //double adc; // [counts], ADC reading
    //enum umn_errdefs err_type; // IMU status
    double temp; // degree C
    double time; // [sec], timestamp of IMU data
};

// GPS Data Structure
struct gps {
    double lat;	// [deg], Geodetic latitude
    double lon;	// [deg], Geodetic longitude
    double alt;	// [m], altitude relative to WGS84
    double ve;	// [m/sec], East velocity
    double vn;	// [m/sec], North velocity
    double vd;	// [m/sec], Down velocity
    //double Xe;	// [m], X position, ECEF
    //double Ye;	// [m], Y position, ECEF
    //double Ze;	// [m], Z position, ECEF
    //double Ue;	// [m/sec], X velocity, ECEF
    //double Ve;	// [m/sec], Y velocity, ECEF
    //double We;	// [m/sec], Z velocity, ECEF
    //double sig_N; // [m], Position error standard deviation in the North direction
    //double sig_E; // [m], Position error standard deviation in the East direction
    //double sig_D; // [m], Position error standard deviation in the Down direction
    //double sig_vn; // [m/sec], Velocity error standard deviation in the North direction
    //double sig_ve; // [m/sec], Velocity error standard deviation in the East direction
    //double sig_vd; // [m/sec], Velocity error standard deviation in the Down direction
    //double GPS_TOW;	// [sec], GPS Time Of Week
    //double courseOverGround;// [rad], course over the ground, relative to true North
    //double speedOverGround;	// [rad], speed over the ground
    double time;	// [sec], timestamp of GPS data
    unsigned short newData;	// [bool], flag set when GPS data has been updated
    unsigned short satVisible; // Number satellites used in the position solution
    //unsigned short navValid;// flag indicating whether the solution is valid, 0 = valid
    //unsigned short GPS_week;// GPS week since current epoch.
    //enum umn_errdefs err_type;	// GPS status
    //int baudRate;		// Baud rate for serial port
    //char* portName;		// Name of serial port
    //int port;			// handle for accessing serial port
    //unsigned char* localBuffer; // local buffer to store partial serial data packets
    //int bytesInLocalBuffer; // number of bytes in the local buffer
    //int readState;			// current state of serial data reader function
    //int read_calls;			// number of times the read_gps function has been called
    double unix_time;
};


// Navigation Filter Data Structure
struct nav {
    double lat;		// [rad], geodetic latitude estimate
    double lon;		// [rad], geodetic longitude estimate
    double alt;		// [m], altitude relative to WGS84 estimate
    double vn;		// [m/sec], north velocity estimate
    double ve;		// [m/sec], east velocity estimate
    double vd;		// [m/sec], down velocity estimate
    double phi;		// [rad], Euler roll angle estimate
    double the;		// [rad], Euler pitch angle estimate
    double psi;		// [rad], Euler yaw angle estimate
    double quat[4];	// Quaternions estimate
    double ab[3];	// [m/sec^2], accelerometer bias estimate
    double gb[3];	// [rad/sec], rate gyro bias estimate
    //double asf[3];	// [m/sec^2], accelerometer scale factor estimate
    //double gsf[3];	// [rad/sec], rate gyro scale factor estimate
    double Pp[3];	// [rad], covariance estimate for position
    double Pv[3];	// [rad], covariance estimate for velocity
    double Pa[3];	// [rad], covariance estimate for angles
    double Pab[3];	// [rad], covariance estimate for accelerometer bias
    double Pgb[3];	// [rad], covariance estimate for rate gyro bias
    //double Pasf[3];	// [rad], covariance estimate for accelerometer scale factor
    //double Pgsf[3];	// [rad], covariance estimate for rate gyro scale factor
    enum umn_errdefs err_type;	// NAV filter status
    double time;			// [sec], timestamp of NAV filter
};


#endif	/* _UMN_GLOBALDEFS_H_ */
