// simple example that loads an imu log, a gps log, and plays it through
// the 15 state plus magnetometer ekf

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <vector>
#include <string>
using std::vector;
using std::string;

#include <iostream>
using std::cout;
using std::endl;

#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/LU>
using namespace Eigen;

#include "../core/coremag.h"
#include "../core/nav_functions.hxx"

#include "EKF_15state_mag.hxx"

#include "linearfit.hxx"

// magnetometer calibration
static Vector3d mag_ned;
static Quaterniond quat;
static Matrix3d C_N2B;
static LinearFitFilter magx(3600);
static LinearFitFilter magy(3600);
static LinearFitFilter magz(3600);

void set_ideal_mag_vector(NAVdata nav) {
    long int jd = now_to_julian_days();
    double field[6];
    calc_magvar( nav.lat, nav.lon,
		 nav.alt / 1000.0, jd, field );
    mag_ned(0) = field[3];
    mag_ned(1) = field[4];
    mag_ned(2) = field[5];
    mag_ned.normalize();
    cout << field[0] << " " << field[1] << " " << field[2] << endl;
    cout << "Ideal mag vector (ned): " << mag_ned << endl;
}

void update_mag_calibration(Vector3d mag_raw, IMUdata imu, NAVdata nav) {
    quat = Quaterniond(nav.qw, nav.qx, nav.qy, nav.qz);
    C_N2B = quat2dcm(quat);
    Vector3d mag_ideal = C_N2B * mag_ned;

    double vel_ms = sqrt(nav.vn*nav.vn + nav.ve*nav.ve);
    if (vel_ms > 4.0) {
        // update calibration when moving
        magx.update(mag_raw(0), mag_ideal(0), 0.01);
	magy.update(mag_raw(1), mag_ideal(1), 0.01);
	magz.update(mag_raw(2), mag_ideal(2), 0.01);
    }

    if ( vel_ms > 4.0 ) {
        // check the calibration

        // if mag uncalibrated
        // double cal_hx = magx.get_value(imu.hx);
	// double cal_hy = magy.get_value(imu.hy);
	// double cal_hz = magz.get_value(imu.hz);
	// Vector3d mag_cal(cal_hx, cal_hy, cal_hz);

        // if mag calibrated
        Vector3d mag_cal(imu.hx, imu.hy, imu.hz);
	mag_cal.normalize();
	Vector3d mag_error = mag_cal - mag_ideal;
	double error = mag_error.norm();
	printf("mag err = %.2f\n", error);
	/*printf("x: %.2f %.4f y: %.2f %.4f z: %.2f %.4f\n",
	       magx.get_a0(), magx.get_a1(),
	       magy.get_a0(), magy.get_a1(),
	       magz.get_a0(), magz.get_a1());*/
    }
}

// split a string into tokens
vector<float>
split( const string& str, const char* sep, int maxsplit = 0 )
{
    vector<float> result;
    int n = strlen( sep );
    if (n == 0) {
        // Error: empty separator string
        return result;
    }
    const char* s = str.c_str();
    string::size_type len = str.length();
    string::size_type i = 0;
    string::size_type j = 0;
    int splitcount = 0;

    while (i+n <= len) {
        if (s[i] == sep[0] && (n == 1 || memcmp(s+i, sep, n) == 0)) {
            result.push_back( atof(str.substr(j,i-j).c_str()) );
            i = j = i + n;
            ++splitcount;
            if (maxsplit && (splitcount >= maxsplit))
                break;
        } else {
            ++i;
        }
    }

    result.push_back( atof(str.substr(j,len-j).c_str()) );
    return result;
}

// simple time measurement
double get_Time()
{
    static struct timespec tstart;
    static bool init = false;
   
    if ( !init ) {
        init = true;
        clock_gettime(CLOCK_MONOTONIC,&tstart);
        return 0.0;
    }

    struct timespec tcur;
    clock_gettime(CLOCK_MONOTONIC, &tcur);
    
    struct timespec tdiff;
    if ( (tcur.tv_nsec - tstart.tv_nsec) < 0 ) {
	tdiff.tv_sec = tcur.tv_sec-tstart.tv_sec-1;
	tdiff.tv_nsec = 1000000000+tcur.tv_nsec-tstart.tv_nsec;
    } else {
	tdiff.tv_sec = tcur.tv_sec-tstart.tv_sec;
	tdiff.tv_nsec = tcur.tv_nsec-tstart.tv_nsec;
    }
    
    double elapsed = (double)tdiff.tv_sec + 1.0e-9*(double)tdiff.tv_nsec;
    
    return elapsed;
}

int main(int argc, char **argv) {
    if (argc != 3) {
        printf("Usage: %s imu.txt gps.txt\n", argv[0]);
    }

    vector<IMUdata> imu_list;
    vector<GPSdata> gps_list;

    char buf[256]; // input buffer
    FILE *f;

    // in demo data, magnetometer is saved already calibrated, but we
    // also save the calibration matrix, so we can undo the
    // calibration which we will now do so we can test the on-the-fly
    // calibration scheme...
    Matrix4d mag_cal;
    mag_cal <<
       0.0025737141, 0.0001152496, 0.0004039434, -0.8824808415,
      -0.0002158941, 0.0024616144, 0.0002816274,  0.1427379365,
       0.0000307758, 0.0001215845, 0.0025342126,  0.1370547336,
       0.0000000000, 0.0000000000, 0.0000000000,  1.0000000000;
    Matrix4d mag_uncal = mag_cal.inverse();
    
    // load the imu data file
    f = fopen(argv[1], "r");
    if (f == NULL) {
        printf("error reading: %s\n", argv[1]);
        return -1;
    }
    while (fgets(buf, 256, f) != NULL) {
        vector<float> tokens = split(buf, ",");
        if (tokens.size() != 12) {
            printf("problem parsing imu file: %s\n", argv[1]);
            return -1;
        }

	// undo the original magnetometer calibration (for testing)
	Vector4d hc(tokens[7], tokens[8], tokens[9], 1.0);
	Vector4d hr = mag_uncal * hc;
	
        IMUdata imu_record;
        imu_record.time = tokens[0];
        imu_record.p = tokens[1];
        imu_record.q = tokens[2];
        imu_record.r = tokens[3];
        imu_record.ax = tokens[4];
        imu_record.ay = tokens[5];
        imu_record.az = tokens[6];
        imu_record.hx = /*tokens[7]*/ round(hr(0));
        imu_record.hy = /*tokens[8]*/ round(hr(1));
        imu_record.hz = /*tokens[9]*/ round(hr(2));
        imu_record.temp = tokens[10];
        imu_list.push_back(imu_record);
    }
    fclose(f);
    printf("Loaded imu records: %d\n", imu_list.size());
    if (imu_list.size() < 1) {
        printf("not enough imu records loaded to continue.\n");
        return -1;
    }
    
    // load the gps data file
    f = fopen(argv[2], "r");
    if (f == NULL) {
        printf("error reading: %s\n", argv[1]);
        return -1;
    }
    while (fgets(buf, 256, f) != NULL) {
        vector<float> tokens = split(buf, ",");
        if (tokens.size() != 13) {
            printf("problem parsing gps file: %s\n", argv[1]);
            return -1;
        }
        GPSdata gps_record;
        gps_record.time = tokens[0];
        gps_record.lat = tokens[1];
        gps_record.lon = tokens[2];
        gps_record.alt = tokens[3];
        gps_record.vn = tokens[4];
        gps_record.ve = tokens[5];
        gps_record.vd = tokens[6];
        gps_record.unix_sec = tokens[7];
        gps_record.sats = tokens[8];
        gps_list.push_back(gps_record);
    }
    fclose(f);
    printf("Loaded gps records: %d\n", gps_list.size());
    if (gps_list.size() < 1) {
        printf("not enough gps records loaded to continue.\n");
        return -1;
    }

    EKF15mag ekf;
    bool ekf_inited = false;
    int imu_index = 0;
    int gps_index = 0;
    IMUdata imu_record;
    GPSdata gps_record;
    NAVdata nav_record;

    double start_sec = get_Time();
    printf("Start time = %.2f\n", start_sec);
    
    while (true) {
        if (imu_index >= imu_list.size() - 1) {
            // exit at end of imu data
            break;
        }
        imu_record = imu_list[imu_index];
        if (gps_index < gps_list.size() - 1) {
            // walk the gps counter forward as needed
            int newData = 0;
            while (gps_index < gps_list.size() - 1 && gps_list[gps_index+1].time <= imu_record.time) {
                gps_index++;
                newData = 1;
            }
            gps_record = gps_list[gps_index];
            gps_record.newData = newData;
        } else {
            // no more gps data, stay on the last record
            gps_record = gps_list[gps_index];
            gps_record.newData = 0;
        }

	// save the raw magnetometer
	Vector3d mag_raw(imu_record.hx, imu_record.hy, imu_record.hz);
	
	// calibrate the mags before calling the ekf
	imu_record.hx = magx.get_value(mag_raw(0));
	imu_record.hy = magy.get_value(mag_raw(1));
	imu_record.hz = magz.get_value(mag_raw(2));
	
        if (!ekf_inited) {
	    nav_record = ekf.init(imu_record, gps_record);
	    set_ideal_mag_vector(nav_record);
	    ekf_inited = true;
        } else {
            nav_record = ekf.update(imu_record, gps_record);
	    update_mag_calibration(mag_raw, imu_record, nav_record);
        }

	// magnetometer self calibration
	
	
        /*double end_sec = get_Time();
        double elapsed_sec = end_sec - start_sec;
        printf("%.8f %.8f %.2f %.1f hz\n",
               nav_record.lat, nav_record.lon, nav_record.alt,
	       (float)imu_index/elapsed_sec);*/
        
        imu_index++;
    }

    double end_sec = get_Time();
    printf("End time = %.2f\n", end_sec);
    double elapsed_sec = end_sec - start_sec;
    printf("Processed %d imu records in %.2f seconds.\n",
           imu_list.size(),
           elapsed_sec);
    
    double start_gps = gps_list[0].time;
    double end_gps = gps_list[gps_list.size()-1].time;
    double elapsed_gps = end_gps - start_gps;
    printf("Data file time span = %.2f seconds.\n", elapsed_gps);
    
    printf("Update rate = %.1f records/second\n",
           (double)imu_list.size() / elapsed_sec);
    
    return 0;
}
