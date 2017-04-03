// simple example that loads an imu log, a gps log, and plays it through
// the 15 state (inertial only) ekf

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <vector>
#include <string>
using std::vector;
using std::string;

#include "EKF_15state.hxx"
#include "netSocket.h"

const double r2d = 180.0 / M_PI;

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

    netSocket sock_imu;

    // open a UDP socket
    if ( ! sock_imu.open( false ) ) {
        printf("error opening imu input socket\n");
    }

    // bind
    if ( sock_imu.bind( "", 55555 ) == -1 ) {
        printf("error binding to port 55555\n");
    }

    IMUdata imu_record;
    GPSdata gps_record;
    NAVdata nav_record;

    // hard code a gps position for testing
    gps_record.time = 0.0;
    gps_record.lat = 44.0;
    gps_record.lon = -93.0;
    gps_record.alt = 278;
    gps_record.vn = 0.0;
    gps_record.ve = 0.0;
    gps_record.vd = 0.0;
    gps_record.unix_sec = 0.0;
    gps_record.sats = 8;
    
    EKF15 ekf;
    bool ekf_inited = false;

    double start_time = get_Time();
    int imu_count = 0;
    double output_time = 0.0;
    
    while ( true ) {
        ssize_t result = sock_imu.recv(&imu_record, sizeof(IMUdata), 0);
	if ( result == sizeof(IMUdata) ) {
            imu_count++;
            // convert to seconds
	    imu_record.time *= 1000.0;
            // convert accels to mps
	    imu_record.p = -imu_record.p;
	    imu_record.q =  imu_record.q;
	    imu_record.r = -imu_record.r;
	    imu_record.ax *= -g;
	    imu_record.ay *=  g;
	    imu_record.az *= -g;
	    imu_record.hx = -imu_record.hx;
	    imu_record.hy =  imu_record.hy;
	    imu_record.hz = -imu_record.hz;

            // printf("received imu packet, len = %d\n", result);
	    // printf( "%0.3f, "
            //         "%0.3f, %0.3f, %0.3f, "
            //         "%0.3f, %0.3f, %0.3f, "
            //         "%0.3f, %0.3f, %0.3f, "
            //         "%0.3f\n",
            //         imu_record.time,
            //         imu_record.p, imu_record.q, imu_record.r,
            //         imu_record.ax, imu_record.ay, imu_record.az,
            //         imu_record.hx, imu_record.hy, imu_record.hz,
            //         imu_record.temp);
	    // fake a gps update @ 10hz
	    if ( imu_record.time >= gps_record.time + 0.1 ) {
	      gps_record.time = gps_record.unix_sec = imu_record.time;
	      gps_record.newData = 1;
	    }
	    if ( gps_record.time > 0 && gps_record.sats > 4 ) {
	      if (!ekf_inited) {
	        nav_record = ekf.init(imu_record, gps_record);
	        ekf_inited = true;
	      } else {
		nav_record = ekf.update(imu_record, gps_record);
	      }
	      if (imu_record.time >= output_time + 0.1) {
		output_time = imu_record.time;
		double elapsed_sec = get_Time() - start_time;
		printf("%.8f %.8f %.2f %.1f %.1f %.1f %.1f\n",
		       nav_record.lat*r2d, nav_record.lon*r2d, nav_record.alt,
		       nav_record.phi*r2d, nav_record.the*r2d,
		       nav_record.psi*r2d,
		       (double)imu_count/elapsed_sec);
	      }
	    }
	}
    }
    
    return 0;
}
