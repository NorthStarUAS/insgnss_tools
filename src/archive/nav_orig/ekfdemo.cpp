// simple compile command line:
//
//     /opt/codesourcery/arm-2009q1/bin/arm-none-linux-gnueabi-g++ -O3 EKF_15state_quat.c ekfdemo.cpp matrix.c nav_functions.c -lm -lrt -o ekfdemo
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <vector>
#include <string>
using std::vector;
using std::string;

#include "../include/globaldefs.h"
#include "nav_interface.h"

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

    vector<imu> imu_list;
    vector<gps> gps_list;

    char buf[256]; // input buffer
    FILE *f;
    
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
        imu imu_record;
        imu_record.time = tokens[0];
        imu_record.p = tokens[1];
        imu_record.q = tokens[2];
        imu_record.r = tokens[3];
        imu_record.ax = tokens[4];
        imu_record.ay = tokens[5];
        imu_record.az = tokens[6];
        imu_record.hx = tokens[7];
        imu_record.hy = tokens[8];
        imu_record.hz = tokens[9];
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
        gps gps_record;
        gps_record.time = tokens[0];
        gps_record.lat = tokens[1];
        gps_record.lon = tokens[2];
        gps_record.alt = tokens[3];
        gps_record.vn = tokens[4];
        gps_record.ve = tokens[5];
        gps_record.vd = tokens[6];
        gps_record.unix_time = tokens[7];
        gps_record.satVisible = tokens[8];
        gps_list.push_back(gps_record);
    }
    fclose(f);
    printf("Loaded gps records: %d\n", gps_list.size());
    if (gps_list.size() < 1) {
        printf("not enough gps records loaded to continue.\n");
        return -1;
    }

    bool ekf_inited = false;
    int imu_index = 0;
    int gps_index = 0;
    imu imu_record;
    gps gps_record;
    nav nav_record;

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

        if (!ekf_inited) {
            init_nav(&imu_record, &gps_record, &nav_record);
	    ekf_inited = true;
        } else {
            get_nav(&imu_record, &gps_record, &nav_record);
        }

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
