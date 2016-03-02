# load sentera csv format

import fileinput
import math
import re

import navpy

import pydefs

d2r = math.pi / 180.0
g = 9.81

def isFloat(string):
    try:
        float(string)
        return True
    except ValueError:
        return False
    
def load(flight_dir):
    imu_data = []
    gps_data = []
    filter_data = []

    # load imu/gps data files
    imu_file = flight_dir + "/imu.csv"
    gps_file = flight_dir + "/gps.csv"

    fimu = fileinput.input(imu_file)
    for line in fimu:
        #print line
        if not re.search('Time', line):
            tokens = line.split(',')
            #print len(tokens)
            if len(tokens) == 11 and isFloat(tokens[10]):
                #print '"' + tokens[10] + '"'
                (time, p, q, r, ax, ay, az, hx, hy, hz, temp) = tokens
                imu = pydefs.IMU( float(time)/1000000.0, 0,
                                  float(p), float(q), float(r),
                                  float(ax)*g, float(ay)*g, float(az)*g,
                                  float(hx), float(hy), float(hz),
                                  float(temp) )
                imu_data.append( imu )

    fgps = fileinput.input(gps_file)
    for line in fgps:
        if not re.search('Timestamp', line):
            #print line
            tokens = line.split(',')
            #print len(tokens)
            (time, itow, ecefx, ecefy, ecefz, ecefvx, ecefvy, ecefvz, fixtype,
             posacc, spdacc, posdop, numsvs, flags) = tokens
            llh = navpy.ecef2lla([float(ecefx)/100.0,
                                  float(ecefy)/100.0,
                                  float(ecefz)/100.0], "deg")
            ned = navpy.ecef2ned([float(ecefvx)/100.0,
                                  float(ecefvy)/100.0,
                                  float(ecefvz)/100.0],
                                 llh[0], llh[1], llh[2])
            if int(numsvs) >= 4:
                gps = pydefs.GPS( float(time)/1000000.0, int(0), float(time)/1000000.0,
                                  llh[0], llh[1], llh[2],
                                  ned[0], ned[1], ned[2])
                gps_data.append(gps)

    print "imu records:", len(imu_data)
    print "gps records:", len(gps_data)

    return imu_data, gps_data, filter_data
