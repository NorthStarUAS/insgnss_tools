# load sentera csv format

import fileinput
import math
import numpy as np
import os
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

    # calibration by plotting and eye-balling (just finding center point, no
    # normalization cooked into calibration.)
    #hx_coeffs = np.array([ 1.0,  -1.5], dtype=np.float64)
    #hy_coeffs = np.array([ 1.0, -78.5], dtype=np.float64)
    #hz_coeffs = np.array([ 1.0, -156.5], dtype=np.float64)
    
    # calibration based on first iteration magEKF output
    hx_coeffs = np.array([ 0.01857768, -0.18006511], dtype=np.float64)
    hy_coeffs = np.array([ 0.01856929, -1.20853805], dtype=np.float64)
    hz_coeffs = np.array([ 0.01559641,  2.81011594], dtype=np.float64)
   
    hx_func = np.poly1d(hx_coeffs)
    hy_func = np.poly1d(hy_coeffs)
    hz_func = np.poly1d(hz_coeffs)

    fimu = fileinput.input(imu_file)
    for line in fimu:
        #print line
        if not re.search('Time', line):
            tokens = line.split(',')
            #print len(tokens)
            if len(tokens) == 11 and isFloat(tokens[10]):
                #print '"' + tokens[10] + '"'
                (time, p, q, r, ax, ay, az, hx, hy, hz, temp) = tokens
                # remap axis before applying mag calibration
                p = -float(p)
                q =  float(q)
                r = -float(r)
                ax = -float(ax)*g
                ay =  float(ay)*g
                az = -float(az)*g
                hx = -float(hx)
                hy =  float(hy)
                hz = -float(hz)
                hx_new = hx_func(float(hx))
                hy_new = hy_func(float(hy))
                hz_new = hz_func(float(hz))
                norm = np.linalg.norm([hx_new, hy_new, hz_new])
                hx_new /= norm
                hy_new /= norm
                hz_new /= norm
                imu = pydefs.IMU( float(time)/1000000.0, 0,
                                  p, q, r, ax, ay, az, hx_new, hy_new, hz_new,
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

def save_filter_post(flight_dir, t_store, data_store):
    filename = os.path.join(flight_dir, 'filter-post.txt')
    f = open(filename, 'w')
    size = len(t_store)
    for i in range(size):
        line = "%.3f,%.10f,%.10f,%.2f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,0" % \
               (t_store[i],
                data_store.nav_lat[i]*180.0/math.pi,
                data_store.nav_lon[i]*180.0/math.pi,
                data_store.nav_alt[i], data_store.nav_vn[i],
                data_store.nav_ve[i], data_store.nav_vd[i],
                data_store.phi[i]*180.0/math.pi,
                data_store.the[i]*180.0/math.pi,
                data_store.psi[i]*180.0/math.pi)
        f.write(line + '\n')
