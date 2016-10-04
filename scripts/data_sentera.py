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
    
    #~/Projects/PILLS/Phantom\ 3\ Flight\ Data/2016-03-22\ --\ imagery_0012\ -\ 400\ ft\ survey
    #hx_coeffs = np.array([ 0.01857771, -0.18006661], dtype=np.float64)
    #hy_coeffs = np.array([ 0.01856938, -1.20854406], dtype=np.float64)
    #hz_coeffs = np.array([ 0.01559645,  2.81011976], dtype=np.float64)

    # ~/Projects/PILLS/Phantom\ 3\ Flight\ Data/imagery_0009 - 0012
    #hx_coeffs = np.array([ 0.01789447,  3.70605872], dtype=np.float64)
    #hy_coeffs = np.array([ 0.017071,    0.7125617], dtype=np.float64)
    #hz_coeffs = np.array([ 0.01447557, -6.54621951], dtype=np.float64)
    
    # ~/Projects/PILLS/2016-04-04\ --\ imagery_0002
    # ~/Projects/PILLS/2016-04-14\ --\ imagery_0003
    # ~/Projects/PILLS/2016-04-14\ --\ imagery_0004
    #hx_coeffs = np.array([ 0.01658555, -0.07790598], dtype=np.float64)
    #hy_coeffs = np.array([ 0.01880532, -1.26548151], dtype=np.float64)
    #hz_coeffs = np.array([ 0.01339084,  2.61905809], dtype=np.float64)

    # ~/Projects/PILLS/2016-05-12\ --\ imagery_0004
    #hx_coeffs = np.array([ 0.01925678,  0.01527908], dtype=np.float64)
    #hy_coeffs = np.array([ 0.01890112, -1.18040666], dtype=np.float64)
    #hz_coeffs = np.array([ 0.01645011,  2.87769626], dtype=np.float64)

    #hx_func = np.poly1d(hx_coeffs)
    #hy_func = np.poly1d(hy_coeffs)
    #hz_func = np.poly1d(hz_coeffs)

    # ~/Projects/PILLS/2016-06-29\ --\ calibration_0002/
    # mag_affine = np.array(
    #     [[ 0.0223062041, -0.0002700799, -0.0001325525,  1.2016235718],
    #      [-0.0002700799,  0.0229484854,  0.0000356172,  0.1177744077],
    #      [-0.0001325525,  0.0000356172,  0.0206129279, -3.2713740483],
    #      [ 0.          ,  0.          ,  0.          ,  1.          ]]
    # )

    # Phantom 3 - Aug 2016 (ellipse cal)
    mag_affine = np.array(
        [[ 0.0189725067,  0.0000203615,  0.0002139272, -0.0134053645],
         [ 0.0000760692,  0.0180178765,  0.0000389461, -1.044762755 ],
         [ 0.0002417847,  0.0000458039,  0.0171450614,  2.647911793 ],
         [ 0.          ,  0.          ,  0.          ,  1.          ]]
    )
    # Phantom 3 - Aug 2016 (ekf cal)
    # mag_affine = np.array(
    #     [[ 0.0181297161,  0.000774339,  -0.002037224 , -0.2576406372],
    #      [ 0.0002434548,  0.018469032,   0.0016475328, -0.8452362072],
    #      [ 0.0000145964,  0.000267444,   0.0159433791,  2.5630653789],
    #      [ 0.          ,  0.         ,   0.          ,  1.          ]]
    # )
    print mag_affine
    
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
                mag_orientation = 'newer'
                if mag_orientation == 'older':
                    #hx_new = hx_func(float(hx))
                    #hy_new = hy_func(float(hy))
                    #hz_new = hz_func(float(hz))
                    s = [hx, hy, hz]
                elif mag_orientation == 'newer':
                    # remap for 2016-05-12 (0004) data set
                    #hx_new = hx_func(float(-hy))
                    #hy_new = hy_func(float(-hx))
                    #hz_new = hz_func(float(-hz))
                    s = [-hy, -hx, -hz]
                # mag calibration mapping via mag_affine matrix
                hs = np.hstack( [s, 1.0] )
                hf = np.dot(mag_affine, hs)
                #print hf[:3]
                #norm = np.linalg.norm([hx_new, hy_new, hz_new])
                #hx_new /= norm
                #hy_new /= norm
                #hz_new /= norm
                
                imu = pydefs.IMU( float(time)/1000000.0, 0,
                                  p, q, r, ax, ay, az, hf[0], hf[1], hf[2],
                                  float(temp) )
                imu_data.append( imu )

    fgps = fileinput.input(gps_file)
    for line in fgps:
        if not re.search('Timestamp', line):
            #print line
            tokens = line.split(',')
            #print len(tokens)
            if len(tokens) == 14:
                (time, itow, ecefx, ecefy, ecefz, ecefvx, ecefvy, ecefvz,
                 fixtype, posacc, spdacc, posdop, numsvs, flags) = tokens
            elif len(tokens) == 19:
                (time, itow, lat, lon, alt, ecefx, ecefy, ecefz,
                 ecefvx, ecefvy, ecefvz,
                 fixtype, posacc, spdacc, posdop, numsvs, flags,
                 diff_status, carrier_status) = tokens
                
            # wgs84 position
            pos_source = 'llh'  # 'llh' or 'ecef'
            llh = navpy.ecef2lla([float(ecefx)/100.0,
                                  float(ecefy)/100.0,
                                  float(ecefz)/100.0], "deg")
            # velocity
            ned = navpy.ecef2ned([float(ecefvx)/100.0,
                                  float(ecefvy)/100.0,
                                  float(ecefvz)/100.0],
                                 llh[0], llh[1], llh[2])
            if int(numsvs) >= 4:
                gps = pydefs.GPS( float(time)/1000000.0, int(0),
                                  float(time)/1000000.0,
                                  # llh[0], llh[1], llh[2],
                                  float(lat), float(lon), float(alt),
                                  ned[0], ned[1], ned[2])
                gps_data.append(gps)

    print "imu records:", len(imu_data)
    print "gps records:", len(gps_data)

    return imu_data, gps_data, filter_data

def save_filter_result(filename, t_store, data_store):
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
    f.close()

def rewrite_pix4d_csv(base_dir, t_store, data_store):
    meta_file = os.path.join(base_dir, 'image-metadata.txt')
    pix4d_file = os.path.join(base_dir, 'pix4d-ekf.csv')

    f_out = open(pix4d_file, 'w')
    f_out.write('File Name,Lat (decimal degrees),Lon (decimal degrees),Alt (meters MSL),Roll (decimal degrees),Pitch (decimal degrees),Yaw (decimal degrees)\n')

    i = 0
    for line in fileinput.input(meta_file):
        if fileinput.isfirstline():
            continue
        tokens = line.split(',')
        image = tokens[0]
        (lat, lon, alt, psi, the, phi, time) = map(float, tokens[1:])
        time /= 1000000.0       # convert seconds
        while t_store[i] < time:
            i += 1
        line = "%s,%.8f,%.8f,%.4f,%.4f,%.4f,%.4f" % \
               (image,
                data_store.nav_lat[i]*180.0/math.pi,
                data_store.nav_lon[i]*180.0/math.pi,
                data_store.nav_alt[i],
                data_store.phi[i]*180.0/math.pi,
                data_store.the[i]*180.0/math.pi,
                data_store.psi[i]*180.0/math.pi)
        f_out.write(line + '\n');
    f_out.close()
