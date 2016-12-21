# load the 2nd alternate sentera format (combination of procerus
# flight controller log file and sentera camera log files.)  The added
# wrinkle is needing to find a way to time correlate between camera
# and flight controller.  Initially I will look at correlating IMU
# data which should produce a reliable correlation.

import fileinput
import math
import numpy as np
import os
import re

import navpy

import sys
sys.path.append('../build/src/nav_core/.libs/')
import libnav_core

d2r = math.pi / 180.0
g = 9.81
mps2kt = 1.94384

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
    air_data = []

    # load imu/gps data files
    imu_file = flight_dir + "/imu.csv"
    procerus_file = flight_dir + "/procerus-metadata.csv"

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

    imu_source = 'camera'
    #imu_source = 'autopilot'
    
    if imu_source == 'camera':
        fimu = fileinput.input(imu_file)
        for line in fimu:
            #print line
            if not re.search('Time', line):
                tokens = line.split(',')
                # print len(tokens)
                if len(tokens) == 15 and isFloat(tokens[0]) and float(tokens[0]) > 0:
                    #(time, p, q, r, ax, ay, az, hx, hy, hz, temp, roll, pitch, yaw, yaw_accuracy) = tokens
                    imu = libnav_core.IMUdata()
                    imu.time = float(tokens[0])/1000000000.0 # nanosec
                    # remap axis before applying mag calibration
                    imu.p =  float(tokens[2])
                    imu.q = -float(tokens[1])
                    imu.r = -float(tokens[3])
                    imu.ax =  float(tokens[5])
                    imu.ay = -float(tokens[4])
                    imu.az = -float(tokens[6])
                    hx =  float(tokens[8])
                    hy = -float(tokens[7])
                    hz = -float(tokens[9])
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
                    imu.hx = hf[0]
                    imu.hy = hf[1]
                    imu.hz = hf[2]
                    if isFloat(tokens[10]):
                        imu.temp = float(tokens[10])
                    else:
                        imu.temp = 15.0
                    imu_data.append( imu )
        do_filter = 'butterworth'
        if do_filter == 'butterworth':
            # filter accel data with butterworth filter
            import scipy.signal as signal
            b, a = signal.butter(2, 15.0/(200.0/2))

            ax = []
            ay = []
            az = []
            for imu in imu_data:
                ax.append(imu.ax)
                ay.append(imu.ay)
                az.append(imu.az)
            ax = np.array(ax)
            ay = np.array(ay)
            az = np.array(az)
            ax = signal.filtfilt(b, a, ax)
            ay = signal.filtfilt(b, a, ay)
            az = signal.filtfilt(b, a, az)
            for i in range(len(imu_data)):
                imu_data[i].ax = ax[i]
                imu_data[i].ay = ay[i]
                imu_data[i].az = az[i]
                print "%.4f, %.4f, %.4f, %.4f" % (imu_data[i].time, ax[i], ay[i], az[i])
        elif do_filter == 'low-pass':
            # filter accel data with simple lowpass filter
            ax_filt = imu_data[0].ax
            ay_filt = imu_data[0].ay
            az_filt = imu_data[0].az
            for imu in imu_data:
                ax_filt = 0.5 * ax_filt + 0.5 * imu.ax
                ay_filt = 0.5 * ay_filt + 0.5 * imu.ay
                az_filt = 0.5 * az_filt + 0.5 * imu.az
                imu.ax = ax_filt
                imu.ay = ay_filt
                imu.az = az_filt
                print "%.4f, %.4f, %.4f, %.4f" % (imu.time, imu.ax, imu.ay, imu.az)
            
    fproc = fileinput.input(procerus_file)
    for line in fproc:
        if not re.search('Timestamp', line):
            #print line
            tokens = line.split(',')
            if len(tokens) == 36:
                if imu_source == 'autopilot':
                    imu = libnav_core.IMUdata()
                    imu.time = float(tokens[0])/1000000000.0 # nanosec
                    imu.p = float(tokens[20]) / 10000.0
                    imu.q = float(tokens[21]) / 10000.0
                    imu.r = float(tokens[22]) / 10000.0
                    imu.ax = float(tokens[23]) / 100.0
                    imu.ay = float(tokens[24]) / 100.0
                    imu.az = float(tokens[25]) / 100.0
                    imu.hx = float(tokens[26])
                    imu.hy = float(tokens[27])
                    imu.hz = float(tokens[28])
                    imu.temp = 15.0
                    imu_data.append( imu )
                
                gps = libnav_core.GPSdata()
                gps.time = float(tokens[0]) / 1000000000.0 # nanosec
                gps.unix_sec = gps.time
                gps.lat = float(tokens[5]) / 10000000.0
                gps.lon = float(tokens[6]) / 10000000.0
                gps.alt = float(tokens[7])
                gps.vn = float(tokens[8])
                gps.ve = float(tokens[9])
                gps.vd = float(tokens[10])
                gps_data.append(gps)

                air = libnav_core.Airdata()
                air.time = float(tokens[0]) / 1000000000.0 # nanosec
                air.airspeed = (float(tokens[29])-2000) * mps2kt / 100.0
                air_data.append( air )
                
                nav = libnav_core.NAVdata()
                nav.time = float(tokens[0]) / 1000000000.0 # nanosec
                nav.lat = (float(tokens[5]) / 10000000.0) * d2r
                nav.lon = (float(tokens[6]) / 10000000.0) * d2r
                nav.alt = float(tokens[7])
                nav.vn = float(tokens[8])
                nav.ve = float(tokens[9])
                nav.vd = float(tokens[10])
                nav.phi = float(tokens[11]) / 10000
                nav.the = float(tokens[12]) / 10000
                nav.psi = float(tokens[13]) / 10000
                filter_data.append(nav)

            else:
                print 'procerus-metadata.csv: unknown structure:', line
                
    return imu_data, gps_data, air_data, filter_data

def save_filter_result(filename, data_store):
    f = open(filename, 'w')
    size = len(data_store.time)
    for i in range(size):
        line = "%.3f,%.10f,%.10f,%.2f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,0" % \
               (data_store.time[i],
                data_store.nav_lat[i]*180.0/math.pi,
                data_store.nav_lon[i]*180.0/math.pi,
                data_store.nav_alt[i], data_store.nav_vn[i],
                data_store.nav_ve[i], data_store.nav_vd[i],
                data_store.phi[i]*180.0/math.pi,
                data_store.the[i]*180.0/math.pi,
                data_store.psi[i]*180.0/math.pi)
        f.write(line + '\n')
    f.close()

def rewrite_image_metadata_txt(base_dir, data_store):
    meta_file = os.path.join(base_dir, 'image-metadata.txt')
    new_file = os.path.join(base_dir, 'image-metadata-ekf.txt')

    f_out = open(new_file, 'w')
    f_out.write('File Name,Lat (decimal degrees),Lon (decimal degrees),Alt (meters MSL),Yaw (decimal degrees),Pitch (decimal degrees),Roll (decimal degrees),GPS Time (us since epoch)\n')

    i = 0
    for line in fileinput.input(meta_file):
        if fileinput.isfirstline():
            continue
        tokens = line.split(',')
        image = tokens[0]
        (lat, lon, alt, psi, the, phi, time_orig) = map(float, tokens[1:])
        time_sec = time_orig / 1000000.0       # convert seconds
        while data_store.time[i] < time_sec:
            i += 1
        line = "%s,%.8f,%.8f,%.4f,%.4f,%.4f,%.4f,%.0f" % \
               (image,
                data_store.nav_lat[i]*180.0/math.pi,
                data_store.nav_lon[i]*180.0/math.pi,
                data_store.nav_alt[i],
                data_store.psi[i]*180.0/math.pi,
                data_store.the[i]*180.0/math.pi,
                data_store.phi[i]*180.0/math.pi,
                time_orig)
        f_out.write(line + '\n');
    f_out.close()

def rewrite_pix4d_csv(base_dir, data_store):
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
        while data_store.time[i] < time:
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
