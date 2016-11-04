# load aura data format

import fileinput
import numpy as np
import os
import math

import imucal

import sys
sys.path.append('../build/src/nav_core/.libs/')
import libnav_core

d2r = math.pi / 180.0

def load(flight_dir, recalibrate=None):
    imu_data = []
    gps_data = []
    filter_data = []

    # load imu/gps data files
    imu_file = flight_dir + "/imu-0.txt"
    imucal_json = flight_dir + "/imucal.json"
    imucal_xml = flight_dir + "/imucal.xml"
    gps_file = flight_dir + "/gps-0.txt"
    filter_file = flight_dir + "/filter-0.txt"
    imu_bias_file = flight_dir + "/imubias.txt"

    # HEY: in the latest aura code, calibrated magnetometer is logged,
    # not raw magnetometer, so we don't need to correct here.  We
    # could 'back correct' if we wanted original values for some
    # reason (using the calibration matrix saved with each flight
    # log.)
    
    # # vireo_01
    # mag_affine = np.array(
    #     [[ 1.6207467043,  0.0228992488,  0.0398638786,  0.1274248748],
    #      [-0.0169905025,  1.7164397411, -0.0001290047, -0.1140304977],
    #      [ 0.0424979948, -0.0038515935,  1.7193766423, -0.1449816095],
    #      [ 0.          ,  0.          ,  0.          ,  1.          ]]
    # )

    # Tyr
    # mag_affine = np.array(
    #     [[ 1.810,  0.109,  0.285, -0.237],
    #      [-0.078,  1.931, -0.171, -0.060],
    #      [ 0.008,  0.109,  1.929,  0.085],
    #      [ 0.   ,  0.   ,  0.   ,  1.      ]]
    # )

    # telemaster apm2_101
    mag_affine = np.array(
        [[ 0.0026424919,  0.0001334248,  0.0000984977, -0.2235908546],
         [-0.000081925 ,  0.0026419229,  0.0000751835, -0.0010757621],
         [ 0.0000219407,  0.0000560341,  0.002541171 ,  0.040221458 ],
         [ 0.          ,  0.          ,  0.          ,  1.          ]]
    )
    
    # skywalker apm2_105
    # mag_affine = np.array(
    #      [[ 0.0025834778, 0.0001434776, 0.0001434961, -0.7900775707 ],
    #       [-0.0001903118, 0.0024796553, 0.0001365238,  0.1881142449 ],
    #       [ 0.0000556437, 0.0001329724, 0.0023791184,  0.1824851582, ],
    #       [ 0.0000000000, 0.0000000000, 0.0000000000,  1.0000000000  ]]
    # )

    #np.set_printoptions(precision=10,suppress=True)
    #print mag_affine
    
    fimu = fileinput.input(imu_file)
    for line in fimu:
        time, p, q, r, ax, ay, az, hx, hy, hz, temp, status = line.split(',')
        # quick hack:
        #hx_new = hx_func(float(hx))
        #hy_new = hy_func(float(hy))
        #hz_new = hz_func(float(hz))
        #norm = np.linalg.norm([hx_new, hy_new, hz_new])
        #hx_new /= norm
        #hy_new /= norm
        #hz_new /= norm

        s = [float(hx), float(hy), float(hz), 1.0]
        hf = np.dot(mag_affine, s)
        #print hf
        imu = libnav_core.IMUdata()
        imu.time = float(time)
        imu.p = float(p)
        imu.q = float(q)
        imu.r = float(r)
        imu.ax = float(ax)
        imu.ay = float(ay)
        imu.az = float(az)
        imu.hx = float(hx)
        imu.hy = float(hy)
        imu.hz = float(hz)
        #float(hf[0]), float(hf[1]), float(hf[2]),
        imu.temp = float(temp)
        imu_data.append( imu )

    fgps = fileinput.input(gps_file)
    last_time = -1.0
    for line in fgps:
        # Note: the aura logs unix time of the gps record, not tow,
        # but for the pruposes of the insgns algorithm, it's only
        # important to have a properly incrementing clock, it doens't
        # really matter what the zero reference point of time is.
        time, lat, lon, alt, vn, ve, vd, unix_sec, sats, status = line.split(',')
        if int(sats) >= 5 and time > last_time:
            gps = libnav_core.GPSdata()
            gps.time = float(time)
            #gps.status = int(status)
            gps.unix_sec = float(unix_sec)
            gps.lat = float(lat)
            gps.lon = float(lon)
            gps.alt = float(alt)
            gps.vn = float(vn)
            gps.ve = float(ve)
            gps.vd = float(vd)
            gps_data.append(gps)
        last_time = time

    # load filter records if they exist (for comparison purposes)
    ffilter = fileinput.input(filter_file)
    for line in ffilter:
        time, lat, lon, alt, vn, ve, vd, phi, the, psi, status = line.split(',')
        if abs(float(lat)) > 0.0001 and abs(float(lon)) > 0.0001:
            psi = float(psi)
            if psi > 180.0:
                psi = psi - 360.0
            if psi < -180.0:
                psi = psi - 360.0
            filter = libnav_core.Filterdata()
            filter.time = float(time)
            filter.lat = float(lat)*d2r
            filter.lon = float(lon)*d2r
            filter.alt = float(alt)
            filter.vn = float(vn)
            filter.ve = float(ve)
            filter.vd = float(vd)
            filter.phi = float(phi)*d2r
            filter.the = float(the)*d2r
            filter.psi = float(psi)*d2r
            filter_data.append(filter)

    print 'back correcting imu data (to get original raw values)'
    cal = imucal.Calibration()
    if os.path.exists(imucal_json):
        imucal_file = imucal_json
    elif os.path.exists(imucal_xml):
        imucal_file = imucal_xml
    cal.load(imucal_file)
    imu_data = cal.back_correct(imu_data)

    if recalibrate:
        print 'recalibrating imu data using alternate calibration file:', recalibrate
        rcal = imucal.Calibration()
        rcal.load(recalibrate)
        imu_data = rcal.correct(imu_data)

    return imu_data, gps_data, filter_data

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
