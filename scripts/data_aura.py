# load aura data format

import fileinput
import numpy as np
import os
import math

import pydefs

d2r = math.pi / 180.0

def load(flight_dir):
    imu_data = []
    gps_data = []
    filter_data = []

    # load imu/gps data files
    imu_file = flight_dir + "/imu-0.txt"
    imucal_file = flight_dir + "/imucal.xml"
    gps_file = flight_dir + "/gps-0.txt"
    filter_file = flight_dir + "/filter-0.txt"
    filter_post_file = flight_dir + "/filter-post.txt"
    imu_bias_file = flight_dir + "/imubias.txt"

    # HEY: in the latest aura code, calibrated magnetometer is logged,
    # not raw magnetometer, so we don't need to correct here.  We
    # could 'back correct' here if we wanted original values for some
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
    # mag_affine = np.array(
    #     [[ 0.0026424919,  0.0001334248,  0.0000984977, -0.2235908546],
    #      [-0.000081925 ,  0.0026419229,  0.0000751835, -0.0010757621],
    #      [ 0.0000219407,  0.0000560341,  0.002541171 ,  0.040221458 ],
    #      [ 0.          ,  0.          ,  0.          ,  1.          ]]
    # )
    
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

        #s = [float(hx), float(hy), float(hz), 1.0]
        #hf = np.dot(mag_affine, s)
        #print hf
        imu = pydefs.IMU( float(time), int(status),
                          float(p), float(q), float(r),
                          float(ax), float(ay), float(az),
                          float(hx), float(hy), float(hz),
                          float(temp) )
        imu_data.append( imu )
        #print hx, hy, hz
        #print '[', hx, hy, hz, '] [', hf[0], hf[1], hf[2], ']'

    fgps = fileinput.input(gps_file)
    last_time = -1.0
    for line in fgps:
        # note the aura logs unix time of the gps record, not tow, but
        # for the pruposes of the insgns algorithm, it's only
        # important to have a properly incrementing clock, it doens't
        # really matter what the zero reference point of time is.
        time, lat, lon, alt, vn, ve, vd, unixsec, sats, status = line.split(',')
        if int(sats) >= 5 and time > last_time:
            gps = pydefs.GPS( float(time), int(status), float(unixsec),
                              float(lat), float(lon), float(alt),
                              float(vn), float(ve), float(vd))
            gps_data.append(gps)
        last_time = time

    # load filter records if they exist (for comparison purposes)
    if os.path.exists(filter_post_file):
        print "Notice: found a post process filter file so using it!"
        ffilter = fileinput.input(filter_post_file)
    else:
        ffilter = fileinput.input(filter_file)
    for line in ffilter:
        time, lat, lon, alt, vn, ve, vd, phi, the, psi, status = line.split(',')
        if abs(float(lat)) > 0.0001 and abs(float(lon)) > 0.0001:
            psi = float(psi)
            if psi > 180.0:
                psi = psi - 360.0
            if psi < -180.0:
                psi = psi - 360.0
            filter = pydefs.FILTER(float(time),
                                   float(lat)*d2r, float(lon)*d2r, float(alt),
                                   float(vn), float(ve), float(vd),
                                   float(phi)*d2r, float(the)*d2r,
                                   float(psi)*d2r)
            filter_data.append(filter)

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

