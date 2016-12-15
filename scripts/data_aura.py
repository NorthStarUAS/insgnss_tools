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
    air_data = []
    filter_data = []
    pilot_data = []
    act_data = []

    # load imu/gps data files
    imu_file = flight_dir + "/imu-0.txt"
    imucal_json = flight_dir + "/imucal.json"
    imucal_xml = flight_dir + "/imucal.xml"
    gps_file = flight_dir + "/gps-0.txt"
    air_file = flight_dir + "/air-0.txt"
    filter_file = flight_dir + "/filter-0.txt"
    pilot_file = flight_dir + "/pilot-0.txt"
    act_file = flight_dir + "/act-0.txt"
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
        tokens = line.split(',')
        time = float(tokens[0])
        lat = float(tokens[1])
        lon = float(tokens[2])
        alt = float(tokens[3])
        vn = float(tokens[4])
        ve = float(tokens[5])
        vd = float(tokens[6])
        unix_sec = float(tokens[7])
        sats = int(tokens[8])
        if len(tokens) == 13:
            status = int(tokens[12])
        else:
            status = int(tokens[8])
        if sats >= 5 and time > last_time:
            gps = libnav_core.GPSdata()
            gps.time = time
            #gps.status = status
            gps.unix_sec = unix_sec
            gps.lat = lat
            gps.lon = lon
            gps.alt = alt
            gps.vn = vn
            gps.ve = ve
            gps.vd = vd
            gps_data.append(gps)
        last_time = time

    fair = fileinput.input(air_file)
    for line in fair:
        tokens = line.split(',')
        air = libnav_core.Airdata()
        air.time = float(tokens[0])
        air.static_press = float(tokens[1])
        air.diff_press = 0.0    # not directly available in flight log
        air.temp = float(tokens[2])
        air.airspeed = float(tokens[3])
        air.altitude = float(tokens[4])
        air_data.append( air )

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

    fpilot = fileinput.input(pilot_file)
    last_time = -1.0
    for line in fpilot:
        time, aileron, elevator, throttle, rudder, gear, flaps, aux1, auto_manual, status = line.split(',')
        pilot = libnav_core.Controldata()
        pilot.time = float(time)
        pilot.aileron = float(aileron)
        pilot.elevator = float(elevator)
        pilot.throttle = float(throttle)
        pilot.rudder = float(rudder)
        pilot.gear = float(gear)
        pilot.flaps = float(flaps)
        pilot.aux1 = float(aux1)
        pilot.auto_manual = float(auto_manual)
        pilot_data.append(pilot)
        last_time = time

    fact = fileinput.input(act_file)
    last_time = -1.0
    for line in fact:
        time, aileron, elevator, throttle, rudder, gear, flaps, aux1, auto_manual, status = line.split(',')
        act = libnav_core.Controldata()
        act.time = float(time)
        act.aileron = float(aileron)
        act.elevator = float(elevator)
        act.throttle = float(throttle)
        act.rudder = float(rudder)
        act.gear = float(gear)
        act.flaps = float(flaps)
        act.aux1 = float(aux1)
        act.auto_manual = float(auto_manual)
        act_data.append(act)
        last_time = time

    cal = imucal.Calibration()
    if os.path.exists(imucal_json):
        imucal_file = imucal_json
    elif os.path.exists(imucal_xml):
        imucal_file = imucal_xml
    cal.load(imucal_file)
    print 'back correcting imu data (to get original raw values)'
    imu_data = cal.back_correct(imu_data)

    if recalibrate:
        print 'recalibrating imu data using alternate calibration file:', recalibrate
        rcal = imucal.Calibration()
        rcal.load(recalibrate)
        imu_data = rcal.correct(imu_data)

    return imu_data, gps_data, air_data, filter_data, pilot_data, act_data

def save_filter_result(filename, data_store):
    f = open(filename, 'w')
    size = len(data_store.time)
    for i in range(size):
        line = "%.3f,%.10f,%.10f,%.2f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,0" % \
               (data_store.time[i],
                data_store.lat[i]*180.0/math.pi,
                data_store.lon[i]*180.0/math.pi,
                data_store.alt[i],
                data_store.vn[i],
                data_store.ve[i],
                data_store.vd[i],
                data_store.phi[i]*180.0/math.pi,
                data_store.the[i]*180.0/math.pi,
                data_store.psi[i]*180.0/math.pi)
        f.write(line + '\n')
    f.close()
