# load aura data format

import fileinput
import numpy as np
import os
import math
import re

import imucal

from nav.structs import IMUdata, GPSdata, Airdata, NAVdata

d2r = math.pi / 180.0

# empty classes we'll fill in with data members
class Controldata: pass
class APdata: pass

def load(flight_dir, recalibrate=None):
    result = {}

    # load imu/gps data files
    imu_file = flight_dir + "/imu-0.txt"
    imucal_json = flight_dir + "/imucal.json"
    imucal_xml = flight_dir + "/imucal.xml"
    gps_file = flight_dir + "/gps-0.txt"
    air_file = flight_dir + "/air-0.txt"
    filter_file = flight_dir + "/filter-0.txt"
    filter_post = flight_dir + "/filter-post.txt"
    pilot_file = flight_dir + "/pilot-0.txt"
    act_file = flight_dir + "/act-0.txt"
    ap_file = flight_dir + "/ap-0.txt"
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
    
    result['imu'] = []
    fimu = fileinput.input(imu_file)
    for line in fimu:
        tokens = re.split('[,\s]+', line.rstrip())
        #s = [float(tokens[7]), float(tokens[8]), float(tokens[9]), 1.0]
        #hf = np.dot(mag_affine, s)
        #print hf
        imu = IMUdata()
        imu.time = float(tokens[0])
        imu.p = float(tokens[1])
        imu.q = float(tokens[2])
        imu.r = float(tokens[3])
        imu.ax = float(tokens[4])
        imu.ay = float(tokens[5])
        imu.az = float(tokens[6])
        imu.hx = float(tokens[7])
        imu.hy = float(tokens[8])
        imu.hz = float(tokens[9])
        imu.temp = float(tokens[10])
        result['imu'].append( imu )

    result['gps'] = []
    fgps = fileinput.input(gps_file)
    last_time = -1.0
    for line in fgps:
        # Note: the aura logs unix time of the gps record, not tow,
        # but for the purposes of the insgns algorithm, it's only
        # important to have a properly incrementing clock, it doesn't
        # really matter what the zero reference point of time is.
        tokens = re.split('[,\s]+', line.rstrip())
        time = float(tokens[0])
        sats = int(tokens[8])
        if sats >= 5 and time > last_time:
            gps = GPSdata()
            gps.time = time
            gps.unix_sec = float(tokens[7])
            gps.lat = float(tokens[1])
            gps.lon = float(tokens[2])
            gps.alt = float(tokens[3])
            gps.vn = float(tokens[4])
            gps.ve = float(tokens[5])
            gps.vd = float(tokens[6])
            gps.sats = sats
            result['gps'].append(gps)
        last_time = time

    if os.path.exists(air_file):
        result['air'] = []
        fair = fileinput.input(air_file)
        for line in fair:
            tokens = re.split('[,\s]+', line.rstrip())
            air = Airdata()
            air.time = float(tokens[0])
            air.static_press = float(tokens[1])
            air.diff_press = 0.0    # not directly available in flight log
            air.temp = float(tokens[2])
            air.airspeed = float(tokens[3])
            air.alt_press = float(tokens[4])
            air.alt_true = float(tokens[5])
            result['air'].append( air )

    # load filter records if they exist (for comparison purposes)
    result['filter'] = []
    ffilter = fileinput.input(filter_file)
    for line in ffilter:
        tokens = re.split('[,\s]+', line.rstrip())
        lat = float(tokens[1])
        lon = float(tokens[2])
        if abs(lat) > 0.0001 and abs(lon) > 0.0001:
            nav = NAVdata()
            nav.time = float(tokens[0])
            nav.lat = lat*d2r
            nav.lon = lon*d2r
            nav.alt = float(tokens[3])
            nav.vn = float(tokens[4])
            nav.ve = float(tokens[5])
            nav.vd = float(tokens[6])
            nav.phi = float(tokens[7])*d2r
            nav.the = float(tokens[8])*d2r
            psi = float(tokens[9])
            if psi > 180.0:
                psi = psi - 360.0
            if psi < -180.0:
                psi = psi + 360.0
            nav.psi = psi*d2r
            result['filter'].append(nav)

    # load filter (post process) records if they exist (for comparison
    # purposes)
    if os.path.exists(filter_post):
        result['filter_post'] = []
        ffilter = fileinput.input(filter_post)
        for line in ffilter:
            tokens = re.split('[,\s]+', line.rstrip())
            lat = float(tokens[1])
            lon = float(tokens[2])
            if abs(lat) > 0.0001 and abs(lon) > 0.0001:
                nav = NAVdata()
                nav.time = float(tokens[0])
                nav.lat = lat*d2r
                nav.lon = lon*d2r
                nav.alt = float(tokens[3])
                nav.vn = float(tokens[4])
                nav.ve = float(tokens[5])
                nav.vd = float(tokens[6])
                nav.phi = float(tokens[7])*d2r
                nav.the = float(tokens[8])*d2r
                psi = float(tokens[9])
                if psi > 180.0:
                    psi = psi - 360.0
                if psi < -180.0:
                    psi = psi + 360.0
                nav.psi = psi*d2r
                result['filter_post'].append(nav)

    if os.path.exists(pilot_file):
        result['pilot'] = []
        fpilot = fileinput.input(pilot_file)
        for line in fpilot:
            tokens = re.split('[,\s]+', line.rstrip())
            pilot = Controldata()
            pilot.time = float(tokens[0])
            pilot.aileron = float(tokens[1])
            pilot.elevator = float(tokens[2])
            pilot.throttle = float(tokens[3])
            pilot.rudder = float(tokens[4])
            pilot.gear = float(tokens[5])
            pilot.flaps = float(tokens[6])
            pilot.aux1 = float(tokens[7])
            pilot.auto_manual = float(tokens[8])
            result['pilot'].append(pilot)

    if os.path.exists(act_file):
        result['act'] = []
        fact = fileinput.input(act_file)
        for line in fact:
            tokens = re.split('[,\s]+', line.rstrip())
            act = Controldata()
            act.time = float(tokens[0])
            act.aileron = float(tokens[1])
            act.elevator = float(tokens[2])
            act.throttle = float(tokens[3])
            act.rudder = float(tokens[4])
            act.gear = float(tokens[5])
            act.flaps = float(tokens[6])
            act.aux1 = float(tokens[7])
            act.auto_manual = float(tokens[8])
            result['act'].append(act)

    if os.path.exists(ap_file):
        result['ap'] = []
        fap = fileinput.input(ap_file)
        for line in fap:
            tokens = re.split('[,\s]+', line.rstrip())
            ap = APdata()
            ap.time = float(tokens[0])
            ap.hdg = float(tokens[1])
            ap.roll = float(tokens[2])
            ap.alt = float(tokens[3])
            ap.pitch = float(tokens[5])
            ap.speed = float(tokens[7])
            #ap.flight_time = float(tokens[8])
            #ap.target_wp = int(tokens[9])
            #ap.wp_lon = float(tokens[10])
            #ap.wp_lat = float(tokens[11])
            #ap.wp_index = int(tokens[12])
            #ap.route_size = int(tokens[13])
            result['ap'].append(ap)

    cal = imucal.Calibration()
    if os.path.exists(imucal_json):
        imucal_file = imucal_json
    elif os.path.exists(imucal_xml):
        imucal_file = imucal_xml
    else:
        imucal_file = None
    if imucal_file:
        cal.load(imucal_file)
        print 'back correcting imu data (to get original raw values)'
        result['imu'] = cal.back_correct(result['imu'])

    if recalibrate:
        print 'recalibrating imu data using alternate calibration file:', recalibrate
        rcal = imucal.Calibration()
        rcal.load(recalibrate)
        result['imu'] = rcal.correct(result['imu'])

    return result

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
