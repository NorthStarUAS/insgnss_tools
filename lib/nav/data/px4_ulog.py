# load a px4_sdlog2 csv file

import csv
import math
import numpy as np
from scipy import interpolate # strait up linear interpolation, nothing fancy
import re

from nav.structs import IMUdata, GPSdata, Airdata, NAVdata

#import transformations

d2r = math.pi / 180.0
r2d = 180.0 / math.pi

# create rotation matrix for the quaternion
def px4_quat2dcm(q):
    R = np.zeros( (3,3) )
    aSq = q[0] * q[0]
    bSq = q[1] * q[1]
    cSq = q[2] * q[2]
    dSq = q[3] * q[3]
    R[0,0] = aSq + bSq - cSq - dSq
    R[0,1] = 2.0 * (q[1] * q[2] - q[0] * q[3])
    R[0,2] = 2.0 * (q[0] * q[2] + q[1] * q[3])
    R[1,0] = 2.0 * (q[1] * q[2] + q[0] * q[3])
    R[1,1] = aSq - bSq + cSq - dSq
    R[1,2] = 2.0 * (q[2] * q[3] - q[0] * q[1])
    R[2,0] = 2.0 * (q[1] * q[3] - q[0] * q[2])
    R[2,1] = 2.0 * (q[0] * q[1] + q[2] * q[3])
    R[2,2] = aSq - bSq - cSq + dSq
    return R
        
def px4_quat2euler(q):
    # create Euler angles vector from the quaternion
    roll = math.atan2(2.0 * (q[0] * q[1] + q[2] * q[3]),
                      1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]))
    pitch = math.asin(2.0 * (q[0] * q[2] - q[3] * q[1]))
    yaw = math.atan2(2.0 * (q[0] * q[3] + q[1] * q[2]),
                     1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]))
    return (roll, pitch, yaw)

def load(csv_base):
    result = {}

    comb_path = csv_base + '_sensor_combined_0.csv'
    gps_path = csv_base + '_vehicle_gps_position_0.csv'
    att_path = csv_base + '_vehicle_attitude_0.csv'
    pos_path = csv_base + '_vehicle_global_position_0.csv'

    result['imu'] = []
    with open(comb_path, 'rb') as f:
        reader = csv.DictReader(f)
        for row in reader:
            imu = IMUdata()
            imu.time = float(row['timestamp']) / 1000000.0
            imu.p = float(row['gyro_rad[0]'])
            imu.q = float(row['gyro_rad[1]'])
            imu.r = float(row['gyro_rad[2]'])
            imu.ax = float(row['accelerometer_m_s2[0]'])
            imu.ay = float(row['accelerometer_m_s2[1]'])
            imu.az = float(row['accelerometer_m_s2[2]'])
            hx = float(row['magnetometer_ga[0]'])
            hy = float(row['magnetometer_ga[0]'])
            hz = float(row['magnetometer_ga[0]'])
            hf = np.array( [hx, hy, hz] )
            norm = np.linalg.norm(hf)
            hf /= norm
            imu.hx = hf[0]
            imu.hy = hf[1]
            imu.hz = hf[2]
            #print imu.hx, imu.hy, imu.hz
            imu.temp = 15.0
            result['imu'].append( imu )

    result['gps'] = []
    with open(gps_path, 'rb') as f:
        reader = csv.DictReader(f)
        for row in reader:
            gps = GPSdata()
            gps.time = float(row['timestamp']) / 1000000.0
            gps.unix_sec = float(row['time_utc_usec']) / 1000000.0
            gps.lat = float(row['lat']) / 1e7
            gps.lon = float(row['lon']) / 1e7
            gps.alt = float(row['alt']) / 1e3
            # print gps.lat, gps.lon, gps.alt
            gps.vn = float(row['vel_n_m_s'])
            gps.ve = float(row['vel_e_m_s'])
            gps.vd = float(row['vel_d_m_s'])
            gps.sats = int(row['satellites_used'])
            if gps.sats >= 5:
                result['gps'].append(gps)

    # result['air'] = []
    # with open(gps_path, 'rb') as f:
    #     reader = csv.DictReader(f)
    #     for row in reader:
    #         air = Airdata()
    #         air.time = imu.time
    #         air.static_press = float(row['SENS_BaroPres'])
    #         air.diff_press = float(row['SENS_DiffPres'])
    #         air.temp = float(row['AIRS_AirTemp'])
    #         air.airspeed = float(row['AIRS_IndSpeed'])
    #         air.alt_press = float(row['SENS_BaroAlt'])
    #         air.alt_true = gps.alt
    #         result['air'].append( air )

    att = []
    with open(att_path, 'rb') as f:
        reader = csv.DictReader(f)
        for row in reader:
            time = float(row['timestamp']) / 1000000.0
            q = [ float(row['q[0]']),
                  float(row['q[1]']),
                  float(row['q[2]']),
                  float(row['q[3]']) ]
            # either of these will work ...
            if True:
                (roll, pitch, yaw) = px4_quat2euler(q)
                att.append( [time, yaw, pitch, roll] )
            else:
                import transformations
                (yaw, pitch, roll) = transformations.euler_from_quaternion(np.array([q[0], -q[1], -q[2], -q[3]]), 'szyx')
                att.append( [time, -yaw, -pitch, -roll] )
        
    pos = []
    with open(pos_path, 'rb') as f:
        reader = csv.DictReader(f)
        for row in reader:
            time = float(row['timestamp']) / 1000000.0
            lat = float(row['lat'])*d2r
            lon = float(row['lon'])*d2r
            alt = float(row['alt'])
            vn = float(row['vel_n'])
            ve = float(row['vel_e'])
            vd = float(row['vel_d'])
            pos.append( [time, lat, lon, alt, vn, ve, vd] )
    pos_array = np.array(pos)
    
    lat_interp = interpolate.interp1d(pos_array[:,0], pos_array[:,1],
                                      bounds_error=False,
                                      fill_value='extrapolate')
    lon_interp = interpolate.interp1d(pos_array[:,0], pos_array[:,2],
                                      bounds_error=False,
                                      fill_value='extrapolate')
    alt_interp = interpolate.interp1d(pos_array[:,0], pos_array[:,3],
                                      bounds_error=False,
                                      fill_value='extrapolate')
    vn_interp = interpolate.interp1d(pos_array[:,0], pos_array[:,4],
                                     bounds_error=False, fill_value=0.0)
    ve_interp = interpolate.interp1d(pos_array[:,0], pos_array[:,5],
                                     bounds_error=False, fill_value=0.0)
    vd_interp = interpolate.interp1d(pos_array[:,0], pos_array[:,6],
                                     bounds_error=False, fill_value=0.0)

    result['filter'] = []
    for a in att:
        nav = NAVdata()
        nav.time = a[0]
        nav.lat = float(lat_interp(nav.time))
        nav.lon = float(lon_interp(nav.time))
        nav.alt = float(alt_interp(nav.time))
        nav.vn = float(vn_interp(nav.time))
        nav.ve = float(ve_interp(nav.time))
        nav.vd = float(vd_interp(nav.time))
        nav.phi = a[3]
        nav.the = a[2]
        nav.psi = a[1]
        result['filter'].append(nav)

    #result['pilot'] = []
    #result['act'] = []
    #result['ap'] = []

    return result
