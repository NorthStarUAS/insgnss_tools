# load a px4_sdlog2 csv file

import csv
import math
import numpy as np
import re

from nav.structs import IMUdata, GPSdata, Airdata, NAVdata

# empty classes we'll fill in with data members
class Controldata: pass
class APdata: pass

d2r = math.pi / 180.0
r2d = 180.0/ math.pi
mps2kt = 1.94384
m2ft = 1.0 / 0.3048

# convert value (string) to float, check for '' and return 0.0
def my_float(value):
    if value != '':
        return float(value)
    else:
        return 0.0
    
def load(csv_file):
    result = {}
    result['imu'] = []
    result['gps'] = []
    result['air'] = []
    result['filter'] = []
    #result['pilot'] = []
    result['act'] = []
    result['ap'] = []

    last_gps_time = -1.0
    with open(csv_file, 'rb') as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row['TIME_StartTime'] == '':
                # empty time
                continue

            imu = IMUdata()
            imu.time = float(row['TIME_StartTime']) / 1000000.0
            imu.p = float(row['IMU_GyroX'])
            imu.q = float(row['IMU_GyroY'])
            imu.r = float(row['IMU_GyroZ'])
            imu.ax = float(row['IMU_AccX'])
            imu.ay = float(row['IMU_AccY'])
            imu.az = float(row['IMU_AccZ'])
            hx = float(row['IMU_MagX'])
            hy = float(row['IMU_MagY'])
            hz = float(row['IMU_MagZ'])
            hf = np.array( [hx, hy, hz] )
            norm = np.linalg.norm(hf)
            hf /= norm
            imu.hx = hf[0]
            imu.hy = hf[1]
            imu.hz = hf[2]
            #print imu.hx, imu.hy, imu.hz
            imu.temp = 15.0
            result['imu'].append( imu )

            if row['GPS_GPSTime'] != '':
                gps = GPSdata()
                gps.time = imu.time
                gps.unix_sec = float(row['GPS_GPSTime']) / 1000000.0
                gps.lat = float(row['GPS_Lat'])
                gps.lon = float(row['GPS_Lon'])
                gps.alt = float(row['GPS_Alt'])
                gps.vn = float(row['GPS_VelN'])
                gps.ve = float(row['GPS_VelE'])
                gps.vd = float(row['GPS_VelD'])
                gps.sats = int(row['GPS_nSat'])
                if gps.sats >= 5 and gps.unix_sec > last_gps_time:
                    result['gps'].append(gps)
                last_gps_time = gps.unix_sec

            if row['SENS_BaroPres'] != '':
                air = Airdata()
                air.time = imu.time
                air.static_press = float(row['SENS_BaroPres'])
                if 'AIR1_DiffPres' in row and row['AIR1_DiffPres'] != '':
                    air.diff_press = float(row['AIR1_DiffPres'])
                else:
                    air.diff_press = 0.0
                if 'AIRS_Temp' in row and row['AIRS_Temp'] != '':
                    air.temp = float(row['AIRS_Temp'])
                else:
                    air.temp = 0.0
                if 'AIRS_IAS' in row and row['AIRS_IAS'] != '':
                    air.airspeed = float(row['AIRS_IAS']) * mps2kt
                else:
                    air.airspeed = 0.0
                if 'AIR1_BaroAlt' in row and row['AIR1_BaroAlt'] != '':
                    air.alt_press = float(row['AIR1_BaroAlt'])
                else:
                    air.alt_press = 0.0
                if 'GPS_Alt' in row and row['GPS_Alt'] != '':
                    air.alt_true = float(row['GPS_Alt'])
                else:
                    air.alt_true = 0.0
                result['air'].append( air )

            if row['GPOS_Lat'] != '':
                nav = NAVdata()
                nav.time = imu.time
                nav.lat = float(row['GPOS_Lat'])*d2r
                nav.lon = float(row['GPOS_Lon'])*d2r
                nav.alt = float(row['GPOS_Alt'])
                nav.vn = float(row['GPOS_VelN'])
                nav.ve = float(row['GPOS_VelE'])
                nav.vd = float(row['GPOS_VelD'])
                nav.phi = float(row['ATT_Roll'])
                nav.the = float(row['ATT_Pitch'])
                psi = float(row['ATT_Yaw'])
                if psi > 180.0:
                    psi = psi - 360.0
                if psi < -180.0:
                    psi = psi + 360.0
                nav.psi = psi
                result['filter'].append(nav)

            if row['GPSP_Alt'] != '':
                ap = APdata()
                ap.time = imu.time
                ap.hdg = my_float(row['ATSP_YawSP']) * r2d
                ap.roll = my_float(row['ATSP_RollSP']) * r2d
                ap.alt = my_float(row['GPSP_Alt']) * m2ft
                ap.pitch = my_float(row['ATSP_PitchSP']) * r2d
                ap.speed = my_float(row['TECS_AsSP']) * mps2kt
                result['ap'].append(ap)
                
            if row['OUT0_Out0'] != '':
                act = Controldata()
                ch0 = (float(row['OUT0_Out0']) - 1500) / 500
                ch1 = (float(row['OUT0_Out1']) - 1500) / 500 
                ch2 = (float(row['OUT0_Out2']) - 1000) / 1000
                ch3 = (float(row['OUT0_Out3']) - 1500) / 500
                #print ch0, ch1, ch2, ch3
                act.time = imu.time
                act.aileron = (ch0 - ch1)
                act.elevator = -(ch0 + ch1)
                act.throttle = ch2
                act.rudder = ch3
                act.gear = 0.0
                act.flaps = 0.0
                act.aux1 = 0.0
                act.auto_manual = 0.0
                result['act'].append(act)

    return result
