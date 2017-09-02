# load umn .mat file data format

# MAT_FILENAME = 'thor_flight75_WaypointTracker_150squareWaypointNew_2012_10_10.mat'

FLAG_UNBIASED_IMU = False             # Choose if accel/gyro should be bias-free.

import math
import os, sys
join = os.path.join
import numpy as np
from scipy import io as sio

from nav.structs import IMUdata, GPSdata, Airdata, NAVdata

mps2kt = 1.94384
r2d = 180.0 / math.pi

class dict2struct():
  pass

def load(mat_filename):
    # Name of .mat file that exists in the directory defined above and
    # has the flight_data and flight_info structures
    filepath = mat_filename

    # Load Flight Data: ## IMPORTANT to have the .mat file in the
    # flight_data and flight_info structures for this function ##
    data = sio.loadmat(filepath, struct_as_record=False, squeeze_me=True)
    for keys in data:
        print keys
        
    # print 'Loaded Data Summary'
    # print '* File: %s' % filepath.split(os.path.sep)[-1]

    # # Fill in time data
    # t = flight_data.time
    # print 't:', t

    # # Magnetometer data - not used hence don't trust
    # hm  = np.vstack((flight_data.hx, -flight_data.hy, -flight_data.hz)).T

    # # Geri mag calibration
    # mag_affine = np.array(
    #     [[ 4.3374191736,  0.9527668819,  0.2106929615,  0.0649324241],
    #      [-3.7617930866,  6.2839014058, -3.5707398622,  2.1616165305],
    #      [-0.8688354599,  0.2205650877,  4.7466115481, -2.7041600008],
    #      [ 0.          ,  0.          ,  0.          ,  1.          ]]
    # )

    # # Populate IMU Data
    # imu = np.vstack((t, flight_data.p, flight_data.q, flight_data.r, 
    #                  flight_data.ax, flight_data.ay, flight_data.az,
    #                  hm[:,0], hm[:,1], hm[:,2])).T

    # # Note that accelerometer and gyro measurements logged by UAV
    # # after 11/17/2011 flight (seemingly, see 
    # # http://trac.umnaem.webfactional.com/wiki/FlightReports/2011_11_17)
    # # have the nav-estimated bias removed before datalogging. So to work with raw
    # # imu-data, we add back the on-board estimated biases.
    # if not FLAG_UNBIASED_IMU:
    #     try:
    #         imu[:, 1:4] += np.vstack((flight_data.p_bias, 
    #                                   flight_data.q_bias, 
    #                                   flight_data.r_bias)).T

    #         imu[:, 4:7] += np.vstack((flight_data.ax_bias,
    #                                   flight_data.ay_bias,
    #                                   flight_data.az_bias)).T
    #     except AttributeError:
    #         print('Note: On board estimated bias not found.')

    # # Air Data
    # ias = flight_data.ias # indicated airspeed (m/s)
    # h = flight_data.h

    # # Populate GPS sensor data
    # try:
    #     vn = flight_data.gps_vn
    # except:
    #     vn = flight_data.vn
    # try:
    #     ve = flight_data.gps_ve
    # except:
    #     ve = flight_data.ve
    # try:
    #     vd = flight_data.gps_vd
    # except:
    #     vd = flight_data.vd
    # lat = flight_data.lat
    # lon = flight_data.lon
    # alt = flight_data.alt

    # # kstart set to when the navigation filter used onboard the aircraft
    # # was initialized and this is accomplished by detecting when navlat is
    # # no longer 0.0. This choice of kstart will ensure the filter being
    # # tested is using the same initialization time step as the onboard
    # # filter allowing for apples to apples comparisons.
    # kstart = (abs(flight_data.navlat) > 0.0).tolist().index(True)
    # k = kstart
    # print('Initialized at Time: %.2f s (k=%i)' % (t[k], k))

    # # Set previous value of GPS altitude to 0.0. This will be used to
    # # trigger GPS newData flag which is commonly used in our
    # # navigation filters for deciding if the GPS data has been
    # # updated. However, in python we have no log of newData
    # # (typically). So a comparison of current GPS altitude to the
    # # previous epoch's GPS altitude is used to determine if GPS has
    # # been updated.
    # last_gps_alt = -9999.9

    # create data structures for ekf processing
    result = {}
    result['imu'] = []
    result['gps'] = []
    result['air'] = []
    result['filter'] = []
    
    last_gps_lon = -9999.0
    last_gps_lat = -9999.0
    
    for i in range( len(data['Time_s']) ):
        # p, q, r = imu[k, 1:4]
        # ax, ay, az = imu[k, 4:7]
        # hx, hy, hz = imu[k, 7:10]
        
        s = [ data['IMU_Hx_uT'][i],
              data['IMU_Hy_uT'][i],
              data['IMU_Hz_uT'][i],
              1.0 ]
        # hf = np.dot(mag_affine, s) # apply mag calibration
        hf = s

        imu_pt = IMUdata()
        imu_pt.time = data['Time_s'][i]
        imu_pt.p = data['IMU_Gx_rads'][i]
        imu_pt.q = data['IMU_Gy_rads'][i]
        imu_pt.r = data['IMU_Gz_rads'][i]
        imu_pt.ax = data['IMU_Ax_mss'][i]
        imu_pt.ay = data['IMU_Ay_mss'][i]
        imu_pt.az = data['IMU_Az_mss'][i]
        imu_pt.hx = float(hf[0])
        imu_pt.hy = float(hf[1])
        imu_pt.hz = float(hf[2])
        imu_pt.temp = data['IMU_Temp_C'][i]
        result['imu'].append(imu_pt)

        lat = data['GPS_Latitude'][i] * r2d
        lon = data['GPS_Longitude'][i] * r2d
        #print lon,lat
        if abs(lat - last_gps_lat) > 0.0000000001 or abs(lon - last_gps_lon) > 0.0000000000001:
            last_gps_lat = lat
            last_gps_lon = lon
            gps_pt = GPSdata()
            gps_pt.time = data['Time_s'][i]
            #gps_pt.status = something
            gps_pt.unix_sec = data['Time_s'][i] # hack an incrementing time here
            gps_pt.lat = lat
            gps_pt.lon = lon
            gps_pt.alt = data['GPS_Altitude'][i]
            gps_pt.vn = data['GPS_North_Velocity'][i]
            gps_pt.ve = data['GPS_East_Velocity'][i]
            gps_pt.vd = data['GPS_Down_Velocity'][i]
            gps_pt.sats = int(data['GPS_NumberSatellites'][i])
            result['gps'].append(gps_pt)
            
        pitot_calibrate = 1.0
        diff_pa = data['Pitot_DiffPressure_Pressure_Pa'][i]
        if diff_pa < 0:
            diff_pa = 0.0
        static_pa = data['Pitot_StaticPressure_Pressure_Pa'][i]
        airspeed_mps = math.sqrt( 2*diff_pa / 1.225 ) * pitot_calibrate;
        P0 = 1013.25
        T = 15.0
        P = static_pa / 100.0   # convert to mbar
        tmp1 = math.pow((P0/P), 1.0/5.257) - 1.0
        alt_m = (tmp1 * (T + 273.15)) / 0.0065
        
        air_pt = Airdata()
        air_pt.time = data['Time_s'][i]
        air_pt.airspeed = airspeed_mps * mps2kt
        air_pt.altitude = alt_m
        result['air'].append(air_pt)
        
        # nav = NAVdata()
        # nav.time = float(t[k])
        # nav.lat = float(flight_data.navlat[k])
        # nav.lon = float(flight_data.navlon[k])
        # nav.alt = float(flight_data.navalt[k])
        # nav.vn = float(flight_data.navvn[k])
        # nav.ve = float(flight_data.navve[k])
        # nav.vd = float(flight_data.navvd[k])
        # nav.phi = float(flight_data.phi[k])
        # nav.the = float(flight_data.theta[k])
        # nav.psi = float(flight_data.psi[k])
        # result['filter'].append(nav)

    dir = os.path.dirname(mat_filename)
    print 'dir:', dir
    
    filename = os.path.join(dir, 'imu-0.txt')
    f = open(filename, 'w')
    for imupt in result['imu']:
        line = [ '%.5f' % imupt.time, '%.4f' % imupt.p, '%.4f' % imupt.q, '%.4f' % imupt.r, '%.4f' % imupt.ax, '%.4f' % imupt.ay, '%.4f' % imupt.az, '%.4f' % imupt.hx, '%.4f' % imupt.hy, '%.4f' % imupt.hz, '%.4f' % imupt.temp, '0' ]
        f.write(','.join(line) + '\n')

    filename = os.path.join(dir, 'gps-0.txt')
    f = open(filename, 'w')
    for gpspt in result['gps']:
        line = [ '%.5f' % gpspt.time, '%.10f' % gpspt.lat, '%.10f' % gpspt.lon, '%.4f' % gpspt.alt, '%.4f' % gpspt.vn, '%.4f' % gpspt.ve, '%.4f' % gpspt.vd, '%.4f' % gpspt.time, '8', '0' ]
        f.write(','.join(line) + '\n')

    if 'filter' in result:
        filename = os.path.join(dir, 'filter-0.txt')
        f = open(filename, 'w')
        for filtpt in result['filter']:
            line = [ '%.5f' % filtpt.time, '%.10f' % filtpt.lat, '%.10f' % filtpt.lon, '%.4f' % filtpt.alt, '%.4f' % filtpt.vn, '%.4f' % filtpt.ve, '%.4f' % filtpt.vd, '%.4f' % (filtpt.phi*r2d), '%.4f' % (filtpt.the*r2d), '%.4f' % (filtpt.psi*r2d), '0' ]
            f.write(','.join(line) + '\n')

    return result
