# load umn .mat file data format

# MAT_FILENAME = 'thor_flight75_WaypointTracker_150squareWaypointNew_2012_10_10.mat'

FLAG_UNBIASED_IMU = False             # Choose if accel/gyro should be bias-free.

import os, sys
join = os.path.join
import numpy as np
from scipy import io as sio

import pydefs

def load(mat_filename):
    # Name of .mat file that exists in the directory defined above and
    # has the flight_data and flight_info structures
    filepath = mat_filename

    # Load Flight Data: ## IMPORTANT to have the .mat file in the
    # flight_data and flight_info structures for this function ##
    data = sio.loadmat(filepath, struct_as_record=False, squeeze_me=True)
    print 'Loaded Data Summary'
    print '* File: %s' % filepath.split(os.path.sep)[-1]
    try:
        flight_data, flight_info = data['flight_data'], data['flight_info']
        print('* Date: %s' % flight_info.date)
        print('* Aircraft: %s' % flight_info.aircraft)
    except KeyError:
        print 'KeyError'
        # Convert from Python dictionary to struct-like before
        flight_data = dict2struct()
        for k in data:
            exec("flight_data.%s = data['%s']" % (k, k))
    del(data)

    # Add both names for pitch: the and theta
    try:
        flight_data.theta = flight_data.the
    except AttributeError:
        pass

    # Fill in time data
    t = flight_data.time

    # Magnetometer data - not used hence don't trust
    hm  = np.vstack((flight_data.hx, -flight_data.hy, -flight_data.hz)).T

    # Populate IMU Data
    imu = np.vstack((t, flight_data.p, flight_data.q, flight_data.r, 
                     flight_data.ax, flight_data.ay, flight_data.az,
                     hm[:,0], hm[:,1], hm[:,2])).T

    # Note that accelerometer and gyro measurements logged by UAV
    # after 11/17/2011 flight (seemingly, see 
    # http://trac.umnaem.webfactional.com/wiki/FlightReports/2011_11_17)
    # have the nav-estimated bias removed before datalogging. So to work with raw
    # imu-data, we add back the on-board estimated biases.
    if not FLAG_UNBIASED_IMU:
        try:
            imu[:, 1:4] += np.vstack((flight_data.p_bias, 
                                      flight_data.q_bias, 
                                      flight_data.r_bias)).T

            imu[:, 4:7] += np.vstack((flight_data.ax_bias,
                                      flight_data.ay_bias,
                                      flight_data.az_bias)).T
        except AttributeError:
            print('Note: On board estimated bias not found.')

    # Air Data
    ias = flight_data.ias # indicated airspeed (m/s)
    h = flight_data.h

    # Populate GPS sensor data
    try:
        vn = flight_data.gps_vn
    except:
        vn = flight_data.vn
    try:
        ve = flight_data.gps_ve
    except:
        ve = flight_data.ve
    try:
        vd = flight_data.gps_vd
    except:
        vd = flight_data.vd
    lat = flight_data.lat
    lon = flight_data.lon
    alt = flight_data.alt

    # kstart set to when the navigation filter used onboard the aircraft
    # was initialized and this is accomplished by detecting when navlat is
    # no longer 0.0. This choice of kstart will ensure the filter being
    # tested is using the same initialization time step as the onboard
    # filter allowing for apples to apples comparisons.
    kstart = (abs(flight_data.navlat) > 0.0).tolist().index(True)
    k = kstart
    print('Initialized at Time: %.2f s (k=%i)' % (t[k], k))

    # Set previous value of GPS altitude to 0.0. This will be used to
    # trigger GPS newData flag which is commonly used in our
    # navigation filters for deciding if the GPS data has been
    # updated. However, in python we have no log of newData
    # (typically). So a comparison of current GPS altitude to the
    # previous epoch's GPS altitude is used to determine if GPS has
    # been updated.
    last_gps_alt = -9999.9

    # create data structures for ekf processing
    imu_data = []
    gps_data = []
    filter_data = []
    
    k = kstart
    while k < len(t):
        p, q, r = imu[k, 1:4]
        ax, ay, az = imu[k, 4:7]
        hx, hy, hz = imu[k, 7:10]
        imu_pt = pydefs.IMU(t[k], 0, p, q, r, ax, ay, az, hx, hy, hz, 15.0)
        imu_data.append(imu_pt)

        if abs(alt[k] - last_gps_alt) > 0.0001:
            last_gps_alt = alt[k]
            gps_pt = pydefs.GPS(t[k], 0, t[k], lat[k], lon[k], alt[k], vn[k], ve[k], vd[k])
            gps_data.append(gps_pt)

        fd_pt = pydefs.FILTER(t[k], flight_data.navlat[k], flight_data.navlon[k], flight_data.navalt[k], flight_data.navvn[k], flight_data.navve[k], flight_data.navvd[k], flight_data.phi[k], flight_data.theta[k], flight_data.psi[k])
        filter_data.append(fd_pt)

        k += 1
    print "imu records:", len(imu_data)
    print "gps records:", len(gps_data)

    return imu_data, gps_data, filter_data
