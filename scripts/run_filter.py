#!/usr/bin/python3

"""run_filter.py

Run a flight data set through a filter and output a few simple plots
Author: Curtis L. Olson, University of Minnesota
"""

import argparse
import math
from matplotlib import pyplot as plt
import numpy as np
import os
import pandas as pd
import time
from tqdm import tqdm

from aurauas.flightdata import flight_loader, flight_interp

# filter interfaces
import navigation.structs
import nav_ekf15
import nav_ekf15_mag
import nav_openloop

parser = argparse.ArgumentParser(description='nav filter')
parser.add_argument('--flight', required=True, help='flight data log')
parser.add_argument('--gps-lag-sec', type=float, default=0.2,
                    help='gps lag (sec)')
args = parser.parse_args()

r2d = 180.0 / math.pi
d2r = math.pi / 180.0

def run_filter(filter, data):
    results = []
    
    # Using while loop starting at k (set to kstart) and going to end
    # of .mat file
    run_start = time.time()
    gpspt = {}
    airpt = {}
    pilotpt = {}
    actpt = {}
    healthpt = {}
    filter_init = False

    iter = flight_interp.IterateGroup(data)
    for i in tqdm(range(iter.size())):
        record = iter.next()
        imupt = record['imu']
        if 'gps' in record:
            gpspt = record['gps']
            gpspt['newData'] = True
        else:
            gpspt = {}
            gpspt['newData'] = False
        if 'air' in record:
            airpt = record['air']
        else:
            airpt = {}
        if 'nav' in record:
            filterpt = record['nav']
        else:
            filterpt = {}
        if 'pilot' in record:
            pilotpt = record['pilot']
        else:
            pilotpt = {}
        if 'act' in record:
            actpt = record['act']
        else:
            actpt = {}
        if 'health' in record:
            healthpt = record['health']
        else:
            healthpt = {}

        # Init the filter if we have gps data (and haven't already init'd)
        if not filter_init and 'time' in gpspt:
            # print("init:", imupt['time'], gpspt['time'])
            navpt = filter.init(imupt, gpspt, filterpt)
            filter_init = True
        elif filter_init:
            navpt = filter.update(imupt, gpspt, filterpt)

        # Store the desired results obtained from the compiled test
        # navigation filter and the baseline filter
        if filter_init:
            results.append(navpt)
            
    # proper cleanup
    filter.close()
    run_end = time.time()
    elapsed_sec = run_end - run_start
    return results, elapsed_sec

path = args.flight
if 'recalibrate' in args:
    recal_file = args.recalibrate
else:
    recal_file = None
data, flight_format = flight_loader.load(path, recal_file)

print("Creating interpolation structures..")
interp = flight_interp.InterpolationGroup(data)

print("imu records:", len(data['imu']))
imu_dt = (data['imu'][-1]['time'] - data['imu'][0]['time']) \
    / float(len(data['imu']))
print("imu dt: %.3f" % imu_dt)
print("gps records:", len(data['gps']))
if 'air' in data:
    print("airdata records:", len(data['air']))
if 'filter' in data:
    print("filter records:", len(data['filter']))
if 'pilot' in data:
    print("pilot records:", len(data['pilot']))
if 'act' in data:
    print("act records:", len(data['act']))
if len(data['imu']) == 0 and len(data['gps']) == 0:
    print("not enough data loaded to continue.")
    quit()

filter1 = nav_ekf15.filter(gps_lag_sec=args.gps_lag_sec, imu_dt=imu_dt)

# Default config
config = navigation.structs.NAVconfig()
config.sig_w_ax = 0.05
config.sig_w_ay = 0.05
config.sig_w_az = 0.05
config.sig_w_gx = 0.00175
config.sig_w_gy = 0.00175
config.sig_w_gz = 0.00175
config.sig_a_d  = 0.02
config.tau_a    = 100.0
config.sig_g_d  = 0.0005
config.tau_g    = 50.0
config.sig_gps_p_ne = 2.0
config.sig_gps_p_d  = 6.0
config.sig_gps_v_ne = 0.5
config.sig_gps_v_d  = 4.0
config.sig_mag      = 1.0
filter1.set_config(config)

# almost no trust in IMU ...
# config = navigation.structs.NAVconfig()
# config.sig_w_ax = 2.0
# config.sig_w_ay = 2.0
# config.sig_w_az = 2.0
# config.sig_w_gx = 0.1
# config.sig_w_gy = 0.1
# config.sig_w_gz = 0.1
# config.sig_a_d  = 0.1
# config.tau_a    = 100.0
# config.sig_g_d  = 0.00873
# config.tau_g    = 50.0
# config.sig_gps_p_ne = 3.0
# config.sig_gps_p_d  = 5.0
# config.sig_gps_v_ne = 0.5
# config.sig_gps_v_d  = 1.0
# config.sig_mag      = 0.2
# filter1.set_config(config)

# less than default trust in IMU ...
# config = navigation.structs.NAVconfig()
# config.sig_w_ax = 0.1
# config.sig_w_ay = 0.1
# config.sig_w_az = 0.1
# config.sig_w_gx = 0.003
# config.sig_w_gy = 0.003
# config.sig_w_gz = 0.003
# config.sig_a_d  = 0.1
# config.tau_a    = 100.0
# config.sig_g_d  = 0.00873
# config.tau_g    = 50.0
# config.sig_gps_p_ne = 3.0
# config.sig_gps_p_d  = 5.0
# config.sig_gps_v_ne = 0.5
# config.sig_gps_v_d  = 1.0
# config.sig_mag      = 0.2
# filter1.set_config(config)

# too high trust in IMU ...
# config = navigation.structs.NAVconfig()
# config.sig_w_ax = 0.02
# config.sig_w_ay = 0.02
# config.sig_w_az = 0.02
# config.sig_w_gx = 0.00175
# config.sig_w_gy = 0.00175
# config.sig_w_gz = 0.00175
# config.sig_a_d  = 0.1
# config.tau_a    = 100.0
# config.sig_g_d  = 0.00873
# config.tau_g    = 50.0
# config.sig_gps_p_ne = 15.0
# config.sig_gps_p_d  = 20.0
# config.sig_gps_v_ne = 2.0
# config.sig_gps_v_d  = 4.0
# config.sig_mag      = 0.3
# filter1.set_config(config)

nav, filter1_sec = run_filter(filter1, data)
print("filter1 time = %.4f" % filter1_sec)

# Plotting Section

plotname = os.path.basename(args.flight)    

df0_gps = pd.DataFrame(data['gps'])
df0_gps.set_index('time', inplace=True, drop=False)
df0_nav = pd.DataFrame(data['filter'])
df0_nav.set_index('time', inplace=True, drop=False)

df1_nav = pd.DataFrame(nav)
df1_nav.set_index('time', inplace=True, drop=False)

r2d = np.rad2deg

# Attitude
att_fig, att_ax = plt.subplots(3, 1, sharex=True)

att_ax[0].set_title("Attitude")
att_ax[0].set_ylabel('Roll (deg)', weight='bold')
att_ax[0].plot(r2d(df1_nav['phi']), label=filter1.name)
att_ax[0].grid()

att_ax[1].set_ylabel('Pitch (deg)', weight='bold')
att_ax[1].plot(r2d(df1_nav['the']), label=filter1.name)
att_ax[1].grid()

att_ax[2].set_ylabel('Yaw (deg)', weight='bold')
att_ax[2].plot(r2d(df1_nav['psi']), label=filter1.name)
att_ax[2].set_xlabel('Time (sec)', weight='bold')
att_ax[2].grid()
att_ax[2].legend(loc=1)

# Velocities
fig, [ax1, ax2, ax3] = plt.subplots(3,1, sharex=True)

# vn Plot
ax1.set_title("Velocity")
ax1.set_ylabel('vn (mps)', weight='bold')
ax1.plot(df0_gps['vn'], '-*', label='GPS Sensor', c='g', alpha=.5)
ax1.plot(df1_nav['vn'], label=filter1.name)
ax1.grid()

# ve Plot
ax2.set_ylabel('ve (mps)', weight='bold')
ax2.plot(df0_gps['ve'], '-*', label='GPS Sensor', c='g', alpha=.5)
ax2.plot(df1_nav['ve'], label=filter1.name)
ax2.grid()

# vd Plot
ax3.set_ylabel('vd (mps)', weight='bold')
ax3.plot(df0_gps['vd'], '-*', label='GPS Sensor', c='g', alpha=.5)
ax3.plot(df1_nav['vd'], label=filter1.name)
ax3.set_xlabel('TIME (SECONDS)', weight='bold')
ax3.grid()
ax3.legend(loc=0)

# Altitude
plt.figure()
plt.title('Altitude')
plt.plot(df0_gps['alt'], '-*', label='GPS Sensor', c='g', alpha=.5)
plt.plot(df1_nav['alt'], label=filter1.name)
plt.ylabel('Altitude (m)', weight='bold')
plt.legend(loc=0)
plt.grid()

if False:
    # Experimental map plot
    import cartopy.crs as ccrs
    import cartopy.io.img_tiles as cimgt
    w = df0_gps['lon'].max() - df0_gps['lon'].min()
    h = df0_gps['lat'].max() - df0_gps['lat'].min()

    request = cimgt.OSM()
    fig, ax = plt.subplots(subplot_kw=dict(projection=request.crs))
    # (xmin, xmax, ymin, ymax)
    extent = [ df0_gps['lon'].min() - 1.1*w,
               df0_gps['lon'].max() + 1.1*w,
               df0_gps['lat'].min() - 1.1*h,
               df0_gps['lat'].max() + 1.1*h ]
    ax.set_extent(extent)
    ax.add_image(request, 10)

    # do coordinate conversion of (x,y)
    xynps = ax.projection.transform_points(ccrs.Geodetic(),
                                           np.array(df0_gps['lon']),
                                           np.array(df0_gps['lat']))
    plt.plot(xynps[:,0], xynps[:,1])        

# Top down flight track plot
plt.figure()
plt.title('Ground track')
plt.ylabel('Latitude (degrees)', weight='bold')
plt.xlabel('Longitude (degrees)', weight='bold')
plt.plot(df0_gps['lon'], df0_gps['lat'], '*', label='GPS Sensor', c='g', alpha=.5)
plt.plot(r2d(df1_nav['lon']), r2d(df1_nav['lat']), label=filter1.name)
plt.grid()
plt.legend(loc=0)

# Biases
bias_fig, bias_ax = plt.subplots(3,2, sharex=True)

# Gyro Biases
bias_ax[0,0].set_title("IMU Biases")
bias_ax[0,0].set_ylabel('p (deg/s)', weight='bold')
bias_ax[0,0].plot(r2d(df1_nav['gbx']), label=filter1.name)
bias_ax[0,0].set_xlabel('Time (secs)', weight='bold')
bias_ax[0,0].grid()

bias_ax[1,0].set_ylabel('q (deg/s)', weight='bold')
bias_ax[1,0].plot(r2d(df1_nav['gby']), label=filter1.name)
bias_ax[1,0].set_xlabel('Time (secs)', weight='bold')
bias_ax[1,0].grid()

bias_ax[2,0].set_ylabel('r (deg/s)', weight='bold')
bias_ax[2,0].plot(r2d(df1_nav['gbz']), label=filter1.name)
bias_ax[2,0].set_xlabel('Time (secs)', weight='bold')
bias_ax[2,0].grid()

# Accel Biases
bias_ax[0,1].set_title("Accel Biases")
bias_ax[0,1].set_ylabel('ax (m/s^2)', weight='bold')
bias_ax[0,1].plot(df1_nav['abx'], label=filter1.name)
bias_ax[0,1].set_xlabel('Time (secs)', weight='bold')
bias_ax[0,1].grid()

bias_ax[1,1].set_ylabel('ay (m/s^2)', weight='bold')
bias_ax[1,1].plot(df1_nav['aby'], label=filter1.name)
bias_ax[1,1].set_xlabel('Time (secs)', weight='bold')
bias_ax[1,1].grid()

bias_ax[2,1].set_ylabel('az (m/s^2)', weight='bold')
bias_ax[2,1].plot(df1_nav['abz'], label=filter1.name)
bias_ax[2,1].set_xlabel('Time (secs)', weight='bold')
bias_ax[2,1].grid()
bias_ax[2,1].legend(loc=1)

plt.show()
