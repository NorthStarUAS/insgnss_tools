#!/usr/bin/python3

"""umn3_add_postnav.py

Run a flight data set through a filter and write the post processed nav
solution back to the original h5 file under a new tree
Author: Curtis L. Olson, University of Minnesota
"""

import argparse
import h5py
import math
from matplotlib import pyplot as plt
import numpy as np
import os
import pandas as pd
from tqdm import tqdm

from aurauas_flightdata import flight_loader, flight_interp

import nav_wrapper

parser = argparse.ArgumentParser(description='nav filter')
parser.add_argument('--flight', required=True, help='flight data log')
parser.add_argument('--gps-lag-sec', type=float, default=0.2,
                    help='gps lag (sec)')
args = parser.parse_args()

r2d = 180.0 / math.pi
d2r = math.pi / 180.0

path = args.flight
if 'recalibrate' in args:
    recal_file = args.recalibrate
else:
    recal_file = None
data, flight_format = flight_loader.load(path)

if flight_format != 'umn3':
    print("This script only runs with UMN Goldy3 HDF5 format flight log files.")
    print("Aborting ...")
    quit()

print("Creating interpolation structures..")
interp = flight_interp.InterpolationGroup(data)

print("imu records:", len(data['imu']))
imu_dt = (data['imu'][-1]['time'] - data['imu'][1]['time']) \
    / float(len(data['imu'])-1)
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

# Default config
config = {
    'sig_w_ax': 0.05,
    'sig_w_ay': 0.05,
    'sig_w_az': 0.05,
    'sig_w_gx': 0.00175,
    'sig_w_gy': 0.00175,
    'sig_w_gz': 0.00175,
    'sig_a_d': 0.02,
    'tau_a': 100.0,
    'sig_g_d': 0.0005,
    'tau_g': 50.0,
    'sig_gps_p_ne': 2.0,
    'sig_gps_p_d': 6.0,
    'sig_gps_v_ne': 0.5,
    'sig_gps_v_d': 3.0,
    'sig_mag': 1.0
}

filter = nav_wrapper.filter(nav='EKF15',
                            gps_lag_sec=args.gps_lag_sec,
                            imu_dt=imu_dt)
filter.set_config(config)

print("Running nav filter:")
filter_init = False
nav = []

iter = flight_interp.IterateGroup(data)
for i in tqdm(range(iter.size())):
    record = iter.next()
    imupt = record['imu']
    if 'gps' in record:
        gpspt = record['gps']
    else:
        gpspt = {}

    # Init the filter if we have gps data (and haven't already init'd)
    if not filter_init and 'time' in gpspt:
        # print("init:", imupt['time'], gpspt['time'])
        navpt = filter.init(imupt, gpspt)
        filter_init = True
    elif filter_init:
        navpt = filter.update(imupt, gpspt)

    # Store the desired results obtained from the compiled test
    # navigation filter and the baseline filter
    if filter_init:
        nav.append(navpt)

# proper cleanup
filter.close()

def add_or_overwrite(h5file, key, pddata):
    if key in h5file:
        del h5file[key]
    h5file.create_dataset(key, (len(pddata), 1), data=pddata.values,
                          compression="gzip", compression_opts=9)

# File Rewriting Section
h5file = h5py.File(path, 'r+')      # read/write
df1_nav = pd.DataFrame(nav)
df1_nav.set_index('time', inplace=True, drop=False)
base='/Sensor-Processing/PostProcess/INS/'
add_or_overwrite(h5file, base + 'Latitude_rad', df1_nav['lat'])
add_or_overwrite(h5file, base + 'Longitude_rad', df1_nav['lon'])
add_or_overwrite(h5file, base + 'Altitude_m', df1_nav['alt'])
add_or_overwrite(h5file, base + 'NorthVelocity_ms', df1_nav['vn'])
add_or_overwrite(h5file, base + 'EastVelocity_ms', df1_nav['ve'])
add_or_overwrite(h5file, base + 'DownVelocity_ms', df1_nav['vd'])
add_or_overwrite(h5file, base + 'Roll_rad', df1_nav['phi'])
add_or_overwrite(h5file, base + 'Pitch_rad', df1_nav['the'])
add_or_overwrite(h5file, base + 'Heading_rad', df1_nav['psi'])
add_or_overwrite(h5file, base + 'GyroXBias_rads', df1_nav['gbx'])
add_or_overwrite(h5file, base + 'GyroYBias_rads', df1_nav['gby'])
add_or_overwrite(h5file, base + 'GyroZBias_rads', df1_nav['gbz'])
add_or_overwrite(h5file, base + 'AccelXBias_mss', df1_nav['abx'])
add_or_overwrite(h5file, base + 'AccelYBias_mss', df1_nav['aby'])
add_or_overwrite(h5file, base + 'AccelZBias_mss', df1_nav['abz'])

if False:
    # special favor for Huginn Flight #02
    print("NOTICE: Also overwriting ublox/DownVelocity_ms for Huginn Flight #02")
    df0_gps = pd.DataFrame(data['gps'])
    add_or_overwrite(h5file, '/Sensors/uBlox/DownVelocity_ms', df0_gps['vd'])
    
h5file.close()

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

att_ax[0].set_title("Attitude Angles")
att_ax[0].set_ylabel('Roll (deg)', weight='bold')
att_ax[0].plot(r2d(df1_nav['phi']), label=filter.name)
att_ax[0].grid()

att_ax[1].set_ylabel('Pitch (deg)', weight='bold')
att_ax[1].plot(r2d(df1_nav['the']), label=filter.name)
att_ax[1].grid()

att_ax[2].set_ylabel('Yaw (deg)', weight='bold')
att_ax[2].plot(r2d(df1_nav['psi']), label=filter.name)
att_ax[2].set_xlabel('Time (sec)', weight='bold')
att_ax[2].grid()
att_ax[2].legend(loc=1)

# Velocities
fig, [ax1, ax2, ax3] = plt.subplots(3,1, sharex=True)

# vn Plot
ax1.set_title("NED Velocities")
ax1.set_ylabel('vn (mps)', weight='bold')
ax1.plot(df0_gps['vn'], '-*', label='GPS Sensor', c='g', alpha=.5)
ax1.plot(df1_nav['vn'], label=filter.name)
ax1.grid()

# ve Plot
ax2.set_ylabel('ve (mps)', weight='bold')
ax2.plot(df0_gps['ve'], '-*', label='GPS Sensor', c='g', alpha=.5)
ax2.plot(df1_nav['ve'], label=filter.name)
ax2.grid()

# vd Plot
ax3.set_ylabel('vd (mps)', weight='bold')
ax3.plot(df0_gps['vd'], '-*', label='GPS Sensor', c='g', alpha=.5)
ax3.plot(df1_nav['vd'], label=filter.name)
ax3.set_xlabel('TIME (SECONDS)', weight='bold')
ax3.grid()
ax3.legend(loc=0)

# Altitude
plt.figure()
plt.title('Altitude')
plt.plot(df0_gps['alt'], '-*', label='GPS Sensor', c='g', alpha=.5)
plt.plot(df1_nav['alt'], label=filter.name)
plt.ylabel('Altitude (m)', weight='bold')
plt.legend(loc=0)
plt.grid()

# Top down flight track plot
plt.figure()
plt.title('Ground track')
plt.ylabel('Latitude (degrees)', weight='bold')
plt.xlabel('Longitude (degrees)', weight='bold')
plt.plot(df0_gps['lon'], df0_gps['lat'], '*', label='GPS Sensor', c='g', alpha=.5)
plt.plot(r2d(df1_nav['lon']), r2d(df1_nav['lat']), label=filter.name)
plt.grid()
plt.legend(loc=0)

# Biases
bias_fig, bias_ax = plt.subplots(3,2, sharex=True)

# Gyro Biases
bias_ax[0,0].set_title("IMU Biases")
bias_ax[0,0].set_ylabel('p (deg/s)', weight='bold')
bias_ax[0,0].plot(r2d(df1_nav['gbx']), label=filter.name)
bias_ax[0,0].set_xlabel('Time (secs)', weight='bold')
bias_ax[0,0].grid()

bias_ax[1,0].set_ylabel('q (deg/s)', weight='bold')
bias_ax[1,0].plot(r2d(df1_nav['gby']), label=filter.name)
bias_ax[1,0].set_xlabel('Time (secs)', weight='bold')
bias_ax[1,0].grid()

bias_ax[2,0].set_ylabel('r (deg/s)', weight='bold')
bias_ax[2,0].plot(r2d(df1_nav['gbz']), label=filter.name)
bias_ax[2,0].set_xlabel('Time (secs)', weight='bold')
bias_ax[2,0].grid()

# Accel Biases
bias_ax[0,1].set_title("Accel Biases")
bias_ax[0,1].set_ylabel('ax (m/s^2)', weight='bold')
bias_ax[0,1].plot(df1_nav['abx'], label=filter.name)
bias_ax[0,1].set_xlabel('Time (secs)', weight='bold')
bias_ax[0,1].grid()

bias_ax[1,1].set_ylabel('ay (m/s^2)', weight='bold')
bias_ax[1,1].plot(df1_nav['aby'], label=filter.name)
bias_ax[1,1].set_xlabel('Time (secs)', weight='bold')
bias_ax[1,1].grid()

bias_ax[2,1].set_ylabel('az (m/s^2)', weight='bold')
bias_ax[2,1].plot(df1_nav['abz'], label=filter.name)
bias_ax[2,1].set_xlabel('Time (secs)', weight='bold')
bias_ax[2,1].grid()
bias_ax[2,1].legend(loc=1)

plt.show()
