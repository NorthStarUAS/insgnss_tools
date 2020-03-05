#!/usr/bin/python3

"""run_filters.py

This script plays flight data through the selected navigation filters.
The filters are compiled as .so objects and wrapped for python with boost.

A set of customizable input flags are defined at the start of the script.

Initial revision: Hamid M.
Many updates: Curtis L. Olson
"""

import argparse
import math
from matplotlib import pyplot as plt
import numpy as np
import os
import pandas as pd
import time
from tqdm import tqdm

from aurauas_flightdata import flight_loader, flight_interp

# filter interfaces
import nav_wrapper

# support routines
import alpha_beta
import wind
import synth_asi
import battery

parser = argparse.ArgumentParser(description='nav filter')
parser.add_argument('flight', help='flight data log')
parser.add_argument('filter', metavar='filter', nargs='+',
                    choices=['EKF15', 'EKF15_mag', 'uNavINS', 'uNavINS_BFS'],
                    help='filters')
parser.add_argument('--gps-lag-sec', type=float, default=0.2,
                    help='gps lag (sec)')
parser.add_argument('--synthetic-airspeed', action='store_true', help='build synthetic airspeed estimator')
args = parser.parse_args()

# Select output plots
PLOT_SENSORS = True
PLOT_ATTITUDE = True
PLOT_VELOCITIES = True
PLOT_GROUNDTRACK = True
PLOT_ALTITUDE = True
PLOT_WIND = True
PLOT_SYNTH_ASI = False
PLOT_BIASES = True

r2d = 180.0 / math.pi
d2r = math.pi / 180.0
mps2kt = 1.94384
gps_settle_secs = 10.0

def run_filter(filter, data, call_init=True):
    results = []
    
    # Using while loop starting at k (set to kstart) and going to end
    # of .mat file
    run_start = time.time()
    if call_init:
        filter_init = False
    else:
        filter_init = True

    gps_init_sec = None
    gpspt = None
    iter = flight_interp.IterateGroup(data)
    for i in tqdm(range(iter.size())):
        record = iter.next()
        imupt = record['imu']
        if 'gps' in record:
            gpspt = record['gps']
            if gps_init_sec is None:
                gps_init_sec = gpspt['time']

        # if not inited or gps not yet reached it's settle time
        if gps_init_sec is None or gpspt['time'] < gps_init_sec + gps_settle_secs:
            continue

        navpt = filter.update(imupt, gpspt)

        # Store the desired results obtained from the compiled test
        # navigation filter and the baseline filter
        results.append(navpt)
    
    run_end = time.time()
    elapsed_sec = run_end - run_start
    return results, elapsed_sec

path = args.flight
data, flight_format = flight_loader.load(path)

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

plotname = os.path.basename(args.flight)    

if False:
    # quick hack estimate gyro biases (would be better to only do this
    # while they are stable or at least not flying
    print('p mean (dps):', data['imu'].loc[:,'p'].mean()*r2d)
    print('q mean (dps):', data['imu'].loc[:,'q'].mean()*r2d)
    print('r mean (dps):', data['imu'].loc[:,'r'].mean()*r2d)

if False:
    # quick rough hack at a magnetometer calibration
    x_min = 1000000.0
    y_min = 1000000.0
    z_min = 1000000.0
    x_max = -1000000.0
    y_max = -1000000.0
    z_max = -1000000.0
    for imu in data['imu']:
        if imu.hx < x_min: x_min = imu.hx
        if imu.hy < y_min: y_min = imu.hy
        if imu.hz < z_min: z_min = imu.hz
        if imu.hx > x_max: x_max = imu.hx
        if imu.hy > y_max: y_max = imu.hy
        if imu.hz > z_max: z_max = imu.hz
    print("x:", x_min, x_max)
    print("y:", y_min, y_max)
    print("z:", z_min, z_max)
    dx = x_max - x_min
    dy = y_max - y_min
    dz = z_max - z_min
    cx = (x_min + x_max) * 0.5
    cy = (y_min + y_max) * 0.5
    cz = (z_min + z_max) * 0.5
    for imu in data['imu']:
        imu.hx = ((imu.hx - x_min) / dx) * 2.0 - 1.0
        imu.hy = ((imu.hy - y_min) / dy) * 2.0 - 1.0
        imu.hz = ((imu.hz - z_min) / dz) * 2.0 - 1.0
        
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
    'sig_mag': 0.3
}

config2 = dict(config)
# more trust in gps (sentera camera, low change in velocity?)
config2['sig_gps_p_ne'] = 2.0
config2['sig_gps_p_d'] = 4.0
config2['sig_gps_v_ne'] = 0.3
config2['sig_gps_v_d'] = 2.0

# almost no trust in IMU ...
# config': navigation.structs.NAVconfig()
# 'sig_w_ax': 2.0,
# 'sig_w_ay': 2.0,
# 'sig_w_az': 2.0,
# 'sig_w_gx': 0.1,
# 'sig_w_gy': 0.1,
# 'sig_w_gz': 0.1,
# 'sig_a_d': 0.1,
# 'tau_a': 100.0,
# 'sig_g_d': 0.00873,
# 'tau_g': 50.0,
# 'sig_gps_p_ne': 3.0,
# 'sig_gps_p_d': 5.0,
# 'sig_gps_v_ne': 0.5,
# 'sig_gps_v_d': 1.0,
# 'sig_mag': 0.2,
# filter2.set_config(config)

# less than default trust in IMU ...
# config': navigation.structs.NAVconfig()
# 'sig_w_ax': 0.1,
# 'sig_w_ay': 0.1
# 'sig_w_az': 0.1,
# 'sig_w_gx': 0.003,
# 'sig_w_gy': 0.003,
# 'sig_w_gz': 0.003,
# 'sig_a_d': 0.1,
# 'tau_a': 100.0,
# 'sig_g_d': 0.00873,
# 'tau_g': 50.0,
# 'sig_gps_p_ne': 3.0,
# 'sig_gps_p_d': 5.0,
# 'sig_gps_v_ne': 0.5,
# 'sig_gps_v_d': 1.0,
# 'sig_mag': 0.2,
# filter1.set_config(config)
# filter2.set_config(config)

# too high trust in IMU ...
# config': navigation.structs.NAVconfig()
# 'sig_w_ax': 0.02,
# 'sig_w_ay': 0.02,
# 'sig_w_az': 0.02,
# 'sig_w_gx': 0.00175,
# 'sig_w_gy': 0.00175,
# 'sig_w_gz': 0.00175,
# 'sig_a_d': 0.1,
# 'tau_a': 100.0,
# 'sig_g_d': 0.00873,
# 'tau_g': 50.0,
# 'sig_gps_p_ne': 15.0,
# 'sig_gps_p_d': 20.0,
# 'sig_gps_v_ne': 2.0,
# 'sig_gps_v_d': 4.0,
# 'sig_mag': 0.3,
# filter1.set_config(config)
nav_list = []
time_list = []

for name in args.filter:
    filter = nav_wrapper.filter(nav=name,
                                gps_lag_sec=args.gps_lag_sec,
                                imu_dt=imu_dt)
    filter.set_config(config)
    nav, filter_sec = run_filter(filter, data)
    nav_list.append(nav)
    time_list.append(filter_sec)

for i in range(len(args.filter)):
    print("filter:", args.filter[i], "= %.3f sec" % time_list[i])

if flight_format == 'aura_csv' or flight_format == 'aura_txt':
    filter_post = os.path.join(args.flight, "filter-post.txt")
    #flight_loader.save(filter_post, nav1)

if flight_format == 'umn3':
    basedir = os.path.dirname(args.flight)
    filter_post = os.path.join(basedir, "filter-post.csv")
    #flight_loader.save(filter_post, nav1)

if flight_format == 'px4_ulog':
    filter_post = args.flight + "_filter_post.txt"
    flight_loader.save(filter_post, nav1)
    
if flight_format == 'sentera':
    filter_post = args.flight + "_filter_post.txt"
    flight_loader.save(filter_post, nav1)

if True:
    w = wind.Wind()
    winds = w.estimate(data, None)

if False:
    print("Estimating alpha/beta (experimental):")
    navpt = {}
    airpt = {}
    iter = flight_interp.IterateGroup(data)
    for i in tqdm(range(iter.size())):
        record = iter.next()
        if len(record):
            imupt = record['imu']
            if 'air' in record:
                airpt = record['air']
                airspeed = airpt['airspeed']
            else:
                airspeed = 0
            if 'filter' in record:
                navpt = record['filter']
            if airspeed > 10.0 and 'psi' in navpt:
                # assumes we've calculated and logged the wind series
                wind = winds[i]
                wind_rad = 0.5*math.pi - wind['wind_deg']*d2r
                we = math.cos(wind_rad)
                wn = math.sin(wind_rad)
                alpha_beta.update(navpt, airpt, imupt, wn, we)
    alpha_beta.gen_stats()

if False:
    print("Generating synthetic airspeed model:")
    actpt = {}
    airpt = {}
    navpt = {}
    iter = flight_interp.IterateGroup(data)
    for i in tqdm(range(iter.size())):
        record = iter.next()
        if len(record):
            imupt = record['imu']
            if 'act' in record:
                actpt = record['act']
            if 'air' in record:
                airpt = record['air']
            if 'filter' in record:
                navpt = record['filter']
            if 'time' in actpt and 'time' in navpt and 'time' in airpt:
                synth_asi.append(imupt['az'], navpt['the'], actpt['throttle'],
                                 actpt['elevator'], imupt['q'],
                                 airpt['airspeed'])

# use synthetic airspeed estimator
# for each record:
#   asi_kt = synth_asi.est_airspeed(imupt['az'], navpt['the'],
#                                   actpt['throttle'],
#                                   actpt['elevator'], imupt['q'])
#   if asi_kt > 100.0:
#       print(imupt['time'], navpt['phi'], navpt['the'], actpt.throttle, actpt.elevator, imupt.q)
#       synth_filt_asi = 0.9 * synth_filt_asi + 0.1 * asi_kt
#       results.add_asi(airpt.airspeed, synth_filt_asi)

if False:
    print("Generating experimental battery model:")
    battery_model = battery.battery(60.0, 0.01)
    actpt = {}
    healthpt = {}
    iter = flight_interp.IterateGroup(data)
    for i in tqdm(range(iter.size())):
        record = iter.next()
        if len(record):
            imupt = record['imu']
            if 'act' in record:
                actpt = record['act']
            if 'health' in record:
                healthpt = record['health']
            if 'time' in actpt and 'time' in healthpt:
                battery_model.update( actpt['throttle'],
                                      healthpt['main_vcc'],
                                      imupt['time'] )

labels = ["On Board"]
for name in args.filter:
    labels.append(name)

df_imu = []
df_imu.append( pd.DataFrame(data['imu']) )
df_imu[0].set_index('time', inplace=True, drop=False)

df_gps = []
df_gps.append( pd.DataFrame(data['gps']) )
df_gps[0].set_index('time', inplace=True, drop=False)

df_nav = []
df_nav.append( pd.DataFrame(data['filter']) )
df_nav[0].set_index('time', inplace=True, drop=False)

for nav in nav_list:
    df_nav.append( pd.DataFrame(nav) )
    df_nav[-1].set_index('time', inplace=True, drop=False)
    
df1_wind = pd.DataFrame(winds)
df1_wind.set_index('time', inplace=True, drop=False)

# Plotting

nsig = 3
r2d = np.rad2deg

# plot raw accels (useful for bench calibration)
if PLOT_SENSORS:
    plt.figure()
    plt.title('Raw Accels')
    plt.plot(df_imu[0]['ax'], label='ax', alpha=0.75)
    plt.plot(df_imu[0]['ay'], label='ay', alpha=0.75)
    plt.plot(df_imu[0]['az'], label='az', alpha=0.75)
    plt.ylabel('mps^2', weight='bold')
    plt.legend(loc=0)
    plt.grid()
    
    plt.figure()
    plt.title('Raw Gyros')
    plt.plot(df_imu[0]['p'], label='p', alpha=0.75)
    plt.plot(df_imu[0]['q'], label='q', alpha=0.75)
    plt.plot(df_imu[0]['r'], label='r', alpha=0.75)
    plt.ylabel('rad/sec', weight='bold')
    plt.legend(loc=0)
    plt.grid()
    # print 'size:', len(q), len(r)
    # for i in range(len(q)):
    #     if q[i] != r[i]:
    #         print q[i], r[i]

if PLOT_ATTITUDE:
    prop_cycle = plt.rcParams['axes.prop_cycle']
    colors = prop_cycle.by_key()['color']
    print("colors:", colors)
    
    att_fig, att_ax = plt.subplots(3,2, sharex=True)

    # Roll Plot
    att_ax[0,0].set_title('Attitude Angles')
    att_ax[0,0].set_ylabel('Roll (deg)', weight='bold')
    att_ax[0,0].plot([],[]) # consume gps color
    for i in range(len(labels)):
        att_ax[0,0].plot(r2d(df_nav[i]['phi']), label=labels[i], alpha=.8)
    att_ax[0,0].grid()

    att_ax[0,1].plot([],[]) # consume gps color
    att_ax[0,1].plot([],[]) # consume on board color
    for i in range(1, len(labels)):
        att_ax[0,1].plot(nsig*np.rad2deg(np.sqrt(df_nav[i]['Pa0'])), c=colors[i+1])
        att_ax[0,1].plot(-nsig*np.rad2deg(np.sqrt(df_nav[i]['Pa0'])), c=colors[i+1])
    att_ax[0,1].set_ylabel('3*stddev', weight='bold')
    att_ax[0,1].grid()
    
    # Pitch Plot
    att_ax[1,0].set_ylabel('Pitch (deg)', weight='bold')
    att_ax[1,0].plot([],[]) # consume gps color
    for i in range(len(labels)):
        att_ax[1,0].plot(r2d(df_nav[i]['the']), label=labels[i], alpha=.8)
    att_ax[1,0].grid()

    att_ax[1,1].plot([],[]) # consume gps color
    att_ax[1,1].plot([],[]) # consume on board color
    for i in range(1, len(labels)):
        att_ax[1,1].plot(nsig*np.rad2deg(np.sqrt(df_nav[i]['Pa1'])), c=colors[i+1])
        att_ax[1,1].plot(-nsig*np.rad2deg(np.sqrt(df_nav[i]['Pa1'])), c=colors[i+1])
    att_ax[1,1].set_ylabel('3*stddev', weight='bold')
    att_ax[1,1].grid()

    # Yaw Plot
    att_ax[2,0].set_ylabel('Yaw (deg)', weight='bold')
    att_ax[2,0].plot([],[]) # consume gps color
    for i in range(len(labels)):
        att_ax[2,0].plot(r2d(df_nav[i]['psi']), label=labels[i], alpha=.8)
    att_ax[2,0].set_xlabel('Time (sec)', weight='bold')
    att_ax[2,0].grid()
    att_ax[2,0].legend(loc=1)
    
    att_ax[2,1].plot([],[]) # consume gps color
    att_ax[2,1].plot([],[]) # consume on board color
    for i in range(1, len(labels)):
        att_ax[2,1].plot(nsig*np.rad2deg(np.sqrt(df_nav[i]['Pa2'])), c=colors[i+1])
        att_ax[2,1].plot(-nsig*np.rad2deg(np.sqrt(df_nav[i]['Pa2'])), c=colors[i+1])
    att_ax[2,1].set_xlabel('Time (sec)', weight='bold')
    att_ax[2,1].set_ylabel('3*stddev', weight='bold')
    att_ax[2,1].grid()

    #fig, [ax1, ax2, ax3] = plt.subplots(3,1, sharex=True)
    
if PLOT_VELOCITIES:
    fig, [ax1, ax2, ax3] = plt.subplots(3,1, sharex=True)

    # vn Plot
    ax1.set_title("NED Velocities")
    ax1.set_ylabel('vn (mps)', weight='bold')
    ax1.plot(df_gps[0]['vn'], '-*', label='GPS Sensor', lw=2, alpha=.5)
    for i in range(len(labels)):
        ax1.plot(df_nav[i]['vn'], label=labels[i], lw=2, alpha=.8)
    ax1.grid()
    ax1.legend(loc=0)

    # ve Plot
    ax2.set_ylabel('ve (mps)', weight='bold')
    ax2.plot(df_gps[0]['ve'], '-*', label='GPS Sensor', lw=2, alpha=.5)
    for i in range(len(labels)):
        ax2.plot(df_nav[i]['ve'], label=labels[i], lw=2, alpha=.8)
    ax2.grid()

    # vd Plot
    ax3.set_ylabel('vd (mps)', weight='bold')
    ax3.plot(df_gps[0]['vd'], '-*', label='GPS Sensor', lw=2, alpha=.5)
    for i in range(len(labels)):
        ax3.plot(df_nav[i]['vd'], label=labels[i], lw=2, alpha=.8)
    ax3.set_xlabel('Time (secs)', weight='bold')
    ax3.grid()

# Altitude Plot
if PLOT_ALTITUDE:
    plt.figure()
    plt.title('Altitude')
    plt.plot(df_gps[0]['alt'], '-*', label='GPS Sensor', lw=2, alpha=.5)
    for i in range(len(labels)):
        plt.plot(df_nav[i]['alt'], label=labels[i], lw=2, alpha=.8)
    plt.ylabel('Altitude (m)', weight='bold')
    plt.legend(loc=0)
    plt.grid()

# Top View (Longitude vs. Latitude) Plot
if PLOT_GROUNDTRACK:
    plt.figure()
    plt.title("Ground Track")
    plt.ylabel('Latitude (degrees)', weight='bold')
    plt.xlabel('Longitude (degrees)', weight='bold')
    plt.plot(df_gps[0]['lon'], df_gps[0]['lat'], '-*', label='GPS Sensor', lw=2, alpha=.5)
    for i in range(len(labels)):
        plt.plot(r2d(df_nav[i]['lon']), r2d(df_nav[i]['lat']), label=labels[i], lw=2, alpha=.8)
    plt.grid()
    plt.legend(loc=0)
    ax = plt.gca()
    ax.axis('equal')

if PLOT_BIASES:
    bias_fig, bias_ax = plt.subplots(3,2, sharex=True)

    # Gyro Biases
    bias_ax[0,0].set_title("Gyro Biases")
    bias_ax[0,0].set_ylabel('p (deg/s)', weight='bold')
    for i in range(len(labels)):
        if 'gbx' in df_nav[i]:
            bias_ax[0,0].plot(r2d(df_nav[i]['gbx']), label=labels[i])
        else:
            bias_ax[0,0].plot(0, label=labels[i])
    bias_ax[0,0].grid()
    
    bias_ax[1,0].set_ylabel('q (deg/s)', weight='bold')
    for i in range(len(labels)):
        if 'gby' in df_nav[i]:
            bias_ax[1,0].plot(r2d(df_nav[i]['gby']), label=labels[i])
        else:
            bias_ax[1,0].plot(0, label=labels[i])
    bias_ax[1,0].grid()
    
    bias_ax[2,0].set_ylabel('r (deg/s)', weight='bold')
    bias_ax[2,0].set_xlabel('Time (secs)', weight='bold')
    for i in range(len(labels)):
        if 'gbz' in df_nav[i]:
            bias_ax[2,0].plot(r2d(df_nav[i]['gbz']), label=labels[i])
        else:
            bias_ax[2,0].plot(0, label=labels[i])
    bias_ax[2,0].grid()
    
    # Accel Biases
    bias_ax[0,1].set_title("Accel Biases")
    bias_ax[0,1].set_ylabel('ax (m/s^2)', weight='bold')
    for i in range(len(labels)):
        if 'abx' in df_nav[i]:
            bias_ax[0,1].plot(df_nav[i]['abx'], label=labels[i])
        else:
            bias_ax[0,1].plot(0, label=labels[i])
    bias_ax[0,1].grid()
    
    bias_ax[1,1].set_ylabel('ay (m/s^2)', weight='bold')
    for i in range(len(labels)):
        if 'aby' in df_nav[i]:
            bias_ax[1,1].plot(df_nav[i]['aby'], label=labels[i])
        else:
            bias_ax[1,1].plot(0, label=labels[i])
    bias_ax[1,1].grid()
    
    bias_ax[2,1].set_ylabel('az (m/s^2)', weight='bold')
    for i in range(len(labels)):
        if 'abz' in df_nav[i]:
            bias_ax[2,1].plot(df_nav[i]['abz'], label=labels[i])
        else:
            bias_ax[2,1].plot(0, label=labels[i])
    bias_ax[2,1].set_xlabel('Time (secs)', weight='bold')
    bias_ax[2,1].grid()
    bias_ax[2,1].legend(loc=1)

def gen_func( coeffs, min, max, steps ):
    miny = None
    xvals = []
    yvals = []
    step = (max - min) / steps
    func = np.poly1d(coeffs)
    for x in np.arange(min, max+step, step):
        y = func(x)
        if miny == None or abs(y) < miny:
            miny = abs(y)
            minx = x
        xvals.append(x)
        yvals.append(y)
    return xvals, yvals, minx, miny

# Wind Plot
if PLOT_WIND:
    fig, ax1 = plt.subplots()
    ax1.set_title('Wind Estimate')
    ax1.set_ylabel('Degrees', weight='bold')
    ax1.plot(df1_wind['wind_deg'], label='Direction (deg)', c='r', lw=2, alpha=.8)

    ax2 = ax1.twinx()
    ax2.plot(df1_wind['wind_kt'], label='Speed (kt)', c='b', lw=2, alpha=.8)
    ax2.plot(df1_wind['pitot_scale'], label='Pitot Scale', c='k', lw=2, alpha=.8)
    ax2.set_ylabel('Knots', weight='bold')
    ax1.legend(loc=4)
    ax2.legend(loc=1)
    ax1.grid()

comments = """
Summary

You specified the following parameters:
filtertype	=	Butterworth
passtype	=	Lowpass
ripple	=	
order	=	2
samplerate	=	100
corner1	=	0.628
corner2	=	
adzero	=	
logmin	=	
Results

Command line: /www/usr/fisher/helpers/mkfilter -Bu -Lp -o 2 -a 6.2800000000e-03 0.0000000000e+00
raw alpha1    =   0.0062800000
raw alpha2    =   0.0062800000
warped alpha1 =   0.0062808149
warped alpha2 =   0.0062808149
gain at dc    :   mag = 2.641105046e+03   phase =   0.0000000000 pi
gain at centre:   mag = 1.867543288e+03   phase =  -0.5000000000 pi
gain at hf    :   mag = 0.000000000e+00

S-plane zeros:

S-plane poles:
	 -0.0279049255 + j   0.0279049255
	 -0.0279049255 + j  -0.0279049255

Z-plane zeros:
	 -1.0000000000 + j   0.0000000000	2 times

Z-plane poles:
	  0.9721056401 + j   0.0271371011
	  0.9721056401 + j  -0.0271371011

Recurrence relation:
y[n] = (  1 * x[n- 2])
     + (  2 * x[n- 1])
     + (  1 * x[n- 0])

     + ( -0.9457257978 * y[n- 2])
     + (  1.9442112802 * y[n- 1])
"""
NZEROS = 2
NPOLES = 2
xv = [0.0] * (NZEROS+1)
yv = [0.0] * (NPOLES+1)
def my_butter(raw):
    GAIN = 2.641105045e+03

    xv[0] = xv[1]
    xv[1] = xv[2]
    xv[2] = raw / GAIN
    yv[0] = yv[1]
    yv[1] = yv[2]
    yv[2] = (xv[0] + xv[2]) + 2 * xv[1] + ( -0.9457257978 * yv[0]) + ( 1.9442112802 * yv[1])
    return yv[2]

if 'act' in data and PLOT_SYNTH_ASI:
    # butterworth filter experiment
    import scipy.signal as signal
    nyq = 0.5 * 100             # 1/2 sample hz
    b1, a1 = signal.butter(2, 2.0 / nyq)
    b2, a2 = signal.butter(2, 0.8 / nyq)
    print('b2:', b2, 'a2:', a2)
    air1 = signal.filtfilt(b1, a1, np.array(data_dict2.asi))
    air2 = signal.filtfilt(b2, a2, np.array(data_dict2.asi))
    air3 = []
    for a in data_dict2.asi:
        a3 = my_butter(a)
        air3.append(a3)
        
    fig, ax1 = plt.subplots()
    asi = data_dict2.asi
    synth_asi = data_dict2.synth_asi
    ax1.set_title('Synthetic Airspeed')
    ax1.set_ylabel('Kts', weight='bold')
    ax1.plot(t_store1, asi, label='Raw ASI', c='r', lw=2, alpha=.8)
    ax1.plot(t_store1, synth_asi, label='Synthetic ASI', c='b', lw=2, alpha=.8)
    ax1.plot(t_store1, air1, label='butterworth 2.0', c='g', lw=2, alpha=.8)
    ax1.plot(t_store1, air2, label='butterworth 0.8', c='y', lw=2, alpha=.8)
    ax1.plot(t_store1, np.array(air3), label='my_butter 0.628', c='purple', lw=2, alpha=.8)
    ax1.legend(loc=0)
    ax1.grid()
    
    # plot roll vs. yaw rate
    roll_array = []
    r_array = []
    for i in range(len(data_dict1.phi)):
        vn = data_dict1.vn[i]
        ve = data_dict1.ve[i]
        vel = math.sqrt(vn*vn + ve*ve)
        phi = data_dict1.phi[i]
        r = data_dict1.r[i]
        if vel > 8 and abs(phi) <= 0.3:
            roll_array.append(phi)
            r_array.append(r)
    roll_array = np.array(roll_array)
    r_array = np.array(r_array)
    roll_cal, res, _, _, _ = np.polyfit( roll_array, r_array, 3, full=True )
    print(roll_cal)
    xvals, yvals, minx, miny = gen_func(roll_cal, roll_array.min(), roll_array.max(), 1000)
    print('bank bias deg (for L1 config) =', -r2d(minx), 'deg')
    print('zero yaw rate @ bank =', r2d(minx), 'deg')
    fig, ax1 = plt.subplots()
    ax1.set_title('Turn Calibration')
    ax1.set_xlabel('Bank angle (rad)', weight='bold')
    ax1.set_ylabel('Turn rate (rad/sec)', weight='bold')
    ax1.plot(roll_array, r_array, 'x', label='bank vs. turn', c='r', lw=2, alpha=.8)
    ax1.plot(xvals, yvals, label='fit', c='b', lw=2, alpha=.8)

# plot alpha vs. CL (estimate)
if len(alpha_beta.cl_array):
    cl_array = np.array(alpha_beta.cl_array)
    alpha_array = np.array(alpha_beta.alpha_array)
    cl_cal, res, _, _, _ = np.polyfit( alpha_array, cl_array, 1, full=True )
    print(cl_cal)
    xvals, yvals, minx, miny = gen_func(cl_cal, alpha_array.min(), alpha_array.max(), 1000)
    fig, ax1 = plt.subplots()
    ax1.set_title('Alpha/CL')
    ax1.set_xlabel('Alpha (est, deg)', weight='bold')
    ax1.set_ylabel('CL', weight='bold')
    ax1.plot(alpha_array, cl_array, 'x', label='alpha vs CL', c='r', lw=2, alpha=.8)
    ax1.plot(xvals, yvals, label='fit', c='b', lw=2, alpha=.8)

plt.show()
