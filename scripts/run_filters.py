#!/usr/bin/python

"""run_filters.py

This script plays flight data through the selected navigation filters.
The filters are compiled as .so objects and wrapped for python with boost.

A set of customizable input flags are defined at the start of the script.

Initial revision: Hamid M.
Many updates: Curtis L. Olson
"""

import argparse
import math
import numpy as np
import time
import os

import nav.structs

from aurauas.flightdata import flight_loader, aura, sentera

import data_store
import wind
import synth_asi
import battery

parser = argparse.ArgumentParser(description='nav filter')
parser.add_argument('--flight', help='load specified aura flight log')
parser.add_argument('--recalibrate', help='recalibrate raw imu from some other calibration file')
parser.add_argument('--gps-lag', type=float, default='0.0', help='gps lag in seconds')
parser.add_argument('--synthetic-airspeed', action='store_true', help='build synthetic airspeed estimator')
args = parser.parse_args()

# # # # # START INPUTS # # # # #

#MAT_FILENAME = 'flightdata_595.4961sec.mat'
T_GPSOFF = 350          # Time, above which, mission->haveGPS set to 0.
                        # To always keep GPS, set to: -1
FLAG_PLOT_ATTITUDE = True
FLAG_PLOT_VELOCITIES = True
FLAG_PLOT_GROUNDTRACK = True
FLAG_PLOT_ALTITUDE = True
FLAG_PLOT_WIND = True
FLAG_PLOT_SYNTH_ASI = False
FLAG_PLOT_BIASES = True
SIGNAL_LIST = [0, 1, 8]  # List of signals [0 to 9] to be plotted
FLAG_WRITE2CSV = False # Write results to CSV file.
# # # # # END INPUTS # # # # #

import os
import csv
import numpy as np
from matplotlib import pyplot as plt
import navpy

# filter interfaces
import nav_eigen_double
import nav_eigen_mag
import nav_eigen_float
import nav_eigen_sep
import nav_openloop
#import MadgwickAHRS

filter1 = nav_eigen_sep.filter()
#filter1 = nav_mag.filter()
#filter1 = nav_eigen.filter()
filter2 = nav_eigen_mag.filter()
#filter2 = nav_openloop.filter()
#filter2 = MadgwickAHRS.filter()

r2d = 180.0 / math.pi
mps2kt = 1.94384

def run_filter(filter, data, call_init=True, start_time=None, end_time=None):
    # for convenience ...
    imu_data = data['imu']
    gps_data = data['gps']
    if 'air' in data:
        air_data = data['air']
    else:
        air_data = []
    filter_data = data['filter']
    if 'pilot' in data:
        pilot_data = data['pilot']
    else:
        pilot_data = []
    if 'act' in data:
        act_data = data['act']
    else:
        act_data = []
    if 'health' in data:
        health_data = data['health']
    else:
        health_data = []
        
    data_dict = data_store.data_store()
    # t_store = []
    
    # Using while loop starting at k (set to kstart) and going to end
    # of .mat file
    run_start = time.time()
    gps_index = 0
    air_index = 0
    airpt = nav.structs.Airdata()
    filter_index = 0
    pilot_index = 0
    pilotpt = None
    act_index = 0
    actpt = None
    health_index = 0
    healthpt = None
    new_gps = 0
    synth_filt_asi = 0
    battery_model = battery.battery(60.0, 0.01)
    if call_init:
        filter_init = False
    else:
        filter_init = True
    k_start = 0
    if start_time != None:
        for k, imu_pt in enumerate(imu_data):
            #print k_start, imu_pt.time, start_time
            if imu_pt.time >= start_time:
                k_start = k
                break
    k_end = len(imu_data)
    if end_time != None:
        for k, imu_pt in enumerate(imu_data):
            if imu_pt.time >= end_time:
                k_end = k
                break
    print k_start, k_end
    for k in range(k_start, k_end):
        imupt = imu_data[k]
        if gps_index < len(gps_data) - 1:
            # walk the gps counter forward as needed
            newData = 0
            while gps_index < len(gps_data) - 1 and gps_data[gps_index+1].time - args.gps_lag <= imupt.time:
                gps_index += 1
                newData = 1
            gpspt = gps_data[gps_index]
            gpspt.newData = newData
        else:
            # no more gps data, stay on the last record
            gpspt = gps_data[gps_index]
            gpspt.newData = 0
        #print gpspt.time
        if air_index < len(air_data) - 1:
            # walk the airdata counter forward as needed
            while air_index < len(air_data) - 1 and air_data[air_index+1].time <= imupt.time:
                air_index += 1
            airpt = air_data[air_index]
        elif len(air_data):
            # no more air data, stay on the last record
            airpt = air_data[air_index]
        # print airpt.time
        # walk the filter counter forward as needed
        if len(filter_data):
            while filter_index < len(filter_data) - 1 and filter_data[filter_index].time <= imupt.time:
                filter_index += 1
            filterpt = filter_data[filter_index]
        else:
            filterpt = nav.structs.NAVdata()
        #print "t(imu) = " + str(imupt.time) + " t(gps) = " + str(gpspt.time)
        if 'pilot' in data:
            while pilot_index < len(pilot_data) - 1 and pilot_data[pilot_index].time <= imupt.time:
                pilot_index += 1
            pilotpt = pilot_data[pilot_index]
        elif 'pilot' in data:
            pilotpt = pilot_data[pilot_index]
        if 'act' in data:
            while act_index < len(act_data) - 1 and act_data[act_index].time <= imupt.time:
                act_index += 1
            actpt = act_data[act_index]
            #print act_index, imupt.time, actpt.time, actpt.throttle, actpt.elevator
        elif 'act' in data:
            actpt = act_data[act_index]
        if 'health' in data:
            while health_index < len(health_data) - 1 and health_data[health_index].time <= imupt.time:
                health_index += 1
            healthpt = health_data[health_index]

        elif 'act' in data:
            actpt = act_data[act_index]

        # If k is at the initialization time init_nav else get_nav
        if not filter_init and gps_index > 0:
            print "init:", imupt.time, gpspt.time
            navpt = filter.init(imupt, gpspt, filterpt)
            filter_init = True
        elif filter_init:
            navpt = filter.update(imupt, gpspt, filterpt)

        if filter_init:
            # experimental: run wind estimator
            # print airpt.airspeed
            (wn, we, ps) = wind.update_wind(imupt.time, airpt.airspeed,
                                            navpt.psi, navpt.vn, navpt.ve)
            #print wn, we, math.atan2(wn, we), math.atan2(wn, we)*r2d
            wind_deg = 90 - math.atan2(wn, we) * r2d
            if wind_deg < 0: wind_deg += 360.0
            wind_kt = math.sqrt( we*we + wn*wn ) * mps2kt
            #print wn, we, ps, wind_deg, wind_kt

            # experimental: synthetic airspeed estimator
            if 'act' in data and synth_asi.rbfi == None:
                # print imupt.time, airpt.airspeed, actpt.throttle, actpt.elevator
                synth_asi.append(navpt.phi, navpt.the, actpt.throttle,
                                 actpt.elevator, imupt.q, airpt.airspeed)
            elif 'act' in data:
                asi_kt = synth_asi.est_airspeed(navpt.phi, navpt.the,
                                                actpt.throttle,
                                                actpt.elevator, imupt.q)
                if asi_kt > 100.0:
                    print imupt.time, navpt.phi, navpt.the, actpt.throttle, actpt.elevator, imupt.q
                synth_filt_asi = 0.9 * synth_filt_asi + 0.1 * asi_kt
                data_dict.add_asi(airpt.airspeed, synth_filt_asi)

            # experimental: battery model / estimator
            if 'health' in data and airpt.airspeed > 10:
                battery_model.update( actpt.throttle, healthpt.main_vcc, imupt.time )
            
        # Store the desired results obtained from the compiled test
        # navigation filter and the baseline filter
        if filter_init:
            data_dict.append(navpt, imupt)
            data_dict.add_wind(wind_deg, wind_kt, ps)

        # Increment time up one step for the next iteration of the
        # while loop.
        k += 1

    # proper cleanup
    filter.close()
    run_end = time.time()
    elapsed_sec = run_end - run_start
    return data_dict, elapsed_sec

path = args.flight
if 'recalibrate' in args:
    recal_file = args.recalibrate
else:
    recal_file = None
data, flight_format = flight_loader.load(path, recal_file)

print "imu records:", len(data['imu'])
print "gps records:", len(data['gps'])
if 'air' in data:
    print "airdata records:", len(data['air'])
if 'filter' in data:
    print "filter records:", len(data['filter'])
if 'pilot' in data:
    print "pilot records:", len(data['pilot'])
if 'act' in data:
    print "act records:", len(data['act'])
if len(data['imu']) == 0 and len(data['gps']) == 0:
    print "not enough data loaded to continue."
    quit()

plotname = os.path.basename(args.flight)    

if False:
    # quick hack estimate gyro biases
    p_sum = 0.0
    q_sum = 0.0
    r_sum = 0.0
    for imu in data['imu']:
        p_sum += imu.p
        q_sum += imu.q
        r_sum += imu.r
    p_bias = p_sum / len(data['imu'])
    q_bias = q_sum / len(data['imu'])
    r_bias = r_sum / len(data['imu'])
    print "bias:", p_bias, q_bias, r_bias
    for imu in data['imu']:
        imu.p -= p_bias
        imu.q -= q_bias
        imu.r -= r_bias

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
    print "x:", x_min, x_max
    print "y:", y_min, y_max
    print "z:", z_min, z_max
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
        
# rearrange flight data for plotting
t_gps = []
lat_gps = []
lon_gps = []
alt_gps = []
vn_gps = []
ve_gps = []
vd_gps = []
for g in data['gps']:
    t_gps.append(g.time)
    lat_gps.append(g.lat)
    lon_gps.append(g.lon)
    alt_gps.append(g.alt)
    vn_gps.append(g.vn)
    ve_gps.append(g.ve)
    vd_gps.append(g.vd)

t_flight = []
psi_flight = []
the_flight = []
phi_flight = []
navlat_flight = []
navlon_flight = []
navalt_flight = []
vn_flight = []
ve_flight = []
vd_flight = []
if 'filter' in data:
    for f in data['filter']:
        t_flight.append(f.time)
        psi_flight.append(f.psi)
        the_flight.append(f.the)
        phi_flight.append(f.phi)
        navlat_flight.append(f.lat)
        navlon_flight.append(f.lon)
        navalt_flight.append(f.alt)
        vn_flight.append(f.vn)
        ve_flight.append(f.ve)
        vd_flight.append(f.vd)

r_flight = []
for i in data['imu']:
    r_flight.append(i.r)
    
# Default config
config = nav.structs.NAVconfig()
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
config.sig_gps_p_ne = 5.0
config.sig_gps_p_d  = 10.0
config.sig_gps_v_ne = 1.0
config.sig_gps_v_d  = 2.0
config.sig_mag      = 1.0
filter1.set_config(config)
filter2.set_config(config)

# almost no trust in IMU ...
# config = nav.structs.NAVconfig()
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
# filter2.set_config(config)

# less than default trust in IMU ...
# config = nav.structs.NAVconfig()
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
# filter2.set_config(config)

# too high trust in IMU ...
# config = nav.structs.NAVconfig()
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

data_dict1, filter1_sec = run_filter(filter1, data)

if args.synthetic_airspeed:
    print "building synthetic air data estimator..."
    if 'act' in data:
        result = synth_asi.build()
        if result:
            FLAG_PLOT_SYNTH_ASI = True
        else:
            FLAG_PLOT_SYNTH_ASI = False

data_dict2, filter2_sec = run_filter(filter2, data)

print "filter1 time = %.4f" % filter1_sec
print "filter2 time = %.4f" % filter2_sec
diff_sec = filter1_sec - filter2_sec
perc = diff_sec / filter1_sec
if perc >= 0.0:
    print "filter2 is %.1f%% faster" % (perc * 100.0)
else:
    print "filter2 is %.1f%% slower" % (-perc * 100.0)

if flight_format == 'aura_csv' or flight_format == 'aura_txt':
    filter_post = os.path.join(args.flight, "filter-post.txt")
    aura.save_filter_result(filter_post, data_dict1)

if flight_format == 'px4_ulog':
    filter_post = args.flight + "_filter_post.txt"
    aura.save_filter_result(filter_post, data_dict1)
    
if flight_format == 'sentera':
    file_ins = os.path.join(args.flight, "filter-post-ins.txt")
    file_mag = os.path.join(args.flight, "filter-post-mag.txt")
    sentera.save_filter_result(file_ins, data_dict1)
    sentera.save_filter_result(file_mag, data_dict2)
    #sentera.rewrite_pix4d_csv(args.flight, data_dict2)
    #sentera.rewrite_image_metadata_txt(args.flight, data_dict2)

nsig = 3
t_store1 = data_dict1.time
t_store2 = data_dict2.time

# Plotting
r2d = np.rad2deg
if FLAG_PLOT_ATTITUDE:
    Patt1 = np.array(data_dict1.Patt, dtype=np.float64)
    Patt2 = np.array(data_dict2.Patt, dtype=np.float64)

    att_fig, att_ax = plt.subplots(3,2, sharex=True)

    # Roll Plot
    phi_nav = data_dict1.phi
    phi_nav_mag = data_dict2.phi
    att_ax[0,0].set_ylabel('Roll (deg)', weight='bold')
    att_ax[0,0].plot(t_flight, r2d(phi_flight), label='On Board', c='g', alpha=.5)
    att_ax[0,0].plot(t_store1, r2d(phi_nav), label=filter1.name, c='r', alpha=.8)
    att_ax[0,0].plot(t_store2, r2d(phi_nav_mag), label=filter2.name, c='b', alpha=.8)
    att_ax[0,0].grid()

    att_ax[0,1].plot(t_store1,nsig*np.rad2deg(np.sqrt(Patt1[:,0])),c='r')
    att_ax[0,1].plot(t_store1,-nsig*np.rad2deg(np.sqrt(Patt1[:,0])),c='r')
    att_ax[0,1].plot(t_store2,nsig*np.rad2deg(np.sqrt(Patt2[:,0])),c='b')
    att_ax[0,1].plot(t_store2,-nsig*np.rad2deg(np.sqrt(Patt2[:,0])),c='b')
    att_ax[0,1].set_ylabel('3*stddev', weight='bold')

    # Pitch Plot
    the_nav = data_dict1.the
    the_nav_mag = data_dict2.the
    att_ax[1,0].set_ylabel('Pitch (deg)', weight='bold')
    att_ax[1,0].plot(t_flight, r2d(the_flight), label='On Board', c='g', alpha=.5)
    att_ax[1,0].plot(t_store1, r2d(the_nav), label=filter1.name, c='r', alpha=.8)
    att_ax[1,0].plot(t_store2, r2d(the_nav_mag), label=filter2.name,c='b', alpha=.8)
    att_ax[1,0].grid()

    att_ax[1,1].plot(t_store1,nsig*np.rad2deg(np.sqrt(Patt1[:,1])),c='r')
    att_ax[1,1].plot(t_store1,-nsig*np.rad2deg(np.sqrt(Patt1[:,1])),c='r')
    att_ax[1,1].plot(t_store2,nsig*np.rad2deg(np.sqrt(Patt2[:,1])),c='b')
    att_ax[1,1].plot(t_store2,-nsig*np.rad2deg(np.sqrt(Patt2[:,1])),c='b')
    att_ax[1,1].set_ylabel('3*stddev', weight='bold')

    # Yaw Plot
    psi_nav = data_dict1.psi
    psi_nav_mag = data_dict2.psi
    att_ax[2,0].set_title(plotname, fontsize=10)
    att_ax[2,0].set_ylabel('Yaw (deg)', weight='bold')
    att_ax[2,0].plot(t_flight, r2d(psi_flight), label='On Board', c='g', alpha=.5)
    att_ax[2,0].plot(t_store1, r2d(psi_nav), label=filter1.name, c='r', alpha=.8)
    att_ax[2,0].plot(t_store2, r2d(psi_nav_mag), label=filter2.name,c='b', alpha=.8)
    att_ax[2,0].set_xlabel('Time (sec)', weight='bold')
    att_ax[2,0].grid()
    att_ax[2,0].legend(loc=1)
    
    att_ax[2,1].plot(t_store1,nsig*np.rad2deg(np.sqrt(Patt1[:,2])),c='r')
    att_ax[2,1].plot(t_store1,-nsig*np.rad2deg(np.sqrt(Patt1[:,2])),c='r')
    att_ax[2,1].plot(t_store2,nsig*np.rad2deg(np.sqrt(Patt2[:,2])),c='b')
    att_ax[2,1].plot(t_store2,-nsig*np.rad2deg(np.sqrt(Patt2[:,2])),c='b')
    att_ax[2,1].set_xlabel('Time (sec)', weight='bold')
    att_ax[2,1].set_ylabel('3*stddev', weight='bold')

    #fig, [ax1, ax2, ax3] = plt.subplots(3,1, sharex=True)

# plot raw accels (useful for bench calibration)
if True:
    ax = data_dict1.ax
    ay = data_dict1.ay
    az = data_dict1.az
    plt.figure()
    plt.title('Raw Accels')
    plt.plot(t_store1, ax, label='ax', c='g', lw=2, alpha=.5)
    plt.plot(t_store1, ay, label='ay', c='b', lw=2, alpha=.5)
    plt.plot(t_store1, az, label='az', c='r', lw=2, alpha=.5)
    plt.ylabel('mps^2', weight='bold')
    plt.legend(loc=0)
    plt.grid()
    
    p = data_dict1.p
    q = data_dict1.q
    r = data_dict1.r
    plt.figure()
    plt.title('Raw Gyros')
    plt.plot(t_store1, p, label='p', c='g', lw=2, alpha=.5)
    plt.plot(t_store1, q, label='q', c='b', lw=2, alpha=.5)
    plt.plot(t_store1, r, label='r', c='r', lw=2, alpha=.5)
    plt.ylabel('rad/sec', weight='bold')
    plt.legend(loc=0)
    plt.grid()
    # print 'size:', len(q), len(r)
    # for i in range(len(q)):
    #     if q[i] != r[i]:
    #         print q[i], r[i]

if FLAG_PLOT_VELOCITIES:
    fig, [ax1, ax2, ax3] = plt.subplots(3,1, sharex=True)

    # vn Plot
    vn_nav = data_dict1.vn
    vn_nav_mag = data_dict2.vn
    ax1.set_title(plotname, fontsize=10)
    ax1.set_ylabel('vn (mps)', weight='bold')
    ax1.plot(t_gps, vn_gps, '-*', label='GPS Sensor', c='g', lw=2, alpha=.5)
    ax1.plot(t_flight, vn_flight, label='On Board', c='k', lw=2, alpha=.5)
    ax1.plot(t_store1, vn_nav, label=filter1.name, c='r', lw=2, alpha=.8)
    ax1.plot(t_store2, vn_nav_mag, label=filter2.name,c='b', lw=2, alpha=.8)
    ax1.grid()
    ax1.legend(loc=0)

    # ve Plot
    ve_nav = data_dict1.ve
    ve_nav_mag = data_dict2.ve
    ax2.set_ylabel('ve (mps)', weight='bold')
    ax2.plot(t_gps, ve_gps, '-*', label='GPS Sensor', c='g', lw=2, alpha=.5)
    ax2.plot(t_flight, ve_flight, label='On Board', c='k', lw=2, alpha=.5)
    ax2.plot(t_store1, ve_nav, label=filter1.name, c='r', lw=2, alpha=.8)
    ax2.plot(t_store2, ve_nav_mag, label=filter2.name,c='b', lw=2, alpha=.8)
    ax2.grid()

    # vd Plot
    vd_nav = data_dict1.vd
    vd_nav_mag = data_dict2.vd
    ax3.set_ylabel('vd (mps)', weight='bold')
    ax3.plot(t_gps, vd_gps, '-*', label='GPS Sensor', c='g', lw=2, alpha=.5)
    ax3.plot(t_flight, vd_flight, label='On Board', c='k', lw=2, alpha=.5)
    ax3.plot(t_store1, vd_nav, label=filter1.name, c='r', lw=2, alpha=.8)
    ax3.plot(t_store2, vd_nav_mag, label=filter2.name, c='b',lw=2, alpha=.8)
    ax3.set_xlabel('TIME (SECONDS)', weight='bold')
    ax3.grid()

# Altitude Plot
if FLAG_PLOT_ALTITUDE:
    navalt = data_dict1.alt
    nav_magalt = data_dict2.alt
    plt.figure()
    plt.title('ALTITUDE')
    plt.plot(t_gps, alt_gps, '-*', label='GPS Sensor', c='g', lw=2, alpha=.5)
    plt.plot(t_flight, navalt_flight, label='On Board', c='k', lw=2, alpha=.5)
    plt.plot(t_store1, navalt, label=filter1.name, c='r', lw=2, alpha=.8)
    plt.plot(t_store2, nav_magalt, label=filter2.name, c='b', lw=2, alpha=.8)
    plt.ylabel('ALTITUDE (METERS)', weight='bold')
    plt.legend(loc=0)
    plt.grid()

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
if FLAG_PLOT_WIND:
    fig, ax1 = plt.subplots()
    wind_deg = data_dict2.wind_deg
    wind_kt = data_dict2.wind_kt
    pitot_scale = data_dict2.pitot_scale
    ax1.set_title('Wind')
    ax1.set_ylabel('Degrees', weight='bold')
    ax1.plot(t_store1, wind_deg, label='Direction (deg)', c='r', lw=2, alpha=.8)

    ax2 = ax1.twinx()
    ax2.plot(t_store1, wind_kt, label='Speed (kt)', c='b', lw=2, alpha=.8)
    ax2.plot(t_store1, pitot_scale, label='Pitot Scale', c='k', lw=2, alpha=.8)
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

if 'act' in data and FLAG_PLOT_SYNTH_ASI:
    # butterworth filter experiment
    import scipy.signal as signal
    nyq = 0.5 * 100             # 1/2 sample hz
    b1, a1 = signal.butter(2, 2.0 / nyq)
    b2, a2 = signal.butter(2, 0.8 / nyq)
    print 'b2:', b2, 'a2:', a2
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
    print roll_cal
    xvals, yvals, minx, miny = gen_func(roll_cal, roll_array.min(), roll_array.max(), 1000)
    print 'bank bias deg (for L1 config) =', -r2d(minx), 'deg'
    print 'zero yaw rate @ bank =', r2d(minx), 'deg'
    fig, ax1 = plt.subplots()
    ax1.set_title('Turn Calibration')
    ax1.set_xlabel('Bank angle (rad)', weight='bold')
    ax1.set_ylabel('Turn rate (rad/sec)', weight='bold')
    ax1.plot(roll_array, r_array, 'x', label='bank vs. turn', c='r', lw=2, alpha=.8)
    ax1.plot(xvals, yvals, label='fit', c='b', lw=2, alpha=.8)

# Top View (Longitude vs. Latitude) Plot
if FLAG_PLOT_GROUNDTRACK:
    navlat = data_dict1.lat
    navlon = data_dict1.lon
    nav_maglat = data_dict2.lat
    nav_maglon = data_dict2.lon
    plt.figure()
    plt.title(plotname, fontsize=10)
    plt.ylabel('LATITUDE (DEGREES)', weight='bold')
    plt.xlabel('LONGITUDE (DEGREES)', weight='bold')
    plt.plot(lon_gps, lat_gps, '*', label='GPS Sensor', c='g', lw=2, alpha=.5)
    plt.plot(r2d(navlon_flight), r2d(navlat_flight), label='On Board', c='k', lw=2, alpha=.5)
    plt.plot(r2d(navlon), r2d(navlat), label=filter1.name, c='r', lw=2, alpha=.8)
    plt.plot(r2d(nav_maglon), r2d(nav_maglat), label=filter2.name, c='b', lw=2, alpha=.8)
    plt.grid()
    plt.legend(loc=0)
    
if FLAG_PLOT_BIASES:
    bias_fig, bias_ax = plt.subplots(3,2, sharex=True)

    # Gyro Biases
    bias_ax[0,0].set_ylabel('p Bias (deg)', weight='bold')
    bias_ax[0,0].plot(t_store1, r2d(data_dict1.p_bias), label=filter1.name, c='r')
    bias_ax[0,0].plot(t_store2, r2d(data_dict2.p_bias), label=filter2.name, c='b')
    bias_ax[0,0].set_xlabel('Time (secs)', weight='bold')
    bias_ax[0,0].grid()
    
    bias_ax[1,0].set_ylabel('q Bias (deg)', weight='bold')
    bias_ax[1,0].plot(t_store1, r2d(data_dict1.q_bias), label=filter1.name, c='r')
    bias_ax[1,0].plot(t_store2, r2d(data_dict2.q_bias), label=filter2.name, c='b')
    bias_ax[1,0].set_xlabel('Time (secs)', weight='bold')
    bias_ax[1,0].grid()
    
    bias_ax[2,0].set_ylabel('r Bias (deg)', weight='bold')
    bias_ax[2,0].plot(t_store1, r2d(data_dict1.r_bias), label=filter1.name, c='r')
    bias_ax[2,0].plot(t_store2, r2d(data_dict2.r_bias), label=filter2.name, c='b')
    bias_ax[2,0].set_xlabel('Time (secs)', weight='bold')
    bias_ax[2,0].grid()
    
    # Accel Biases
    bias_ax[0,1].set_ylabel('ax Bias (m/s^2)', weight='bold')
    bias_ax[0,1].plot(t_store1, data_dict1.ax_bias, label=filter1.name, c='r')
    bias_ax[0,1].plot(t_store2, data_dict2.ax_bias, label=filter2.name, c='b')
    bias_ax[0,1].set_xlabel('Time (secs)', weight='bold')
    bias_ax[0,1].grid()
    
    bias_ax[1,1].set_ylabel('ay Bias (m/s^2)', weight='bold')
    bias_ax[1,1].plot(t_store1, data_dict1.ay_bias, label=filter1.name, c='r')
    bias_ax[1,1].plot(t_store2, data_dict2.ay_bias, label=filter2.name, c='b')
    bias_ax[1,1].set_xlabel('Time (secs)', weight='bold')
    bias_ax[1,1].grid()
    
    bias_ax[2,1].set_ylabel('az Bias (m/s^2)', weight='bold')
    bias_ax[2,1].plot(t_store1, data_dict1.az_bias, label=filter1.name, c='r')
    bias_ax[2,1].plot(t_store2, data_dict2.az_bias, label=filter2.name, c='b')
    bias_ax[2,1].set_xlabel('Time (secs)', weight='bold')
    bias_ax[2,1].grid()
    bias_ax[2,1].legend(loc=1)

plt.show()
