#!/usr/bin/python

"""WRAP_NAV_FILTER.PY
This script plays flight data through navigation filter C-Code.
Both baseline navigation and researchNavigation are compiled into `.so` 
shared objects and wrapped in Python.  

This code automatically calls necessary terminal calls to gcc to compile
the functions.  However, more details on compiling the C-Code to make the 
`.so` manually can be found in `README.md`.

A set of customizable input flags are defined at the start of the script.

**Note:** Rerunning this in interactive mode has unexpected results!
          It doesn't seem to reload the latest `.so`.  This script
          should be called from the terminal.  For example:
          >> python wrap_nav_filter.py

Author: Hamid M.
Last Update: April 22, 2015
"""

import argparse
import numpy as np
import time
import os

import flight_data

parser = argparse.ArgumentParser(description='nav filter')
parser.add_argument('--flight', help='load specified aura flight log')
parser.add_argument('--aura-flight', help='load specified aura flight log')
parser.add_argument('--umn-flight', help='load specified .mat flight log')
parser.add_argument('--sentera-flight', help='load specified sentera flight log')
args = parser.parse_args()

# # # # # START INPUTS # # # # #

#MAT_FILENAME = 'flightdata_595.4961sec.mat'
T_GPSOFF = 350          # Time, above which, mission->haveGPS set to 0.
                        # To always keep GPS, set to: -1
FLAG_PLOT_ATTITUDE = True
FLAG_PLOT_VELOCITIES = True
FLAG_PLOT_GROUNDTRACK = True
FLAG_PLOT_ALTITUDE = True
FLAG_PLOT_BIASES = True
SIGNAL_LIST = [0, 1, 8]  # List of signals [0 to 9] to be plotted
FLAG_WRITE2CSV = False # Write results to CSV file.
# # # # # END INPUTS # # # # #

import os
import csv
import numpy as np
from matplotlib import pyplot as plt
import navpy
r2d = np.rad2deg

# filter interfaces
import nav_orig
import nav_mag
import nav_eigen
import nav_eigen_mag
import nav_openloop
import MadgwickAHRS

#filter2 = nav_eigen_mag_old.filter()
#filter1 = nav_orig.filter()
filter1 = nav_eigen.filter()
filter2 = nav_eigen_mag.filter()
#filter2 = nav_openloop.filter()
#filter2 = MadgwickAHRS.filter()

class data_store():
    def __init__(self):
        self.psi = []
        self.the = []
        self.phi = []
        self.nav_lat = []
        self.nav_lon = []
        self.nav_alt = []
        self.nav_vn = []
        self.nav_ve = []
        self.nav_vd = []

        self.ax_bias = []
        self.ay_bias = [] 
        self.az_bias = []
        self.p_bias = []
        self.q_bias = []
        self.r_bias = []

        self.Pp = []
        self.Pvel = []
        self.Patt = []
        self.Pab = []
        self.Pgb = []

    def append(self, insgps):
        self.psi.append(insgps.psi)
        self.the.append(insgps.the)
        self.phi.append(insgps.phi)
        self.nav_lat.append(insgps.lat)
        self.nav_lon.append(insgps.lon)
        self.nav_alt.append(insgps.alt)
        self.nav_vn.append(insgps.vn)
        self.nav_ve.append(insgps.ve)
        self.nav_vd.append(insgps.vd)

        self.ax_bias.append(insgps.abx)
        self.ay_bias.append(insgps.aby)
        self.az_bias.append(insgps.abz)
        self.p_bias.append(insgps.gbx)
        self.q_bias.append(insgps.gby)
        self.r_bias.append(insgps.gbz)
        
        self.Pp.append( np.array([insgps.Pp0, insgps.Pp1, insgps.Pp2]) )
        self.Pvel.append( np.array([insgps.Pv0, insgps.Pv1, insgps.Pv2]) )
        self.Patt.append( np.array([insgps.Pa0, insgps.Pa1, insgps.Pa2]) )
        self.Pab.append( np.array([insgps.Pabx, insgps.Paby, insgps.Pabz]) )
        self.Pgb.append( np.array([insgps.Pgbx, insgps.Pgby, insgps.Pgbz]) )

# Values (Calculated by compiled test navigation filter) need to be
# stored in python variables and they need to be in the globaldefs.c
# and cdefs.py to allow for pulling them out and saving. These
# python variables need to be initialized to work properly in the
# while loop.
data_dict1 = data_store()
data_dict2 = data_store()
t_store = []

imu_data, gps_data, filter_data = flight_data.load(args)
print "imu records:", len(imu_data)
print "gps records:", len(gps_data)
print "filter records:", len(filter_data)
if len(imu_data) == 0 and len(gps_data) == 0:
    print "not enough data loaded to continue."
    quit()

if args.flight:
    plotname = os.path.basename(args.flight)    
elif args.aura_flight:
    plotname = os.path.basename(args.aura_flight)
elif args.sentera_flight:
    plotname = os.path.basename(args.sentera_flight)
elif args.umn_flight:
    plotname = os.path.basename(args.umn_flight)

if False:
    # quick hack estimate gyro biases
    p_sum = 0.0
    q_sum = 0.0
    r_sum = 0.0
    for imu in imu_data:
        p_sum += imu.p
        q_sum += imu.q
        r_sum += imu.r
    p_bias = p_sum / len(imu_data)
    q_bias = q_sum / len(imu_data)
    r_bias = r_sum / len(imu_data)
    print "bias:", p_bias, q_bias, r_bias
    for imu in imu_data:
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
    for imu in imu_data:
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
    for imu in imu_data:
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
for g in gps_data:
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
for f in filter_data:
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

concurrent_run = False
if concurrent_run:
    # Using while loop starting at k (set to kstart) and going to end
    # of .mat file
    gps_index = 0
    new_gps = 0
    filter_init = False
    for k, imupt in enumerate(imu_data):
        # walk the gps counter forward as needed
        if imupt.time >= gps_data[gps_index].time:
            gps_index += 1
            new_gps = 1
        else:
            new_gps = 0
        if gps_index >= len(gps_data):
            # no more gps data, stay on the last record
            gps_index = len(gps_data)-1
        gpspt = gps_data[gps_index-1]
        gpspt.newData = new_gps
        #print "t(imu) = " + str(imupt.time) + " t(gps) = " + str(gpspt.time)

        # If k is at the initialization time init_nav else get_nav
        if not filter_init and gps_index > 0:
            print "init:", imupt.time, gpspt.time
            insgps1 = filter1.init(imupt, gpspt)
            insgps2 = filter2.init(imupt, gpspt)
            filter_init = True
        elif filter_init:
            insgps1 = filter1.update(imupt, gpspt)
            insgps2 = filter2.update(imupt, gpspt)

        # Store the desired results obtained from the compiled test
        # navigation filter and the baseline filter
        if filter_init:
            data_dict1 = data_dict1.append(insgps1)
            data_dict2 = data_dict2.append(insgps2)
            t_store.append(imupt.time)

        # Increment time up one step for the next iteration of the
        # while loop.
        k += 1

    # proper cleanup
    filter1.close()
    filter2.close()
else:
    # Using while loop starting at k (set to kstart) and going to end
    # of .mat file
    start_time = time.time()
    gps_index = 0
    filter_index = 0
    new_gps = 0
    filter_init = False
    for k, imupt in enumerate(imu_data):
        # walk the gps counter forward as needed
        if imupt.time >= gps_data[gps_index].time:
            gps_index += 1
            new_gps = 1
        else:
            new_gps = 0
        if gps_index >= len(gps_data):
            # no more gps data, stay on the last record
            gps_index = len(gps_data)-1
        gpspt = gps_data[gps_index-1]
        gpspt.newData = new_gps
        # walk the filter counter forward as needed
        if len(filter_data):
            if imupt.time > filter_data[filter_index].time:
                filter_index += 1
            if filter_index >= len(filter_data):
                # no more filter data, stay on the last record
                filter_index = len(filter_data)-1
            filterpt = filter_data[filter_index]
        else:
            filterpt = None
        #print "t(imu) = " + str(imupt.time) + " t(gps) = " + str(gpspt.time)

        # If k is at the initialization time init_nav else get_nav
        if not filter_init and gps_index > 0:
            print "init:", imupt.time, gpspt.time
            insgps1 = filter1.init(imupt, gpspt, filterpt)
            filter_init = True
        elif filter_init:
            insgps1 = filter1.update(imupt, gpspt, filterpt)

        # Store the desired results obtained from the compiled test
        # navigation filter and the baseline filter
        if filter_init:
            data_dict1.append(insgps1)
            t_store.append(imupt.time)
            # print imupt.time, imupt.p - insgps1.estGB[0], imupt.q - insgps1.estGB[1], imupt.r - insgps1.estGB[2], imupt.ax - insgps1.estAB[0], imupt.ay - insgps1.estAB[1], imupt.az - insgps1.estAB[2], imupt.hx, imupt.hy, imupt.hz, imupt.temp

        # Increment time up one step for the next iteration of the
        # while loop.
        k += 1

    # proper cleanup
    filter1.close()
    end_time = time.time()
    filter1_sec = end_time - start_time

    start_time = time.time()
    gps_index = 0
    filter_index = 0
    new_gps = 0
    filter_init = False
    for k, imupt in enumerate(imu_data):
        # walk the gps counter forward as needed
        if imupt.time >= gps_data[gps_index].time:
            gps_index += 1
            new_gps = 1
        else:
            new_gps = 0
        if gps_index >= len(gps_data):
            # no more gps data, stay on the last record
            gps_index = len(gps_data)-1
        gpspt = gps_data[gps_index-1]
        gpspt.newData = new_gps
        if len(filter_data):
            # walk the filter counter forward as needed
            if imupt.time > filter_data[filter_index].time:
                filter_index += 1
            if filter_index >= len(filter_data):
                # no more filter data, stay on the last record
                filter_index = len(filter_data)-1
            filterpt = filter_data[filter_index]
        else:
            filterpt = None
        #print "t(imu) = " + str(imupt.time) + " t(gps) = " + str(gpspt.time)

        # If k is at the initialization time init_nav else get_nav
        if not filter_init and gps_index > 0:
            insgps2 = filter2.init(imupt, gpspt, filterpt)
            filter_init = True
        elif filter_init:
            insgps2 = filter2.update(imupt, gpspt, filterpt)

        # Store the desired results obtained from the compiled test
        # navigation filter and the baseline filter
        if filter_init:
            data_dict2.append(insgps2)

        # Increment time up one step for the next iteration of the
        # while loop.
        k += 1

    # proper cleanup
    filter2.close()
    end_time = time.time()
    filter2_sec = end_time - start_time
    print "filter1 time = %.4f" % filter1_sec
    print "filter2 time = %.4f" % filter2_sec
    diff_sec = filter1_sec - filter2_sec
    perc = diff_sec / filter1_sec
    if perc >= 0.0:
        print "filter2 is %.1f%% faster" % (perc * 100.0)
    else:
        print "filter2 is %.1f%% slower" % (-perc * 100.0)

if args.flight or args.aura_flight:
    import data_aura
    if args.flight:
        filter_post = os.path.join(args.flight, "filter-post.txt")
    elif args.aura_flight:
        filter_post = os.path.join(args.aura_flight, "filter-post.txt")
    data_aura.save_filter_result(filter_post, t_store, data_dict2)
    
if args.sentera_flight:
    import data_sentera
    file_ins = os.path.join(args.sentera_flight, "filter-post-ins.txt")
    file_mag = os.path.join(args.sentera_flight, "filter-post-mag.txt")
    data_sentera.save_filter_result(file_ins, t_store, data_dict1)
    data_sentera.save_filter_result(file_mag, t_store, data_dict2)
    data_sentera.rewrite_pix4d_csv(args.sentera_flight, t_store, data_dict2)
    data_sentera.rewrite_image_metadata_txt(args.sentera_flight, t_store, data_dict2)

nsig = 3

# Plotting
if FLAG_PLOT_ATTITUDE:
    Patt1 = np.array(data_dict1.Patt, dtype=np.float64)
    Patt2 = np.array(data_dict2.Patt, dtype=np.float64)

    att_fig, att_ax = plt.subplots(3,2, sharex=True)

    # Roll PLot
    phi_nav = data_dict1.phi
    phi_nav_mag = data_dict2.phi
    att_ax[0,0].set_ylabel('Roll (deg)', weight='bold')
    att_ax[0,0].plot(t_flight, r2d(phi_flight), label='On Board', c='g', alpha=.5)
    att_ax[0,0].plot(t_store, r2d(phi_nav), label=filter1.name, c='r', alpha=.8)
    att_ax[0,0].plot(t_store, r2d(phi_nav_mag), label=filter2.name, c='b', alpha=.8)
    att_ax[0,0].grid()
    
    att_ax[0,1].plot(t_store,nsig*np.rad2deg(np.sqrt(Patt1[:,0])),c='r')
    att_ax[0,1].plot(t_store,-nsig*np.rad2deg(np.sqrt(Patt1[:,0])),c='r')
    att_ax[0,1].plot(t_store,nsig*np.rad2deg(np.sqrt(Patt2[:,0])),c='b')
    att_ax[0,1].plot(t_store,-nsig*np.rad2deg(np.sqrt(Patt2[:,0])),c='b')
    att_ax[0,1].set_ylabel('3*stddev', weight='bold')

    # Pitch PLot
    the_nav = data_dict1.the
    the_nav_mag = data_dict2.the
    att_ax[1,0].set_ylabel('Pitch (deg)', weight='bold')
    att_ax[1,0].plot(t_flight, r2d(the_flight), label='On Board', c='g', alpha=.5)
    att_ax[1,0].plot(t_store, r2d(the_nav), label=filter1.name, c='r', alpha=.8)
    att_ax[1,0].plot(t_store, r2d(the_nav_mag), label=filter2.name,c='b', alpha=.8)
    att_ax[1,0].grid()

    att_ax[1,1].plot(t_store,nsig*np.rad2deg(np.sqrt(Patt1[:,1])),c='r')
    att_ax[1,1].plot(t_store,-nsig*np.rad2deg(np.sqrt(Patt1[:,1])),c='r')
    att_ax[1,1].plot(t_store,nsig*np.rad2deg(np.sqrt(Patt2[:,1])),c='b')
    att_ax[1,1].plot(t_store,-nsig*np.rad2deg(np.sqrt(Patt2[:,1])),c='b')
    att_ax[1,1].set_ylabel('3*stddev', weight='bold')

    # Yaw Plot
    psi_nav = data_dict1.psi
    psi_nav_mag = data_dict2.psi
    att_ax[2,0].set_title(plotname, fontsize=10)
    att_ax[2,0].set_ylabel('Yaw (deg)', weight='bold')
    att_ax[2,0].plot(t_flight, r2d(psi_flight), label='On Board', c='g', alpha=.5)
    att_ax[2,0].plot(t_store, r2d(psi_nav), label=filter1.name, c='r', alpha=.8)
    att_ax[2,0].plot(t_store, r2d(psi_nav_mag), label=filter2.name,c='b', alpha=.8)
    att_ax[2,0].set_xlabel('Time (sec)', weight='bold')
    att_ax[2,0].grid()
    att_ax[2,0].legend(loc=1)
    
    att_ax[2,1].plot(t_store,nsig*np.rad2deg(np.sqrt(Patt1[:,2])),c='r')
    att_ax[2,1].plot(t_store,-nsig*np.rad2deg(np.sqrt(Patt1[:,2])),c='r')
    att_ax[2,1].plot(t_store,nsig*np.rad2deg(np.sqrt(Patt2[:,2])),c='b')
    att_ax[2,1].plot(t_store,-nsig*np.rad2deg(np.sqrt(Patt2[:,2])),c='b')
    att_ax[2,1].set_xlabel('Time (sec)', weight='bold')
    att_ax[2,1].set_ylabel('3*stddev', weight='bold')

if FLAG_PLOT_VELOCITIES:
    fig, [ax1, ax2, ax3] = plt.subplots(3,1, sharex=True)

    # vn Plot
    vn_nav = data_dict1.nav_vn
    vn_nav_mag = data_dict2.nav_vn
    ax1.set_title(plotname, fontsize=10)
    ax1.set_ylabel('vn (mps)', weight='bold')
    ax1.plot(t_gps, vn_gps, '-*', label='GPS Sensor', c='g', lw=2, alpha=.5)
    ax1.plot(t_flight, vn_flight, label='On Board', c='k', lw=2, alpha=.5)
    ax1.plot(t_store, vn_nav, label=filter1.name, c='r', lw=2, alpha=.8)
    ax1.plot(t_store, vn_nav_mag, label=filter2.name,c='b', lw=2, alpha=.8)
    ax1.grid()
    ax1.legend(loc=0)

    # ve Plot
    ve_nav = data_dict1.nav_ve
    ve_nav_mag = data_dict2.nav_ve
    ax2.set_ylabel('ve (mps)', weight='bold')
    ax2.plot(t_gps, ve_gps, '-*', label='GPS Sensor', c='g', lw=2, alpha=.5)
    ax2.plot(t_flight, ve_flight, label='On Board', c='k', lw=2, alpha=.5)
    ax2.plot(t_store, ve_nav, label=filter1.name, c='r', lw=2, alpha=.8)
    ax2.plot(t_store, ve_nav_mag, label=filter2.name,c='b', lw=2, alpha=.8)
    ax2.grid()

    # vd Plot
    vd_nav = data_dict1.nav_vd
    vd_nav_mag = data_dict2.nav_vd
    ax3.set_ylabel('vd (mps)', weight='bold')
    ax3.plot(t_gps, vd_gps, '-*', label='GPS Sensor', c='g', lw=2, alpha=.5)
    ax3.plot(t_flight, vd_flight, label='On Board', c='k', lw=2, alpha=.5)
    ax3.plot(t_store, vd_nav, label=filter1.name, c='r', lw=2, alpha=.8)
    ax3.plot(t_store, vd_nav_mag, label=filter2.name, c='b',lw=2, alpha=.8)
    ax3.set_xlabel('TIME (SECONDS)', weight='bold')
    ax3.grid()

# Altitude Plot
if FLAG_PLOT_ALTITUDE:
    navalt = data_dict1.nav_alt
    nav_magalt = data_dict2.nav_alt
    plt.figure()
    plt.title('ALTITUDE')
    plt.plot(t_gps, alt_gps, '-*', label='GPS Sensor', c='g', lw=2, alpha=.5)
    plt.plot(t_flight, navalt_flight, label='On Board', c='k', lw=2, alpha=.5)
    plt.plot(t_store, navalt, label=filter1.name, c='r', lw=2, alpha=.8)
    plt.plot(t_store, nav_magalt, label=filter2.name,c='b', lw=2, alpha=.8)
    plt.ylabel('ALTITUDE (METERS)', weight='bold')
    plt.legend(loc=0)
    plt.grid()


# Top View (Longitude vs. Latitude) Plot
if FLAG_PLOT_GROUNDTRACK:
    navlat = data_dict1.nav_lat
    navlon = data_dict1.nav_lon
    nav_maglat = data_dict2.nav_lat
    nav_maglon = data_dict2.nav_lon
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
    bias_ax[0,0].plot(t_store, r2d(data_dict1.p_bias), label=filter1.name, c='r')
    bias_ax[0,0].plot(t_store, r2d(data_dict2.p_bias), label=filter2.name, c='b')
    bias_ax[0,0].set_xlabel('Time (secs)', weight='bold')
    bias_ax[0,0].grid()
    
    bias_ax[1,0].set_ylabel('q Bias (deg)', weight='bold')
    bias_ax[1,0].plot(t_store, r2d(data_dict1.q_bias), label=filter1.name, c='r')
    bias_ax[1,0].plot(t_store, r2d(data_dict2.q_bias), label=filter2.name, c='b')
    bias_ax[1,0].set_xlabel('Time (secs)', weight='bold')
    bias_ax[1,0].grid()
    
    bias_ax[2,0].set_ylabel('r Bias (deg)', weight='bold')
    bias_ax[2,0].plot(t_store, r2d(data_dict1.r_bias), label=filter1.name, c='r')
    bias_ax[2,0].plot(t_store, r2d(data_dict2.r_bias), label=filter2.name, c='b')
    bias_ax[2,0].set_xlabel('Time (secs)', weight='bold')
    bias_ax[2,0].grid()
    
    # Accel Biases
    bias_ax[0,1].set_ylabel('ax Bias (m/s^2)', weight='bold')
    bias_ax[0,1].plot(t_store, data_dict1.ax_bias, label=filter1.name, c='r')
    bias_ax[0,1].plot(t_store, data_dict2.ax_bias, label=filter2.name, c='b')
    bias_ax[0,1].set_xlabel('Time (secs)', weight='bold')
    bias_ax[0,1].grid()
    
    bias_ax[1,1].set_ylabel('ay Bias (m/s^2)', weight='bold')
    bias_ax[1,1].plot(t_store, data_dict1.ay_bias, label=filter1.name, c='r')
    bias_ax[1,1].plot(t_store, data_dict2.ay_bias, label=filter2.name, c='b')
    bias_ax[1,1].set_xlabel('Time (secs)', weight='bold')
    bias_ax[1,1].grid()
    
    bias_ax[2,1].set_ylabel('az Bias (m/s^2)', weight='bold')
    bias_ax[2,1].plot(t_store, data_dict1.az_bias, label=filter1.name, c='r')
    bias_ax[2,1].plot(t_store, data_dict2.az_bias, label=filter2.name, c='b')
    bias_ax[2,1].set_xlabel('Time (secs)', weight='bold')
    bias_ax[2,1].grid()
    bias_ax[2,1].legend(loc=1)

plt.show()
