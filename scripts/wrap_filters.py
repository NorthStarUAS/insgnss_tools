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

parser = argparse.ArgumentParser(description='nav filter')
parser.add_argument('--aura-dir', help='load specified aura flight log')
parser.add_argument('--mat-flight', help='load specified .mat flight log')
parser.add_argument('--sentera-dir', help='load specified sentera flight log')
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
import nav_polarity
import nav_mag
import nav_eigen
import nav_eigen_mag
import nav_eigen_old
import nav_eigen_mag_old
import MadgwickAHRS

#filter2 = nav_eigen_mag_old.filter()
#filter1 = nav_orig.filter()
filter1 = nav_eigen.filter()
filter2 = nav_eigen_mag.filter()
#filter2 = MadgwickAHRS.filter()

import pydefs
insgps1 = pydefs.INSGPS(0, 0.0, np.zeros(3), np.zeros(3), np.zeros(3),
                       np.zeros(3), np.zeros(3), np.eye(15), np.zeros(6))
insgps2 = pydefs.INSGPS(0, 0.0, np.zeros(3), np.zeros(3), np.zeros(3),
                        np.zeros(3), np.zeros(3), np.eye(15), np.zeros(6))

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
        self.navStatus = []

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
        self.psi.append(insgps.estATT[0])
        self.the.append(insgps.estATT[1])
        self.phi.append(insgps.estATT[2])
        self.nav_lat.append(insgps.estPOS[0])
        self.nav_lon.append(insgps.estPOS[1])
        self.nav_alt.append(insgps.estPOS[2])
        self.nav_vn.append(insgps.estVEL[0])
        self.nav_ve.append(insgps.estVEL[1])
        self.nav_vd.append(insgps.estVEL[2])
        self.navStatus.append(insgps.valid) #fixme: was err_type

        self.ax_bias.append(insgps.estAB[0])
        self.ay_bias.append(insgps.estAB[1])
        self.az_bias.append(insgps.estAB[2])
        self.p_bias.append(insgps.estGB[0])
        self.q_bias.append(insgps.estGB[1])
        self.r_bias.append(insgps.estGB[2])
        
        self.Pp.append( np.diag(insgps.P[0:3,0:3]) )
        self.Pvel.append( np.diag(insgps.P[3:6,3:6]) )
        self.Patt.append( np.diag(insgps.P[6:9,6:9]) )
        self.Pab.append( np.diag(insgps.P[9:12,9:12]) )
        self.Pgb.append( np.diag(insgps.P[12:15,12:15]) )

# Values (Calculated by compiled test navigation filter) need to be
# stored in python variables and they need to be in the globaldefs.c
# and cdefs.py to allow for pulling them out and saving. These
# python variables need to be initialized to work properly in the
# while loop.
data_dict1 = data_store()
data_dict2 = data_store()
t_store = []

import data_aura
import data_sentera
import data_umn
if args.aura_dir:
    imu_data, gps_data, filter_data = data_aura.load(args.aura_dir)
    plotname = os.path.basename(args.aura_dir)
elif args.sentera_dir:
    imu_data, gps_data, filter_data = data_sentera.load(args.sentera_dir)
    plotname = os.path.basename(args.sentera_dir)
elif args.mat_flight:
    imu_data, gps_data, filter_data = data_umn.load(args.mat_flight)
    plotname = os.path.basename(args.mat_flight)
else:
    print "no input file / dir specifed"
    quit()

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
for g in gps_data:
    t_gps.append(g.time)
    lat_gps.append(g.lat)
    lon_gps.append(g.lon)
    alt_gps.append(g.alt)

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
        if imupt.time > filter_data[filter_index].time:
            filter_index += 1
        if filter_index >= len(filter_data):
            # no more filter data, stay on the last record
            filter_index = len(filter_data)-1
        filterpt = filter_data[filter_index]
        #print "t(imu) = " + str(imupt.time) + " t(gps) = " + str(gpspt.time)

        # If k is at the initialization time init_nav else get_nav
        if not filter_init and gps_index > 0:
            print "init:", imupt.time, gpspt.time
            insgps1 = filter1.init(imupt, gpspt)
            filter_init = True
        elif filter_init:
            insgps1 = filter1.update(imupt, gpspt)

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
            insgps2 = filter2.init(imupt, gpspt)
            filter_init = True
        elif filter_init:
            insgps2 = filter2.update(imupt, gpspt)

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

if args.aura_dir:
    filter_post = os.path.join(args.aura_dir, "filter-post.txt")
    data_aura.save_filter_result(filter_post, t_store, data_dict2)
    
if args.sentera_dir:
    file_ins = os.path.join(args.sentera_dir, "filter-post-ins.txt")
    file_mag = os.path.join(args.sentera_dir, "filter-post-mag.txt")
    data_sentera.save_filter_result(file_ins, t_store, data_dict1)
    data_sentera.save_filter_result(file_mag, t_store, data_dict2)
    data_sentera.rewrite_pix4d_csv(args.sentera_dir, t_store, data_dict2)
    data_sentera.rewrite_image_metadata_txt(args.sentera_dir, t_store, data_dict2)

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
    att_ax[0,0].plot(t_store, r2d(phi_nav), label='nav', c='red')
    att_ax[0,0].plot(t_store, r2d(phi_nav_mag), label='nav_mag', c='blue')
    att_ax[0,0].plot(t_flight, r2d(phi_flight), label='On-Board', c='green', alpha=.5)
    att_ax[0,0].grid()
    
    att_ax[0,1].plot(t_store,nsig*np.rad2deg(np.sqrt(Patt1[:,0])),c='red')
    att_ax[0,1].plot(t_store,-nsig*np.rad2deg(np.sqrt(Patt1[:,0])),c='red')
    att_ax[0,1].plot(t_store,nsig*np.rad2deg(np.sqrt(Patt2[:,0])),c='blue')
    att_ax[0,1].plot(t_store,-nsig*np.rad2deg(np.sqrt(Patt2[:,0])),c='blue')
    att_ax[0,1].set_ylabel('3*stddev', weight='bold')

    # Pitch PLot
    the_nav = data_dict1.the
    the_nav_mag = data_dict2.the
    att_ax[1,0].set_ylabel('Pitch (deg)', weight='bold')
    att_ax[1,0].plot(t_store, r2d(the_nav), label='nav', c='red')
    att_ax[1,0].plot(t_store, r2d(the_nav_mag), label='nav_mag',c='blue')
    att_ax[1,0].plot(t_flight, r2d(the_flight), label='On-Board', c='green', alpha=.5)
    att_ax[1,0].grid()

    att_ax[1,1].plot(t_store,nsig*np.rad2deg(np.sqrt(Patt1[:,1])),c='red')
    att_ax[1,1].plot(t_store,-nsig*np.rad2deg(np.sqrt(Patt1[:,1])),c='red')
    att_ax[1,1].plot(t_store,nsig*np.rad2deg(np.sqrt(Patt2[:,1])),c='blue')
    att_ax[1,1].plot(t_store,-nsig*np.rad2deg(np.sqrt(Patt2[:,1])),c='blue')
    att_ax[1,1].set_ylabel('3*stddev', weight='bold')

    # Yaw Plot
    psi_nav = data_dict1.psi
    psi_nav_mag = data_dict2.psi
    att_ax[2,0].set_title(plotname, fontsize=10)
    att_ax[2,0].set_ylabel('Yaw (deg)', weight='bold')
    att_ax[2,0].plot(t_store, r2d(psi_nav), label='EKF', c='red')
    att_ax[2,0].plot(t_store, r2d(psi_nav_mag), label='EKF+Mag',c='blue')
    att_ax[2,0].plot(t_flight, r2d(psi_flight), label='On-Board', c='green', alpha=.5)
    att_ax[2,0].set_xlabel('Time (sec)', weight='bold')
    att_ax[2,0].grid()
    att_ax[2,0].legend(loc=1)
    
    att_ax[2,1].plot(t_store,nsig*np.rad2deg(np.sqrt(Patt1[:,2])),c='red')
    att_ax[2,1].plot(t_store,-nsig*np.rad2deg(np.sqrt(Patt1[:,2])),c='red')
    att_ax[2,1].plot(t_store,nsig*np.rad2deg(np.sqrt(Patt2[:,2])),c='blue')
    att_ax[2,1].plot(t_store,-nsig*np.rad2deg(np.sqrt(Patt2[:,2])),c='blue')
    att_ax[2,1].set_xlabel('Time (sec)', weight='bold')
    att_ax[2,1].set_ylabel('3*stddev', weight='bold')

if FLAG_PLOT_VELOCITIES:
    fig, [ax1, ax2, ax3] = plt.subplots(3,1)

    # vn Plot
    vn_nav = data_dict1.nav_vn
    vn_nav_mag = data_dict2.nav_vn
    ax1.set_title(plotname, fontsize=10)
    ax1.set_ylabel('vn (mps)', weight='bold')
    ax1.plot(t_store, vn_nav, label='nav', c='k', lw=3, alpha=.5)
    ax1.plot(t_store, vn_nav_mag, label='nav_mag',c='blue', lw=2)
    ax1.plot(t_flight, vn_flight, label='On-Board', c='green', lw=2, alpha=.5)
    ax1.grid()
    ax1.legend(loc=0)

    # ve Plot
    ve_nav = data_dict1.nav_ve
    ve_nav_mag = data_dict2.nav_ve
    ax2.set_ylabel('ve (mps)', weight='bold')
    ax2.plot(t_store, ve_nav, label='nav', c='k', lw=3, alpha=.5)
    ax2.plot(t_store, ve_nav_mag, label='nav_mag',c='blue', lw=2)
    ax2.plot(t_flight, ve_flight, label='On-Board', c='green', lw=2, alpha=.5)
    ax2.grid()

    # vd Plot
    vd_nav = data_dict1.nav_vd
    vd_nav_mag = data_dict2.nav_vd
    ax3.set_ylabel('vd (mps)', weight='bold')
    ax3.plot(t_store, vd_nav, label='nav', c='k', lw=3, alpha=.5)
    ax3.plot(t_store, vd_nav_mag, label='nav_mag', c='blue',lw=2)
    ax3.plot(t_flight, vd_flight, label='On-Board', c='green', lw=2, alpha=.5)
    ax3.set_xlabel('TIME (SECONDS)', weight='bold')
    ax3.grid()

# Altitude Plot
if FLAG_PLOT_ALTITUDE:
    navalt = data_dict1.nav_alt
    nav_magalt = data_dict2.nav_alt
    plt.figure()
    plt.title('ALTITUDE')
    plt.plot(t_gps, alt_gps, '-*', label='GPS Sensor', c='green', lw=3, alpha=.5)
    plt.plot(t_flight, navalt_flight, label='On-Board', c='green', lw=2, alpha=.5)
    plt.plot(t_store, navalt, label='nav', c='k', lw=3, alpha=.5)
    plt.plot(t_store, nav_magalt, label='nav_mag',c='blue', lw=2)
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
    plt.plot(lon_gps, lat_gps, '*', label='GPS Sensor', c='red', lw=2, alpha=.5)
    plt.plot(r2d(navlon_flight), r2d(navlat_flight), label='On-Board', c='green', lw=1, alpha=.85)
    plt.plot(r2d(navlon), r2d(navlat), label='nav', c='k', lw=3, alpha=.5)
    plt.plot(r2d(nav_maglon), r2d(nav_maglat), label='nav_mag', c='blue', lw=2)
    plt.grid()
    plt.legend(loc=0)
    
if FLAG_PLOT_BIASES:
    bias_fig, bias_ax = plt.subplots(3,2, sharex=True)

    # Gyro Biases
    bias_ax[0,0].set_ylabel('p Bias (deg)', weight='bold')
    bias_ax[0,0].plot(t_store, r2d(data_dict1.p_bias), label='nav', c='red')
    bias_ax[0,0].plot(t_store, r2d(data_dict2.p_bias), label='nav_mag', c='blue')
    bias_ax[0,0].set_xlabel('Time (secs)', weight='bold')
    bias_ax[0,0].grid()
    
    bias_ax[1,0].set_ylabel('q Bias (deg)', weight='bold')
    bias_ax[1,0].plot(t_store, r2d(data_dict1.q_bias), label='nav', c='red')
    bias_ax[1,0].plot(t_store, r2d(data_dict2.q_bias), label='nav_mag', c='blue')
    bias_ax[1,0].set_xlabel('Time (secs)', weight='bold')
    bias_ax[1,0].grid()
    
    bias_ax[2,0].set_ylabel('r Bias (deg)', weight='bold')
    bias_ax[2,0].plot(t_store, r2d(data_dict1.r_bias), label='nav', c='red')
    bias_ax[2,0].plot(t_store, r2d(data_dict2.r_bias), label='nav_mag', c='blue')
    bias_ax[2,0].set_xlabel('Time (secs)', weight='bold')
    bias_ax[2,0].grid()
    
    # Accel Biases
    bias_ax[0,1].set_ylabel('ax Bias (m/s^2)', weight='bold')
    bias_ax[0,1].plot(t_store, data_dict1.ax_bias, label='nav', c='red')
    bias_ax[0,1].plot(t_store, data_dict2.ax_bias, label='nav_mag', c='blue')
    bias_ax[0,1].set_xlabel('Time (secs)', weight='bold')
    bias_ax[0,1].grid()
    
    bias_ax[1,1].set_ylabel('ay Bias (m/s^2)', weight='bold')
    bias_ax[1,1].plot(t_store, data_dict1.ay_bias, label='nav', c='red')
    bias_ax[1,1].plot(t_store, data_dict2.ay_bias, label='nav_mag', c='blue')
    bias_ax[1,1].set_xlabel('Time (secs)', weight='bold')
    bias_ax[1,1].grid()
    
    bias_ax[2,1].set_ylabel('az Bias (m/s^2)', weight='bold')
    bias_ax[2,1].plot(t_store, data_dict1.az_bias, label='nav', c='red')
    bias_ax[2,1].plot(t_store, data_dict2.az_bias, label='nav_mag', c='blue')
    bias_ax[2,1].set_xlabel('Time (secs)', weight='bold')
    bias_ax[2,1].grid()

plt.show()
