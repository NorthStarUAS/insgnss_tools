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
FLAG_PLOT_WIND     = True
# FLAG_PLOT_HAVEGPS  = True
FLAG_PLOT_SIGNALS  = True
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
import MadgwickAHRS

#filter1 = nav_orig.filter()
#filter2 = nav_eigen.filter()

filter1 = nav_eigen.filter()
filter2 = nav_eigen_mag.filter()
#filter2 = MadgwickAHRS.filter()

import pydefs
insgps1 = pydefs.INSGPS(0, 0.0, np.zeros(3), np.zeros(3), np.zeros(3),
                       np.zeros(3), np.zeros(3), np.eye(15), np.zeros(6))
insgps2 = pydefs.INSGPS(0, 0.0, np.zeros(3), np.zeros(3), np.zeros(3),
                           np.zeros(3), np.zeros(3), np.eye(15), np.zeros(6))

class dict2struct():
    pass

# Values (Calculated by compiled test navigation filter) need to be
# stored in python variables and they need to be in the globaldefs.c
# and cdefs.py to allow for pulling them out and saving. These
# python variables need to be initialized to work properly in the
# while loop.
data_dict1 = {}
data_dict2 = {}
haveGPS_store = []
t_store = []

def store_data(data_dict, insgps):
    """
    Append current elements from `insgps` into
    `data_dict`.  
    """
    # Initialize dictionary if needed (e.g.) first iteration.
    if len(data_dict) == 0:
        data_dict['psi_store'] = []
        data_dict['psi_store'] = []
        data_dict['the_store'] = []
        data_dict['phi_store'] = []
        data_dict['navlat_store'] = []
        data_dict['navlon_store'] = []
        data_dict['navalt_store'] = []
        data_dict['nav_vn_store'] = []
        data_dict['nav_ve_store'] = []
        data_dict['nav_vd_store'] = []
        data_dict['navStatus_store'] = []
        data_dict['wn_store'] = []
        data_dict['we_store'] = []
        data_dict['wd_store'] = []
        data_dict['signal_store'] = []

        data_dict['ax_bias'] = []
        data_dict['ay_bias'] = [] 
        data_dict['az_bias'] = []
        data_dict['p_bias'] = []
        data_dict['q_bias'] = []
        data_dict['r_bias'] = []

        data_dict['NS_std'] = []
        data_dict['WE_std'] = []
        data_dict['alt_std'] = []

        # Attitude errors (small angle errors about N-E-D)
        # Note: epsN and epsE are in general different than roll, pitch uncertainty.  
        data_dict['epsN_std'] = []
        data_dict['epsE_std'] = []
        data_dict['epsD_std'] = [] # yaw uncertainty [rad]

    # Store data
    data_dict['psi_store'].append(insgps.estATT[0])
    data_dict['the_store'].append(insgps.estATT[1])
    data_dict['phi_store'].append(insgps.estATT[2])
    data_dict['navlat_store'].append(insgps.estPOS[0])
    data_dict['navlon_store'].append(insgps.estPOS[1])
    data_dict['navalt_store'].append(insgps.estPOS[2])
    data_dict['nav_vn_store'].append(insgps.estVEL[0])
    data_dict['nav_ve_store'].append(insgps.estVEL[1])
    data_dict['nav_vd_store'].append(insgps.estVEL[2])
    data_dict['navStatus_store'].append(insgps.valid) #fixme: was err_type
    #data_dict['wn_store'].append(insgps.wn)
    #data_dict['we_store'].append(insgps.we)
    #data_dict['wd_store'].append(insgps.wd)
    #data_dict['signal_store'].append([insgps.signal_0, insgps.signal_1,
    #                                  insgps.signal_2, insgps.signal_3,
    #                                  insgps.signal_4, insgps.signal_5, 
    #                                  insgps.signal_6, insgps.signal_7,
    #                                  insgps.signal_8, insgps.signal_9])

    data_dict['ax_bias'].append(insgps.estAB[0])
    data_dict['ay_bias'].append(insgps.estAB[1])
    data_dict['az_bias'].append(insgps.estAB[2])
    data_dict['p_bias'].append(insgps.estGB[0])
    data_dict['q_bias'].append(insgps.estGB[1])
    data_dict['r_bias'].append(insgps.estGB[2])

    data_dict['NS_std'].append(np.sqrt(insgps.P[0]))
    data_dict['WE_std'].append(np.sqrt(insgps.P[1]))
    data_dict['alt_std'].append(np.sqrt(insgps.P[2]))

    data_dict['epsN_std'].append(np.sqrt(insgps.P[6]))
    data_dict['epsE_std'].append(np.sqrt(insgps.P[7]))
    data_dict['epsD_std'].append(np.sqrt(insgps.P[8])) # yaw uncertainty [rad]

    return data_dict

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
            data_dict1 = store_data(data_dict1, insgps1)
            data_dict2 = store_data(data_dict2, insgps2)
            # haveGPS_store.append(mission.haveGPS)
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
            filter_init = True
        elif filter_init:
            insgps1 = filter1.update(imupt, gpspt)

        # Store the desired results obtained from the compiled test
        # navigation filter and the baseline filter
        if filter_init:
            data_dict1 = store_data(data_dict1, insgps1)
            # haveGPS_store.append(mission.haveGPS)
            t_store.append(imupt.time)

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
            data_dict2 = store_data(data_dict2, insgps2)
            # haveGPS_store.append(mission.haveGPS)
            # not on second run ... t_store.append(imupt.time)

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
        
       
# Plotting
if FLAG_PLOT_ATTITUDE:
    fig, [ax1, ax2, ax3] = plt.subplots(3,1)

    # Yaw Plot
    psi_nav = data_dict1['psi_store']
    psi_nav_mag = data_dict2['psi_store']
    ax1.set_title(plotname, fontsize=10)
    ax1.set_ylabel('YAW (DEGREES)', weight='bold')
    ax1.plot(t_store, r2d(psi_nav), label='nav', c='k', lw=3, alpha=.5)
    ax1.plot(t_store, r2d(psi_nav_mag), label='nav_mag',c='blue', lw=2)
    ax1.plot(t_flight, r2d(psi_flight), label='On-Board', c='green', lw=2, alpha=.5)
    ax1.grid()
    ax1.legend(loc=0)

    # Pitch PLot
    the_nav = data_dict1['the_store']
    the_nav_mag = data_dict2['the_store']  
    ax2.set_ylabel('PITCH (DEGREES)', weight='bold')
    ax2.plot(t_store, r2d(the_nav), label='nav', c='k', lw=3, alpha=.5)
    ax2.plot(t_store, r2d(the_nav_mag), label='nav_mag',c='blue', lw=2)
    ax2.plot(t_flight, r2d(the_flight), label='On-Board', c='green', lw=2, alpha=.5)
    ax2.grid()

    # Roll PLot
    phi_nav = data_dict1['phi_store']
    phi_nav_mag = data_dict2['phi_store']   
    ax3.set_ylabel('ROLL (DEGREES)', weight='bold')
    ax3.plot(t_store, r2d(phi_nav), label='nav', c='k', lw=3, alpha=.5)
    ax3.plot(t_store, r2d(phi_nav_mag), label='nav_mag', c='blue',lw=2)
    ax3.plot(t_flight, r2d(phi_flight), label='On-Board', c='green', lw=2, alpha=.5)
    ax3.set_xlabel('TIME (SECONDS)', weight='bold')
    ax3.grid()

if FLAG_PLOT_VELOCITIES:
    fig, [ax1, ax2, ax3] = plt.subplots(3,1)

    # vn Plot
    vn_nav = data_dict1['nav_vn_store']
    vn_nav_mag = data_dict2['nav_vn_store']
    ax1.set_title(plotname, fontsize=10)
    ax1.set_ylabel('vn (mps)', weight='bold')
    ax1.plot(t_store, vn_nav, label='nav', c='k', lw=3, alpha=.5)
    ax1.plot(t_store, vn_nav_mag, label='nav_mag',c='blue', lw=2)
    ax1.plot(t_flight, vn_flight, label='On-Board', c='green', lw=2, alpha=.5)
    ax1.grid()
    ax1.legend(loc=0)

    # ve PLot
    ve_nav = data_dict1['nav_ve_store']
    ve_nav_mag = data_dict2['nav_ve_store']  
    ax2.set_ylabel('ve (mps)', weight='bold')
    ax2.plot(t_store, ve_nav, label='nav', c='k', lw=3, alpha=.5)
    ax2.plot(t_store, ve_nav_mag, label='nav_mag',c='blue', lw=2)
    ax2.plot(t_flight, ve_flight, label='On-Board', c='green', lw=2, alpha=.5)
    ax2.grid()

    # vd PLot
    vd_nav = data_dict1['nav_vd_store']
    vd_nav_mag = data_dict2['nav_vd_store']   
    ax3.set_ylabel('vd (mps)', weight='bold')
    ax3.plot(t_store, vd_nav, label='nav', c='k', lw=3, alpha=.5)
    ax3.plot(t_store, vd_nav_mag, label='nav_mag', c='blue',lw=2)
    ax3.plot(t_flight, vd_flight, label='On-Board', c='green', lw=2, alpha=.5)
    ax3.set_xlabel('TIME (SECONDS)', weight='bold')
    ax3.grid()

# Altitude Plot
if FLAG_PLOT_ALTITUDE:
    navalt = data_dict1['navalt_store']
    nav_magalt = data_dict2['navalt_store']
    plt.figure()
    plt.title('ALTITUDE')
    plt.plot(t_gps, alt_gps, '-*', label='GPS Sensor', c='green', lw=3, alpha=.5)
    plt.plot(t_flight, navalt_flight, label='On-Board', c='green', lw=2, alpha=.5)
    plt.plot(t_store, navalt, label='nav', c='k', lw=3, alpha=.5)
    plt.plot(t_store, nav_magalt, label='nav_mag',c='blue', lw=2)
    plt.ylabel('ALTITUDE (METERS)', weight='bold')
    plt.legend(loc=0)
    plt.grid()

# Wind Plot
#if FLAG_PLOT_WIND:
#    wn = data_dict2['wn_store']
#    we = data_dict2['we_store']
#    wd = data_dict2['wd_store']
#    plt.figure()
#    plt.title('WIND ESTIMATES - Only from nav_mag')
#    plt.plot(t_store, wn, label='North',c='gray', lw=2)
#    plt.plot(t_store, we, label='East',c='black', lw=2)
#    plt.plot(t_store, wd, label='Down',c='blue', lw=2)
#    plt.ylabel('WIND (METERS/SECOND)', weight='bold')
#    plt.legend(loc=0)
#    plt.grid()

# Top View (Longitude vs. Latitude) Plot
if FLAG_PLOT_GROUNDTRACK:
    navlat = data_dict1['navlat_store']
    navlon = data_dict1['navlon_store']
    nav_maglat = data_dict2['navlat_store']
    nav_maglon = data_dict2['navlon_store']
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

# if FLAG_PLOT_SIGNALS:
#     plt.figure()
#     plt.title('SIGNAL PLOTS - Only from nav_mag')
#     signal_store = data_dict2['signal_store']
#     signal_store = np.array(signal_store)
#     for sig in SIGNAL_LIST:
#         plt.plot(t_store, signal_store[:,sig], label=str(sig), lw=2, alpha=.5)
#     plt.ylabel('SIGNAL UNITS', weight='bold')
#     plt.legend(loc=0)
#     plt.grid()

# haveGPS Plot
# if FLAG_PLOT_HAVEGPS:
#   plt.figure()
#   plt.title('MISSION HAVEGPS FLAG')
#   plt.plot(t_store, haveGPS_store, c='black', lw=2)
#   plt.ylim([-2,2])
#   plt.grid()

plt.show()


# Save Results to CSV File
if FLAG_WRITE2CSV:
    OUTPUT_FILENAME = filepath + '_postprocess.csv'
    hdr_list = ['OMAP Timestamp (microseconds since epoch)', 
                'Lat (1e-7 deg)', 'Lon (1e-7 deg)', 'Alt (m)',
                'Aircraft Roll (1e-4 rad)', 'Aircraft Pitch (1e-4 rad)', 'Aircraft Yaw (1e-4 rad)',
                'North-South std (m)', 'West-East std (m)', 'Alt std (m)',
                'Yaw std (deg)', 'Pitch std (deg)', 'Roll std (deg)']
    with open(OUTPUT_FILENAME, 'w') as fobj:
        # TODO: Print header
        csv_writer = csv.writer(fobj)
        csv_writer.writerow(hdr_list)
        for k in range(len(t_store)):
            # Convert eps_NED to eps_YPR
            yaw_rad   = data_dict1['psi_store'][k]
            pitch_rad = data_dict1['the_store'][k]
            roll_rad  = data_dict1['phi_store'][k]

            # Note, as part of transformation we are
            # ignoring uncertinty in the mapping.
            epsNED_std_deg = [r2d(data_dict1['epsN_std'][k]),
                              r2d(data_dict1['epsE_std'][k]),
                              r2d(data_dict1['epsD_std'][k])]
            yaw_std_deg = epsNED_std_deg[2]
            pitch_std_deg = navpy.angle2dcm(yaw_rad, 0, 0, input_unit='rad').dot(epsNED_std_deg)[1]
            roll_std_deg = navpy.angle2dcm(yaw_rad, pitch_rad, 0, input_unit='rad').dot(epsNED_std_deg)[0]

            row = [int(t_store[k]*1e6), 
                   int(r2d(data_dict1['navlat_store'][k])*1e7),
                   int(r2d(data_dict1['navlon_store'][k])*1e7),
                   data_dict1['navalt_store'][k],
                   int(roll_rad*1e4),
                   int(pitch_rad*1e4),
                   int(yaw_rad*1e4),
                   data_dict1['NS_std'][k],
                   data_dict1['WE_std'][k],
                   data_dict1['alt_std'][k],
                   yaw_std_deg,
                   pitch_std_deg,
                   roll_std_deg]
            csv_writer.writerow(row)
    print("Playback results written to: %s" % OUTPUT_FILENAME)
