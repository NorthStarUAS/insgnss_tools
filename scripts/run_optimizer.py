#!/usr/bin/python

"""run_filters.py

This script plays flight data through the selected navigation filters.
The filters are compiled as .so objects and wrapped for python with boos.

A set of customizable input flags are defined at the start of the script.

Initial revision: Hamid M.
Many updates: Curtis L. Olson
"""

import argparse
import math
import numpy as np
import time
import os
from scipy.optimize import minimize

import navpy

import flight_data

parser = argparse.ArgumentParser(description='nav filter')
parser.add_argument('--flight', help='load specified aura flight log')
parser.add_argument('--aura-flight', help='load specified aura flight log')
parser.add_argument('--umn-flight', help='load specified .mat flight log')
parser.add_argument('--sentera-flight', help='load specified sentera flight log')
parser.add_argument('--sentera2-flight', help='load specified sentera2 flight log')
args = parser.parse_args()

# # # # # START INPUTS # # # # #

FLAG_PLOT_ATTITUDE = True
FLAG_PLOT_VELOCITIES = True
FLAG_PLOT_GROUNDTRACK = True
FLAG_PLOT_ALTITUDE = True
FLAG_PLOT_BIASES = True
SIGNAL_LIST = [0, 1, 8]  # List of signals [0 to 9] to be plotted
FLAG_WRITE2CSV = False # Write results to CSV file.
# # # # # END INPUTS # # # # #

import csv
from matplotlib import pyplot as plt

# filter interfaces
import nav_orig
import nav_mag
import nav_eigen
import nav_eigen_mag
import nav_openloop
import MadgwickAHRS

filter_ref = nav_eigen_mag.filter()
filter_opt = nav_openloop.filter()

# this class organizes the filter output in a way that is more
# convenient and dirct for matplotlib
class data_store():
    def __init__(self):
        self.time = []
        self.psi = []
        self.the = []
        self.phi = []
        self.lat = []
        self.lon = []
        self.alt = []
        self.vn = []
        self.ve = []
        self.vd = []

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
        r2d = 180.0 / math.pi
        self.time.append(insgps.time)
        
        self.psi.append(insgps.psi)
        self.the.append(insgps.the)
        self.phi.append(insgps.phi)
        self.lat.append(insgps.lat*r2d)
        self.lon.append(insgps.lon*r2d)
        self.alt.append(insgps.alt)
        self.vn.append(insgps.vn)
        self.ve.append(insgps.ve)
        self.vd.append(insgps.vd)

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

    def append_from_filter(self, filterpt):
        r2d = 180.0 / math.pi
        self.time.append(filterpt.time)
        self.psi.append(filterpt.psi)
        self.the.append(filterpt.the)
        self.phi.append(filterpt.phi)
        self.lat.append(filterpt.lat*r2d)
        self.lon.append(filterpt.lon*r2d)
        self.alt.append(filterpt.alt)
        self.vn.append(filterpt.vn)
        self.ve.append(filterpt.ve)
        self.vd.append(filterpt.vd)
        
    def append_from_gps(self, gpspt):
        self.time.append(gpspt.time)
        self.lat.append(gpspt.lat)
        self.lon.append(gpspt.lon)
        self.alt.append(gpspt.alt)
        self.vn.append(gpspt.vn)
        self.ve.append(gpspt.ve)
        self.vd.append(gpspt.vd)
        
def run_filter(filter, imu_data, gps_data, filter_data, config=None):
    data_dict = data_store()
    errors = []
    
    # Using while loop starting at k (set to kstart) and going to end
    # of .mat file
    run_start = time.time()
    gps_index = 0
    filter_index = 0
    new_gps = 0
    if config and config['call_init']:
        filter_init = False
    else:
        filter_init = True
    if config and 'start_time' in config:
        for k, imu_pt in enumerate(imu_data):
            if imu_pt.time >= config['start_time']:
                k_start = k
                break
    else:
        k_start = 0
    if config and 'end_time' in config:
        for k, imu_pt in enumerate(imu_data):
            if imu_pt.time >= config['end_time']:
                k_end = k
                break
    else:
        k_end = len(imu_data)
    #print k_start, k_end
    for k in range(k_start, k_end):
        imupt = imu_data[k]
        if gps_index < len(gps_data) - 1:
            # walk the gps counter forward as needed
            newData = 0
            while gps_data[gps_index+1].time <= imupt.time:
                gps_index += 1
                newData = 1
            gpspt = gps_data[gps_index]
            gpspt.newData = newData
        else:
            # no more gps data, stay on the last record
            gpspt = gps_data[gps_index]
            gpspt.newData = 0
        #print gpspt.time`
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
            navpt = filter.init(imupt, gpspt, filterpt)
            filter_init = True
        elif filter_init:
            navpt = filter.update(imupt, gpspt, filterpt)

        # Store the desired results obtained from the compiled test
        # navigation filter and the baseline filter
        if filter_init:
            data_dict.append(navpt)

        if gpspt.newData:
            # compute error metric with each new gps report
            p1 = navpy.lla2ecef(gpspt.lat, gpspt.lon, gpspt.alt,
                                latlon_unit='deg')
            p2 = navpy.lla2ecef(navpt.lat, navpt.lon, navpt.alt,
                                latlon_unit='rad')
            pe = np.linalg.norm(p1 - p2)
            #print gpspt.time
            #print 'gps:', gpspt.lat, gpspt.lon, gpspt.alt
            #print 'nav:', navpt.lat, navpt.lon, navpt.alt
            #print p1, p2, pe

            # it is always tempting to fit to the velocity vector
            # (especially when seeing some of the weird velocity fits
            # that the optimizer spews out), but it never helps
            # ... seems to make the solution convergence much more
            # shallow, ultimately never seems to produce a better fit
            # than using position directly.  Weird fits happen when
            # the inertial errors just don't fit the gps errors.
            # Fitting to velocity doesn't seem to improve that
            # problem.
            v1 = np.array( [gpspt.vn, gpspt.ve, gpspt.vd] )
            v2 = np.array( [navpt.vn, navpt.ve, navpt.vd] )
            ve = np.linalg.norm(v1 - v2)
            
            errors.append(pe)   # 3d position error
            
        # Increment time up one step for the next iteration of the
        # while loop.
        k += 1

    # proper cleanup
    filter.close()
    run_end = time.time()
    elapsed_sec = run_end - run_start
    # print 'iteration:', elapsed_sec
    return errors, data_dict, elapsed_sec

att_fig, att_ax = plt.subplots(3,1, sharex=True)
att_ax[0].grid()
att_ax[1].grid()
att_ax[2].grid()
att_ax[2].set_xlabel('Time (sec)', weight='bold')

vel_fig, vel_ax = plt.subplots(3,1, sharex=True)
vel_ax[0].grid()
vel_ax[1].grid()
vel_ax[2].grid()
vel_ax[2].set_xlabel('Time (sec)', weight='bold')

pos_fig, pos_ax = plt.subplots(3,1, sharex=True)
pos_ax[0].grid()
pos_ax[1].grid()
pos_ax[2].grid()
pos_ax[2].set_xlabel('Time (sec)', weight='bold')

bias_fig, bias_ax = plt.subplots(3,2, sharex=True)

def update_plot(data_dict, label='Unknown', ls='-', marker=' ', c='g', alpha=0.5):
    plt.ion()
    r2d = np.rad2deg
    nsig = 3
    
    t_flight = data_dict.time
    
    # Roll Plot
    if len(data_dict.phi):
        phi = data_dict.phi
        att_ax[0].set_ylabel('Roll (deg)', weight='bold')
        att_ax[0].plot(t_flight, r2d(phi), label=label, ls=ls, c=c, alpha=alpha)

    # Pitch Plot
    if len(data_dict.the):
        the = data_dict.the
        att_ax[1].set_ylabel('Pitch (deg)', weight='bold')
        att_ax[1].plot(t_flight, r2d(the), label=label, ls=ls, c=c, alpha=alpha)

    # Yaw Plot
    if len(data_dict.psi):
        psi = data_dict.psi
        att_ax[2].set_title(plotname, fontsize=10)
        att_ax[2].set_ylabel('Yaw (deg)', weight='bold')
        att_ax[2].plot(t_flight, r2d(psi), label=label, ls=ls, c=c, alpha=alpha)

    att_ax[2].legend(loc=1)
    att_fig.canvas.draw()

    # vn Plot
    print label, len(data_dict.vn)
    vn = data_dict.vn
    vel_ax[0].set_title(plotname, fontsize=10)
    vel_ax[0].set_ylabel('vn (mps)', weight='bold')
    vel_ax[0].plot(t_flight, vn, ls=ls, marker=marker, label=label, c=c, lw=2, alpha=alpha)
    vel_ax[0].legend(loc=0)

    # ve Plot
    ve = data_dict.ve
    vel_ax[1].set_ylabel('ve (mps)', weight='bold')
    vel_ax[1].plot(t_flight, ve, ls=ls, marker=marker, label=label, c=c, lw=2, alpha=alpha)

    # vd Plot
    vd = data_dict.vd
    vel_ax[2].set_ylabel('vd (mps)', weight='bold')
    vel_ax[2].plot(t_flight, vd, ls=ls, marker=marker, label=label, c=c, lw=2, alpha=alpha)

    vel_fig.canvas.draw()
    
    # lat plot
    lat = data_dict.lat
    pos_ax[0].set_title('Position')
    pos_ax[0].set_ylabel('Lat (deg)', weight='bold')
    pos_ax[0].plot(t_flight, lat, ls=ls, marker=marker, label=label, c=c, lw=2, alpha=alpha)
    pos_ax[0].legend(loc=0)

    # lon plot
    lon = data_dict.lon
    pos_ax[1].set_ylabel('Lon (deg)', weight='bold')
    pos_ax[1].plot(t_flight, lon, ls=ls, marker=marker, label=label, c=c, lw=2, alpha=alpha)

    # alt plot
    alt = data_dict.alt
    pos_ax[2].set_ylabel('Alt (m)', weight='bold')
    pos_ax[2].plot(t_flight, alt, ls=ls, marker=marker, label=label, c=c, lw=2, alpha=alpha)

    pos_fig.canvas.draw()

    # Top View (Longitude vs. Latitude) Plot
    # if FLAG_PLOT_GROUNDTRACK:
    #     navlat = data_dict1.lat
    #     navlon = data_dict1.lon
    #     nav_maglat = data_dict2.lat
    #     nav_maglon = data_dict2.lon
    #     plt.figure()
    #     plt.title(plotname, fontsize=10)
    #     plt.ylabel('LATITUDE (DEGREES)', weight='bold')
    #     plt.xlabel('LONGITUDE (DEGREES)', weight='bold')
    #     plt.plot(lon_gps, lat_gps, '*', label='GPS Sensor', c='g', lw=2, alpha=.5)
    #     plt.plot(r2d(navlon_flight), r2d(navlat_flight), label='On Board', c='k', lw=2, alpha=.5)
    #     plt.plot(r2d(navlon), r2d(navlat), label=filter1.name, c='r', lw=2, alpha=.8)
    #     plt.plot(r2d(nav_maglon), r2d(nav_maglat), label=filter2.name, c='b', lw=2, alpha=.8)
    #     plt.grid()
    #     plt.legend(loc=0)

    if len(data_dict.p_bias) and len(data_dict.q_bias) and len(data_dict.r_bias):
        # Gyro Biases
        bias_ax[0,0].set_ylabel('p Bias (deg/s)', weight='bold')
        bias_ax[0,0].plot(t_flight, r2d(data_dict.p_bias), label=label, c=c)
        bias_ax[0,0].set_xlabel('Time (secs)', weight='bold')
        bias_ax[0,0].grid()

        bias_ax[1,0].set_ylabel('q Bias (deg/s)', weight='bold')
        bias_ax[1,0].plot(t_flight, r2d(data_dict.q_bias), label=label, c=c)
        bias_ax[1,0].set_xlabel('Time (secs)', weight='bold')
        bias_ax[1,0].grid()

        bias_ax[2,0].set_ylabel('r Bias (deg/s)', weight='bold')
        bias_ax[2,0].plot(t_flight, r2d(data_dict.r_bias), label=label, c=c)
        bias_ax[2,0].set_xlabel('Time (secs)', weight='bold')
        bias_ax[2,0].grid()

    if len(data_dict.ax_bias) and len(data_dict.ay_bias) and len(data_dict.az_bias):
        # Accel Biases
        bias_ax[0,1].set_ylabel('ax Bias (m/s^2)', weight='bold')
        bias_ax[0,1].plot(t_flight, data_dict.ax_bias, label=label, c=c)
        bias_ax[0,1].set_xlabel('Time (secs)', weight='bold')
        bias_ax[0,1].grid()

        bias_ax[1,1].set_ylabel('ay Bias (m/s^2)', weight='bold')
        bias_ax[1,1].plot(t_flight, data_dict.ay_bias, label=label, c=c)
        bias_ax[1,1].set_xlabel('Time (secs)', weight='bold')
        bias_ax[1,1].grid()

        bias_ax[2,1].set_ylabel('az Bias (m/s^2)', weight='bold')
        bias_ax[2,1].plot(t_flight, data_dict.az_bias, label=label, c=c)
        bias_ax[2,1].set_xlabel('Time (secs)', weight='bold')
        bias_ax[2,1].grid()
        bias_ax[2,1].legend(loc=1)
    
    bias_fig.canvas.draw()

    plt.pause(0.25)

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
elif args.sentera2_flight:
    plotname = os.path.basename(args.sentera2_flight)
elif args.umn_flight:
    plotname = os.path.basename(args.umn_flight)

# plot onboard filter
data_ob = data_store()
for filterpt in filter_data:
    data_ob.append_from_filter(filterpt)
update_plot(data_ob, 'On Board', c='g', alpha=0.5)

# plot gps
data_gps = data_store()
for gpspt in gps_data:
    data_gps.append_from_gps(gpspt)
update_plot(data_gps, 'GPS', marker='*', c='g', alpha=0.5)

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

# find the range of gps time stamps (starting with the first point
# with some significant velocity
k_start = 0
for k, gpspt in enumerate(gps_data):
    gps_vel = math.sqrt(gpspt.vn*gpspt.vn + gpspt.ve*gpspt.ve + gpspt.vd*gpspt.vd)
    if gps_vel > 3.0:
        k_start = k
        break
gps_begin = gps_data[k_start].time
gps_end = gps_data[len(gps_data)-1].time
print "gps time span:", gps_begin, gps_end

# display current parameter vector
def printParams(xk):
    r2d = 180.0 / math.pi
    print 'initial vel (m/s): %.4f, %.4f, %.4f' % (xk[0], xk[1], xk[2])
    print 'initial att (rad): %.3f, %.3f, %.3f' % (xk[3], xk[4], xk[5])
    print 'gyro bias (rad/s): %.4f, %.4f, %.4f' % (xk[6], xk[7], xk[8])
    print 'accel bias (m/s^2): %.4f, %.4f, %.4f' % (xk[9], xk[10], xk[11])
    
# run the filter to optimize, then for each gps_record find the result
# position and compute an error distance.  Return the list of error
# values.
data_opt = None
def errorFunc(xk, config, imu_data, gps_data, filter_data):
    global data_opt
    
    filter_opt.set_pos(config['start_lat'],
                       config['start_lon'],
                       config['start_alt'])

    filter_opt.set_vel(xk[0], xk[1], xk[2])
    
    filter_opt.set_att(xk[3], xk[4], xk[5])
    filter_opt.set_gyro_calib(xk[6], xk[7], xk[8],     # bias
                              0.0, 0.0, 0.0)   # delta
    filter_opt.set_accel_calib(xk[9], xk[10], xk[11], # bias
                               0.0, 0.0, 0.0) # delta
    filter_opt.set_G(0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0)
    errors, data_dict, filter_sec = \
        run_filter(filter_opt, imu_data, gps_data, filter_data,
                   config)
    data_opt = data_dict
    if len(errors) > 0:
        sum = 0.0
        for e in errors:
            sum += e*e
        return math.sqrt(sum / len(errors))
    else:
        return 0.0

segment_length = 25
start_time = gps_begin
biases = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

while start_time < gps_end:
    # define time span for this iteration
    end_time = start_time + segment_length
    if end_time > gps_end:
        end_time = gps_end
        
    # find starting position
    for k, gpspt in enumerate(gps_data):
        if gpspt.time >= start_time:
            k_start = k
            break
    d2r = math.pi/ 180.0
    config = {}
    config['start_lat'] = gps_data[k_start].lat*d2r
    config['start_lon'] = gps_data[k_start].lon*d2r
    config['start_alt'] = gps_data[k_start].alt

    config['start_time'] = start_time
    config['end_time'] = end_time
    config['call_init'] = False
    
    # use filter solution to estimate intial attitude and velocity
    for k, filterpt in enumerate(filter_data):
        if filterpt.time >= start_time:
            k_start = k
            break
    initial = (filter_data[k_start].vn, filter_data[k_start].ve,
               filter_data[k_start].vd,    # vel
               filter_data[k_start].phi, filter_data[k_start].the,
               filter_data[k_start].psi, # att
               biases[0], biases[1], biases[2], # gyro
               biases[3], biases[4], biases[5]) # accel
    bounds = ( (filter_data[k_start].vn-2.0, filter_data[k_start].vn+2.0),
               (filter_data[k_start].ve-2.0, filter_data[k_start].ve+2.0),
               (filter_data[k_start].vd-2.0, filter_data[k_start].vd+2.0),
               (filter_data[k_start].phi-0.2, filter_data[k_start].phi+0.2),
               (filter_data[k_start].the-0.2, filter_data[k_start].the+0.2),
               (filter_data[k_start].psi-math.pi, filter_data[k_start].psi+math.pi),
               (-0.2, 0.2), (-0.2, 0.2), (-0.2, 0.2), # gyro bias
               (-1.0, 1.0), (-1.0, 1.0), (-1.0, 1.0)) # accel bias

    from scipy.optimize import minimize
    res = minimize(errorFunc, initial[:], bounds=bounds,
                   args=(config,imu_data,gps_data,filter_data),
                   options={'disp': True},
                   callback=printParams)
    print res
    print "time span:", start_time, end_time
    printParams(res['x'])

    if res['success'] or res['nit'] > 5:
        # we either succeeded or tried real hard
        label = "%.0f-%.0f" % (start_time, end_time)
        update_plot(data_opt, label=label, c='b')
    
        # advance to next segment
        start_time += (segment_length * 0.9)
        # don't carry biases forward to next segment for now ...
        biases = res['x'][6:12]     # or do carry them forward ...
    else:
        # let's rerun the segment and see if we have better luck on
        # the next try ...
        pass

print "Finished fitting all segments, you may now explore the plots."
plt.ioff()    
plt.show()

# quit for now ... we should rerun with the optimized parameters to
# get the results for plotting ...
quit()

nsig = 3
t_store = data_dict1.time
r2d = np.rad2deg

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
    vn_nav = data_dict1.vn
    vn_nav_mag = data_dict2.vn
    ax1.set_title(plotname, fontsize=10)
    ax1.set_ylabel('vn (mps)', weight='bold')
    ax1.plot(t_gps, vn_gps, '-*', label='GPS Sensor', c='g', lw=2, alpha=.5)
    ax1.plot(t_flight, vn_flight, label='On Board', c='k', lw=2, alpha=.5)
    ax1.plot(t_store, vn_nav, label=filter1.name, c='r', lw=2, alpha=.8)
    ax1.plot(t_store, vn_nav_mag, label=filter2.name,c='b', lw=2, alpha=.8)
    ax1.grid()
    ax1.legend(loc=0)

    # ve Plot
    ve_nav = data_dict1.ve
    ve_nav_mag = data_dict2.ve
    ax2.set_ylabel('ve (mps)', weight='bold')
    ax2.plot(t_gps, ve_gps, '-*', label='GPS Sensor', c='g', lw=2, alpha=.5)
    ax2.plot(t_flight, ve_flight, label='On Board', c='k', lw=2, alpha=.5)
    ax2.plot(t_store, ve_nav, label=filter1.name, c='r', lw=2, alpha=.8)
    ax2.plot(t_store, ve_nav_mag, label=filter2.name,c='b', lw=2, alpha=.8)
    ax2.grid()

    # vd Plot
    vd_nav = data_dict1.vd
    vd_nav_mag = data_dict2.vd
    ax3.set_ylabel('vd (mps)', weight='bold')
    ax3.plot(t_gps, vd_gps, '-*', label='GPS Sensor', c='g', lw=2, alpha=.5)
    ax3.plot(t_flight, vd_flight, label='On Board', c='k', lw=2, alpha=.5)
    ax3.plot(t_store, vd_nav, label=filter1.name, c='r', lw=2, alpha=.8)
    ax3.plot(t_store, vd_nav_mag, label=filter2.name, c='b',lw=2, alpha=.8)
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
    plt.plot(t_store, navalt, label=filter1.name, c='r', lw=2, alpha=.8)
    plt.plot(t_store, nav_magalt, label=filter2.name,c='b', lw=2, alpha=.8)
    plt.ylabel('ALTITUDE (METERS)', weight='bold')
    plt.legend(loc=0)
    plt.grid()

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
    bias_ax[0,0].set_ylabel('p Bias (deg/s)', weight='bold')
    bias_ax[0,0].plot(t_store, r2d(data_dict1.p_bias), label=filter1.name, c='r')
    bias_ax[0,0].plot(t_store, r2d(data_dict2.p_bias), label=filter2.name, c='b')
    bias_ax[0,0].set_xlabel('Time (secs)', weight='bold')
    bias_ax[0,0].grid()
    
    bias_ax[1,0].set_ylabel('q Bias (deg/s)', weight='bold')
    bias_ax[1,0].plot(t_store, r2d(data_dict1.q_bias), label=filter1.name, c='r')
    bias_ax[1,0].plot(t_store, r2d(data_dict2.q_bias), label=filter2.name, c='b')
    bias_ax[1,0].set_xlabel('Time (secs)', weight='bold')
    bias_ax[1,0].grid()
    
    bias_ax[2,0].set_ylabel('r Bias (deg/s)', weight='bold')
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
