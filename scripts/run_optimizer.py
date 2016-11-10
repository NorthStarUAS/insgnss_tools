#!/usr/bin/python

"""run_filters.py

This script plays flight data through the selected navigation filters.
The filters are compiled as .so objects and wrapped for python with boos.

A set of customizable input flags are defined at the start of the script.

Initial revision: Hamid M.
Many updates: Curtis L. Olson
"""

import argparse
import csv
import math
import numpy as np
import time
import os
from scipy.optimize import minimize

import navpy

import data_store
import flight_data
import plots

# filter interfaces
import nav_orig
import nav_mag
import nav_eigen
import nav_eigen_mag
import nav_openloop
import MadgwickAHRS

parser = argparse.ArgumentParser(description='nav filter')
parser.add_argument('--flight', help='load specified aura flight log')
parser.add_argument('--aura-flight', help='load specified aura flight log')
parser.add_argument('--umn-flight', help='load specified .mat flight log')
parser.add_argument('--sentera-flight', help='load specified sentera flight log')
parser.add_argument('--sentera2-flight', help='load specified sentera2 flight log')
args = parser.parse_args()

filter_ref = nav_eigen_mag.filter()
filter_opt = nav_openloop.filter()

def run_filter(filter, imu_data, gps_data, filter_data, config=None):
    data_dict = data_store.data_store()
    errors = []
    
    # Using while loop starting at k (set to kstart) and going to end
    # of .mat file
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
    return errors, data_dict

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
    errors, data_dict = \
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

# plots
plt = plots.Plots(plotname)

# plot onboard filter
data_ob = data_store.data_store()
for filterpt in filter_data:
    data_ob.append_from_filter(filterpt)
plt.update(data_ob, 'On Board', c='g', alpha=0.5)

# plot gps
data_gps = data_store.data_store()
for gpspt in gps_data:
    data_gps.append_from_gps(gpspt)
plt.update(data_gps, 'GPS', marker='*', c='g', alpha=0.5)

# # rearrange flight data for plotting
# t_gps = []
# lat_gps = []
# lon_gps = []
# alt_gps = []
# vn_gps = []
# ve_gps = []
# vd_gps = []
# for g in gps_data:
#     t_gps.append(g.time)
#     lat_gps.append(g.lat)
#     lon_gps.append(g.lon)
#     alt_gps.append(g.alt)
#     vn_gps.append(g.vn)
#     ve_gps.append(g.ve)
#     vd_gps.append(g.vd)

# t_flight = []
# psi_flight = []
# the_flight = []
# phi_flight = []
# navlat_flight = []
# navlon_flight = []
# navalt_flight = []
# vn_flight = []
# ve_flight = []
# vd_flight = []
# for f in filter_data:
#     t_flight.append(f.time)
#     psi_flight.append(f.psi)
#     the_flight.append(f.the)
#     phi_flight.append(f.phi)
#     navlat_flight.append(f.lat)
#     navlon_flight.append(f.lon)
#     navalt_flight.append(f.alt)
#     vn_flight.append(f.vn)
#     ve_flight.append(f.ve)
#     vd_flight.append(f.vd)

# find the range of gps time stamps (starting with the first point
# with some significant velocity
k_start = 0
for k, gpspt in enumerate(gps_data):
    gps_vel = math.sqrt(gpspt.vn*gpspt.vn + gpspt.ve*gpspt.ve)
    if gps_vel > 5.0:
        k_start = k
        break
gps_begin = gps_data[k_start].time
k_end = 0
for k, gpspt in enumerate(gps_data):
    gps_vel = math.sqrt(gpspt.vn*gpspt.vn + gpspt.ve*gpspt.ve)
    if gps_vel > 5.0:
        k_end = k
gps_end = gps_data[k_end].time
print "gps time span:", gps_begin, gps_end

segment_length = 60
start_time = gps_begin
biases = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# store the segment config and optimal params so we can use the
# solution for something at the end of all this.
segments = []

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
    print "segment time span:", start_time, end_time
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
               (-2.0, 2.0), (-2.0, 2.0), (-2.0, 2.0)) # accel bias

    from scipy.optimize import minimize
    res = minimize(errorFunc, initial[:], bounds=bounds,
                   args=(config,imu_data,gps_data,filter_data),
                   options={'disp': True, 'maxiter': 6},
                   callback=printParams)
    print res
    printParams(res['x'])

    if res['success'] or res['nit'] > 5:
        # we either succeeded or tried real hard
        segment = dict()
        segment['config'] = config.copy()
        segment['results'] = res.copy()
        segments.append(segment)
        
        label = "%.0f-%.0f" % (start_time, end_time)
        plt.update(data_opt, label=label, c='b')
    
        # advance to next segment
        start_time += (segment_length * 0.9)
        # don't carry biases forward to next segment for now ...
        biases = res['x'][6:12]     # or do carry them forward ...
    else:
        # let's rerun the segment and see if we have better luck on
        # the next try ...
        pass

print segments

for segment in segments:
    xk = segment['results']['x']
    config = segment['config']
    errorFunc(xk, config, imu_data, gps_data, filter_data)
    
print "Finished fitting all segments, you may now explore the plots."
plt.explore()

