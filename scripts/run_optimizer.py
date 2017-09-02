#!/usr/bin/python

"""run_filters.py

This script plays flight data through the selected navigation filters.
The filters are compiled as .so objects and wrapped for python with boos.

A set of customizable input flags are defined at the start of the script.

Initial revision: Hamid M.
Many updates: Curtis L. Olson
"""

import argparse
import copy
import csv
import math
import numpy as np
import time
import os
from scipy.optimize import minimize

import navpy

import data_store
from aurauas.flightdata import flight_loader
import plots

# filter interfaces
import nav_eigen
import nav_eigen_mag
import nav_openloop
#import MadgwickAHRS

parser = argparse.ArgumentParser(description='nav filter')
parser.add_argument('--flight', help='load specified aura flight log')
parser.add_argument('--aura-flight', help='load specified aura flight log')
parser.add_argument('--px4-sdlog2', help='load specified px4 sdlog2 (csv) flight log')
parser.add_argument('--px4-ulog', help='load specified px4 ulog (csv) base path')
parser.add_argument('--umn-flight', help='load specified .mat flight log')
parser.add_argument('--sentera-flight', help='load specified sentera flight log')
parser.add_argument('--sentera2-flight', help='load specified sentera2 flight log')
args = parser.parse_args()

# This could be an important parameter to pay attention to:
gps_latency = 0.4               # seconds

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
            while gps_data[gps_index+1].time - gps_latency <= imupt.time:
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
            data_dict.append(navpt, imupt)

        if gpspt.newData:
            # compute error metric with each new gps report
            # full 3d distance error (in ecef)
            p1 = navpy.lla2ecef(gpspt.lat, gpspt.lon, gpspt.alt,
                                latlon_unit='deg')
            p2 = navpy.lla2ecef(navpt.lat, navpt.lon, navpt.alt,
                                latlon_unit='rad')
            pe = np.linalg.norm(p1 - p2)
            
            # weight horizontal error more highly than vertical error
            ref = gps_data[0]
            n1 = navpy.lla2ned(gpspt.lat, gpspt.lon, gpspt.alt,
                               ref.lat, ref.lon, 0.0,
                               latlon_unit='deg')
            n2 = navpy.lla2ned(navpt.lat*r2d, navpt.lon*r2d, navpt.alt,
                               ref.lat, ref.lon, 0.0,
                               latlon_unit='deg')
            dn = n2 - n1
            ne = math.sqrt(dn[0]*dn[0] + dn[1]*dn[1] + dn[2]*dn[2]*0.5)

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
            
            errors.append(ne)   # ned error weighted towards horizontal
            
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
    errors, data_opt = \
        run_filter(filter_opt, imu_data, gps_data, filter_data,
                   config)
    if len(errors) > 0:
        sum = 0.0
        for e in errors:
            sum += e*e
        return math.sqrt(sum / len(errors))
    else:
        return 0.0

if args.flight:
    loader = 'aura'
    path = args.flight
elif args.aura_flight:
    loader = 'aura'
    path = args.aura_flight
elif args.px4_sdlog2:
    loader = 'px4_sdlog2'
    path = args.px4_sdlog2
elif args.px4_ulog:
    loader = 'px4_ulog'
    path = args.px4_ulog
elif args.sentera_flight:
    loader = 'sentera1'
    path = args.sentera_flight
elif args.sentera2_flight:
    loader = 'sentera2'
    path = args.sentera2_flight
elif args.umn_flight:
    loader = 'umn1'
    path = args.umn_flight
else:
    loader = None
    path = None

data = flight_loader.load(loader, path, None)
print "imu records:", len(data['imu'])
print "gps records:", len(data['gps'])
if 'air' in data:
    print "air records:", len(data['air'])
print "filter records:", len(data['filter'])
if len(data['imu']) == 0 and len(data['gps']) == 0:
    print "not enough data loaded to continue."
    quit()

if args.flight:
    plotname = os.path.basename(args.flight)    
elif args.aura_flight:
    plotname = os.path.basename(args.aura_flight)
elif args.px4_sdlog2:
    plotname = os.path.basename(args.px4_sdlog2)
elif args.sentera_flight:
    plotname = os.path.basename(args.sentera_flight)
elif args.sentera2_flight:
    plotname = os.path.basename(args.sentera2_flight)
elif args.umn_flight:
    plotname = os.path.basename(args.umn_flight)
else:
    plotname = "plotname not set correctly"

# plots
plt = plots.Plots(plotname)

# plot onboard filter
data_ob = data_store.data_store()
for filterpt in data['filter']:
    data_ob.append_from_filter(filterpt)
plt.update(data_ob, 'On Board', c='g', alpha=0.5)

# plot gps
data_gps = data_store.data_store()
for gpspt in data['gps']:
    data_gps.append_from_gps(gpspt)
plt.update(data_gps, 'GPS', marker='*', c='g', alpha=0.5)

# find the range of gps time stamps that represent some significant
# amount of velocity
k_start = 0
for k, gpspt in enumerate(data['gps']):
    gps_vel = math.sqrt(gpspt.vn*gpspt.vn + gpspt.ve*gpspt.ve)
    if gps_vel > 5.0:
        k_start = k
        break
gps_begin = data['gps'][k_start].time + gps_latency
k_end = 0
for k, gpspt in enumerate(data['gps']):
    gps_vel = math.sqrt(gpspt.vn*gpspt.vn + gpspt.ve*gpspt.ve)
    if gps_vel > 5.0:
        k_end = k
gps_end = data['gps'][k_end].time + gps_latency
print "gps time span:", gps_begin, gps_end

# store the segment config and optimal params so we can use the
# solution for something at the end of all this.
segments = []
segment_length = 30             # seconds
segment_overlap = 0.1           # 10%

start_time = gps_begin
biases = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

d2r = math.pi/ 180.0
r2d = 180.0 / math.pi
                
while start_time < gps_end:
    # define time span for this iteration
    end_time = start_time + segment_length
    if end_time > gps_end:
        end_time = gps_end
        
    # find starting position
    for k, gpspt in enumerate(data['gps']):
        if gpspt.time - gps_latency >= start_time:
            k_start = k
            break
    print "segment time span:", start_time, end_time
    config = {}
    config['start_lat'] = data['gps'][k_start].lat*d2r
    config['start_lon'] = data['gps'][k_start].lon*d2r
    config['start_alt'] = data['gps'][k_start].alt

    config['start_time'] = start_time
    config['end_time'] = end_time
    config['call_init'] = False
    
    # use filter solution to estimate intial attitude and velocity
    for k, filterpt in enumerate(data['filter']):
        if filterpt.time >= start_time:
            k_start = k
            break
    initial = (data['filter'][k_start].vn, data['filter'][k_start].ve,
               data['filter'][k_start].vd,    # vel
               data['filter'][k_start].phi, data['filter'][k_start].the,
               data['filter'][k_start].psi, # att
               biases[0], biases[1], biases[2], # gyro
               biases[3], biases[4], biases[5]) # accel
    max_gyro_bias = 0.1
    max_accel_bias = 0.2
    bounds = ( (data['filter'][k_start].vn-2.0, data['filter'][k_start].vn+2.0),
               (data['filter'][k_start].ve-2.0, data['filter'][k_start].ve+2.0),
               (data['filter'][k_start].vd-2.0, data['filter'][k_start].vd+2.0),
               (data['filter'][k_start].phi-0.2, data['filter'][k_start].phi+0.2),
               (data['filter'][k_start].the-0.2, data['filter'][k_start].the+0.2),
               (data['filter'][k_start].psi-math.pi, data['filter'][k_start].psi+math.pi),
               (-max_gyro_bias, max_gyro_bias), (-max_gyro_bias, max_gyro_bias), (-max_gyro_bias, max_gyro_bias), # gyro bias
               (-max_accel_bias, max_accel_bias), (-max_accel_bias, max_accel_bias), (-max_accel_bias, max_accel_bias)) # accel bias

    from scipy.optimize import minimize
    res = minimize(errorFunc, initial[:], bounds=bounds,
                   args=(config,data['imu'],data['gps'],data['filter']),
                   #options={'disp': True, 'maxiter': 5},
                   options={'disp': True},
                   callback=printParams)
    print res
    printParams(res['x'])

    if res['success'] or res['nit'] > 5:
        # we either succeeded or tried real hard
        segment = dict()
        segment['config'] = config.copy()
        segment['results'] = res.copy()
        segment['data'] = data_opt
        segments.append(segment)
        
        label = "%.0f-%.0f" % (start_time, end_time)
        plt.update(data_opt, label=label, c='b')
    
        # advance to next segment
        start_time += (segment_length * (1 - segment_overlap))

        # carry biases forward as the starting value for the next
        # segment optimization.
        biases = res['x'][6:12]
    else:
        # let's rerun the segment and see if we have better luck on
        # the next try ...
        pass

result_opt = data_store.data_store()
for i in range(0, len(segments)):
    #xk = segment['results']['x']
    #errorFunc(xk, config, data['imu'], data['gps'], data['filter'])
    config = segments[i]['config']
    print 'start:', config['start_time'], 'end:', config['end_time']
    if i == 0:
        s1 = segments[i]
        data1 = s1['data']
        t1 = data1.find_index(config['start_time'])
        n1 = data1.data[t1]
        start_k = t1
        start_err = data_store.diff_split(n1, n1) # should be all zeros
    else:
        s0 = segments[i-1]
        data0 = s0['data']
        t0 = data0.find_index(config['start_time'] + segment_length * (segment_overlap * 0.5))
        n0 = data0.data[t0]
        
        s1 = segments[i]
        data1 = s1['data']
        t1 = data1.find_index(config['start_time'] + segment_length * (segment_overlap * 0.5))
        n1 = data1.data[t1]

        start_k = t1
        start_err = data_store.diff_split(n0, n1)
        
    if i == len(segments) - 1:
        s1 = segments[i]
        data1 = s1['data']
        t1 = data1.find_index(config['end_time'])
        n1 = data1.data[t1]
        end_k = t1
        end_err = data_store.diff_split(n1, n1) # should be all zeros
    else:
        s1 = segments[i]
        data1 = s1['data']
        t1 = data1.find_index(config['end_time'] - segment_length * (segment_overlap * 0.5))
        n1 = data1.data[t1]

        s2 = segments[i+1]
        data2 = s2['data']
        t2 = data2.find_index(config['end_time'] - segment_length * (segment_overlap * 0.5))
        n2 = data2.data[t2]

        end_k = t1
        end_err = data_store.diff_split(n2, n1)        
    print 'start_k:', start_k, 'end_k:', end_k

    for k in range(start_k, end_k):
        perc = float(k - start_k) / float(end_k - start_k)
        err = data_store.weighted_avg(start_err, end_err, 1-perc)
        joined = data_store.sum(data1.data[k], err)
        result_opt.append(joined, None)

# write out 'filter-post.txt' based on optimized result
if args.flight or args.aura_flight:
    import data_aura
    if args.flight:
        filter_post = os.path.join(args.flight, "filter-post.txt")
    elif args.aura_flight:
        filter_post = os.path.join(args.aura_flight, "filter-post.txt")
    data_aura.save_filter_result(filter_post, result_opt)

# plot final result
plt.update(result_opt, 'Optimized', c='r', alpha=0.5)
        
print "Finished fitting all segments, you may now explore the plots."
plt.explore()

