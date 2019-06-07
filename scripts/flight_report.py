#!/usr/bin/python3

"""run_filter.py

Run a flight data set through a filter and output a few simple plots
Author: Curtis L. Olson, University of Minnesota
"""

import argparse
import datetime
import math
from matplotlib import pyplot as plt
import matplotlib.transforms
#import mpld3
import numpy as np
import os
import pandas as pd
from tqdm import tqdm

from aurauas_flightdata import flight_loader, flight_interp

parser = argparse.ArgumentParser(description='nav filter')
parser.add_argument('--flight', required=True, help='flight data log')
args = parser.parse_args()

r2d = 180.0 / math.pi
d2r = math.pi / 180.0
m2nm = 0.0005399568034557235    # meters to nautical miles
mps2kt = 1.94384               # m/s to kts

path = args.flight
data, flight_format = flight_loader.load(path)

print("imu records:", len(data['imu']))
imu_dt = (data['imu'][-1]['time'] - data['imu'][0]['time']) \
    / float(len(data['imu']))
print("imu dt: %.3f" % imu_dt)
print("gps records:", len(data['gps']))
if 'air' in data:
    print("airdata records:", len(data['air']))
if len(data['imu']) == 0 and len(data['gps']) == 0:
    print("not enough data loaded to continue.")
    quit()

# make data frames for easier plotting
df0_gps = pd.DataFrame(data['gps'])
df0_gps.set_index('time', inplace=True, drop=False)
df0_nav = pd.DataFrame(data['filter'])
df0_nav.set_index('time', inplace=True, drop=False)
df0_air = pd.DataFrame(data['air'])
df0_air.set_index('time', inplace=True, drop=False)
if 'act' in data:
    df0_act = pd.DataFrame(data['act'])
    df0_act.set_index('time', inplace=True, drop=False)


airborne = None
mission = None
land = None
odometer = 0.0
flight_time = 0.0
log_time = data['imu'][-1]['time'] - data['imu'][0]['time']

# Scan events log if it exists
if 'event' in data:
    messages = []
    for event in data['event']:
        time = event['time']
        msg = event['message']
        # print(time, msg)
        tokens = msg.split()
        if len(tokens) == 2 and tokens[1] == 'airborne' and not airborne:
            print("airborne (launch) at t =", time)
            airborne = time
        elif len(tokens) == 4 and tokens[2] == 'complete:' and tokens[3] == 'launch' and not mission:
            # haven't found a mission start yet, so update time
            print("launch complete at t =", time)
            mission = time
        elif len(tokens) == 3 and time > 0 and tokens[1] == 'on' and tokens[2] == 'ground' and not land:
            t = time
            if t - airborne > 60:
                print("flight complete at t =", time)
                land = time
            else:
                print("warning ignoring sub 1 minute flight")
        elif len(tokens) == 5 and (tokens[0] == 'APM2:' or tokens[0] == 'Aura3:') and tokens[1] == 'Serial' and tokens[2] == 'Number':
            auto_sn = int(tokens[4])
        elif len(tokens) == 4 and tokens[0] == 'APM2' and tokens[1] == 'Serial' and tokens[2] == 'Number:':
            auto_sn = int(tokens[3])

# make time regions
label = "n/a"
startE = 0.0
regions = []
for event in data['event']:
    if event['message'][:7] == "Test ID":
        label = event['message']
    if event['message'] == "Excitation Start":
        startE = event['time']
    if event['message'] == "Excitation End":
        regions.append( [startE, event['time'], label] )
print(regions)
    
# Iterate through the flight and collect some stats
print("Collecting flight stats:")
in_flight = False
ap_time = 0.0
ap_enabled = False
last_time = 0.0
total_mah = 0.0
iter = flight_interp.IterateGroup(data)
for i in tqdm(range(iter.size())):
    record = iter.next()
    imu = record['imu']
    if 'gps' in record:
        gps = record['gps']
    if 'air' in record:
        air = record['air']
        if not airborne and air['airspeed'] >= 15:
            airborne = air['time']
        if airborne and not land and air['airspeed'] <= 10:
            land = air['time']
        if air['airspeed'] >= 15:
            in_flight = True
        else:
            in_flight = False
    if 'pilot' in record:
        pilot = record['pilot']
        if pilot['auto_manual'] > 0.0:
            ap_enabled = True
        else:
            ap_enable = False
    if 'health' in record:
        health = record['health']
        if 'total_mah' in health:
            total_mah = health['total_mah']
    if 'filter' in record:
        nav = record['filter']
        current_time = nav['time']
        dt = current_time - last_time
        last_time = current_time
        if in_flight:
            flight_time += dt
            vn = nav['vn']
            ve = nav['ve']
            vel_ms = math.sqrt(vn*vn + ve*ve)
            odometer += vel_ms * dt
        if in_flight and ap_enabled:
            ap_time += dt
        
# Generate markdown report
f = open("report.md", "w")

plotname = os.path.basename(args.flight.rstrip('/'))

f.write("# Flight Report\n")
f.write("\n")
f.write("## Summary\n")
f.write("- File: %s\n" % plotname)
d = datetime.datetime.utcfromtimestamp( data['gps'][0]['unix_sec'] )
f.write("- Date: %s (UTC)\n" % d.strftime("%Y-%m-%d %H:%M:%S"))
f.write("- Log time: %.1f minutes\n" % (log_time / 60.0))
f.write("- Flight time: %.1f minutes\n" % (flight_time / 60.0))
if ap_time > 0.0:
    f.write("- Autopilot time: %.1f minutes\n" % (ap_time / 60.0))
if odometer > 0.0:
    f.write("- Flight distance: %.2f nm (%.2f km)\n" % (odometer*m2nm, odometer/1000.0))
if odometer > 0.0 and flight_time > 0.0:
    gs_mps = odometer / flight_time
    f.write("- Average ground speed: %.1f kts (%.1f m/s)\n" % (gs_mps * mps2kt, gs_mps))
if total_mah > 0.0:
    f.write("- Total Battery: " + "%0f" % total_mah + " (mah)\n")
f.write("\n")

# Weather Summary

apikey = None
try:
    from os.path import expanduser
    home = expanduser("~")
    fio = open(home + '/.forecastio')
    apikey = fio.read().rstrip()
    fio.close()
except:
    print("you must sign up for a free apikey at forecast.io and insert it as a single line inside a file called ~/.forecastio (with no other text in the file)")

unix_sec = data['gps'][0]['unix_sec']
lat = data['gps'][0]['lat']
lon = data['gps'][0]['lon']

if not apikey:
    print("Cannot lookup weather because no forecastio apikey found.")
elif unix_sec < 1:
    print("Cannot lookup weather because gps didn't report unix time.")
else:
    f.write("## Weather\n")
    d = datetime.datetime.utcfromtimestamp(unix_sec)
    print(d.strftime("%Y-%m-%d-%H:%M:%S"))

    url = 'https://api.darksky.net/forecast/' + apikey + '/%.8f,%.8f,%.d' % (lat, lon, unix_sec)

    import urllib.request, json
    response = urllib.request.urlopen(url)
    wx = json.loads(response.read())
    mph2kt = 0.868976
    mb2inhg = 0.0295299830714
    if 'currently' in wx:
        currently = wx['currently']
        #for key in currently:
        #    print key, ':', currently[key]
        if 'icon' in currently:
            icon = currently['icon']
            f.write("- Conditions: " + icon + "\n")
        if 'temperature' in currently:
            tempF = currently['temperature']
            tempC = (tempF - 32.0) * 5 / 9
            f.write("- Temperature: %.1f F" % tempF + " (%.1f C)" % tempC + "\n")
        if 'dewPoint' in currently:
            dewF = currently['dewPoint']
            dewC = (dewF - 32.0) * 5 / 9
            f.write("- Dewpoint: %.1f F" % dewF + " (%.1f C)" % dewC + "\n")
        if 'humidity' in currently:
            hum = currently['humidity']
            f.write("- Humidity: %.0f%%" % (hum * 100.0) + "\n")
        if 'pressure' in currently:
            mbar = currently['pressure']
            inhg = mbar * mb2inhg
            f.write("- Pressure: %.2f inhg" % inhg + " (%.1f mbar)" % mbar + "\n")
        if 'windSpeed' in currently:
            wind_mph = currently['windSpeed']
            wind_kts = wind_mph * mph2kt
        else:
            wind_mph = 0
            wind_kts = 0
        if 'windBearing' in currently:
            wind_deg = currently['windBearing']
        else:
            wind_deg = 0
        f.write("- Wind %d deg @ %.1f kt (%.1f mph)" % (wind_deg, wind_kts, wind_mph) + "\n")
        if 'visibility' in currently:
            vis = currently['visibility']
            f.write("- Visibility: %.1f miles" % vis + "\n")
        if 'cloudCover' in currently:
            cov = currently['cloudCover']
            f.write("- Cloud Cover: %.0f%%" % (cov * 100.0) + "\n")
        f.write("- METAR: KXYZ " + d.strftime("%d%H%M") + "Z" +
                " %03d%02dKT" % (round(wind_deg/10)*10, wind_kts) +
                " " + ("%.1f" % vis).rstrip('0').rstrip(".") + "SM" +
                " " + ("%.0f" % tempC).replace('-', 'M') + "/" +
                ("%.0f" % dewC).replace('-', 'M') +
                " A%.0f=\n" % (inhg*100)
        )
    f.write("\n")

# add a shaded time region(s) to plot
blend = matplotlib.transforms.blended_transform_factory
def add_regions(plot, regions):
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b',
              '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']
    trans = blend(plot.transData, plot.transAxes)
    for i, region in enumerate(regions):
        plot.axvspan(region[0], region[1], color=colors[i % len(colors)],
                     alpha=0.25)
        plot.text(region[0], 0.5, region[2], transform=trans,
                  verticalalignment='center',
                  rotation=90, color=colors[i % len(colors)])


        
if not 'wind_dir' in data['air'][0]:
    # run a quick wind estimate
    import wind
    winds = wind.estimate(data)
    df1_wind = pd.DataFrame(winds)
    time = df1_wind['time']
    wind_dir = df1_wind['wind_deg']
    wind_speed = df1_wind['wind_kt']
    pitot_scale = df1_wind['pitot_scale']
else:
    time = df0_air['time']
    wind_dir = df0_air['wind_dir']
    wind_speed = df0_air['wind_speed']
    pitot_scale = df0_air['pitot_scale']
    
wind_fig, wind_ax = plt.subplots(2, 1, sharex=True)
wind_ax[0].set_title("Winds Aloft")
wind_ax[0].set_ylabel("Heading (degrees)", weight='bold')
wind_ax[0].plot(time, wind_dir)
#wind_ax[0].axvspan(airborne, land, color='green', alpha=0.25)
#wind_ax[0].axvspan(airborne, land, alpha=0.25)
add_regions(wind_ax[0], [[airborne, land, "Airborne"]])
wind_ax[0].grid()
wind_ax[0].legend()
wind_ax[1].set_xlabel("Time (secs)", weight='bold')
wind_ax[1].set_ylabel("Speed (kts)", weight='bold')
wind_ax[1].plot(time, wind_speed)
wind_ax[1].plot(time, pitot_scale)
#wind_ax[1].axvspan(airborne, land, color='green', alpha=0.25)
add_regions(wind_ax[1], [[airborne, land, "Airborne"]])
wind_ax[1].grid()
wind_ax[1].legend()

# How bad are your magnetometers?
import mags
result = mags.estimate(data)
df1_mags = pd.DataFrame(result)
plt.figure()
plt.title("Magnetometer Norm vs. Throttle")
plt.plot(df1_mags['time'], df1_mags['throttle'])
avg = df1_mags['mag_norm'].mean()
plt.plot(df1_mags['time'], df1_mags['mag_norm']/avg)
plt.xlabel("Time (secs)", weight='bold')
plt.ylabel("Mag Norm", weight='bold')
plt.legend()
plt.grid()

plt.figure()
plt.title("Magnetometer Norm vs. Throttle Correlation")
plt.plot(df1_mags['throttle'], df1_mags['mag_norm']/avg, '*')
plt.xlabel("Throttle (norm)", weight='bold')
plt.ylabel("Magnetometer norm", weight='bold')
plt.grid()

r2d = np.rad2deg

# Attitude
att_fig, att_ax = plt.subplots(3, 1, sharex=True)

att_ax[0].set_title("Attitude Angles")
att_ax[0].set_ylabel('Roll (deg)', weight='bold')
att_ax[0].plot(r2d(df0_nav['phi']))
add_regions(att_ax[0], regions)
att_ax[0].grid()

att_ax[1].set_ylabel('Pitch (deg)', weight='bold')
att_ax[1].plot(r2d(df0_nav['the']))
add_regions(att_ax[1], regions)
att_ax[1].grid()

att_ax[2].set_ylabel('Yaw (deg)', weight='bold')
att_ax[2].plot(r2d(df0_nav['psi']))
add_regions(att_ax[2], regions)
att_ax[2].set_xlabel('Time (sec)', weight='bold')
att_ax[2].grid()
att_ax[2].legend(loc=1)

f.close()

#mpld3.show()

# Velocities
fig, [ax1, ax2, ax3] = plt.subplots(3,1, sharex=True)

# vn Plot
ax1.set_title("NED Velocities")
ax1.set_ylabel('vn (mps)', weight='bold')
ax1.plot(df0_gps['vn'], '-*', label='GPS Sensor', c='g', alpha=.5)
ax1.plot(df0_nav['vn'], label='EKF')
add_regions(ax1, regions)
ax1.grid()

# ve Plot
ax2.set_ylabel('ve (mps)', weight='bold')
ax2.plot(df0_gps['ve'], '-*', label='GPS Sensor', c='g', alpha=.5)
ax2.plot(df0_nav['ve'], label='EKF')
add_regions(ax2, regions)
ax2.grid()

# vd Plot
ax3.set_ylabel('vd (mps)', weight='bold')
ax3.plot(df0_gps['vd'], '-*', label='GPS Sensor', c='g', alpha=.5)
ax3.plot(df0_nav['vd'], label='EKF')
add_regions(ax3, regions)
ax3.set_xlabel('Time (secs)', weight='bold')
ax3.grid()
ax3.legend(loc=0)

if 'temp' in df0_air:
    plt.figure()
    plt.title("Air Temp")
    plt.plot(df0_air['temp'])
    plt.grid()

plt.figure()
plt.title("Airspeed (kt)")
plt.plot(df0_air['airspeed'])
plt.grid()

if 'alt_press' in df0_air:
    plt.figure()
    plt.title("Altitude (press)")
    plt.plot(df0_air['alt_press'])
    plt.grid()

if 'act' in data:
    plt.figure()
    plt.title("Throttle")
    plt.plot(df0_act['throttle'])
    plt.grid()

# Altitude
fig = plt.figure()
plt.title('Altitude')
plt.plot(df0_gps['alt'], '-*', label='GPS Sensor', c='g', alpha=.5)
plt.plot(df0_nav['alt'], label='EKF')
add_regions(fig.gca(), regions)
plt.ylabel('Altitude (m)', weight='bold')
plt.legend(loc=0)
plt.grid()

# Top down flight track plot
plt.figure()
plt.title('Ground track')
plt.ylabel('Latitude (degrees)', weight='bold')
plt.xlabel('Longitude (degrees)', weight='bold')
plt.plot(df0_gps['lon'], df0_gps['lat'], '*', label='GPS Sensor', c='g', alpha=.5)
plt.plot(r2d(df0_nav['lon']), r2d(df0_nav['lat']), label='EKF')
plt.grid()
plt.legend(loc=0)

# Biases
bias_fig, bias_ax = plt.subplots(3,2, sharex=True)

# Gyro Biases
bias_ax[0,0].set_title("IMU Biases")
bias_ax[0,0].set_ylabel('p (deg/s)', weight='bold')
bias_ax[0,0].plot(r2d(df0_nav['p_bias']))
bias_ax[0,0].set_xlabel('Time (secs)', weight='bold')
bias_ax[0,0].grid()

bias_ax[1,0].set_ylabel('q (deg/s)', weight='bold')
bias_ax[1,0].plot(r2d(df0_nav['q_bias']))
bias_ax[1,0].set_xlabel('Time (secs)', weight='bold')
bias_ax[1,0].grid()

bias_ax[2,0].set_ylabel('r (deg/s)', weight='bold')
bias_ax[2,0].plot(r2d(df0_nav['r_bias']))
bias_ax[2,0].set_xlabel('Time (secs)', weight='bold')
bias_ax[2,0].grid()

# Accel Biases
bias_ax[0,1].set_title("Accel Biases")
bias_ax[0,1].set_ylabel('ax (m/s^2)', weight='bold')
bias_ax[0,1].plot(df0_nav['ax_bias'])
bias_ax[0,1].set_xlabel('Time (secs)', weight='bold')
bias_ax[0,1].grid()

bias_ax[1,1].set_ylabel('ay (m/s^2)', weight='bold')
bias_ax[1,1].plot(df0_nav['ay_bias'])
bias_ax[1,1].set_xlabel('Time (secs)', weight='bold')
bias_ax[1,1].grid()

bias_ax[2,1].set_ylabel('az (m/s^2)', weight='bold')
bias_ax[2,1].plot(df0_nav['az_bias'])
bias_ax[2,1].set_xlabel('Time (secs)', weight='bold')
bias_ax[2,1].grid()
bias_ax[2,1].legend(loc=1)

plt.show()
