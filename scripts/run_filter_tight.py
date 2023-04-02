#!/usr/bin/python3

"""run_filter2.py

Run a flight data set through a filter and output a few simple plots
Author: Curtis L. Olson, University of Minnesota
"""

import argparse
import math
from matplotlib import pyplot as plt
import numpy as np
import os
import pandas as pd
from tqdm import tqdm

from rcUAS.flightdata import flight_loader, flight_interp

import new_wrapper2
import json 

import sys

# 
def EphemerisData2PosVelClock(time, ephem_data):
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~# 
    # Author: Kerry Sun 
    # Input:
    # curent time [s]
    # Entire emphemeris data set 
    # Output: 
    # Function process entire emphemeris data set and produce position and velocity in ECEF
    # Ref: All the equations are based ON: https://www.gps.gov/technical/icwg/IS-GPS-200H.pdf
    # pg. 104-105, Also Grove  p335-338

    # Constant 
    MU = 3.986005e14
    OMEGA_DOT_EARTH = 7.2921151467e-5
    # Process subframe 1,2,3 information
    A_semiMajorAxis = pow(ephem_data['sqrtA'], 2)  # Semi-major axis
    n_0_computedMeanMotion = math.sqrt(MU / pow(A_semiMajorAxis, 3)) # Computed mean motion
    n_correctedMeanMotion = n_0_computedMeanMotion + ephem_data['deltan'] # Corrected mean motion
    e_eccentricity = ephem_data['e'] # Eccentricity
    #double phi_k_argumentOfLattitude;   # Argument of latitude
    M_0_trueAnomalyAtRef = ephem_data['M0']
    omega0_longitudeofAscendingNodeofOrbitPlane = ephem_data['Omega0']
    omega_argumentOfPerigee = ephem_data['omega']
    omegaDot_argumentOfPerigee = ephem_data['Omegad']
    i_0_inclinationAtRef = ephem_data['i0']
    iDot_rateOfInclination = ephem_data['IDOT']
    t_OE = ephem_data['toe']
    # 2nd harmonic terms
    C_us = ephem_data['Cus']
    C_uc = ephem_data['Cuc']
    C_rs = ephem_data['Crs']
    C_rc = ephem_data['Crc']
    C_is = ephem_data['Cis']
    C_ic = ephem_data['Cic']

    # Compute the time from the ephemeris reference epoch
    t_k_timeFromReferenceEpoch = time - t_OE
    # Correct that time for end-of-week crossovers
    if t_k_timeFromReferenceEpoch > 302400:
        t_k_timeFromReferenceEpoch -= 604800
    if t_k_timeFromReferenceEpoch < -302400:
        t_k_timeFromReferenceEpoch += 604800

    # Compute the mean anomaly
    M_k_meanAnomaly = M_0_trueAnomalyAtRef + n_correctedMeanMotion * t_k_timeFromReferenceEpoch

    # Below, we iteratively solve for E_k_eccentricAnomaly using Newton-Raphson method
    solutionError = 1000000
    E_k_eccentricAnomaly = 1
    currentDerivative = 0
    iterationCount = 0

    solutionError = (E_k_eccentricAnomaly -
                     (e_eccentricity * math.sin(E_k_eccentricAnomaly)) -
                     M_k_meanAnomaly)

    while (abs(solutionError) > 1.0e-6) and iterationCount < 1000:
        currentDerivative = (1.0 - (e_eccentricity * math.cos(E_k_eccentricAnomaly)))
        E_k_eccentricAnomaly = E_k_eccentricAnomaly - solutionError / currentDerivative
        solutionError = (E_k_eccentricAnomaly -
                         (e_eccentricity * math.sin(E_k_eccentricAnomaly)) -
                         M_k_meanAnomaly)
        iterationCount += 1
    
    cos_E_k = math.cos(E_k_eccentricAnomaly)
    sin_E_k = math.sin(E_k_eccentricAnomaly)
    nu_k_trueAnomaly = math.atan2(
        (math.sqrt(1.0 - pow(e_eccentricity, 2)) * sin_E_k) /
            (1.0 - (e_eccentricity * cos_E_k)),
        (cos_E_k - e_eccentricity) /
            (1.0 - e_eccentricity * cos_E_k))

    phi_k_argumentOfLatitude = nu_k_trueAnomaly + omega_argumentOfPerigee

    # Compute the corrective 2nd order terms
    sin2PhiK = math.sin(2.0 * phi_k_argumentOfLatitude)
    cos2PhiK = math.cos(2.0 * phi_k_argumentOfLatitude)

    deltaU_argumentOfLatCorrection = (C_us * sin2PhiK) + (C_uc * cos2PhiK)
    deltaR_radiusCorrection = (C_rs * sin2PhiK) + (C_rc * cos2PhiK)
    deltaI_inclinationCorrection = (C_is * sin2PhiK) + (C_ic * cos2PhiK)

    # Now compute the updated corrected orbital elements
    u_argumentOfLat = phi_k_argumentOfLatitude + deltaU_argumentOfLatCorrection
    r_radius = (A_semiMajorAxis * (1 - (e_eccentricity * cos_E_k))) + deltaR_radiusCorrection
    i_inclination = i_0_inclinationAtRef + (iDot_rateOfInclination * t_k_timeFromReferenceEpoch) + deltaI_inclinationCorrection

    # Compute the satellite position within the orbital plane
    xPositionOrbitalPlane = r_radius * math.cos(u_argumentOfLat)
    yPositionOrbitalPlane = r_radius * math.sin(u_argumentOfLat)
    omegaK_longitudeAscendingNode = omega0_longitudeofAscendingNodeofOrbitPlane + ((omegaDot_argumentOfPerigee - OMEGA_DOT_EARTH) * t_k_timeFromReferenceEpoch) - (OMEGA_DOT_EARTH * t_OE)

    sinOmegaK = math.sin(omegaK_longitudeAscendingNode)
    cosOmegaK = math.cos(omegaK_longitudeAscendingNode)

    sinIK = math.sin(i_inclination)
    cosIK = math.cos(i_inclination)
    # Earth-fixed coordinates:
    x = (xPositionOrbitalPlane * cosOmegaK) - (yPositionOrbitalPlane * cosIK * sinOmegaK)
    y = (xPositionOrbitalPlane * sinOmegaK) + (yPositionOrbitalPlane * cosIK * cosOmegaK)
    z = (yPositionOrbitalPlane * sinIK)

    # ECEF velocity calculation:
    E_dot_k_eccentricAnomaly = n_correctedMeanMotion / (1.0 - (e_eccentricity * cos_E_k))           # Eq.(8.21)
    phi_dot_k_argumentOfLatitude = math.sin(nu_k_trueAnomaly) / sin_E_k * E_dot_k_eccentricAnomaly       # Eq.(8.22)
    r_dot_o_os = (A_semiMajorAxis * e_eccentricity * sin_E_k) * E_dot_k_eccentricAnomaly + 2 * ((C_rs * cos2PhiK) - (C_rc * sin2PhiK)) * phi_dot_k_argumentOfLatitude # Eq.(8.23a)
    u_dot_o_os = (1 + 2 * C_us * cos2PhiK - 2 * C_uc * sin2PhiK) * phi_dot_k_argumentOfLatitude    # Eq.(8.23b)

    x_dot_o_os = r_dot_o_os * math.cos(u_argumentOfLat) - r_radius * u_dot_o_os * math.sin(u_argumentOfLat)  # Eq.(8.24a)
    y_dot_o_os = r_dot_o_os * math.sin(u_argumentOfLat) + r_radius * u_dot_o_os * math.cos(u_argumentOfLat)  # Eq.(8.24b)

    omega_dot_K_longitudeAscendingNode = omegaDot_argumentOfPerigee - OMEGA_DOT_EARTH              # Eq. (8.25)
    i_dot_inclination = iDot_rateOfInclination + 2 * ((C_is * cos2PhiK) - (C_ic * sin2PhiK)) * phi_dot_k_argumentOfLatitude # Eq. (8.26)

    # Eq. (8.27)
    vx = (x_dot_o_os * cosOmegaK - y_dot_o_os * cosIK * sinOmegaK + i_dot_inclination * yPositionOrbitalPlane * sinIK * sinOmegaK) - omega_dot_K_longitudeAscendingNode * (xPositionOrbitalPlane * sinOmegaK + yPositionOrbitalPlane * cosIK * cosOmegaK)
    vy = (x_dot_o_os * sinOmegaK + y_dot_o_os * cosIK * cosOmegaK - i_dot_inclination * yPositionOrbitalPlane * sinIK * cosOmegaK) - omega_dot_K_longitudeAscendingNode * (-xPositionOrbitalPlane * cosOmegaK + yPositionOrbitalPlane * cosIK * sinOmegaK)
    vz = (y_dot_o_os * sinIK + i_dot_inclination * yPositionOrbitalPlane * cosIK)
     
    #lambda1 = 2*c / (1575.4282e6)  # L1 according ublox8
    #PseudorangeRate = lambda1 * gnss_raw_measurement.doppler
    #gnss_raw_measurement.pseudorange 
    return [x, y, z, vx, vy, vz]

# command line arguments
parser = argparse.ArgumentParser(description='nav filter')
parser.add_argument('flight', help='flight data log')
parser.add_argument('--gps-lag-sec', type=float, default=0.2,
                    help='gps lag (sec)')
args = parser.parse_args()

# constants
r2d = 180.0 / math.pi
d2r = math.pi / 180.0
gps_settle_secs = 10.0

ephem_file = os.path.join(args.flight, "ephemeris.json")

# test ephemeris json file 
with open(ephem_file) as json_file:
    ephem_data = json.load(json_file)

print("converting semicircles to radians as needed")
cvt = ('deltan', 'i0', 'IDOT', 'M0', 'omega', 'Omega0', 'Omegad')
for key in ephem_data:
    #print("sat:", key)
    #print(ephem_data[key])
    sat = ephem_data[key]
    for field in cvt:
        if field in sat:
            sat[field] *= math.pi
        else:
            print("cannot find:", field, "in", sat)
    #print(ephem_data[key])
# for k in ephem_data.keys():
#     if(ephem_data[k]['frame1'] == 'True' and ephem_data[k]['frame2'] == 'True'
#        and ephem_data[k]['frame3'] == 'True'):
#         print (ephem_data[k]['AODO'])

#print("Type: ", type(ephem_data))
#print(ephem_data)
#print("0-12:", ephem_data['0-12'])
#print(ephem_data['0-12']['AODO']) 
# check validity of frame 1, 2 and 3
#d = EphemerisData2PosVelClock(29400, ephem_data['0-12'])
#print(d)

## using set to match keys between two dictionaries 
# https://stackoverflow.com/questions/1317410/finding-matching-keys-in-two-large-dictionaries-and-doing-it-fast

# load the flight data
path = args.flight
data, flight_format = flight_loader.load(path)

print("imu records:", len(data['imu']))
imu_dt = (data['imu'][-1]['time'] - data['imu'][0]['time']) \
    / float(len(data['imu']))
print("imu dt: %.3f" % imu_dt)
print("gps records:", len(data['gps']))
print("gps raw records:", len(data['gpsraw']))
#print("gnss records:",len(data['gnss_measument']))
#print("no. of gnss satellite at each time step:",len(data['gnss_measument'][0]['gnss_measurement']))
if 'air' in data:
    print("airdata records:", len(data['air']))
if len(data['imu']) == 0 and len(data['gnss_measument']) == 0:
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
    'sig_a_d': 0.01,
    'tau_a': 100.0,
    'sig_g_d': 0.00025,
    'tau_g': 50.0,
    'sig_gps_p_ne': 3.0,
    'sig_gps_p_d': 6.0,
    'sig_gps_v_ne': 0.5,
    'sig_gps_v_d': 1.0,
    'sig_mag': 1.0,
    'sig_pseudorange': 1,
    'sig_pseudorangeRate': 1
}
# uNavINS default config
# config = {
#     'sig_w_ax': 0.05,
#     'sig_w_ay': 0.05,
#     'sig_w_az': 0.05,
#     'sig_w_gx': 0.00175,
#     'sig_w_gy': 0.00175,
#     'sig_w_gz': 0.00175,
#     'sig_a_d': 0.01,
#     'tau_a': 100.0,
#     'sig_g_d': 0.00025,
#     'tau_g': 50.0,
#     'sig_gps_p_ne': 3.0,
#     'sig_gps_p_d': 6.0,
#     'sig_gps_v_ne': 0.5,
#     'sig_gps_v_d': 1.0,
#     'sig_mag': 1.0
# }

# select filter
#filter_name = "EKF15"
#filter_name = "EKF15_mag"
#filter_name = "uNavINS"
#filter_name = "uNavINS_BFS"
filter_name = "EKF17"

filter = new_wrapper2.filter(nav=filter_name,
                            gps_lag_sec=args.gps_lag_sec,
                            imu_dt=imu_dt)
filter.set_config(config)
filter.set_ephemeris(ephem_data)

print("Running nav filter:")
results = []

gps_init_sec = None
gpsrawpt = None

iter = flight_interp.IterateGroup(data)
for i in tqdm(range(iter.size())):
    record = iter.next()
    imupt = record['imu']
    if "gpsraw" in record:
        gpsrawpt = record['gpsraw']
        if gps_init_sec is None:
            gps_init_sec = gpsrawpt['time']

    # if not inited or gps not yet reached it's settle time
    if gps_init_sec is None or gpsrawpt['time'] < gps_init_sec + gps_settle_secs:
        continue

    navpt = filter.update(imupt, gpsrawpt)

    # Store the desired results obtained from the compiled test
    # navigation filter and the baseline filter
    results.append(navpt)

# Plotting Section

plotname = os.path.basename(args.flight)    

df0_gps = pd.DataFrame(data['gps'])
df0_gps.set_index('time', inplace=True, drop=False)
df0_nav = pd.DataFrame(data['filter'])
df0_nav.set_index('time', inplace=True, drop=False)

df1_nav = pd.DataFrame(results)
df1_nav.set_index('time', inplace=True, drop=False)

r2d = np.rad2deg

# Attitude
att_fig, att_ax = plt.subplots(3, 1, sharex=True)

att_ax[0].set_title("Attitude Angles")
att_ax[0].set_ylabel('Roll (deg)', weight='bold')
att_ax[0].plot(r2d(df0_nav['phi']), color='g', label='On Board')
att_ax[0].plot(r2d(df1_nav['phi']), label=filter.name)
att_ax[0].grid()

att_ax[1].set_ylabel('Pitch (deg)', weight='bold')
att_ax[1].plot(r2d(df0_nav['the']), color='g', label='On Board')
att_ax[1].plot(r2d(df1_nav['the']), label=filter.name)
att_ax[1].grid()

att_ax[2].set_ylabel('Yaw (deg)', weight='bold')
att_ax[2].plot(r2d(df0_nav['psi']), color='g', label='On Board')
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
ax1.plot(df0_nav['vn'], label="On Board")
ax1.plot(df1_nav['vn'], label=filter.name)
ax1.grid()

# ve Plot
ax2.set_ylabel('ve (mps)', weight='bold')
ax2.plot(df0_gps['ve'], '-*', label='GPS Sensor', c='g', alpha=.5)
ax2.plot(df0_nav['ve'], label="On Board")
ax2.plot(df1_nav['ve'], label=filter.name)
ax2.grid()

# vd Plot
ax3.set_ylabel('vd (mps)', weight='bold')
ax3.plot(df0_gps['vd'], '-*', label='GPS Sensor', c='g', alpha=.5)
ax3.plot(df0_nav['vd'], label="On Board")
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
ax = plt.gca()
ax.axis('equal')

# Biases
bias_fig, bias_ax = plt.subplots(3,2, sharex=True)

# Gyro Biases
bias_ax[0,0].set_title("IMU Biases")
bias_ax[0,0].set_ylabel('p (deg/s)', weight='bold')
bias_ax[0,0].plot(r2d(df0_nav['p_bias']), c='g', label='On Board')
bias_ax[0,0].plot(r2d(df1_nav['gbx']), label=filter.name)
bias_ax[0,0].set_xlabel('Time (secs)', weight='bold')
bias_ax[0,0].grid()

bias_ax[1,0].set_ylabel('q (deg/s)', weight='bold')
bias_ax[1,0].plot(r2d(df0_nav['q_bias']), c='g', label='On Board')
bias_ax[1,0].plot(r2d(df1_nav['gby']), label=filter.name)
bias_ax[1,0].set_xlabel('Time (secs)', weight='bold')
bias_ax[1,0].grid()

bias_ax[2,0].set_ylabel('r (deg/s)', weight='bold')
bias_ax[2,0].plot(r2d(df0_nav['r_bias']), c='g', label='On Board')
bias_ax[2,0].plot(r2d(df1_nav['gbz']), label=filter.name)
bias_ax[2,0].set_xlabel('Time (secs)', weight='bold')
bias_ax[2,0].grid()

# Accel Biases
bias_ax[0,1].set_title("Accel Biases")
bias_ax[0,1].set_ylabel('ax (m/s^2)', weight='bold')
bias_ax[0,1].plot(df0_nav['ax_bias'], c='g', label='On Board')
bias_ax[0,1].plot(df1_nav['abx'], label=filter.name)
bias_ax[0,1].set_xlabel('Time (secs)', weight='bold')
bias_ax[0,1].grid()

bias_ax[1,1].set_ylabel('ay (m/s^2)', weight='bold')
bias_ax[1,1].plot(df0_nav['ay_bias'], c='g', label='On Board')
bias_ax[1,1].plot(df1_nav['aby'], label=filter.name)
bias_ax[1,1].set_xlabel('Time (secs)', weight='bold')
bias_ax[1,1].grid()

bias_ax[2,1].set_ylabel('az (m/s^2)', weight='bold')
bias_ax[2,1].plot(df0_nav['az_bias'], c='g', label='On Board')
bias_ax[2,1].plot(df1_nav['abz'], label=filter.name)
bias_ax[2,1].set_xlabel('Time (secs)', weight='bold')
bias_ax[2,1].grid()
bias_ax[2,1].legend(loc=1)

plt.show()
