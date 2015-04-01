"""WRAP_NAV_FILTER.PY

This script wraps the a navigation filter C-Code (assuming it is compiled 
into a `.so` shared object) and plays `.mat` flight data that has been
converted to have the flight_data and flight_info structures through the 
code in python.

More details on compiling the C-Code to make the `.so` can be found in
`README.md`.

June 19, 2014      [Hamid M. ] initial version for experimental AHRS/air-speed dead-reckoning filter
August 19, 2014    [Trevor L.] modified and extended to work with nominal INS/GPS
September 25, 2014 [Hamid M. ] clean up comments and naming
March 31, 2014     [Hamid M. ] add commands to build .so on run
"""
# Build the 'Cbuild/EKF_15state_quat.so' object
# Note: Rerunning this in interactive mode has unexpected results.  
#       It doesn't seem to reload the latest `.so`
import subprocess, os, sys

if not os.path.isdir('Cbuild'):
    os.mkdir('Cbuild')

BUILD_FAILED_FLAG = 0
join = os.path.join
## Build `nav_functions.o`
cmd = 'gcc -o '+ join('Cbuild', 'nav_functions.o') + ' -c ' + join('Csources', 'nav_functions.c') +' -fPIC'
p = subprocess.Popen(cmd, shell=True)
BUILD_FAILED_FLAG |= p.wait() # p.wait() returns a '1' if process failed

## Build `matrix.o`
cmd = 'gcc -o ' + join('Cbuild', 'matrix.o') + ' -c ' + join('Csources', 'matrix.c') + ' -fPIC'
p = subprocess.Popen(cmd, shell=True)
BUILD_FAILED_FLAG |= p.wait()

## Build `EKF_15state_quat.o`
cmd = 'gcc -o ' + join('Cbuild', 'EKF_15state_quat.o') + ' -c ' + join('Csources', 'EKF_15state_quat.c') + ' -fPIC'
p = subprocess.Popen(cmd, shell=True)
BUILD_FAILED_FLAG |= p.wait()

## Link into shared object
cmd = 'gcc -lm -shared -Wl",-soname,EKF_15state_quat" -o ' + join('Cbuild', 'EKF_15state_quat.so') + ' ' \
                                                           + join('Cbuild', 'EKF_15state_quat.o')  + ' ' \
                                                           + join('Cbuild', 'matrix.o') + ' ' \
                                                           + join('Cbuild', 'nav_functions.o')
p = subprocess.Popen(cmd, shell=True)
BUILD_FAILED_FLAG |= p.wait()

if BUILD_FAILED_FLAG:
    sys.exit('Ending Program.  Failed to build C-code.')




# Import these ctypes for proper declaration of globaldefs.py structures
import ctypes
# Import the globaldefs.py file
import globaldefs
# Abbreviate these ctypes commands
POINTER = ctypes.POINTER
byref   = ctypes.byref

# Declare Structures from globaldefs.py
sensordata = globaldefs.SENSORDATA()
imu = globaldefs.IMU()
nav = globaldefs.NAV()
controlData = globaldefs.CONTROL()
gpsData     = globaldefs.GPS()
gpsData_l   = globaldefs.GPS()
gpsData_r   = globaldefs.GPS()
airData     = globaldefs.AIRDATA()
surface     = globaldefs.SURFACE()

# Assign pointers that use the structures just declared
sensordata.imuData_ptr = ctypes.pointer(imu)
sensordata.gpsData_ptr   = ctypes.pointer(gpsData)
sensordata.gpsData_l_ptr = ctypes.pointer(gpsData_l)
sensordata.gpsData_r_ptr = ctypes.pointer(gpsData_r)
sensordata.adData_ptr   = ctypes.pointer(airData)
sensordata.surfData_ptr = ctypes.pointer(surface)


# Load compilied `.so` file.
compiled_test_nav_filter = ctypes.CDLL(os.path.abspath('Cbuild/EKF_15state_quat.so'))

# Name the init_nav function defined as the init_nav from the compiled nav filter
init_nav = compiled_test_nav_filter.init_nav
# Declare inputs to the init_nav function
init_nav.argtypes = [POINTER(globaldefs.SENSORDATA), 
                          POINTER(globaldefs.NAV), 
                          POINTER(globaldefs.CONTROL)]

# Name the get_nav function defined as the get_nav from the compiled nav filter
get_nav = compiled_test_nav_filter.get_nav
# Declare inputs to the get_nav function
get_nav.argtypes = [POINTER(globaldefs.SENSORDATA), 
                         POINTER(globaldefs.NAV), 
                         POINTER(globaldefs.CONTROL)]

# Name the close_nav function defined as the close_nav from the compiled nav filter
close_nav = compiled_test_nav_filter.close_nav

# Import modules including the numpy and scipy.  Matplotlib is used for plotting results.
import os
import numpy as np
from scipy import io as sio
from matplotlib import pyplot as plt
r2d = np.rad2deg

# Directory to converted flight data that contains the flight_data and flight_info structures
directory = 'flight_data'

# Name of .mat file that exists in the directory defined above and has the flight_data and flight_info structures
filename = 'thor_flight75_WaypointTracker_150squareWaypointNew_2012_10_10'
filepath = directory+os.sep+filename

# Load Flight Data: ## IMPORTANT to have the .mat file in the flight_data and flight_info structures for this function ##
data = sio.loadmat(filepath, struct_as_record=False, squeeze_me=True)
flight_data, flight_info = data['flight_data'], data['flight_info']
del(data)
print('Loaded Data Summary')
print('* File: %s' % filepath.split(os.path.sep))[-1]
print('* Date: %s' % flight_info.date)
print('* Aircraft: %s' % flight_info.aircraft)

# Fill in time data
t = flight_data.time

# Magnetometer data - not used hence don't trust
hm  = np.vstack((flight_data.hx, flight_data.hy, flight_data.hz)).T

# Populate IMU Data
imu = np.vstack((t, flight_data.p, flight_data.q, flight_data.r, 
                    flight_data.ax, flight_data.ay, flight_data.az,
                    hm[:,0], hm[:,1], hm[:,2])).T

# Note that accelerometer and gyro measurements logged by UAV
# after 11/17/2011 flight (seemingly, see 
# http://trac.umnaem.webfactional.com/wiki/FlightReports/2011_11_17)
# have the nav-estimated bias removed before datalogging. So to work with raw
# imu-data, we add back the on-board estimated biases.
try:
   imu[:, 1:4] += np.vstack((flight_data.p_bias, 
                           flight_data.q_bias, 
                           flight_data.r_bias)).T
                           
   imu[:, 4:7] += np.vstack((flight_data.ax_bias,
                           flight_data.ay_bias,
                           flight_data.az_bias)).T
except AttributeError:
   print('Note: On board estimated bias not found.')
   pass

# Air Data
ias = flight_data.ias # indicated airspeed (m/s)
h = flight_data.h

# Populate GPS sensor data
vn = flight_data.vn
ve = flight_data.ve
vd = flight_data.vd
lat = flight_data.lat
lon = flight_data.lon
alt = flight_data.alt

# kstart set to when the navigation filter used onboard the aircraft was initialized
# and this is accomplished by detecting when navlat is no longer 0.0. This choice
# of kstart will ensure the filter being tested is using the same initialization
# time step as the onboard filter allowing for apples to apples comparisons.
kstart = (abs(flight_data.navlat) > 0.0).tolist().index(True)
k = kstart
print('Initialized at Time: %.2f s (k=%i)' % (t[k], k))

# Set previous value of GPS altitude to 0.0. This will be used to trigger GPS newData
# flag which is commonly used in our navigation filters for deciding if the GPS
# data has been updated. However, in python we have no log of newData (typically). So
# a comparison of current GPS altitude to the previous epoch's GPS altitude is used to
# determine if GPS has been updated.
old_GPS_alt = 0.0

# Values (Calculated by compiled test navigation filter) need to be stored in python
# variables and they need to be in the globaldefs.c and globaldefs.py to allow for
# pulling them out and saving. These python variables need to be initialized to work
# properly in the while loop.
psi_store, the_store, phi_store = [],[],[]
navlat_store, navlon_store, navalt_store = [],[],[]
navStatus_store = []
t_store = []

# Using while loop starting at k (set to kstart) and going to end of .mat file
while k < len(t):
    # Populate this epoch's IMU data
    p ,  q,  r = imu[k, 1:4]
    ax, ay, az = imu[k, 4:7]
    hx, hy, hz = imu[k, 7:10]

    # Assign that IMU data extracted from the .mat file at the current epoch to the 
    # pointer values and structures to be passed into "get_" functions of the c-code.
    sensordata.imuData_ptr.contents.p = p
    sensordata.imuData_ptr.contents.q = q
    sensordata.imuData_ptr.contents.r = r


    sensordata.imuData_ptr.contents.ax = ax
    sensordata.imuData_ptr.contents.ay = ay
    sensordata.imuData_ptr.contents.az = az

    sensordata.imuData_ptr.contents.hx = hx
    sensordata.imuData_ptr.contents.hy = hy
    sensordata.imuData_ptr.contents.hz = hz

    # Assign the current time
    sensordata.imuData_ptr.contents.time = t[k]    

    # Assign Air Data
    sensordata.adData_ptr.contents.ias = ias[k]
    sensordata.adData_ptr.contents.h = h[k]

    # Assign GPS Data
    sensordata.gpsData_ptr.contents.vn = vn[k]
    sensordata.gpsData_ptr.contents.ve = ve[k]
    sensordata.gpsData_ptr.contents.vd = vd[k]

    sensordata.gpsData_ptr.contents.lat = lat[k]
    sensordata.gpsData_ptr.contents.lon = lon[k]
    sensordata.gpsData_ptr.contents.alt = alt[k]

    # Set GPS newData flag
    if ((abs(flight_data.alt[k] - old_GPS_alt))>.0001):
        sensordata.gpsData_ptr.contents.newData = 1
        old_GPS_alt = flight_data.alt[k]
    else:
        sensordata.gpsData_ptr.contents.newData = 0

    # If k is at the initialization time init_nav else get_nav
    if k == kstart:
        init_nav(sensordata, nav, controlData)
    else:
        get_nav(sensordata, nav, controlData)

    # Store the desired results obtained from the compiled test navigation filter
    psi_store.append(nav.psi)
    the_store.append(nav.the)
    phi_store.append(nav.phi)
    navlat_store.append(nav.lat)
    navlon_store.append(nav.lon)
    navalt_store.append(nav.alt)
    navStatus_store.append(nav.err_type)
    t_store.append(t[k])

    # Increment time up one step for the next iteration of the while loop.    
    k+=1

# When k = len(t) execute the close_nav function freeing up memory from matrices.
close_nav()


# Plotting
fig, [ax1, ax2, ax3] = plt.subplots(3,1)

# Yaw Plot
ax1.set_title(filename, fontsize=10)
ax1.set_ylabel('YAW (DEGREES)', weight='bold')
ax1.plot(t, r2d(flight_data.psi), label='onboard', c='k', lw=3, alpha=.5)
ax1.plot(t_store, r2d(psi_store), label='PythonWrap',c='blue', lw=2)
ax1.grid()
ax1.legend(loc=0)

# Pitch PLot
ax2.set_ylabel('PITCH (DEGREES)', weight='bold')
ax2.plot(t, r2d(flight_data.theta), label='onboard', c='k', lw=3, alpha=.5)
ax2.plot(t_store, r2d(the_store), label='PythonWrap',c='blue', lw=2)
ax2.grid()

# Roll PLot
ax3.set_ylabel('ROLL (DEGREES)', weight='bold')
ax3.plot(t, r2d(flight_data.phi), label='onboard', c='k', lw=3, alpha=.5)
ax3.plot(t_store, r2d(phi_store), label='PythonWrap', c='blue',lw=2)
ax3.set_xlabel('TIME (SECONDS)', weight='bold')
ax3.grid()

# Altitude Plot
plt.figure()
plt.plot(t[kstart:len(t)], flight_data.alt[kstart:len(t)], label='GPS Sensor', c='green', lw=3, alpha=.5)
plt.plot(t[kstart:len(t)], flight_data.navalt[kstart:len(t)], label='onboard', c='k', lw=3, alpha=.5)
plt.plot(t_store[kstart:len(t)], navalt_store[kstart:len(t)], label='PythonWrap',c='blue', lw=2)
plt.ylabel('ALTITUDE (METERS)', weight='bold')
plt.legend(loc=0)
plt.grid()

# Top View (Longitude vs. Latitude) Plot
plt.figure()
plt.title(filename, fontsize=10)
plt.ylabel('LATITUDE (DEGREES)', weight='bold')
plt.xlabel('LONGITUDE (DEGREES)', weight='bold')
plt.plot(flight_data.lon[kstart:len(t)], flight_data.lat[kstart:len(t)], label='GPS Sensor', c='green', lw=2, alpha=.5)
plt.plot(r2d(flight_data.navlon[kstart:len(t)]), r2d(flight_data.navlat[kstart:len(t)]), label='onboard', c='k', lw=3, alpha=.5)
plt.plot(r2d(navlon_store[kstart:len(t)]), r2d(navlat_store[kstart:len(t)]), label='PythonWrap', c='blue', lw=2)
plt.grid()
plt.legend(loc=0)
plt.show()