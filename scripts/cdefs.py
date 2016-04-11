"""
This wraps the structures in the globaldefs.c of the UAV flightcode.

**NOTE**
It's IMPORTANT that the structures and fields declared in the globaldefs.c
that gets compiled in the .so file match exactly with the structures and 
fields declared in this globaldefs.py file. Even the ones never used in
a nav filter.

June 4, 2014
Hamid

Revised August 19, 2014 by Trevor Layh
"""
from ctypes import *

# IMU Data Structure
class IMU(Structure):
    _fields_ = [
        (  'p',	c_double), # [rad/sec], body X axis angular rate (roll)
        (  'q',	c_double), # [rad/sec], body Y axis angular rate (pitch)
        (  'r',	c_double), # [rad/sec], body Z axis angular rate (yaw)
        ( 'ax',	c_double), # [m/sec^2], body X axis acceleration
        ( 'ay',	c_double), # [m/sec^2], body Y axis acceleration
        ( 'az',	c_double), # [m/sec^2], body Z axis acceleration
        ( 'hx',	c_double), # [Gauss], body X axis magnetic field
        ( 'hy',	c_double), # [Gauss], body Y axis magnetic field
        ( 'hz',	c_double), # [Gauss], body Z axis magnetic field
        ('phi', c_double), # [rad], Euler roll angle. Only used if IMU sensor reports attitude.
        ('the', c_double), # [rad], Euler pitch angle. Only used if IMU sensor reports attitude.
        ('psi', c_double), # [rad], Euler yaw angle. Only used if IMU sensor reports attitude.
        (  'T', c_float),  # [degC], temperature of IMU sensor
        ( 'Vs', c_float),  # [Volts], supply voltage of IMU sensor
        ('adc', c_double), # [counts], ADC reading
        ('err_type', c_int), #enum errdefs - IMU status
        ('time', c_double)  # [sec], timestamp of IMU data
    ]

# IMU Data Structure
class newIMU(Structure):
    _fields_ = [
        ('time', c_double),  # [sec], timestamp of IMU data
        (  'p',	c_double), # [rad/sec], body X axis angular rate (roll)
        (  'q',	c_double), # [rad/sec], body Y axis angular rate (pitch)
        (  'r',	c_double), # [rad/sec], body Z axis angular rate (yaw)
        ( 'ax',	c_double), # [m/sec^2], body X axis acceleration
        ( 'ay',	c_double), # [m/sec^2], body Y axis acceleration
        ( 'az',	c_double), # [m/sec^2], body Z axis acceleration
        ( 'hx',	c_double), # [Gauss], body X axis magnetic field
        ( 'hy',	c_double), # [Gauss], body Y axis magnetic field
        ( 'hz',	c_double) # [Gauss], body Z axis magnetic field
    ]

# GPS Data Structure
class GPS(Structure):
    _fields_ = [
        (    'lat', c_double), # [deg], Geodetic latitude
        (    'lon', c_double), # [deg], Geodetic longitude
        (    'alt', c_double), # [m], altitude relative to WGS84
        (     've', c_double), # [m/sec], East velocity
        (     'vn', c_double), # [m/sec], North velocity
        (     'vd', c_double), # [m/sec], Down velocity
        (     'Xe', c_double), # [m], X position, ECEF
        (     'Ye', c_double), # [m], Y position, ECEF
        (     'Ze', c_double), # [m], Z position, ECEF
        (     'Ue', c_double), # [m/sec], X velocity, ECEF
        (     'Ve', c_double), # [m/sec], Y velocity, ECEF
        (     'We', c_double), # [m/sec], Z velocity, ECEF
        (  'sig_N', c_double), # [m], Position error standard deviation in the North direction
        (  'sig_E', c_double), # [m], Position error standard deviation in the East direction
        (  'sig_D', c_double), # [m], Position error standard deviation in the Down direction
        ( 'sig_vn', c_double), # [m/sec], Velocity error standard deviation in the North direction
        ( 'sig_ve', c_double), # [m/sec], Velocity error standard deviation in the East direction
        ( 'sig_vd', c_double), # [m/sec], Velocity error standard deviation in the Down direction
        ('GPS_TOW', c_double), #[sec], GPS Time Of Week
        ('courseOverGround', c_double), # [rad], course over the ground, relative to true North
        ('speedOverGround',  c_double), # [rad], speed over the ground
        (      'time', c_double), # [sec], timestamp of GPS data
        (   'newData', c_ushort), # [bool], flag set when GPS data has been updated
        ('satVisible', c_ushort), # Number satellites used in the position solution
        (  'navValid', c_ushort), # flag indicating whether the solution is valid, 0 = valid
        (  'GPS_week', c_ushort), # GPS week since current epoch.
        ( 'err_type', c_int), # GPS status
        ( 'baudRate', c_int), # Baud rate for serial port
        ('portName', c_char_p), # Name of serial port
        ('port', c_int),        # handle for accessing serial port
        ('localBuffer', c_char_p), # local buffer to store partial serial data packets
        ( 'bytesInLocalBuffer', c_int), # number of bytes in the local buffer
        (          'readState', c_int), # current state of serial data reader function
        (         'read_calls', c_int) # number of times the read_gps function has been called
    ]


# GPS Data Structure
class newGPS(Structure):
    _fields_ = [
        (   'time', c_double), # [sec], timestamp of GPS data
        (    'lat', c_double), # [deg], Geodetic latitude
        (    'lon', c_double), # [deg], Geodetic longitude
        (    'alt', c_double), # [m], altitude relative to WGS84
        (     'vn', c_double), # [m/sec], North velocity
        (     've', c_double), # [m/sec], East velocity
        (     'vd', c_double), # [m/sec], Down velocity
        ('newData', c_ushort)  # [bool], flag set when GPS data has been updated
    ]


# Air Data Structure
class AIRDATA(Structure):
    _fields_ = [
        (       'h', c_double), # [m], barometric altitude above ground level (AGL)
        (     'ias', c_double), # [m/sec], indicated airspeed
        (  'h_filt', c_double), # [m], filtered altitude
        ('ias_filt', c_double), # [m/s], filtered airspeed
        (      'Ps', c_double), # [KPa], static pressure
        (      'Pd', c_double), # [KPa], dynamic pressure
        (     'aoa', c_double), # [rad], angle of attack from 5-hole Pitot probe
        (     'aos', c_double), # [rad], angle of sideslip from 5-hole Pitot probe
        ( 'l_alpha', c_double), # [rad], angle of attack, from left vane
        ( 'r_alpha', c_double), # [rad], angle of attack, from right vane
        (  'l_beta', c_double), # [rad], angle of sideslip, from left vane
        (  'r_beta', c_double), # [rad], angle of sideslip, from right vane
        (  'Pd_aoa', c_double), # [KPa], dynamic pressure for aoa, from 5-hole Pitot probe
        (  'Pd_aos', c_double), # [KPa], dynamic pressure for aos, from 5-hole Pitot probe
        ( 'bias', c_double*10), # array for storing biases for air data.
        ( 'status', c_ushort)   # status bitfield for air data sensors.
    ]


# Control surface deflections
class SURFACE(Structure):
    _fields_ = [
        ('dthr_pos', c_double), # [0-1], measured throttle position
        (  'de_pos', c_double), # [rad], measured elevator position, +TED
        (  'dr_pos', c_double), # [rad], measured rudder position, +TEL
        ('da_l_pos', c_double), # [rad], measured left aileron position, +TED
        ('da_r_pos', c_double), # [rad], measured right aileron position, +TED
        ('df_l_pos', c_double), # [rad], measured left flap position, +TED
        ('df_r_pos', c_double)  # [rad], measured right flap position, +TED
    ]

# Pilot inceptor data structure
class INCEPTOR(Structure):
    _fields_ = [
        ('throttle', c_double), # throttle stick command from the pilot, ND
        (   'pitch', c_double), # pitch stick command from the pilot, ND
        (     'yaw', c_double), # yaw stick command from the pilot, ND
        (    'roll', c_double)  # roll stick command from the pilot, ND
    ]
    
# Mission manager data structure
class MISSION(Structure):
    _fields_ = [
        (       'mode', c_ushort), # mode variable; 0 = dump data, 1 = manual control, 2 = autopilot control
        (    'run_num', c_ushort), # counter for number of autopilot engagements
        ('researchNav', c_ushort), # mode variable; 0 = standard nav filter, 1 = research nav filter
        ('researchGuidance', c_ushort), # mode variable; 0 = standard guidance, 1 = research guidance
        ('haveGPS', c_ushort)      # mode variable; 0 = no GPS, 1 = have GPS
    ]

# Control Data structure
class CONTROL(Structure):
    _fields_ = [
        (      'dthr', c_double), #   ///< [0-1], throttle command
        (        'de', c_double), #     ///< [rad], elevator command, +TED
        (        'dr', c_double), #     ///< [rad], rudder command, +TEL
        (      'da_l', c_double), #   ///< [rad], left aileron command, +TED
        (      'da_r', c_double), #   ///< [rad], right aileron command, +TED
        (      'df_l', c_double), #   ///< [rad], left flap command, +TED
        (      'df_r', c_double), #   ///< [rad], right flap command, +TED
        (   'phi_cmd', c_double), # //< [rad], Euler roll angle command
        ( 'theta_cmd', c_double), # < [rad], Euler pitch angle command
        (   'psi_cmd', c_double), # //< [rad], Euler yaw angle command
        (     'p_cmd', c_double), #  ///< [rad/sec], body axis roll rate command
        (     'q_cmd', c_double), #  ///< [rad/sec], body axis pitch rate command
        (     'r_cmd', c_double), #  ///< [rad/sec], body axis yaw rate command
        (   'ias_cmd', c_double), # //< [m/sec], airspeed command
        (     'h_cmd', c_double), #  ///< [m], altitude command
        ('gndtrk_cmd', c_double), #  [rad], ground track angle command, relative to true north
        (   'aoa_cmd', c_double), # //< [rad], angle of attack command
        (   'aos_cmd', c_double), # //< [rad], angle of sideslip command
        ( 'gamma_cmd', c_double), # < [rad], flight path angle command
        (  'signal_0', c_double), # //< user defined dummy variable
        (  'signal_1', c_double), # //< user defined dummy variable
        (  'signal_2', c_double), # //< user defined dummy variable
        (  'signal_3', c_double), # //< user defined dummy variable
        (  'signal_4', c_double), # //< user defined dummy variable
        (  'signal_5', c_double), # //< user defined dummy variable
        (  'signal_6', c_double), # //< user defined dummy variable
        (  'signal_7', c_double), # //< user defined dummy variable
        (  'signal_8', c_double), # //< user defined dummy variable
        (  'signal_9', c_double)  # //< user defined dummy variable
    ]

# Research Control Data structure
class RESEARCHCONTROL(Structure):
    _fields_ = [
        (      'dthr', c_double), #   ///< [0-1], throttle command
        (        'de', c_double), #     ///< [rad], elevator command, +TED
        (        'dr', c_double), #     ///< [rad], rudder command, +TEL
        (      'da_l', c_double), #   ///< [rad], left aileron command, +TED
        (      'da_r', c_double), #   ///< [rad], right aileron command, +TED
        (      'df_l', c_double), #   ///< [rad], left flap command, +TED
        (      'df_r', c_double), #   ///< [rad], right flap command, +TED
        (   'phi_cmd', c_double), # //< [rad], Euler roll angle command
        ( 'theta_cmd', c_double), # < [rad], Euler pitch angle command
        (   'psi_cmd', c_double), # //< [rad], Euler yaw angle command
        (     'p_cmd', c_double), #  ///< [rad/sec], body axis roll rate command
        (     'q_cmd', c_double), #  ///< [rad/sec], body axis pitch rate command
        (     'r_cmd', c_double), #  ///< [rad/sec], body axis yaw rate command
        (   'ias_cmd', c_double), # //< [m/sec], airspeed command
        (     'h_cmd', c_double), #  ///< [m], altitude command
        ('gndtrk_cmd', c_double), #  [rad], ground track angle command, relative to true north
        (   'aoa_cmd', c_double), # //< [rad], angle of attack command
        (   'aos_cmd', c_double), # //< [rad], angle of sideslip command
        ( 'gamma_cmd', c_double), # < [rad], flight path angle command
        (  'signal_0', c_double), # //< user defined dummy variable
        (  'signal_1', c_double), # //< user defined dummy variable
        (  'signal_2', c_double), # //< user defined dummy variable
        (  'signal_3', c_double), # //< user defined dummy variable
        (  'signal_4', c_double), # //< user defined dummy variable
        (  'signal_5', c_double), # //< user defined dummy variable
        (  'signal_6', c_double), # //< user defined dummy variable
        (  'signal_7', c_double), # //< user defined dummy variable
        (  'signal_8', c_double), # //< user defined dummy variable
        (  'signal_9', c_double)  # //< user defined dummy variable
    ]


# Navigation Filter Data Structure
class NAV(Structure):
    _fields_ = [
        (   'lat', c_double), # [rad], geodetic latitude estimate
        (   'lon', c_double), # [rad], geodetic longitude estimate
        (   'alt', c_double), # [m], altitude relative to WGS84 estimate
        (    'vn', c_double), # [m/sec], north velocity estimate
        (    've', c_double), # [m/sec], east velocity estimate
        (    'vd', c_double), # [m/sec], down velocity estimate
        (   'phi', c_double), # [rad], Euler roll angle estimate
        (   'the', c_double), # [rad], Euler pitch angle estimate
        (   'psi', c_double), # [rad], Euler yaw angle estimate
        (  'quat', c_double*4), # Quaternions estimate
        (    'ab', c_double*3), # [m/sec^2], accelerometer bias estimate
        (    'gb', c_double*3), # [rad/sec], rate gyro bias estimate
        (   'asf', c_double*3), # [m/sec^2], accelerometer scale factor estimate
        (   'gsf', c_double*3), # [rad/sec], rate gyro scale factor estimate
        (    'Pp', c_double*3), # [rad], covariance estimate for position
        (    'Pv', c_double*3), # [rad], covariance estimate for velocity
        (    'Pa', c_double*3), # [rad], covariance estimate for angles
        (   'Pab', c_double*3), # [rad], covariance estimate for accelerometer bias
        (   'Pgb', c_double*3), # [rad], covariance estimate for rate gyro bias
        (  'Pasf', c_double*3), # [rad], covariance estimate for accelerometer scale factor
        (  'Pgsf', c_double*3), # [rad], covariance estimate for rate gyro scale factor
        ('err_type', c_int),     # NAV filter status
        (      'time', c_double), # [sec], timestamp of NAV filter
        (  'wn', c_double), # [m/s], estimated wind speed in the north direction
        (  'we', c_double), # [m/s], estimated wind speed in the east direction
        (  'wd', c_double), # [m/s], estimated wind speed in the down direction
        (  'signal_0', c_double), # //< user defined dummy variable
        (  'signal_1', c_double), # //< user defined dummy variable
        (  'signal_2', c_double), # //< user defined dummy variable
        (  'signal_3', c_double), # //< user defined dummy variable
        (  'signal_4', c_double), # //< user defined dummy variable
        (  'signal_5', c_double), # //< user defined dummy variable
        (  'signal_6', c_double), # //< user defined dummy variable
        (  'signal_7', c_double), # //< user defined dummy variable
        (  'signal_8', c_double), # //< user defined dummy variable
        (  'signal_9', c_double)  # //< user defined dummy variable    
    ]

# Navigation Filter Data Structure
class newNAV(Structure):
    _fields_ = [
        (  'time', c_double), # [sec], timestamp of NAV filter
        (   'lat', c_double), # [rad], geodetic latitude estimate
        (   'lon', c_double), # [rad], geodetic longitude estimate
        (   'alt', c_double), # [m], altitude relative to WGS84 estimate
        (    'vn', c_double), # [m/sec], north velocity estimate
        (    've', c_double), # [m/sec], east velocity estimate
        (    'vd', c_double), # [m/sec], down velocity estimate
        (   'phi', c_double), # [rad], Euler roll angle estimate
        (   'the', c_double), # [rad], Euler pitch angle estimate
        (   'psi', c_double), # [rad], Euler yaw angle estimate
        (  'quat', c_double*4), # Quaternions estimate
        (    'ab', c_double*3), # [m/sec^2], accelerometer bias estimate
        (    'gb', c_double*3), # [rad/sec], rate gyro bias estimate
        (   'asf', c_double*3), # [m/sec^2], accelerometer scale factor estimate
        (   'gsf', c_double*3), # [rad/sec], rate gyro scale factor estimate
        (    'Pp', c_double*3), # [rad], covariance estimate for position
        (    'Pv', c_double*3), # [rad], covariance estimate for velocity
        (    'Pa', c_double*3), # [rad], covariance estimate for angles
        (   'Pab', c_double*3), # [rad], covariance estimate for accelerometer bias
        (   'Pgb', c_double*3), # [rad], covariance estimate for rate gyro bias
        (  'Pasf', c_double*3), # [rad], covariance estimate for accelerometer scale factor
        (  'Pgsf', c_double*3), # [rad], covariance estimate for rate gyro scale factor
        ('err_type', c_int)     # NAV filter status
    ]

# Research Navigation Filter Data Structure
class RESEARCHNAV(Structure):
    _fields_ = [
        (   'lat', c_double), # [rad], geodetic latitude estimate
        (   'lon', c_double), # [rad], geodetic longitude estimate
        (   'alt', c_double), # [m], altitude relative to WGS84 estimate
        (    'vn', c_double), # [m/sec], north velocity estimate
        (    've', c_double), # [m/sec], east velocity estimate
        (    'vd', c_double), # [m/sec], down velocity estimate
        (   'phi', c_double), # [rad], Euler roll angle estimate
        (   'the', c_double), # [rad], Euler pitch angle estimate
        (   'psi', c_double), # [rad], Euler yaw angle estimate
        (  'quat', c_double*4), # Quaternions estimate
        (    'ab', c_double*3), # [m/sec^2], accelerometer bias estimate
        (    'gb', c_double*3), # [rad/sec], rate gyro bias estimate
        (   'asf', c_double*3), # [m/sec^2], accelerometer scale factor estimate
        (   'gsf', c_double*3), # [rad/sec], rate gyro scale factor estimate
        (    'Pp', c_double*3), # [rad], covariance estimate for position
        (    'Pv', c_double*3), # [rad], covariance estimate for velocity
        (    'Pa', c_double*3), # [rad], covariance estimate for angles
        (   'Pab', c_double*3), # [rad], covariance estimate for accelerometer bias
        (   'Pgb', c_double*3), # [rad], covariance estimate for rate gyro bias
        (  'Pasf', c_double*3), # [rad], covariance estimate for accelerometer scale factor
        (  'Pgsf', c_double*3), # [rad], covariance estimate for rate gyro scale factor
        ('err_type', c_int),     # NAV filter status
        (      'time', c_double), # [sec], timestamp of NAV filter
        (  'wn', c_double), # [m/s], estimated wind speed in the north direction
        (  'we', c_double), # [m/s], estimated wind speed in the east direction
        (  'wd', c_double), # [m/s], estimated wind speed in the down direction
        (  'signal_0', c_double), # //< user defined dummy variable
        (  'signal_1', c_double), # //< user defined dummy variable
        (  'signal_2', c_double), # //< user defined dummy variable
        (  'signal_3', c_double), # //< user defined dummy variable
        (  'signal_4', c_double), # //< user defined dummy variable
        (  'signal_5', c_double), # //< user defined dummy variable
        (  'signal_6', c_double), # //< user defined dummy variable
        (  'signal_7', c_double), # //< user defined dummy variable
        (  'signal_8', c_double), # //< user defined dummy variable
        (  'signal_9', c_double)  # //< user defined dummy variable    
    ]

class SENSORDATA(Structure):
    _fields_ = [
	('imuData_ptr',   POINTER(IMU)), # pointer to imu data structure		
	('gpsData_ptr',   POINTER(GPS)), # pointer to gps data structure
	('gpsData_l_ptr', POINTER(GPS)), # pointer to left gps data structure
	('gpsData_r_ptr', POINTER(GPS)), # pointer to right gps data structure
	('adData_ptr',    POINTER(AIRDATA)), # pointer to airdata data structure
	('surfData_ptr',  POINTER(SURFACE)), # pointer to surface data structure
        ('inceptorData_ptr', POINTER(INCEPTOR))  # pointer to pilot inceptor data structure
    ]
