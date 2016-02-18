import os

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
controlData = globaldefs.CONTROL()
gpsData     = globaldefs.GPS()
airData     = globaldefs.AIRDATA()
surface     = globaldefs.SURFACE()
mission     = globaldefs.MISSION()

sensordata_mag = globaldefs.SENSORDATA()
imu_mag = globaldefs.IMU()
gpsData_mag     = globaldefs.GPS()

# Assign pointers that use the structures just declared
sensordata.imuData_ptr = ctypes.pointer(imu)
sensordata.gpsData_ptr   = ctypes.pointer(gpsData)
sensordata.adData_ptr   = ctypes.pointer(airData)
sensordata.surfData_ptr = ctypes.pointer(surface)

sensordata_mag.imuData_ptr = ctypes.pointer(imu_mag)
sensordata_mag.gpsData_ptr   = ctypes.pointer(gpsData_mag)


class filter():
    def __init__(self):
        # Load compilied `.so` file.
        sharedobj = ctypes.CDLL(os.path.abspath('../build/src/navigation/.libs/libnavigation.so'))
        
        # Name the init_nav function defined as the init_nav from the
        # compiled nav filter
        self.init_nav = sharedobj.init_nav
        # Declare inputs to the init_nav function
        self.init_nav.argtypes = [POINTER(globaldefs.SENSORDATA), 
                                  POINTER(globaldefs.NAV), 
                                  POINTER(globaldefs.CONTROL)]

        # Name the get_nav function defined as the get_nav from the
        # compiled nav filter
        self.get_nav = sharedobj.get_nav
        # Declare inputs to the get_nav function
        self.get_nav.argtypes = [POINTER(globaldefs.SENSORDATA), 
                                 POINTER(globaldefs.NAV), 
                                 POINTER(globaldefs.CONTROL)]
        
        # Name the close_nav function defined as the close_nav from
        # the compiled nav filter
        self.close_nav = sharedobj.close_nav

    def init(self):
        pass

