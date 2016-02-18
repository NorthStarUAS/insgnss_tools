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
gpsData     = globaldefs.GPS()

sensordata_mag = globaldefs.SENSORDATA()
imu_mag = globaldefs.IMU()
gpsData_mag     = globaldefs.GPS()

# Assign pointers that use the structures just declared
sensordata.imuData_ptr = ctypes.pointer(imu)
sensordata.gpsData_ptr   = ctypes.pointer(gpsData)

sensordata_mag.imuData_ptr = ctypes.pointer(imu_mag)
sensordata_mag.gpsData_ptr   = ctypes.pointer(gpsData_mag)

class filter():
    def __init__(self):
        # Load compilied `.so` file.
        self.sharedobj = ctypes.CDLL(os.path.abspath('../build/src/magnav/.libs/libnavigation_mag.so'))
        
        # Declare inputs to the init_nav function
        self.sharedobj.init_nav.argtypes = [POINTER(globaldefs.IMU),
                                            POINTER(globaldefs.GPS),
                                            POINTER(globaldefs.NAV)]

        # Declare inputs to the get_nav function
        self.sharedobj.get_nav.argtypes = [POINTER(globaldefs.IMU),
                                           POINTER(globaldefs.GPS),
                                           POINTER(globaldefs.NAV)]

    def init(self, imup, gpsp, navp):
        self.sharedobj.init_nav(imup, gpsp, navp)

    def update(self, imup, gpsp, navp):
        self.sharedobj.get_nav(imup, gpsp, navp)

    def close(self):
        self.sharedobj.close_nav()
