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

# Assign pointers that use the structures just declared
sensordata.imuData_ptr = ctypes.pointer(imu)
sensordata.gpsData_ptr   = ctypes.pointer(gpsData)

class filter():
    def __init__(self):
        # Load compilied `.so` file.
        self.sharedobj = ctypes.CDLL(os.path.abspath('../build/src/navigation/.libs/libnavigation.so'))
        
        # Declare inputs to the init_nav function
        self.sharedobj.init_nav.argtypes = [POINTER(globaldefs.SENSORDATA), 
                                            POINTER(globaldefs.NAV)]

        # Declare inputs to the get_nav function
        self.sharedobj.get_nav.argtypes = [POINTER(globaldefs.SENSORDATA), 
                                           POINTER(globaldefs.NAV)]
        
    def init(self, sensorp, navp):
        self.sharedobj.init_nav(sensorp, navp)

    def update(self, sensorp, navp):
        self.sharedobj.get_nav(sensorp, navp)

    def close(self):
        self.sharedobj.close_nav()


