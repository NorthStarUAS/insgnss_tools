import os

# Import these ctypes for proper declaration of globaldefs.py structures
import ctypes
# Import the globaldefs.py file
import globaldefs
# Abbreviate these ctypes commands
POINTER = ctypes.POINTER
byref   = ctypes.byref

class filter():
    def __init__(self):
        # Load compilied `.so` file.
        self.sharedobj = ctypes.CDLL(os.path.abspath('../build/src/navigation/.libs/libnavigation.so'))
        
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


