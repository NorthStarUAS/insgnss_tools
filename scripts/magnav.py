import numpy as np
import os

# Import the structures
import cdefs
import pydefs

# Import these ctypes for proper declaration of cdefs.py structures
import ctypes
# Abbreviate these ctypes commands
POINTER = ctypes.POINTER
byref   = ctypes.byref

class filter():
    def __init__(self):
        # Load compilied `.so` file.
        self.sharedobj = ctypes.CDLL(os.path.abspath('../build/src/magnav/.libs/libnavigation_mag.so'))
        
        # Declare inputs to the init_nav function
        self.sharedobj.init_nav.argtypes = [POINTER(cdefs.IMU),
                                            POINTER(cdefs.GPS),
                                            POINTER(cdefs.NAV)]

        # Declare inputs to the get_nav function
        self.sharedobj.get_nav.argtypes = [POINTER(cdefs.IMU),
                                           POINTER(cdefs.GPS),
                                           POINTER(cdefs.NAV)]

        self.cnav = cdefs.NAV()

    def python2c(self, imu, gps):
        cimu = cdefs.IMU()
        cimu.time = imu.time
        cimu.p = imu.p
        cimu.q = imu.q
        cimu.r = imu.r
        cimu.ax = imu.ax
        cimu.ay = imu.ay
        cimu.az = imu.az
        cimu.hx = imu.hx
        cimu.hy = imu.hy
        cimu.hz = imu.hz
        
        cgps = cdefs.GPS()
        cgps.time = gps.time
        cgps.tow = gps.tow
        cgps.newData = gps.newData
        cgps.vn = gps.vn
        cgps.ve = gps.ve
        cgps.vd = gps.vd
        cgps.lat = gps.lat
        cgps.lon = gps.lon
        cgps.alt = gps.alt

        return cimu, cgps

    def c2python(self):
        P = np.diag([self.cnav.Pp[0], self.cnav.Pp[1], self.cnav.Pp[2],
                     self.cnav.Pv[0], self.cnav.Pv[1], self.cnav.Pv[2],
                     self.cnav.Pa[0], self.cnav.Pa[1], self.cnav.Pa[2],
                     self.cnav.Pab[0], self.cnav.Pab[1], self.cnav.Pab[2],
                     self.cnav.Pgb[0], self.cnav.Pgb[1], self.cnav.Pgb[2]])
        stateInnov = np.nan*np.ones(6)
        insgps = pydefs.INSGPS(1, # fixme: valid/init
                               self.cnav.time,
                               [self.cnav.lat, self.cnav.lon, self.cnav.alt],
                               [self.cnav.vn, self.cnav.ve, self.cnav.vd],
                               [self.cnav.psi, self.cnav.the, self.cnav.phi],
                               self.cnav.ab,
                               self.cnav.gb,
                               P,
                               stateInnov)
        return insgps
    
    def init(self, imu, gps):
        cimu, cgps = self.python2c(imu, gps)
        self.sharedobj.init_nav(cimu, cgps, self.cnav)
        nav = self.c2python()
        return nav

    def update(self, imu, gps):
        cimu, cgps = self.python2c(imu, gps)
        self.sharedobj.get_nav(cimu, cgps, self.cnav)
        nav = self.c2python()
        return nav

    def close(self):
        self.sharedobj.close_nav()
