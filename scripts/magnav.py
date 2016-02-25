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
        self.sharedobj.init_nav.argtypes = [POINTER(cdefs.cIMU),
                                            POINTER(cdefs.cGPS),
                                            POINTER(cdefs.cNAV)]

        # Declare inputs to the get_nav function
        self.sharedobj.get_nav.argtypes = [POINTER(cdefs.cIMU),
                                           POINTER(cdefs.cGPS),
                                           POINTER(cdefs.cNAV)]

        self.navp = cdefs.cNAV()

    def python2c(self, imu, gps):
        cimu = cdefs.cIMU()
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
        
        cgps = cdefs.cGPS()
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
        P = np.diag([self.navp.Pp[0], self.navp.Pp[1], self.navp.Pp[2],
                     self.navp.Pv[0], self.navp.Pv[1], self.navp.Pv[2],
                     self.navp.Pa[0], self.navp.Pa[1], self.navp.Pa[2],
                     self.navp.Pab[0], self.navp.Pab[1], self.navp.Pab[2],
                     self.navp.Pgb[0], self.navp.Pgb[1], self.navp.Pgb[2]])
        stateInnov = np.nan*np.ones(6)
        insgps = pydefs.INSGPS(1, # fixme: valid/init
                               self.navp.time,
                               [self.navp.lat, self.navp.lon, self.navp.alt],
                               [self.navp.vn, self.navp.ve, self.navp.vd],
                               [self.navp.psi, self.navp.the, self.navp.phi],
                               self.navp.ab,
                               self.navp.gb,
                               P,
                               stateInnov)
        return insgps
    
    def init(self, imu, gps):
        cimu, cgps = self.python2c(imu, gps)
        self.sharedobj.init_nav(cimu, cgps, self.navp)
        nav = self.c2python()
        return nav

    def update(self, imu, gps):
        cimu, cgps = self.python2c(imu, gps)
        self.sharedobj.get_nav(cimu, cgps, self.navp)
        nav = self.c2python()
        return nav

    def close(self):
        self.sharedobj.close_nav()
