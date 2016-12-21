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

import sys
sys.path.append('../build/src/nav_core/.libs/')
import libnav_core

class filter():
    def __init__(self):
        # Load compilied `.so` file.
        self.sharedobj = ctypes.CDLL(os.path.abspath('../build/src/nav_orig/.libs/libnav_orig.so'))
        
        # Declare inputs to the init_nav function
        self.sharedobj.init_nav.argtypes = [POINTER(cdefs.IMU),
                                            POINTER(cdefs.GPS),
                                            POINTER(cdefs.NAV)]

        # Declare inputs to the get_nav function
        self.sharedobj.get_nav.argtypes = [POINTER(cdefs.IMU),
                                           POINTER(cdefs.GPS),
                                           POINTER(cdefs.NAV)]

        self.cnav = cdefs.NAV()
        self.name = 'EKF15 (C)'
        
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
        cgps.tow = gps.time
        cgps.newData = gps.newData
        cgps.vn = gps.vn
        cgps.ve = gps.ve
        cgps.vd = gps.vd
        cgps.lat = gps.lat
        cgps.lon = gps.lon
        cgps.alt = gps.alt

        return cimu, cgps

    def c2python(self):
        nav = libnav_core.NAVdata()
        nav.lat = self.cnav.lat
        nav.lon = self.cnav.lon
        nav.alt = self.cnav.alt
        nav.vn = self.cnav.vn
        nav.ve = self.cnav.ve
        nav.vd = self.cnav.vd
        nav.phi = self.cnav.phi
        nav.the = self.cnav.the
        nav.psi = self.cnav.psi
        nav.abx = self.cnav.ab[0]
        nav.aby = self.cnav.ab[1]
        nav.abz = self.cnav.ab[2]
        nav.gbx = self.cnav.gb[0]
        nav.gby = self.cnav.gb[1]
        nav.gbz = self.cnav.gb[2]
        nav.Pp0 = self.cnav.Pp[0]
        nav.Pp1 = self.cnav.Pp[1]
        nav.Pp2 = self.cnav.Pp[2]
        nav.Pv0 = self.cnav.Pv[0]
        nav.Pv1 = self.cnav.Pv[1]
        nav.Pv2 = self.cnav.Pv[2]
        nav.Pa0 = self.cnav.Pa[0]
        nav.Pa1 = self.cnav.Pa[1]
        nav.Pa2 = self.cnav.Pa[2]
        nav.Pabx = self.cnav.Pab[0]
        nav.Paby = self.cnav.Pab[1]
        nav.Pabz = self.cnav.Pab[2]
        nav.Pgbx = self.cnav.Pgb[0]
        nav.Pgby = self.cnav.Pgb[1]
        nav.Pgbz = self.cnav.Pgb[2]
        return nav
    
    def init(self, imu, gps, filterpt=None):
        cimu, cgps = self.python2c(imu, gps)
        self.sharedobj.init_nav(cimu, cgps, self.cnav)
        nav = self.c2python()
        nav.time = imu.time
        return nav

    def update(self, imu, gps, filterpt=None):
        cimu, cgps = self.python2c(imu, gps)
        self.sharedobj.get_nav(cimu, cgps, self.cnav)
        nav = self.c2python()
        nav.time = imu.time
        return nav

    def close(self):
        self.sharedobj.close_nav()
