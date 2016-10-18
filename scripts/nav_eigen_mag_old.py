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
        sharedobj = ctypes.CDLL(os.path.abspath('../build/src/nav_eigen_mag_old/.libs/libnav_eigen_mag_old.so'))

        self.init_func = sharedobj._Z8init_nav7IMUdata7GPSdata
        self.update_func = sharedobj._Z7get_nav7IMUdata7GPSdata
        
        # Declare inputs to the init_nav function
        self.init_func.argtypes = [cdefs.newIMU,
                                   cdefs.newGPS]
        self.init_func.restype = cdefs.newNAV
        
        # Declare inputs to the get_nav function
        self.update_func.argtypes = [cdefs.newIMU,
                                     cdefs.newGPS]
        self.update_func.restype = cdefs.newNAV

    def python2c(self, imu, gps):
        cimu = cdefs.newIMU()
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
        
        cgps = cdefs.newGPS()
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

    def c2python(self, cnav):
        P = np.diag([cnav.Pp[0], cnav.Pp[1], cnav.Pp[2],
                     cnav.Pv[0], cnav.Pv[1], cnav.Pv[2],
                     cnav.Pa[0], cnav.Pa[1], cnav.Pa[2],
                     cnav.Pab[0], cnav.Pab[1], cnav.Pab[2],
                     cnav.Pgb[0], cnav.Pgb[1], cnav.Pgb[2]])
        stateInnov = np.nan*np.ones(6)
        insgps = pydefs.INSGPS(1, # fixme: valid/init
                               cnav.time,
                               [cnav.lat, cnav.lon, cnav.alt],
                               [cnav.vn, cnav.ve, cnav.vd],
                               [cnav.psi, cnav.the, cnav.phi],
                               cnav.ab,
                               cnav.gb,
                               P,
                               stateInnov)
        return insgps
    
    def init(self, imu, gps):
        cimu, cgps = self.python2c(imu, gps)
        cnav = self.init_func(cimu, cgps)
        return self.c2python(cnav)

    def update(self, imu, gps):
        cimu, cgps = self.python2c(imu, gps)
        cnav = self.update_func(cimu, cgps)
        return self.c2python(cnav)

    def close(self):
        pass
