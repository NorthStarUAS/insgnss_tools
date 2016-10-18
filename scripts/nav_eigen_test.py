import numpy as np
import os
import sys

# Import the structures
import cdefs
import pydefs

# Import these ctypes for proper declaration of cdefs.py structures
import ctypes
# Abbreviate these ctypes commands
POINTER = ctypes.POINTER
byref   = ctypes.byref

sys.path.append('../build/src/nav_eigen_test/.libs/')
import libnav_eigen_test

class filter():
    def __init__(self):
        # Load compilied `.so` file.
        # sharedobj = ctypes.CDLL(os.path.abspath('../build/src/nav_eigen_test/.libs/libnav_eigen_test.so'))

        # self.init_func = sharedobj._Z8init_nav7IMUdata7GPSdata
        # self.update_func = sharedobj._Z7get_nav7IMUdata7GPSdata
        
        self.ekf = libnav_eigen_test.EKF15()

        # Declare inputs to the init_nav function
        # self.init_func.argtypes = [cdefs.newIMU,
        #                            cdefs.newGPS]
        # self.init_func.restype = cdefs.newNAV
        
        # # Declare inputs to the get_nav function
        # self.update_func.argtypes = [cdefs.newIMU,
        #                              cdefs.newGPS]
        # self.update_func.restype = cdefs.newNAV

    def python2c(self, imu, gps):
        cimu = libnav_eigen_test.IMUdata()
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
        
        cgps = libnav_eigen_test.GPSdata()
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
        P = np.diag([cnav.Pp0, cnav.Pp1, cnav.Pp2,
                     cnav.Pv0, cnav.Pv1, cnav.Pv2,
                     cnav.Pa0, cnav.Pa1, cnav.Pa2,
                     cnav.Pabx, cnav.Paby, cnav.Pabz,
                     cnav.Pgbx, cnav.Pgby, cnav.Pgbz])
        stateInnov = np.nan*np.ones(6)
        insgps = pydefs.INSGPS(1, # fixme: valid/init
                               cnav.time,
                               [cnav.lat, cnav.lon, cnav.alt],
                               [cnav.vn, cnav.ve, cnav.vd],
                               [cnav.psi, cnav.the, cnav.phi],
                               [cnav.abx, cnav.aby, cnav.abz],
                               [cnav.gbx, cnav.gby, cnav.gbz],
                               P,
                               stateInnov)
        return insgps
    
    def init(self, imu, gps):
        cimu, cgps = self.python2c(imu, gps)
        cnav = self.ekf.init(cimu, cgps)
        return self.c2python(cnav)

    def update(self, imu, gps):
        cimu, cgps = self.python2c(imu, gps)
        cnav = self.ekf.update(cimu, cgps)
        return self.c2python(cnav)

    def close(self):
        pass
