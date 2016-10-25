import numpy as np
import os
import sys

sys.path.append('../build/src/nav_core/.libs/')
sys.path.append('../build/src/nav_openloop/.libs/')
import libnav_core
import libnav_openloop

class filter():
    def __init__(self):
        self.ekf = libnav_openloop.OpenLoop()
        self.name = 'Open Loop'
        self.filter_sync = 0.0

    def init(self, imu, gps, filt):
        self.ekf.init(filt.lat, filt.lon, filt.alt,
                      filt.vn, filt.ve, filt.vd,
                      filt.phi, filt.the, filt.psi)
        nav = self.ekf.update(imu)
        return nav

    def update(self, imu, gps, filt):
        if filt.time > self.filter_sync + 300:
            print 'resync:', filt.time
            self.filter_sync = filt.time
            nav = self.init(imu, gps, filt)
        else:
            nav = self.ekf.update(imu)
        return nav

    def close(self):
        pass
