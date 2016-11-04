import numpy as np
import os
import sys

sys.path.append('../build/src/nav_eigen/.libs/')
import libnav_eigen

class filter():
    def __init__(self):
        self.ekf = libnav_eigen.EKF15()
        self.name = 'EKF15'

    def set_config(self, config):
        self.ekf.set_config(config)
        
    def init(self, imu, gps, filterpt=None):
        nav = self.ekf.init(imu, gps)
        return nav

    def update(self, imu, gps, filterpt=None):
        nav = self.ekf.update(imu, gps)
        return nav

    def close(self):
        pass
