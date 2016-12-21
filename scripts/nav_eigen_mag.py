import numpy as np
import os
import sys

import nav.EKF15_mag

class filter():
    def __init__(self):
        self.ekf = nav.EKF15_mag.EKF15_mag()
        self.name = 'EKF15_mag'

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
