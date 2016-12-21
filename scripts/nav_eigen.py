import numpy as np
import os
import sys

import nav.EKF15

class filter():
    def __init__(self):
        self.ekf = nav.EKF15.EKF15()
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
