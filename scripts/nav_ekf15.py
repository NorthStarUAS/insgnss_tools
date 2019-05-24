import numpy as np
import os
import sys

from navigation.structs import IMUdata, GPSdata
import navigation.filters

class filter():
    def __init__(self, gps_lag_sec=0.0, imu_dt=0.02):
        self.ekf = navigation.filters.EKF15()
        self.openloop = navigation.filters.OpenLoop()
        self.gps_lag_frames = int(round(gps_lag_sec / imu_dt))
        print("gps lag frame:", self.gps_lag_frames)
        self.imu_queue = []
        self.name = 'EKF15'

    def set_config(self, config):
        self.ekf.set_config(config)
        
    def init(self, imu, gps, filterpt=None):
        Cimu = IMUdata()
        Cimu.from_dict(imu)
        Cgps = GPSdata()
        Cgps.from_dict( gps )
        while len(self.imu_queue) < self.gps_lag_frames:
            self.imu_queue.append(Cimu)
        self.ekf.init(Cimu, Cgps)
        nav = self.ekf.get_nav()
        return nav.as_dict()

    def update(self, imu, gps, filterpt=None):
        Cimu = IMUdata()
        Cimu.from_dict(imu)

        # queue delay
        self.imu_queue.insert(0, Cimu)
        Cimu = self.imu_queue.pop()
        
        self.ekf.time_update(Cimu)
        if gps['newData']:
            Cgps = GPSdata()
            Cgps.from_dict( gps )
            self.ekf.measurement_update(Cgps)
        nav = self.ekf.get_nav()

        # forward propagate
        self.openloop.init_by_nav(nav)
        for imu in reversed(self.imu_queue):
            nav = self.openloop.update(imu)
            
        return nav.as_dict()

    def close(self):
        pass
