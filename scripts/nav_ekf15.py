import numpy as np
import os
import sys

import navigation.filters

def mkIMUdata( src ):
    result = navigation.structs.IMUdata()
    result.time = src.time
    result.p = src.p
    result.q = src.q
    result.r = src.r
    result.ax = src.ax
    result.ay = src.ay
    result.az = src.az
    result.hx = src.hx
    result.hy = src.hy
    result.hz = src.hz
    result.temp = src.temp
    return result

def mkGPSdata( src ):
    result = navigation.structs.GPSdata()
    result.time = src.time
    result.lat = src.lat
    result.lon = src.lon
    result.alt = src.alt
    result.vn = src.vn
    result.ve = src.ve
    result.vd = src.vd
    result.sats = src.sats
    result.newData = src.newData
    return result
    
class filter():
    
    def __init__(self):
        self.ekf = navigation.filters.EKF15()
        self.openloop = navigation.filters.OpenLoop()
        self.gps_lag_frames = 10
        self.imu_queue = []
        
        self.name = 'EKF15'

    def set_config(self, config):
        self.ekf.set_config(config)
        
    def init(self, imu, gps, filterpt=None):
        Cimu = mkIMUdata( imu )
        Cgps = mkGPSdata( gps )
        while len(self.imu_queue) < self.gps_lag_frames:
            self.imu_queue.append(Cimu)
        self.ekf.init(Cimu, Cgps)
        nav = self.ekf.get_nav()
        return nav

    def update(self, imu, gps, filterpt=None):
        Cimu = mkIMUdata( imu )

        # queue delay
        self.imu_queue.insert(0, Cimu)
        Cimu = self.imu_queue.pop()
        
        self.ekf.time_update(Cimu)
        if gps.newData:
            Cgps = mkGPSdata( gps )
            self.ekf.measurement_update(Cgps)
        nav = self.ekf.get_nav()

        # forward propagate
        self.openloop.init_by_nav(nav)
        for imu in reversed(self.imu_queue):
            nav = self.openloop.update(imu)
            
        return nav

    def close(self):
        pass
