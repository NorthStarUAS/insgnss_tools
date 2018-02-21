import numpy as np
import os
import sys

import nav.EKF15_sep

def mkIMUdata( src ):
    result = nav.structs.IMUdata()
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
    result = nav.structs.GPSdata()
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
        self.ekf = nav.EKF15_sep.EKF15_sep()
        self.name = 'EKF15_float_sep'

    def set_config(self, config):
        self.ekf.set_config(config)
        
    def init(self, imu, gps, filterpt=None):
        Cimu = mkIMUdata( imu )
        Cgps = mkGPSdata( gps )
        self.ekf.init(Cimu, Cgps)
        return self.ekf.get_nav()

    def update(self, imu, gps, filterpt=None):
        Cimu = mkIMUdata( imu )
        self.ekf.time_update(Cimu)
        if gps.newData:
            Cgps = mkGPSdata( gps )
            self.ekf.measurement_update(Cgps)
        return self.ekf.get_nav()

    def close(self):
        pass
