import numpy as np
import os
import sys

import nav.EKF15_mag_unified

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
        self.ekf = nav.EKF15_mag_unified.EKF15_mag_unified()
        self.name = 'EKF15_mag_unified'

    def set_config(self, config):
        self.ekf.set_config(config)
        
    def init(self, imu, gps, filterpt=None):
        Cimu = mkIMUdata( imu )
        Cgps = mkGPSdata( gps )
        nav = self.ekf.init(Cimu, Cgps)
        return nav

    def update(self, imu, gps, filterpt=None):
        Cimu = mkIMUdata( imu )
        Cgps = mkGPSdata( gps )
        nav = self.ekf.update(Cimu, Cgps)
        return nav

    def close(self):
        pass
