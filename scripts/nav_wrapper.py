# nav_wrapper.py
# 1. simplify calling the C++ ekf filters
# 2. add gps lag support

from aurauas_navigation.structs import IMUdata, GPSdata, NAVconfig
from aurauas_navigation.filters import EKF15, EKF15_mag, OpenLoop

class filter():
    def __init__(self, nav='EKF15', gps_lag_sec=0.0, imu_dt=0.02):
        if nav == 'EKF15':
            self.filter = EKF15()
            self.name = 'EKF15'
        elif nav == 'EKF15_mag':
            self.filter = EKF15_mag()
            self.name = 'EKF15_mag'
        else:
            print("Unknown nav filter specified aborting:", nav)
            quit()

        self.openloop = OpenLoop()
        self.gps_lag_frames = int(round(gps_lag_sec / imu_dt))
        print("gps lag frame:", self.gps_lag_frames)
        self.imu_queue = []

    def set_config(self, config):
        Cconfig = NAVconfig()
        Cconfig.from_dict(config)
        self.filter.set_config(Cconfig)
        
    def init(self, imu, gps):
        Cimu = IMUdata()
        Cimu.from_dict(imu)
        Cgps = GPSdata()
        Cgps.from_dict( gps )
        while len(self.imu_queue) < self.gps_lag_frames:
            self.imu_queue.append(Cimu)
        self.filter.init(Cimu, Cgps)
        nav = self.filter.get_nav()
        return nav.as_dict()

    def update(self, imu, gps):
        Cimu = IMUdata()
        Cimu.from_dict(imu)

        # queue delay
        self.imu_queue.insert(0, Cimu)
        Cimu = self.imu_queue.pop()
        
        self.filter.time_update(Cimu)
        if 'time' in gps:
            Cgps = GPSdata()
            Cgps.from_dict( gps )
            self.filter.measurement_update(Cimu, Cgps)
        nav = self.filter.get_nav()

        if len(self.imu_queue):
            # forward propagate from the lagged solution to new
            self.openloop.init_by_nav(nav)
            for imu in reversed(self.imu_queue):
                nav = self.openloop.update(imu)
            
        return nav.as_dict()

    def close(self):
        pass
