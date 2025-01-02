# nav_wrapper.py
# 1. simplify calling the C++ ekf filters
# 2. optional gps lag support (external to the filter)

from .nav_structs import IMUdata, GPSdata, NAVconfig
from .ekf15 import EKF15
from .ekf15_mag import EKF15_mag
from .uNavINS import uNavINS
from .uNavINS_BFS import uNavINS_BFS
from .openloop import OpenLoop
from .pyekf import pyEKF


class filter():
    def __init__(self, nav='EKF15', gps_lag_sec=0.0, imu_dt=0.02):
        self.name = nav
        if nav == 'EKF15':
            self.filter = EKF15()
        elif nav == 'EKF15_mag':
            self.filter = EKF15_mag()
        elif nav == 'uNavINS':
            self.filter = uNavINS()
        elif nav == 'uNavINS_BFS':
            self.filter = uNavINS_BFS()
        elif nav == "pyNavEKF15":
            self.filter = pyEKF()
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

    def update(self, imu, gps):
        Cimu = IMUdata()
        Cimu.from_dict(imu)

        # queue delay
        self.imu_queue.insert(0, Cimu)
        Cimu = self.imu_queue.pop()
        self.Cgps = GPSdata()
        self.Cgps.from_dict( gps )

        self.filter.update(Cimu, self.Cgps)
        nav = self.filter.get_nav()

        if len(self.imu_queue):
            # forward propagate from the lagged solution to new
            self.openloop.init_by_nav(nav)
            for imu in reversed(self.imu_queue):
                nav = self.openloop.update(imu)

        return nav.as_dict()

    def close(self):
        pass
