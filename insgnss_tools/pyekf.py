from math import pi

from insgnss_tools.structs import NAVdata

from .NavEKF15 import NavEKF15

d2r = pi / 180.0
r2d = 180.0 / pi

class pyEKF():
    def __init__(self):
        self.filter = NavEKF15()
        self.filter.Configure()
        self.first_frame = True
        self.nav = NAVdata()
        self.last_imu_time = 0
        self.last_gps_time = 0

    def set_config(self, config):
        pass

    def update(self, imu, gps):
        dt = 0
        if self.last_imu_time > 0:
            dt = imu.time - self.last_imu_time
        self.last_imu_time = imu.time

        if self.first_frame and imu.time > 0 and gps.time > 0:
            self.first_frame = False
            self.filter.Initialize([imu.p, imu.q, imu.r], [imu.ax, imu.ay, imu.az],
                                   [gps.lat*d2r, gps.lon*d2r, gps.alt], [gps.vn, gps.ve, gps.vd])

        measUpdate = False
        if gps.time > self.last_gps_time:
            # print("gps:", gps.as_dict())
            measUpdate = True
            self.last_gps_time = gps.time
        accel_corr, gyro_corr, vel, rpy, pos = \
            self.filter.Update(measUpdate,
                               [imu.p, imu.q, imu.r], [imu.ax, imu.ay, imu.az],
                               [gps.lat*d2r, gps.lon*d2r, gps.alt], [gps.vn, gps.ve, gps.vd],
                               dt)
        self.nav.time = imu.time
        self.nav.abx = self.filter.aBias_mps2[0]
        self.nav.aby = self.filter.aBias_mps2[1]
        self.nav.abz = self.filter.aBias_mps2[2]
        self.nav.gbx = self.filter.wBias_rps[0]
        self.nav.gby = self.filter.wBias_rps[1]
        self.nav.gbz = self.filter.wBias_rps[2]
        self.nav.vn = vel[0]
        self.nav.ve = vel[1]
        self.nav.vd = vel[2]
        self.nav.phi = rpy[0]
        self.nav.the = rpy[1]
        self.nav.psi = rpy[2]
        self.nav.lat = pos[0]
        self.nav.lon = pos[1]
        self.nav.alt = pos[2]

    def get_nav(self):
        # print("nav:", self.nav.as_dict())
        return self.nav