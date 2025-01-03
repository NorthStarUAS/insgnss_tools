from math import pi

from insgnss_tools.nav_structs import NAVdata

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
            dt = imu.time_sec - self.last_imu_time
        self.last_imu_time = imu.time_sec

        if self.first_frame and imu.time_sec > 0 and gps.time_sec > 0:
            self.first_frame = False
            self.filter.Initialize([imu.p, imu.q, imu.r], [imu.ax, imu.ay, imu.az],
                                   [gps.lat*d2r, gps.lon*d2r, gps.alt], [gps.vn, gps.ve, gps.vd])

        measUpdate = False
        if gps.time_sec > self.last_gps_time:
            # print("gps:", gps.as_dict())
            measUpdate = True
            self.last_gps_time = gps.time_sec
        accel_corr, gyro_corr, vel, rpy, pos = \
            self.filter.Update(measUpdate,
                               [imu.p, imu.q, imu.r], [imu.ax, imu.ay, imu.az],
                               [gps.lat*d2r, gps.lon*d2r, gps.alt], [gps.vn, gps.ve, gps.vd],
                               dt)
        self.nav.time_sec = imu.time_sec
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

        self.nav.Pp0 = self.filter.P[0,0]
        self.nav.Pp1 = self.filter.P[1,1]
        self.nav.Pp2 = self.filter.P[2,2]
        self.nav.Pv0 = self.filter.P[3,3]
        self.nav.Pv1 = self.filter.P[4,4]
        self.nav.Pv2 = self.filter.P[5,5]
        self.nav.Pa0 = self.filter.P[6,6]
        self.nav.Pa1 = self.filter.P[7,7]
        self.nav.Pa2 = self.filter.P[8,8]
        self.nav.Pabx = self.filter.P[9,9]
        self.nav.Paby = self.filter.P[10,10]
        self.nav.Pabz = self.filter.P[11,11]
        self.nav.Pgbx = self.filter.P[12,12]
        self.nav.Pgby = self.filter.P[13,13]
        self.nav.Pgbz = self.filter.P[14,14]

    def get_nav(self):
        # print("nav:", self.nav.as_dict())
        return self.nav