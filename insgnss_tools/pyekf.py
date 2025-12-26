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
            self.filter.Initialize([imu.p_rps, imu.q_rps, imu.r_rps], [imu.ax_mps2, imu.ay_mps2, imu.az_mps2],
                                   [gps.latitude_deg*d2r, gps.longitude_deg*d2r, gps.altitude_m], [gps.vn_mps, gps.ve_mps, gps.vd_mps])

        measUpdate = False
        if gps.time_sec > self.last_gps_time:
            # print("gps:", gps.as_dict())
            measUpdate = True
            self.last_gps_time = gps.time_sec
        accel_corr, gyro_corr, vel, rpy, pos = \
            self.filter.Update(measUpdate,
                               [imu.p_rps, imu.q_rps, imu.r_rps], [imu.ax_mps2, imu.ay_mps2, imu.az_mps2],
                               [gps.latitude_deg*d2r, gps.longitude_deg*d2r, gps.altitude_m], [gps.vn_mps, gps.ve_mps, gps.vd_mps],
                               dt)
        self.nav.time_sec = imu.time_sec
        self.nav.abx = self.filter.aBias_mps2[0]
        self.nav.aby = self.filter.aBias_mps2[1]
        self.nav.abz = self.filter.aBias_mps2[2]
        self.nav.gbx = self.filter.wBias_rps[0]
        self.nav.gby = self.filter.wBias_rps[1]
        self.nav.gbz = self.filter.wBias_rps[2]
        self.nav.vn_mps = vel[0]
        self.nav.ve_mps = vel[1]
        self.nav.vd_mps = vel[2]
        self.nav.phi_deg = rpy[0]*r2d
        self.nav.theta_deg = rpy[1]*r2d
        self.nav.psi_deg = rpy[2]*r2d
        self.nav.latitude_deg = pos[0]*r2d
        self.nav.longitude_deg = pos[1]*r2d
        self.nav.altitude_m = pos[2]

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