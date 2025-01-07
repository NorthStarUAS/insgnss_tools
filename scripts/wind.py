# Estimate wind vector given indicated airspeed, aircraft heading
# (true), and gps ground velocity vector.  This function is designed
# to be called repeatedly to update the wind estimate in real time.

import math
from tqdm import tqdm

from flightdata import flight_interp

import lowpass

# useful constants
d2r = math.pi / 180.0
r2d = 180.0 / math.pi
mps2kt = 1.94384
kt2mps = 1 / mps2kt

class Wind():
    def __init__(self):
        self.wind_time_factor = 60
        self.pitot_time_factor = 240
        self.last_time = 0.0

    def update(self, time, airspeed_mps, yaw_rad, vn, ve):
        dt = 0.0
        if self.last_time > 0:
            dt = time - self.last_time
        self.last_time = time

        if dt > 0.0 and airspeed_mps >= 8.0:
            # update values if "flying" and time has elapsed
            psi = 0.5*math.pi - yaw_rad

            # estimate body velocity
            ue = math.cos(psi) * (airspeed_mps * self.filt_ps.value)
            un = math.sin(psi) * (airspeed_mps * self.filt_ps.value)

            # instantaneous wind velocity
            we = ue - ve
            wn = un - vn

            # filtered wind velocity
            self.filt_wn.update(wn, dt)
            self.filt_we.update(we, dt)

            # estimate true airspeed
            true_e = self.filt_we.value + ve
            true_n = self.filt_wn.value + vn

            # estimate pitot tube bias
            ps = 1.0
            if airspeed_mps > 9.0:
                true_speed_mps = math.sqrt( true_e*true_e + true_n*true_n )
                ps = true_speed_mps / airspeed_mps
                # don't let the scale factor exceed some reasonable limits
                if ps < 0.75: ps = 0.75
                if ps > 1.25: ps = 1.25
            self.filt_ps.update(ps, dt)

        return self.filt_wn.value, self.filt_we.value, self.filt_ps.value

    # run a quick wind estimate and pitot calibration based on nav
    # estimate + air data
    def estimate(self, data, wind_time_factor):
        print("Estimating winds aloft:")
        if wind_time_factor:
            self.wind_time_factor = wind_time_factor
        self.filt_wn = lowpass.LowPassFilter(self.wind_time_factor, 1)
        self.filt_we = lowpass.LowPassFilter(self.wind_time_factor, -5.0)
        self.filt_ps = lowpass.LowPassFilter(self.pitot_time_factor, 1.0)
        self.last_time = 0.0
        winds = []
        airspeed_mps = 0
        psi = 0
        vn = 0
        ve = 0
        wind_deg = 0
        wind_kt = 0
        wn = 0
        we = 0
        ps = 1.0
        iter = flight_interp.IterateGroup(data)
        for i in tqdm(range(iter.size())):
            record = iter.next()
            if len(record):
                t = record["imu"]["time_sec"]
                if "airdata" in record:
                    airspeed_mps = record["airdata"]["airspeed_mps"]
                if "nav" in record:
                    psi_rad = record["nav"]["psi_deg"] * d2r
                    vn = record["nav"]["vn_mps"]
                    ve = record["nav"]["ve_mps"]
                if airspeed_mps > 8.0:
                    (wn, we, ps) = self.update(t, airspeed_mps, psi_rad, vn, ve)
                    #print wn, we, math.atan2(wn, we), math.atan2(wn, we)*r2d
                    wind_deg = 90 - math.atan2(wn, we) * r2d
                    if wind_deg < 0: wind_deg += 360.0
                    wind_kt = math.sqrt( we*we + wn*wn ) * mps2kt
                    #print wn, we, ps, wind_deg, wind_kt
                # make sure we log one record per each imu record
                winds.append( { "time_sec": t,
                                "wind_deg": wind_deg,
                                "wind_kt": wind_kt,
                                "pitot_scale": ps,
                                "wn_mps": wn,
                                "we_mps": we } )
        return winds
