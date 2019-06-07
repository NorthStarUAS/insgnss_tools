# Estimate wind vector given indicated airspeed, aircraft heading
# (true), and gps ground velocity vector.  This function is designed
# to be called repeatedly to update the wind estimate in real time.

import math
from tqdm import tqdm

from aurauas_flightdata import flight_interp

import lowpass

# useful constants
d2r = math.pi / 180.0
r2d = 180.0 / math.pi
mps2kt = 1.94384
kt2mps = 1 / mps2kt

time_factor = 60
filt_wn = lowpass.LowPassFilter(time_factor, 0.0)
filt_we = lowpass.LowPassFilter(time_factor, 0.0)
filt_ps = lowpass.LowPassFilter(time_factor*4.0, 1.0)
last_time = 0.0

def update(time, airspeed_kt, yaw_rad, vn, ve):
    global last_time
    global filt_wn
    global filt_we
    global filt_ps
    
    dt = 0.0
    if last_time > 0:
        dt = time - last_time
    last_time = time

    if dt > 0.0 and airspeed_kt >= 10.0:
        # update values if 'flying' and time has elapsed
        psi = 0.5*math.pi - yaw_rad

        # estimate body velocity
        ue = math.cos(psi) * (airspeed_kt * filt_ps.value * kt2mps)
        un = math.sin(psi) * (airspeed_kt * filt_ps.value * kt2mps)

        # instantaneous wind velocity
        we = ue - ve
        wn = un - vn

        # filtered wind velocity
        filt_wn.update(wn, dt)
        filt_we.update(we, dt)

        # estimate true airspeed
        true_e = filt_we.value + ve
        true_n = filt_wn.value + vn

        # estimate pitot tube bias
        ps = 1.0
        if airspeed_kt > 1.0:
            true_speed_kt = math.sqrt( true_e*true_e + true_n*true_n ) * mps2kt
            ps = true_speed_kt / airspeed_kt
            # don't let the scale factor exceed some reasonable limits
            if ps < 0.75: ps = 0.75
            if ps > 1.25: ps = 1.25
        filt_ps.update(ps, dt)

    return filt_wn.value, filt_we.value, filt_ps.value

# run a quick wind estimate and pitot calibration based on nav
# estimate + air data
def estimate(data):
    print("Estimating winds aloft:")
    winds = []
    airspeed = 0
    psi = 0
    vn = 0
    ve = 0
    wind_deg = 0
    wind_kt = 0
    ps = 1.0
    iter = flight_interp.IterateGroup(data)
    for i in tqdm(range(iter.size())):
        record = iter.next()
        if len(record):
            t = record['imu']['time']
            if 'air' in record:
                airspeed = record['air']['airspeed']
            if 'filter' in record:
                psi = record['filter']['psi']
                vn = record['filter']['vn']
                ve = record['filter']['ve']
            if airspeed > 10.0:
                (wn, we, ps) = update(t, airspeed, psi, vn, ve)
                #print wn, we, math.atan2(wn, we), math.atan2(wn, we)*r2d
                wind_deg = 90 - math.atan2(wn, we) * r2d
                if wind_deg < 0: wind_deg += 360.0
                wind_kt = math.sqrt( we*we + wn*wn ) * mps2kt
                #print wn, we, ps, wind_deg, wind_kt
            # make sure we log one record per each imu record
            winds.append( { 'time': t,
                            'wind_deg': wind_deg,
                            'wind_kt': wind_kt,
                            'pitot_scale': ps } )
    return winds
