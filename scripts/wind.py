# Estimate wind vector given indicated airspeed, aircraft heading
# (true), and gps ground velocity vector.  This function is designed
# to be called repeatedly to update the wind estimate in real time.

import math

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
        psi = math.pi * 0.5 - yaw_rad

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
