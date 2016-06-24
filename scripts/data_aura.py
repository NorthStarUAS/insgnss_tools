# load aura data format

import fileinput
import numpy as np
import math

import pydefs

d2r = math.pi / 180.0

def load(flight_dir):
    imu_data = []
    gps_data = []
    filter_data = []

    # load imu/gps data files
    imu_file = flight_dir + "/imu-0.txt"
    imucal_file = flight_dir + "/imucal.xml"
    gps_file = flight_dir + "/gps-0.txt"
    filter_file = flight_dir + "/filter-0.txt"
    imu_bias_file = flight_dir + "/imubias.txt"

    hx_coeffs = np.array([ 0.00269528, -0.21279657], dtype=np.float64)
    hy_coeffs = np.array([ 0.00263963,  0.01225428], dtype=np.float64)
    hz_coeffs = np.array([ 0.00300658, -0.11184107], dtype=np.float64)
    print hx_coeffs
    hx_func = np.poly1d(hx_coeffs)
    hy_func = np.poly1d(hy_coeffs)
    hz_func = np.poly1d(hz_coeffs)

    fimu = fileinput.input(imu_file)
    for line in fimu:
        time, p, q, r, ax, ay, az, hx, hy, hz, temp, status = line.split(',')
        # quick hack:
        hx_new = hx_func(float(hx))
        hy_new = hy_func(float(hy))
        hz_new = hz_func(float(hz))
        norm = np.linalg.norm([hx_new, hy_new, hz_new])
        hx_new /= norm
        hy_new /= norm
        hz_new /= norm
        imu = pydefs.IMU( float(time), int(status),
                          float(p), float(q), float(r),
                          float(ax), float(ay), float(az),
                          float(hx_new), float(hy_new), float(hz_new),
                          float(temp) )
        imu_data.append( imu )

    fgps = fileinput.input(gps_file)
    for line in fgps:
        # note the aura logs unix time of the gps record, not tow, but
        # for the pruposes of the insgns algorithm, it's only
        # important to have a properly incrementing clock, it doens't
        # really matter what the zero reference point of time is.
        time, lat, lon, alt, vn, ve, vd, unixsec, sats, status = line.split(',')
        if int(sats) >= 5:
            gps = pydefs.GPS( float(time), int(status), float(unixsec),
                              float(lat), float(lon), float(alt),
                              float(vn), float(ve), float(vd))
            gps_data.append(gps)

    # load filter records if they exist (for comparison purposes)
    ffilter = fileinput.input(filter_file)
    for line in ffilter:
        time, lat, lon, alt, vn, ve, vd, phi, the, psi, status = line.split(',')
        if abs(float(lat)) > 0.0001 and abs(float(lon)) > 0.0001:
            psi = float(psi)
            if psi > 180.0:
                psi = psi - 360.0
            if psi < -180.0:
                psi = psi - 360.0
            filter = pydefs.FILTER(float(time),
                                   float(lat)*d2r, float(lon)*d2r, float(alt),
                                   float(vn), float(ve), float(vd),
                                   float(phi)*d2r, float(the)*d2r,
                                   float(psi)*d2r)
            filter_data.append(filter)

    print "imu records:", len(imu_data)
    print "gps records:", len(gps_data)

    return imu_data, gps_data, filter_data
