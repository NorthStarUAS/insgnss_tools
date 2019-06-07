# Estimate magnetometer vector length vs. throttle position looking for
# interference

import math
import numpy as np
from tqdm import tqdm

from aurauas_flightdata import flight_interp

import lowpass

def estimate(data):
    print("Estimating magnetometer/throttle correlation:")
    result = []
    throttle = 0.0
    iter = flight_interp.IterateGroup(data)
    for i in tqdm(range(iter.size())):
        record = iter.next()
        if len(record):
            t = record['imu']['time']
            mag_norm = np.linalg.norm(np.array([record['imu']['hx'],
                                                record['imu']['hy'],
                                                record['imu']['hz']]))
            if 'act' in record:
                throttle = record['act']['throttle']
            result.append( { 'time': t,
                             'throttle': throttle,
                             'mag_norm': mag_norm } )
    return result
