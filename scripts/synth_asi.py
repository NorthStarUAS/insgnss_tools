# synthetic airspeed experiment

import math
import numpy as np
from scipy.interpolate import Rbf

# x, y, z, d = np.random.rand(4, 50)
# rbfi = Rbf(x, y, z, d)  # radial basis function interpolator instance
# xi = yi = zi = np.linspace(0, 1, 20)
# di = rbfi(xi, yi, zi)   # interpolated values
# di.shape
# (20,)

phi_list = []
thr_list = []
ele_list = []
q_list = []
asi_list = []
rbfi = None

min_airspeed = 15               # kts

def append(phi, thr, ele, q, airspeed):
    if airspeed >= min_airspeed:
        phi_list.append(math.sqrt(math.cos(phi)))
        thr_list.append(thr)
        ele_list.append(1.0/ele)
        q_list.append(q)
        asi_list.append(airspeed)

def build():
    bins = 40
    
    phi_array = np.array(phi_list)
    thr_array = np.array(thr_list)
    ele_array = np.array(ele_list)
    q_array = np.array(q_list)
    asi_array = np.array(asi_list)

    phi_min = phi_array.min()
    thr_min = thr_array.min()
    ele_min = ele_array.min()
    q_min = q_array.min()
    asi_min = asi_array.min()

    phi_max = phi_array.max()
    thr_max = thr_array.max()
    ele_max = ele_array.max()
    q_max = q_array.max()
    asi_max = asi_array.max()

    phi_range = phi_max - phi_min
    thr_range = thr_max - thr_min
    ele_range = ele_max - ele_min
    q_range = q_max - q_min
    asi_range = asi_max - asi_min

    phi_d = phi_range / bins
    thr_d = thr_range / bins
    ele_d = ele_range / bins
    q_d = q_range / bins
    asi_d = asi_range / bins

    print 'phi:', phi_range, phi_d
    print 'thr:', thr_range, thr_d
    print 'ele:', ele_range, ele_d
    print 'q:', q_range, q_d
    print 'asi:', asi_range, asi_d

    ba_array = np.zeros((bins, bins, bins, bins))
    ba_count = np.zeros((bins, bins, bins, bins))

    size = len(phi_array)
    for i in range(size):
        phi = phi_array[i]
        thr = thr_array[i]
        ele = ele_array[i]
        q = q_array[i]
        asi = asi_array[i]
        phi_idx = int((phi - phi_min) / phi_d)
        if phi_idx == bins: phi_idx = bins - 1
        thr_idx = int((thr - thr_min) / thr_d)
        if thr_idx == bins: thr_idx = bins - 1
        ele_idx = int((ele - ele_min) / ele_d)
        if ele_idx == bins: ele_idx = bins - 1
        q_idx = int((q - q_min) / q_d)
        if q_idx == bins: q_idx = bins - 1
        asi_idx = int((asi - asi_min) / asi_d)
        if asi_idx == bins: asi_idx = bins - 1
        ba_array[phi_idx, thr_idx, ele_idx, q_idx] += asi
        ba_count[phi_idx, thr_idx, ele_idx, q_idx] += 1

    phi_short = []
    thr_short = []
    ele_short = []
    q_short = []
    asi_short = []
    for i1 in range(bins):
        for i2 in range(bins):
            for i3 in range(bins):
                for i4 in range(bins):
                    count = ba_count[i1, i2, i3, i4]
                    if count > 0:
                        val = ba_array[i1, i2, i3, i4] / count
                        phi_short.append(phi_min + phi_d * (i1 + 0.5))
                        thr_short.append(thr_min + thr_d * (i2 + 0.5))
                        ele_short.append(ele_min + ele_d * (i3 + 0.5))
                        q_short.append(q_min + q_d * (i4 + 0.5))
                        asi_short.append(val)
    print 'short len:', len(asi_short)

    global rbfi
    rbfi = Rbf(np.array(phi_short),
               np.array(thr_short),
               np.array(ele_short),
               np.array(q_short),
               np.array(asi_short),
               smooth=1.0)

def est_airspeed(phi, thr, ele, q):
    return rbfi(math.sqrt(math.cos(phi)), thr, 1.0/ele, q)
