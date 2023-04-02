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
the_list = []
thr_list = []
ele_list = []
q_list = []
asi_list = []
rbfi = None

min_airspeed = 15               # kts

def append(phi, the, thr, ele, q, airspeed):
    if airspeed >= min_airspeed:
        try:
            # phi_list.append(math.sqrt(math.cos(phi)))
            # phi_list.append(math.cos(phi))
            # phi_list.append(math.cos(abs(phi)))
            phi_list.append(abs(phi))
        except:
            phi_list.append(0.0)
            print('error:', phi, math.cos(phi))
        the_list.append(the)
        if thr > 1.1:
            # catch a umn3 glitch
            thr = 0
        thr_list.append(thr)
        ele_list.append(ele)
        q_list.append(q)
        asi_list.append(airspeed)

def build():
    if len(phi_list) == 0:
        return False
    
    bins = 16
    
    phi_array = np.array(phi_list)
    the_array = np.array(the_list)
    thr_array = np.array(thr_list)
    ele_array = np.array(ele_list)
    q_array = np.array(q_list)
    asi_array = np.array(asi_list)

    phi_min = phi_array.min()
    the_min = the_array.min()
    thr_min = thr_array.min()
    ele_min = ele_array.min()
    q_min = q_array.min()
    asi_min = asi_array.min()

    phi_max = phi_array.max()
    the_max = the_array.max()
    thr_max = thr_array.max()
    ele_max = ele_array.max()
    q_max = q_array.max()
    asi_max = asi_array.max()

    phi_range = phi_max - phi_min
    the_range = the_max - the_min
    thr_range = thr_max - thr_min
    ele_range = ele_max - ele_min
    q_range = q_max - q_min
    asi_range = asi_max - asi_min

    phi_d = phi_range / bins
    the_d = the_range / bins
    thr_d = thr_range / bins
    ele_d = ele_range / bins
    q_d = q_range / bins
    asi_d = asi_range / bins

    if abs(thr_d) < 0.0001 or abs(ele_d) < 0.0001:
        return False
    
    print('phi:', phi_range, phi_d)
    print('the:', the_range, the_d)
    print('thr:', thr_range, thr_d)
    print('ele:', ele_range, ele_d)
    print('q:', q_range, q_d)
    print('asi:', asi_range, asi_d)

    ba_array = np.zeros((bins, bins, bins, bins, bins))
    ba_count = np.zeros((bins, bins, bins, bins, bins))

    size = len(phi_array)
    for i in range(size):
        phi = phi_array[i]
        the = the_array[i]
        thr = thr_array[i]
        ele = ele_array[i]
        q = q_array[i]
        asi = asi_array[i]
        phi_idx = int((phi - phi_min) / phi_d)
        if phi_idx == bins: phi_idx = bins - 1
        the_idx = int((the - the_min) / the_d)
        if the_idx == bins: the_idx = bins - 1
        thr_idx = int((thr - thr_min) / thr_d)
        if thr_idx == bins: thr_idx = bins - 1
        ele_idx = int((ele - ele_min) / ele_d)
        if ele_idx == bins: ele_idx = bins - 1
        q_idx = int((q - q_min) / q_d)
        if q_idx == bins: q_idx = bins - 1
        asi_idx = int((asi - asi_min) / asi_d)
        if asi_idx == bins: asi_idx = bins - 1
        ba_array[phi_idx, the_idx, thr_idx, ele_idx, q_idx] += asi
        ba_count[phi_idx, the_idx, thr_idx, ele_idx, q_idx] += 1

    phi_short = []
    the_short = []
    thr_short = []
    ele_short = []
    q_short = []
    asi_short = []
    for i1 in range(bins):
        for i2 in range(bins):
            for i3 in range(bins):
                for i4 in range(bins):
                    for i5 in range(bins):
                        count = ba_count[i1, i2, i3, i4, i5]
                        if count > 0:
                            val = ba_array[i1, i2, i3, i4, i5] / count
                            phi_short.append(phi_min + phi_d * (i1 + 0.5))
                            the_short.append(the_min + the_d * (i2 + 0.5))
                            thr_short.append(thr_min + thr_d * (i3 + 0.5))
                            ele_short.append(ele_min + ele_d * (i4 + 0.5))
                            q_short.append(q_min + q_d * (i5 + 0.5))
                            asi_short.append(val)
    print('short len:', len(asi_short))

    global rbfi
    rbfi = Rbf(np.array(phi_short),
               np.array(the_short),
               np.array(thr_short),
               np.array(ele_short),
               np.array(q_short),
               np.array(asi_short),
               smooth=2.0)
    return True

def est_airspeed(phi, the, thr, ele, q):
    val = rbfi(abs(phi), the, thr, ele, q)
    return val
