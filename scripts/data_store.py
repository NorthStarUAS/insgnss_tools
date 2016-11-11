import math
import numpy as np

import sys
sys.path.append('../build/src/nav_core/.libs/')
import libnav_core

r2d = 180.0 / math.pi

# this class organizes the filter output in a way that is more
# convenient and direct for matplotlib
class data_store():
    def __init__(self):
        self.data = []
        
        self.time = []
        self.psi = []
        self.the = []
        self.phi = []
        self.lat = []
        self.lon = []
        self.alt = []
        self.vn = []
        self.ve = []
        self.vd = []

        self.ax_bias = []
        self.ay_bias = [] 
        self.az_bias = []
        self.p_bias = []
        self.q_bias = []
        self.r_bias = []

        self.Pp = []
        self.Pvel = []
        self.Patt = []
        self.Pab = []
        self.Pgb = []

    def append(self, insgps):
        self.data.append(insgps)
        
        self.time.append(insgps.time)
        
        self.psi.append(insgps.psi)
        self.the.append(insgps.the)
        self.phi.append(insgps.phi)
        self.lat.append(insgps.lat*r2d)
        self.lon.append(insgps.lon*r2d)
        self.alt.append(insgps.alt)
        self.vn.append(insgps.vn)
        self.ve.append(insgps.ve)
        self.vd.append(insgps.vd)

        self.ax_bias.append(insgps.abx)
        self.ay_bias.append(insgps.aby)
        self.az_bias.append(insgps.abz)
        self.p_bias.append(insgps.gbx)
        self.q_bias.append(insgps.gby)
        self.r_bias.append(insgps.gbz)
        
        self.Pp.append( np.array([insgps.Pp0, insgps.Pp1, insgps.Pp2]) )
        self.Pvel.append( np.array([insgps.Pv0, insgps.Pv1, insgps.Pv2]) )
        self.Patt.append( np.array([insgps.Pa0, insgps.Pa1, insgps.Pa2]) )
        self.Pab.append( np.array([insgps.Pabx, insgps.Paby, insgps.Pabz]) )
        self.Pgb.append( np.array([insgps.Pgbx, insgps.Pgby, insgps.Pgbz]) )

    def append_from_filter(self, filterpt):
        self.time.append(filterpt.time)
        self.psi.append(filterpt.psi)
        self.the.append(filterpt.the)
        self.phi.append(filterpt.phi)
        self.lat.append(filterpt.lat*r2d)
        self.lon.append(filterpt.lon*r2d)
        self.alt.append(filterpt.alt)
        self.vn.append(filterpt.vn)
        self.ve.append(filterpt.ve)
        self.vd.append(filterpt.vd)
        
    def append_from_gps(self, gpspt):
        self.time.append(gpspt.time)
        self.lat.append(gpspt.lat)
        self.lon.append(gpspt.lon)
        self.alt.append(gpspt.alt)
        self.vn.append(gpspt.vn)
        self.ve.append(gpspt.ve)
        self.vd.append(gpspt.vd)

    # return the index corresponding to the given time (or the next
    # index if there is no exact match
    def find_index(self, time):
        for k, t in enumerate(self.time):
            if t >= time:
                return k
        # every time in the set is earlier than the given time, so
        # return the index of the last entry.
        return len(self.time) - 1

# return a record filled in with half the difference between 
def diff_split(nav1, nav2):
    diff = libnav_core.NAVdata()
    
    diff.time = nav1.time
    print ' err t =', diff.time
    diff.psi = (nav1.psi - nav2.psi) * 0.5
    diff.the = (nav1.the - nav2.the) * 0.5
    diff.phi = (nav1.phi - nav2.phi) * 0.5
    print ' att:', diff.phi, diff.the, diff.psi
    diff.lat = (nav1.lat - nav2.lat) * 0.5
    diff.lon = (nav1.lon - nav2.lon) * 0.5
    diff.alt = (nav1.alt - nav2.alt) * 0.5
    print ' pos:', diff.lat, diff.lon, diff.alt
    diff.vn = (nav1.vn - nav2.vn) * 0.5
    diff.ve = (nav1.ve - nav2.ve) * 0.5
    diff.vd = (nav1.vd - nav2.vd) * 0.5
    print ' vel:', diff.vn, diff.ve, diff.vd

    diff.abx = (nav1.abx - nav2.abx) * 0.5
    diff.aby = (nav1.aby - nav2.aby) * 0.5
    diff.abz = (nav1.abz - nav2.abz) * 0.5
    print ' accel bias:', diff.abx, diff.aby, diff.abz
    diff.gbx = (nav1.gbx - nav2.gbx) * 0.5
    diff.gby = (nav1.gby - nav2.gby) * 0.5
    diff.gbz = (nav1.gbz - nav2.gbz) * 0.5
    print ' gyro bias:', diff.gbx, diff.gby, diff.gbz
        
    # [insgps.Pp0, insgps.Pp1, insgps.Pp2]
    # [insgps.Pv0, insgps.Pv1, insgps.Pv2]
    # [insgps.Pa0, insgps.Pa1, insgps.Pa2]
    # [insgps.Pabx, insgps.Paby, insgps.Pabz]
    # [insgps.Pgbx, insgps.Pgby, insgps.Pgbz]
    return diff

# return a weighted average of the two records
def weighted_avg(nav1, nav2, w):
    avg = libnav_core.NAVdata()

    a = w
    b = 1 - w
    
    avg.time = nav1.time
    print ' avg t =', avg.time
    avg.psi = a*nav1.psi + b*nav2.psi
    avg.the = a*nav1.the + b*nav2.the
    avg.phi = a*nav1.phi + b*nav2.phi
    print ' att:', avg.phi, avg.the, avg.psi
    avg.lat = a*nav1.lat + b*nav2.lat
    avg.lon = a*nav1.lon + b*nav2.lon
    avg.alt = a*nav1.alt + b*nav2.alt
    print ' pos:', avg.lat, avg.lon, avg.alt
    avg.vn = a*nav1.vn + b*nav2.vn
    avg.ve = a*nav1.ve + b*nav2.ve
    avg.vd = a*nav1.vd + b*nav2.vd
    print ' vel:', avg.vn, avg.ve, avg.vd

    avg.abx = a*nav1.abx + b*nav2.abx
    avg.aby = a*nav1.aby + b*nav2.aby
    avg.abz = a*nav1.abz + b*nav2.abz
    print ' accel bias:', avg.abx, avg.aby, avg.abz
    avg.gbx = a*nav1.gbx + b*nav2.gbx
    avg.gby = a*nav1.gby + b*nav2.gby
    avg.gbz = a*nav1.gbz + b*nav2.gbz
    print ' gyro bias:', avg.gbx, avg.gby, avg.gbz
    return avg
    
# return a weighted average of the two records
def sum(nav1, nav2):
    result = libnav_core.NAVdata()

    result.time = nav1.time
    print ' sum t =', result.time
    result.psi = nav1.psi + nav2.psi
    result.the = nav1.the + nav2.the
    result.phi = nav1.phi + nav2.phi
    print ' att:', result.phi, result.the, result.psi
    result.lat = nav1.lat + nav2.lat
    result.lon = nav1.lon + nav2.lon
    result.alt = nav1.alt + nav2.alt
    print ' pos:', result.lat, result.lon, result.alt
    result.vn = nav1.vn + nav2.vn
    result.ve = nav1.ve + nav2.ve
    result.vd = nav1.vd + nav2.vd
    print ' vel:', result.vn, result.ve, result.vd

    result.abx = nav1.abx + nav2.abx
    result.aby = nav1.aby + nav2.aby
    result.abz = nav1.abz + nav2.abz
    print ' accel bias:', result.abx, result.aby, result.abz
    result.gbx = nav1.gbx + nav2.gbx
    result.gby = nav1.gby + nav2.gby
    result.gbz = nav1.gbz + nav2.gbz
    print ' gyro bias:', result.gbx, result.gby, result.gbz
    return result
