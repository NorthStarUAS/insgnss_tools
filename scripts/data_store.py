import math

r2d = 180.0 / math.pi

# this class organizes the filter output in a way that is more
# convenient and direct for matplotlib
class data_store():
    def __init__(self):
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
