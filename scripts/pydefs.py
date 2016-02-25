class IMU():
    def __init__(self, time, valid, p, q, r, ax, ay, az, hx, hy, hz, temp):
        self.time = time
        self.valid = valid
        self.p = p
        self.q = q
        self.r = r
        self.ax = ax
        self.ay = ay
        self.az = az
        self.hx = hx
        self.hy = hy
        self.hz = hz
        self.temp = temp

class GPS():
    def __init__(self, time, valid, tow, lat, lon, alt, vn, ve, vd):
        self.time = time
        self.valid = valid
        self.tow = tow
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.vn = vn
        self.ve = ve
        self.vd = vd

class INSGPS():
    def __init__(self, valid, time, estPOS, estVEL, estATT, estAB, estGB,
                 P, stateInnov):
        self.valid = valid
        self.time = time
        self.estPOS = estPOS[:]
        self.estVEL = estVEL[:]
        self.estATT = estATT[:]
        self.estAB = estAB[:]
        self.estGB = estGB[:]
        self.P = P
        self.stateInnov = stateInnov[:]

class FILTER():
    def __init__(self, time, lat, lon, alt, vn, ve, vd, phi, the, psi):
        self.time = time
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.vn = vn
        self.ve = ve
        self.vd = vd
        self.phi = phi
        self.the = the
        self.psi = psi
    
