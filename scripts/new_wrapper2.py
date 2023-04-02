# nav_wrapper2.py
# 1. simplify calling the C++ ekf filters
# 2. optional gps lag support

import math
import numpy as np

from aurauas_navigation.structs import IMUdata, GPSdata, NAVconfig, GNSS_measurement
#from aurauas_navigation.ekf15 import EKF15
# from aurauas_navigation.ekf15_mag import EKF15_mag
#from aurauas_navigation.uNavINS import uNavINS
#from aurauas_navigation.uNavINS_BFS import uNavINS_BFS
from aurauas_navigation.openloop import OpenLoop
from aurauas_navigation.ekf17 import EKF17

def EphemerisData2PosVelClock(time, ephem_data):
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~# 
    # Author: Kerry Sun 
    # Input:
    # curent time [s]
    # Entire emphemeris data set 
    # Output: 
    # Function process entire emphemeris data set and produce position and velocity in ECEF
    # Ref: All the equations are based ON: https://www.gps.gov/technical/icwg/IS-GPS-200H.pdf
    # pg. 104-105, Also Grove  p335-338

    # Constant 
    MU = 3.986005e14
    OMEGA_DOT_EARTH = 7.2921151467e-5
    # Process subframe 1,2,3 information
    A_semiMajorAxis = pow(ephem_data['sqrtA'], 2)  # Semi-major axis
    n_0_computedMeanMotion = math.sqrt(MU / pow(A_semiMajorAxis, 3)) # Computed mean motion
    n_correctedMeanMotion = n_0_computedMeanMotion + ephem_data['deltan'] # Corrected mean motion
    e_eccentricity = ephem_data['e'] # Eccentricity
    #double phi_k_argumentOfLattitude;   # Argument of latitude
    M_0_trueAnomalyAtRef = ephem_data['M0']
    omega0_longitudeofAscendingNodeofOrbitPlane = ephem_data['Omega0']
    omega_argumentOfPerigee = ephem_data['omega']
    omegaDot_argumentOfPerigee = ephem_data['Omegad']
    i_0_inclinationAtRef = ephem_data['i0']
    iDot_rateOfInclination = ephem_data['IDOT']
    t_OE = ephem_data['toe']
    # 2nd harmonic terms
    C_us = ephem_data['Cus']
    C_uc = ephem_data['Cuc']
    C_rs = ephem_data['Crs']
    C_rc = ephem_data['Crc']
    C_is = ephem_data['Cis']
    C_ic = ephem_data['Cic']

    # Compute the time from the ephemeris reference epoch
    t_k_timeFromReferenceEpoch = time - t_OE
    print("t_k_timeFromReferenceEpoch:", t_k_timeFromReferenceEpoch)
    # Correct that time for end-of-week crossovers
    if t_k_timeFromReferenceEpoch > 302400:
        t_k_timeFromReferenceEpoch -= 604800
    if t_k_timeFromReferenceEpoch < -302400:
        t_k_timeFromReferenceEpoch += 604800

    # Compute the mean anomaly
    M_k_meanAnomaly = M_0_trueAnomalyAtRef + n_correctedMeanMotion * t_k_timeFromReferenceEpoch

    # Below, we iteratively solve for E_k_eccentricAnomaly using Newton-Raphson method
    solutionError = 1000000
    E_k_eccentricAnomaly = 1
    currentDerivative = 0
    iterationCount = 0

    solutionError = (E_k_eccentricAnomaly -
                     (e_eccentricity * math.sin(E_k_eccentricAnomaly)) -
                     M_k_meanAnomaly)

    while (abs(solutionError) > 1.0e-6) and iterationCount < 1000:
        currentDerivative = (1.0 - (e_eccentricity * math.cos(E_k_eccentricAnomaly)))
        E_k_eccentricAnomaly = E_k_eccentricAnomaly - solutionError / currentDerivative
        solutionError = (E_k_eccentricAnomaly -
                         (e_eccentricity * math.sin(E_k_eccentricAnomaly)) -
                         M_k_meanAnomaly)
        iterationCount += 1
    
    cos_E_k = math.cos(E_k_eccentricAnomaly)
    sin_E_k = math.sin(E_k_eccentricAnomaly)
    nu_k_trueAnomaly = math.atan2(
        (math.sqrt(1.0 - pow(e_eccentricity, 2)) * sin_E_k) /
            (1.0 - (e_eccentricity * cos_E_k)),
        (cos_E_k - e_eccentricity) /
            (1.0 - e_eccentricity * cos_E_k))

    phi_k_argumentOfLatitude = nu_k_trueAnomaly + omega_argumentOfPerigee

    # Compute the corrective 2nd order terms
    sin2PhiK = math.sin(2.0 * phi_k_argumentOfLatitude)
    cos2PhiK = math.cos(2.0 * phi_k_argumentOfLatitude)

    deltaU_argumentOfLatCorrection = (C_us * sin2PhiK) + (C_uc * cos2PhiK)
    deltaR_radiusCorrection = (C_rs * sin2PhiK) + (C_rc * cos2PhiK)
    deltaI_inclinationCorrection = (C_is * sin2PhiK) + (C_ic * cos2PhiK)

    # Now compute the updated corrected orbital elements
    u_argumentOfLat = phi_k_argumentOfLatitude + deltaU_argumentOfLatCorrection
    r_radius = (A_semiMajorAxis * (1 - (e_eccentricity * cos_E_k))) + deltaR_radiusCorrection
    i_inclination = i_0_inclinationAtRef + (iDot_rateOfInclination * t_k_timeFromReferenceEpoch) + deltaI_inclinationCorrection

    # Compute the satellite position within the orbital plane
    xPositionOrbitalPlane = r_radius * math.cos(u_argumentOfLat)
    yPositionOrbitalPlane = r_radius * math.sin(u_argumentOfLat)
    omegaK_longitudeAscendingNode = omega0_longitudeofAscendingNodeofOrbitPlane + ((omegaDot_argumentOfPerigee - OMEGA_DOT_EARTH) * t_k_timeFromReferenceEpoch) - (OMEGA_DOT_EARTH * t_OE)

    sinOmegaK = math.sin(omegaK_longitudeAscendingNode)
    cosOmegaK = math.cos(omegaK_longitudeAscendingNode)

    sinIK = math.sin(i_inclination)
    cosIK = math.cos(i_inclination)
    # Earth-fixed coordinates:
    x = (xPositionOrbitalPlane * cosOmegaK) - (yPositionOrbitalPlane * cosIK * sinOmegaK)
    y = (xPositionOrbitalPlane * sinOmegaK) + (yPositionOrbitalPlane * cosIK * cosOmegaK)
    z = (yPositionOrbitalPlane * sinIK)

    # ECEF velocity calculation:
    E_dot_k_eccentricAnomaly = n_correctedMeanMotion / (1.0 - (e_eccentricity * cos_E_k))           # Eq.(8.21)
    phi_dot_k_argumentOfLatitude = math.sin(nu_k_trueAnomaly) / sin_E_k * E_dot_k_eccentricAnomaly       # Eq.(8.22)
    r_dot_o_os = (A_semiMajorAxis * e_eccentricity * sin_E_k) * E_dot_k_eccentricAnomaly + 2 * ((C_rs * cos2PhiK) - (C_rc * sin2PhiK)) * phi_dot_k_argumentOfLatitude # Eq.(8.23a)
    u_dot_o_os = (1 + 2 * C_us * cos2PhiK - 2 * C_uc * sin2PhiK) * phi_dot_k_argumentOfLatitude    # Eq.(8.23b)

    x_dot_o_os = r_dot_o_os * math.cos(u_argumentOfLat) - r_radius * u_dot_o_os * math.sin(u_argumentOfLat)  # Eq.(8.24a)
    y_dot_o_os = r_dot_o_os * math.sin(u_argumentOfLat) + r_radius * u_dot_o_os * math.cos(u_argumentOfLat)  # Eq.(8.24b)

    omega_dot_K_longitudeAscendingNode = omegaDot_argumentOfPerigee - OMEGA_DOT_EARTH              # Eq. (8.25)
    i_dot_inclination = iDot_rateOfInclination + 2 * ((C_is * cos2PhiK) - (C_ic * sin2PhiK)) * phi_dot_k_argumentOfLatitude # Eq. (8.26)

    # Eq. (8.27)
    vx = (x_dot_o_os * cosOmegaK - y_dot_o_os * cosIK * sinOmegaK + i_dot_inclination * yPositionOrbitalPlane * sinIK * sinOmegaK) - omega_dot_K_longitudeAscendingNode * (xPositionOrbitalPlane * sinOmegaK + yPositionOrbitalPlane * cosIK * cosOmegaK)
    vy = (x_dot_o_os * sinOmegaK + y_dot_o_os * cosIK * cosOmegaK - i_dot_inclination * yPositionOrbitalPlane * sinIK * cosOmegaK) - omega_dot_K_longitudeAscendingNode * (-xPositionOrbitalPlane * cosOmegaK + yPositionOrbitalPlane * cosIK * sinOmegaK)
    vz = (y_dot_o_os * sinIK + i_dot_inclination * yPositionOrbitalPlane * cosIK)
     
    #lambda1 = 2*c / (1575.4282e6)  # L1 according ublox8
    #PseudorangeRate = lambda1 * gnss_raw_measurement.doppler
    #gnss_raw_measurement.pseudorange 
    return [x, y, z, vx, vy, vz]

# Physical parameters of the Earth
EARTH_GM = 3.986005e14  # m^3/s^2 (gravitational constant * mass of earth)
EARTH_RADIUS = 6.3781e6  # m
EARTH_ROTATION_RATE = 7.2921151467e-005  # rad/s (WGS84 earth rotation rate)

def get_sat_info(time, eph):
    tdiff = time - eph['toc']  # Time of clock
    clock_err = eph['af0'] + tdiff * (eph['af1'] + tdiff * eph['af2'])
    clock_rate_err = eph['af1'] + 2 * tdiff * eph['af2']

    # Orbit propagation
    tdiff = time - eph['toe']  # Time of ephemeris (might be different from time of clock)

    # Calculate position per IS-GPS-200D p 97 Table 20-IV
    a = eph['sqrtA'] * eph['sqrtA']  # [m] Semi-major axis
    ma_dot = math.sqrt(EARTH_GM / (a * a * a)) + eph['deltan']  # [rad/sec] Corrected mean motion
    ma = eph['M0'] + ma_dot * tdiff  # [rad] Corrected mean anomaly

    # Iteratively solve for the Eccentric Anomaly (from Keith Alter and David Johnston)
    ea = ma  # Starting value for E

    ea_old = 2222
    while math.fabs(ea - ea_old) > 1.0E-14:
      ea_old = ea
      tempd1 = 1.0 - eph['e'] * math.cos(ea_old)
      ea = ea + (ma - ea_old + eph['e'] * math.sin(ea_old)) / tempd1
    ea_dot = ma_dot / tempd1

    # Relativistic correction term
    einstein = -4.442807633E-10 * eph['e'] * eph['sqrtA'] * math.sin(ea)

    # Begin calc for True Anomaly and Argument of Latitude
    tempd2 = math.sqrt(1.0 - eph['e'] * eph['e'])
    # [rad] Argument of Latitude = True Anomaly + Argument of Perigee
    al = math.atan2(tempd2 * math.sin(ea), math.cos(ea) - eph['e']) + eph['omega']
    al_dot = tempd2 * ea_dot / tempd1

    # Calculate corrected argument of latitude based on position
    cal = al + eph['Cus'] * math.sin(2.0 * al) + eph['Cuc'] * math.cos(2.0 * al)
    cal_dot = al_dot * (1.0 + 2.0 * (eph['Cus'] * math.cos(2.0 * al) -
                                     eph['Cuc'] * math.sin(2.0 * al)))

    # Calculate corrected radius based on argument of latitude
    r = a * tempd1 + eph['Crc'] * math.cos(2.0 * al) + eph['Crs'] * math.sin(2.0 * al)
    r_dot = (a * eph['e'] * math.sin(ea) * ea_dot +
             2.0 * al_dot * (eph['Crs'] * math.cos(2.0 * al) -
                             eph['Crc'] * math.sin(2.0 * al)))

    # Calculate inclination based on argument of latitude
    inc = (eph['i0'] + eph['IDOT'] * tdiff +
           eph['Cic'] * math.cos(2.0 * al) +
           eph['Cis'] * math.sin(2.0 * al))
    print("inc:", inc, inc*180/math.pi)
    inc_dot = (eph['IDOT'] +
               2.0 * al_dot * (eph['Cis'] * math.cos(2.0 * al) -
                               eph['Cic'] * math.sin(2.0 * al)))

    # Calculate position and velocity in orbital plane
    x = r * math.cos(cal)
    y = r * math.sin(cal)
    x_dot = r_dot * math.cos(cal) - y * cal_dot
    y_dot = r_dot * math.sin(cal) + x * cal_dot

    # Corrected longitude of ascending node
    om_dot = eph['Omegad'] - EARTH_ROTATION_RATE
    print("om_dot:", om_dot)
    #om = eph['Omega0'] + tdiff * om_dot - EARTH_ROTATION_RATE * eph['toe'].tow
    om = eph['Omega0'] + tdiff * om_dot - EARTH_ROTATION_RATE * eph['toe']
    print("lon asc node:", om)

    # Compute the satellite's position in Earth-Centered Earth-Fixed coordiates
    pos = np.empty(3)
    pos[0] = x * math.cos(om) - y * math.cos(inc) * math.sin(om)
    pos[1] = x * math.sin(om) + y * math.cos(inc) * math.cos(om)
    pos[2] = y * math.sin(inc)

    tempd3 = y_dot * math.cos(inc) - y * math.sin(inc) * inc_dot

    # Compute the satellite's velocity in Earth-Centered Earth-Fixed coordiates
    vel = np.empty(3)
    vel[0] = -om_dot * pos[1] + x_dot * math.cos(om) - tempd3 * math.sin(om)
    vel[1] = om_dot * pos[0] + x_dot * math.sin(om) + tempd3 * math.cos(om)
    vel[2] = y * math.cos(inc) * inc_dot + y_dot * math.sin(inc)

    clock_err += einstein

    return pos, vel, clock_err, clock_rate_err

class filter():
    def __init__(self, nav, gps_lag_sec=0.0, imu_dt=0.02):
        self.name = nav
        if nav == 'EKF15':
            self.filter = EKF15()
        elif nav == 'EKF15_mag':
            self.filter = EKF15_mag()
        elif nav == 'EKF17':
            self.filter = EKF17()
        elif nav == 'uNavINS':
            self.filter = uNavINS()
        elif nav == 'uNavINS_BFS':
            self.filter = uNavINS_BFS()
        else:
            print("Unknown nav filter specified aborting:", nav)
            quit()

        self.openloop = OpenLoop()
        self.gps_lag_frames = int(round(gps_lag_sec / imu_dt))
        print("gps lag frame:", self.gps_lag_frames)
        self.imu_queue = []
        self.ephemeris = {}
        
    def set_config(self, config):
        Cconfig = NAVconfig()
        Cconfig.from_dict(config)
        self.filter.set_config(Cconfig)

    def set_ephemeris(self, ephem):
        self.ephemeris = ephem

    def gpsraw2gnss(self, gpsraw):
        c = 299792458           # Speed of light in m/s
        print("gpsraw:", gpsraw)
        gnss = []
        for i, id in enumerate(gpsraw['svid']):
            if id < 0:
                break
            ephemid = "0-%d" % id
            ephem = self.ephemeris[ephemid]
            #print(ephemid, ephem)
            if ephem["frame1"] == "False" or ephem["frame2"] == "False" or ephem["frame3"] == "False":
                continue
            print("receiver tow:", gpsraw['receiver_tow'])
            posvel = EphemerisData2PosVelClock(gpsraw['receiver_tow'],
                                               self.ephemeris[ephemid])
            print(ephemid, posvel)
            print("alt:", get_sat_info(gpsraw['receiver_tow'],
                                       self.ephemeris[ephemid]))
            _lambda = 2*c / 1575.4282e6 # L1 according ublox8
            # https://www.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_%28UBX-13003221%29.pdf
            pseudorange_rate = _lambda * gpsraw['doppler'][i]
    
            gnss.append( [ gpsraw['pseudorange'][i], pseudorange_rate ]
                         + posvel )
        #print("gnss:", np.array(gnss))
        result = {}
        result['time'] = gpsraw['receiver_tow']
        result['gnss_measurement'] = np.array(gnss)
        print(result['gnss_measurement'].shape)
        return result
    
    def update(self, imu, gpsraw):
        Cimu = IMUdata()
        Cimu.from_dict(imu)

        # queue delay
        self.imu_queue.insert(0, Cimu)
        Cimu = self.imu_queue.pop()
        #self.Cgps = GPSdata()
        #self.Cgps.from_dict( gps )
        #self.filter.update(Cimu, self.Cgps
        #assert "time" in gnss

        gnss = self.gpsraw2gnss(gpsraw)
        print(gnss['time'])
        print(type(gnss.get('gnss_measurement')), gnss.get('gnss_measurement').shape, gnss.get('gnss_measurement'))
        
        self.Cgnss = GNSS_measurement(gnss['time'], gnss['gnss_measurement'])
        # self.Cgnss = GNSS_measurement()
        # self.Cgnss.create(gnss["gnss_measument.time"], gnss["gnss_measument"])
        # self.Cgnss.from_dict(gnss) 
        # self.Cgnss.from_dict(gnss['time'], gnss.get('gnss_measurement'))
        self.filter.update(Cimu, self.Cgnss)

        nav = self.filter.get_nav()

        if len(self.imu_queue):
            # forward propagate from the lagged solution to new
            self.openloop.init_by_nav(nav)
            for imu in reversed(self.imu_queue):
                nav = self.openloop.update(imu)
            
        return nav.as_dict()

    def close(self):
        pass
