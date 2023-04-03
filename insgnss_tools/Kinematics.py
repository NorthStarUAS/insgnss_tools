"""
Cirrus Aircraft
Copyright 2022 Cirrus Aircraft
See: LICENSE.md for complete license details

Author: Chris Regan
"""

import numpy as np

#%%
from scipy import signal
import matplotlib.pyplot as plt

def SolveTimeShift(x, y):
    indx = np.arange(len(x))
    
    # Fill any missing data in x and y
    b, a = signal.butter(2, [1/len(x), 0.1], btype='bandpass')
    
    indxFinite = np.where(np.isfinite(x))[0]
    xFill = np.interp(indx, indx[indxFinite], x[indxFinite])
    # xFill -= xFill[0]
    xFilt = signal.filtfilt(b, a, xFill)
            
    
    indxFinite = np.where(np.isfinite(y))[0]
    yFill = np.interp(indx, indx[indxFinite], y[indxFinite])
    # yFill -= yFill[0]
    yFilt = signal.filtfilt(b, a, yFill)
    
    if False:
        plt.figure()
        plt.plot(x, 'b.')
        plt.plot(xFill, 'b')
        plt.plot(xFilt, '--b')
        plt.plot(y, 'r.')
        plt.plot(yFill, 'r')
        plt.plot(yFilt, '--r')
    
    # Compute the Correlation between x and y
    corr = signal.correlate(xFilt, yFilt, mode="full")
    
    # Find points near the peak correlation
    indxLag = np.argmax(corr) + np.array([-2,-1,0,1,2])
    
    # Fit a second order polynomial to the area around the peak
    corrPolyFit = np.polyfit(indxLag, corr[indxLag], 2 )
    
    # Find the index of the peak of the polynomial fit
    corrPolyFitDeriv = np.polyder(corrPolyFit)
    indxPeak = np.roots(corrPolyFitDeriv)[0]
    
    # Convert the peak index into a lag
    indxLags = signal.correlation_lags(xFilt.size, yFilt.size, mode="full")
    lagPeak = np.interp(indxPeak, np.arange(len(indxLags)), indxLags)
    
    if False:
        # Create a series of point from the polynomial fit, just for plotting
        indxVal = np.linspace(indxLag[0], indxLag[-1], 100)
        corrVal = np.polyval(corrPolyFit, indxVal)
        
        # Correlation at the solved peak
        corrPeak = np.polyval(corrPolyFit, indxPeak)
    
        plt.figure()
        plt.plot(corr, '.')
        plt.plot(indxLag, corr[indxLag])
        plt.plot(indxVal, corrVal)
        plt.plot(indxPeak, corrPeak, '*')
    
    return lagPeak


#%%
# Relative Velocity
def RelVel(vel_AO, omega_AO, pos_BA, vel_BA = np.asarray([0, 0, 0])):
    '''Compute the relative velocity.
    
    Inputs:
      vel_AO   - Velocity of frame A wrt frame O
      omega_AO - Rotation rate of frame A wrt O
      pos_BA   - Position of frame B wrt A
      vel_BA   - Velocity of frame B wrt A [0, 0, 0]
    
    Outputs:
      vel_BO   - Velocity of frame B wrt frame O
    
    Notes:
      There are no internal unit conversions.
      The vectors must all be in the same coordinate system
    '''
    
    # Compute the Relative Velocity
    vel_BO = vel_AO + vel_BA + np.cross(omega_AO, pos_BA)
    
    return vel_BO

# Relative Acceleration
def RelAccel(accel_AO, omega_AO, omegaDot_AO, pos_BA, accel_BA = np.asarray([0, 0, 0]), vel_BA = np.asarray([0, 0, 0])):
    '''Compute the relative acceleration.
    
    Inputs:
      accel_AO    - Acceleration of frame A wrt frame O
      omega_AO    - Rotation rate of frame A wrt O
      omegaDot_AO - Rotation rate of frame A wrt O
      pos_BA      - Position of frame B wrt A
      accel_BA    - Acceleration of frame B wrt A [0, 0, 0]
      vel_BA      - Velocity of frame B wrt A [0, 0, 0]
    
    Outputs:
      accel_BO   - Acceleration of frame B wrt frame O
    
    Notes:
      There are no internal unit conversions.
      The vectors must all be in the same coordinate system
    '''
    
    # Compute the Relative Acceleration
    accel_BO = accel_AO + accel_BA + np.cross(omegaDot_AO, pos_BA) + np.cross(omega_AO, np.cross(omega_AO, pos_BA)) + 2 * np.cross(omega_AO, vel_BA)
    
    return accel_BO


def TransPitot(v_PA_P_mps, w_BA_B_rps, s_PB_rad, r_PB_B_m):
    ''' Transform Pitot Measurements from Probe location to Body.
    
    Inputs:
      v_PA_P_mps - Velocity of the Probe wrt Atm in Probe Coordinates [P/A]P (m/s2)
      w_BA_B_rps - Rotation rate of the Body Frame wrt Atm in Body Coordinates [B/A]B (rad/s)
      s_PB_rad   - Orientation (321) of the Probe Frame wrt Body Frame [P/B] (rad)
      r_PB_B_m   - Position of the Probe Frame wrt Body Frame in Body Coordinates [P/B]B (m)
    
    Outputs:
      v_BA_B_mps - Velocity of the Body wrt Atm in Body Coordinates [B/A]B (m/s2)
      v_BA_L_mps - Velocity of the Body wrt Atm in Local Level Coordinates [B/A]L (m/s2)
    '''
    
    # Parameterize transformation from P to B
    T_B2P = R.from_euler('ZYX', s_PB_rad[[2,1,0]], degrees = False).as_matrix()
    T_P2B = T_B2P.T
    
    v_PB_B_mps = np.asarray([0, 0, 0]) # Velocity of the Probe wrt the Body Frame [P/B]B (m/s)
    v_BP_B_mps = -v_PB_B_mps
    
    r_BP_B_m = -r_PB_B_m
    
    #
    v_BA_B_mps = np.nan * np.ones_like(v_PA_P_mps)
    numSamp = v_PA_P_mps.shape[-1]
    for indx in range(0, numSamp):
        # Transform from P to B
        v_PA_B_mps = T_P2B @ v_PA_P_mps[:,indx]
        v_BA_B_mps[:,indx] = RelVel(v_PA_B_mps, w_BA_B_rps[:,indx], r_BP_B_m, v_BP_B_mps)
    
    return v_BA_B_mps

def VelocityNED(time_s, rB_L_m):
    ''' Compute the NED Velocity based on NED positions.
    
    Inputs:
        rB_L_m - Position in NED (m)
    
    Outputs:
        vB_L_mps - Velocity in NED (m/s)
    '''
    
    indxFinite = np.where(np.isfinite(rB_L_m[0]))[0]

    # Compute the NED Velocity from Geodetic Positions
    vB_L_mps = np.nan * np.ones_like(rB_L_m)
    
    dr = np.diff(rB_L_m[:,indxFinite], axis=1)
    dt = np.diff(time_s[indxFinite])
    
    vB_L_mps[:,indxFinite[:-1]] = dr / dt
    
    if False: # Place the velocity midway between indices
        indxMid = int(indxFinite[0:-1] + 0.5*np.diff(indxFinite))
        vB_L_mps[:, indxMid] = vB_L_mps
        
    return vB_L_mps


#%% Coordinate Transforms
# Reference Frames and Coordinates
# I - ECI (Earch Center Inertial): origin at Earth center
# E - ECEF (Earch Center Earth Fixed): origin at Earth center
# D - Geodetic: origin at Earth center, Uses earth ellisoid definition (WGS84)
# G - Geocentric: origin at Earth center, Uses spheroid definition
# L - Local Level: origin at specified reference, [x- North, y- East, z- Down]
# B - Body: origin at Body CG, [x- Fwd, y- Starboard, z- Down]
# All units meters and radians

from scipy.spatial.transform import Rotation as R

# Transform coordinates: NED to/from Body
def TransL2B(s_BL_rad):
    
    # T_L2B = R.from_euler('XYZ', -s_BL_rad, degrees = False).as_matrix()

    phi = s_BL_rad[0]
    theta = s_BL_rad[1]
    psi = s_BL_rad[2]
    
    R3 = R.from_euler('Z', -psi, degrees = False).as_matrix() # Stevens and Lewis 3rd (1.3-10)
    R2 = R.from_euler('Y', -theta, degrees = False).as_matrix()
    R1 = R.from_euler('X', -phi, degrees = False).as_matrix()

    T_L2B = R1 @ R2 @ R3 # Stevens and Lewis 3rd (1.3-10)
    
    if len(s_BL_rad.shape) > 1:
        T_L2B = np.moveaxis(T_L2B, 0, -1)

    return T_L2B

def TransB2L(s_BL_rad):
    
    # T_B2L = R.from_euler('ZYX', s_BL_rad[[2,1,0]], degrees = False).as_matrix()
    
    # if len(s_BL_rad.shape) > 1:
    #     T_B2L = np.moveaxis(T_B2L, 0, -1)
    
    T_B2L = TransL2B(s_BL_rad).T
    
    return T_B2L


# Transform coordinates: Body to/from Wind
def TransB2W(alpha_rad, beta_rad):

    R2 = R.from_euler('Y', -alpha_rad, degrees = False).as_matrix()
    R3 = R.from_euler('Z', beta_rad, degrees = False).as_matrix()
    
    T_B2W = R2 @ R3
        
    return T_B2W

def TransW2B(alpha_rad, beta_rad):
    
    T_W2B = TransB2W(alpha_rad, beta_rad).T
    
    return T_W2B


# WGS84 Parameters
def WGS84Param():
    R_m = 6378137.0 # earth semi-major axis radius (m)
    f_nd = 1/298.257223563 # reciprocal flattening (nd)
    
    return R_m, f_nd

# Earth Radius
def EarthRad(lat):
    R_m, f_nd = WGS84Param()
    e2 = 2*f_nd - f_nd**2 # eccentricity squared
    
    denom = np.abs(1.0 - (e2 * np.sin(lat)**2))
    sqrt_denom = np.sqrt(denom)
    
    Rew = R_m / sqrt_denom # Transverse (along East-West)
    Rns = R_m * (1 - e2) / (denom * sqrt_denom) # Merdian (along North-South)
    
    return Rew, Rns

# Transform coordinates: Geodetic to ECEF
def D2E(r_D_, degrees = False):
    ''' Convert Geodetic to ECEF Coordinates
    
    Inputs:
      r_D_ - Position in Geodetic Coordinates (-, -, m)
      degrees - r_D_ is input in degrees [True]
    
    Outputs:
      r_E_m - Position in ECEF Coordinates (m, m, m)
    '''
    
    # Change units
    if degrees:
        d2r = np.pi / 180.0
        r_D_ = (r_D_.T * np.asarray([d2r, d2r, 1.0])).T
    
    lat = r_D_[0]
    lon = r_D_[1]
    alt = r_D_[2]
    
    # Parameters for WGS84
    R_m, f_nd = WGS84Param()
    e2 = 2*f_nd - f_nd**2 # eccentricity squared
    
    # Earth East-West radius at specified latitude
    Rew = R_m / np.sqrt(1 - e2 * np.sin(lat)**2)
        
    ## Convert
    r_E_m = np.nan * np.ones_like(r_D_)
    r_E_m[0] = (Rew + alt) * np.cos(lat) * np.cos(lon)
    r_E_m[1] = (Rew + alt) * np.cos(lat) * np.sin(lon)
    r_E_m[2] = (Rew * (1 - e2) + alt) * np.sin(lat)
    
    return r_E_m

# Calculate the Latitude, Longitude and Altitude given the ECEF Coordinates.
def E2D( p_E, degrees = False):
    R_m, f_nd = WGS84Param()

    ra2 = 1.0 / R_m**2
    e2 = 2*f_nd - f_nd**2 # eccentricity squared
    e4 = e2**2

    # according to: H. Vermeille,
    # Direct transformation from geocentric to geodetic ccordinates,
    # Journal of Geodesy (2002) 76:451-454
    p_D = np.zeros(3)
    
    X = p_E[0]
    Y = p_E[1]
    Z = p_E[2]

    XXpYY = X*X+Y*Y;
    if( XXpYY + Z*Z < 25 ):
       	# This function fails near the geocenter region, so catch
       	# that special case here.  Define the innermost sphere of
       	# small radius as earth center and return the coordinates
       	# 0/0/-EQURAD. It may be any other place on geoide's surface,
       	# the Northpole, Hawaii or Wentorf. This one was easy to code ;-)
       	p_D[0] = 0.0
       	p_D[1] = 0.0
       	p_D[2] = -R_m
    

    sqrtXXpYY = np.sqrt(XXpYY)
    p = XXpYY*ra2
    q = Z*Z*(1-e2)*ra2
    r = 1/6.0*(p+q-e4)
    s = e4*p*q/(4*r*r*r)

    # s*(2+s) is negative for s = [-2..0]
    # slightly negative values for s due to floating point rounding errors
    # cause nan for sqrt(s*(2+s))
    # We can probably clamp the resulting parable to positive numbers
    if( s >= -2.0) & (s <= 0.0 ) :
        s = 0.0

    t = pow(1+s+np.sqrt(s*(2+s)), 1/3.0);
    u = r*(1+t+1/t);
    v = np.sqrt(u*u+e4*q);
    w = e2*(u+v-q)/(2*v);
    k = np.sqrt(u+v+w*w)-w;
    D = k*sqrtXXpYY/(k+e2);
    sqrtDDpZZ = np.sqrt(D*D+Z*Z);

    p_D[1] = 2*np.arctan2(Y, X+sqrtXXpYY);
    p_D[0] = 2*np.arctan2(Z, D+sqrtDDpZZ);
    p_D[2] = (k+e2-1)*sqrtDDpZZ/k;

    # Change units to Degrees
    if degrees:
        r2d = 180.0 / np.pi
        p_D = (p_D.T * np.asarray([r2d, r2d, 1.0])).T
        
    return p_D


# Transform coordinates: Geodetic to NED
def D2L(rRef_LD_D_, r_PD_D_, degrees = False):
    ''' Convert ECEF Coordinates to Local Level
    
    Inputs:
      rRef_LD_D_ - Reference Position in Geodetic Coordinates [-, -, m]
      r_PD_D_   - Position in Geodetic Coordinates
    
    Outputs:
      r_PL_L_m - Position in Local Level Coordinates (m)
    
    Notes:
      Uses D2E and E2L
    '''
    
    # Change units to Radians
    if degrees:
        d2r = np.pi / 180.0
        rRef_LD_D_ = (rRef_LD_D_.T * np.asarray([d2r, d2r, 1.0])).T
        r_PD_D_ = (r_PD_D_.T * np.asarray([d2r, d2r, 1.0])).T
    
    # Reference location of L wrt D in ECEF
    r_LD_E_m = D2E(rRef_LD_D_, degrees = False)
    
    # Position of P wrt E in ECEF
    r_PE_E_m = D2E(r_PD_D_, degrees = False)
    
    # Distance from Ref in ECEF
    r_PL_E_m = r_PE_E_m - r_LD_E_m
    
    r_PL_L_m = E2L(rRef_LD_D_, r_PL_E_m, degrees = False)
    
    return r_PL_L_m


# Calculate the rate of change of latitude, longitude,
# and altitude using the velocity in NED coordinates and WGS-84.
def L2D_Rate(v_L, rRef_D_, degrees = False):
    
    # Change units
    d2r = np.pi / 180.0
    if degrees:
        rRef_D_ = (rRef_D_.T * np.asarray([d2r, d2r, 1.0])).T

    lat = rRef_D_[0]
    # lon = rRef_D_[1]
    alt = rRef_D_[2]
    
    Rew, Rns = EarthRad(lat)
    
    pDot_D = np.zeros((3))
    pDot_D[0] = v_L[0] / (Rns + alt) # latDot = vNorth / (Rns + alt)
    pDot_D[1] = v_L[1] / ((Rew + alt) * np.cos(lat)) # lonDot = vEast / ((Rew + alt)*np.cos(lat))
    pDot_D[2] = -v_L[2]

    return pDot_D

# Calculate the angular velocity of the NED frame,
# also known as the navigation rate using WGS-84.
# Rotation rate of the NED frame wrt Earth Center, in NED coordinates.
def NavRate(v_L, rRef_D_, degrees = False):
        
    # Change units
    d2r = np.pi / 180.0
    if degrees:
        rRef_D_ = (rRef_D_.T * np.asarray([d2r, d2r, 1.0])).T

    lat = rRef_D_[0]
    # lon = rRef_D_[1]
    alt = rRef_D_[2]
    
    Rew, Rns = EarthRad(lat)

    w_LE = np.zeros((3))
    w_LE[0] = v_L[1] / (Rew + alt) # rotation rate about North = vEast / (Rew + alt)
    w_LE[1] = -v_L[0] / (Rns + alt) # rotation rate about East = -vNorth / (Rns + alt)
    w_LE[2] = -v_L[1] * np.tan(lat) / (Rew + alt) # rotation rate about Down

    return w_LE

# Transform coordinates: ECEF to NED
def E2L(rRef_LD_D_, r_PL_E_m, degrees = False):
    ''' Convert ECEF Coordinates to Local Level
    
    Inputs:
      rRef_LD_D_ - Reference Position in Geodetic Coordinates (ddm)
      r_PE_E_m   - Position in ECEF Coordinates (m)
    
    Outputs:
      T_E2L - Transformation Matrix from ECEF to Local Level Coordinates
      r_L_m - Position in Local Level Coordinates (m)
    '''
    
    # Transform the Reference Coordinates
    rRef_LD_E_m = D2E(rRef_LD_D_, degrees)
    
    # Compute the Transformation Matrix at the Reference Location
    T_E2L = TransE2L(rRef_LD_D_, degrees)
    
    # Transform Coordinates from ECEF to NED
    r_PL_L_m = T_E2L @ r_PL_E_m
    
    return r_PL_L_m

# Transform coordinates: NED to ECEF
def L2E(rRef_LD_D_, r_PL_L_m, degrees = False):
    ''' Convert Local Level Coordinates to ECEF
    
    Inputs:
      rRef_LD_D_ - Reference Position in Geodetic Coordinates (ddm)
      r_PL_L_m   - Position in Local Level Coordinates (m)
    
    Outputs:
      T_L2E - Transformation Matrix from Local Level to ECEF Coordinates
      r_E_m - Position in Local Level Coordinates (m)
    '''
    # Transform the Reference Coordinates
    rRef_LD_E_m = D2E(rRef_LD_D_, degrees)
    
    # Compute the Transformation Matrix at the Reference Location
    T_E2L = TransE2L(rRef_LD_D_, degrees)
    T_L2E = T_E2L.T
    
    # Transform Coordinates from ECEF to NED
    r_PL_E_m = rRef_LD_E_m + T_L2E @ r_PL_L_m
    
    return r_PL_E_m

# Calculate the ECEF to NED coordinate transform DCM, rotation only
def TransE2L(pRef_D_, degrees = False):
    ''' Rotation Matrix to convert ECEF Coordinates to Local Level
    
    Inputs:
      pRef_D_ - Reference Position in Geodetic Coordinates (ddm)
    
    Outputs:
      T_E2L - Transformation Matrix from ECEF to Local Level Coordinates
    
    Notes:
      T_E2L = R2(270-lat)*R3(long)
    '''
    
    # Change units
    d2r = np.pi / 180.0
    if degrees:
        pRef_D_ = pRef_D_ * np.asarray([d2r, d2r, 1.0])
        
    lat = pRef_D_[0]
    lon = pRef_D_[1]
        
    # T_E2L = np.zeros((3,3))
    # T_E2L[0,0] = -np.sin(lat)*np.cos(lon)
    # T_E2L[0,1] = -np.sin(lat)*np.sin(lon)
    # T_E2L[0,2] = np.cos(lat)
    
    # T_E2L[1,0] = -np.sin(lon)
    # T_E2L[1,1] = np.cos(lon)
    # T_E2L[1,2] = 0.0
    
    # T_E2L[2,0] = -np.cos(lat)*np.cos(lon)
    # T_E2L[2,1] = -np.cos(lat)*np.sin(lon)
    # T_E2L[2,2] = -np.sin(lat)
        
    sRef_ = np.asarray([270.0 * d2r - lat, lon])
    T_E2L = R.from_euler('YZ', -sRef_, degrees = False).as_matrix()
    
    return T_E2L

# Attitude Interpolation
def AttInterp(s_rad, time_s):
    from scipy.spatial.transform import Slerp
    
    # Find all the good Attitudes
    indx = np.arange(s_rad.shape[-1])
    indxValid = np.any(~np.isnan(s_rad), axis=0)
    indxSlerp = ~indxValid
    
    if any(indxSlerp):
        indxSlerp[:min(indx[indxValid])] = False
        indxSlerp[max(indx[indxValid]):] = False
    
        # Slerp to fill the nan in the time history
        rot = R.from_euler('xyz', s_rad.T)
        slerpObj = Slerp(time_s[indxValid], rot[indxValid])
        rotSlerp = slerpObj(time_s[indxSlerp])
        
        # Convert back to Euler angles
        # s_rad = rot.as_euler('xyz').T
        s_rad[:, indxSlerp] = rotSlerp.as_euler('xyz').T
        
        # heading as 0 to 2Pi
        s_rad[2] = np.mod(s_rad[2], 2*np.pi)

    return s_rad

# Naive Derivative - XXX use a polynomial fit or filter-based derivative
def Derive(data, time):
        
    dataDot = np.nan * np.ones_like(data)
    if len(data.shape) > 1:
        indxFinite = np.where(np.all(np.isfinite(data), axis=0))[0]
        dTime = np.diff(time[indxFinite])
        dData = np.diff(data[:, indxFinite], axis=-1)
        dataDot[:, indxFinite[:-1]] = dData / dTime
    else:
        indxFinite = np.where(np.isfinite(data))[0]
        dTime = np.diff(time[indxFinite])
        dData = np.diff(data[indxFinite])
        dataDot[indxFinite[:-1]] = dData / dTime
    
    return dataDot


# def DeriveConv(z, dt):
#     #  Analytic expressions for the derivative of local 
#     #  polynomial fits to the noisy measured data.  
#     #  The expressions are different near the endpoints 
#     #  because there are not enough neighboring points on one side.
    
#     [m,n] = z.shape()
#     zd = np.zeros((m,n))

    
#     zd[1,:]     = (-54*z[1,:] + 13*z[2,  :] + 40*z[3,  :] + 27*z[4,  :] - 26*z[5,  :]) / (70*dt)
#     zd[2,:]     = (-34*z[1,:] +  3*z[2,  :] + 20*z[3,  :] + 17*z[4,  :] -  6*z[5,  :]) / (70*dt)
#     zd[3:n-2,:] = ( -2*z[1:n-4,:] - z[2:n-3,:] + z[4:n-1,:] + 2.*z[5:n,:]) / (10*dt)
#     zd[n-1,:]   = ( 34*z[n,:] -  3*z[n-1,:] - 20*z[n-2,:] - 17*z[n-3,:] +  6*z[n-4,:]) / (70*dt)
#     zd[n,:]     = ( 54*z[n,:] - 13*z[n-1,:] - 40*z[n-2,:] - 27*z[n-3,:] + 26*z[n-4,:]) / (70*dt)
    
#     return


def ComputeAccelGravNED(faData, g_mps2=9.81):
    
    if 'T_L2B' not in faData:
        faData['T_L2B'] = TransL2B(faData['sB_L_rad'])
    
    faData['aGrav_L_mps2'] = np.array([0, 0, -g_mps2])
    
    faData['aGrav_B_mps2'] = np.nan * np.ones_like(faData['sB_L_rad'])
    for i in range(faData['T_L2B'].shape[-1]):
        faData['aGrav_B_mps2'][..., i] = faData['T_L2B'][..., i] @ faData['aGrav_L_mps2']
    
    return



#%% Rotation Transforms, Quaternions
def QuatNormalize(q):
    qNorm = np.linalg.norm(q)
    if qNorm != 0:
        q = q / qNorm
    
    return q

def QuatMult(p, q):
    p0, p1, p2, p3 = p
        
    P = np.array([
        [p0, -p1, -p2, -p3], 
        [p1,  p0, -p3,  p2],
        [p2,  p3,  p0, -p1],
        [p3, -p2,  p1,  p0]])
    
    r = QuatNormalize(P @ q)
    
    return r

def QuatInv(q):
    
    q = QuatNormalize(q)
    
    # qInv = 1/qNorm * (q * np.array([1, -1, -1, -1]))
    qInv = q * np.array([1, -1, -1, -1])
    
    return qInv

def QuatRotate(q, v):
    # Rotate q by v, where v = [v0 i + v1 j + v2 k]
    
    qInv = QuatInv(q)
    qRot = np.hstack((0, v))
    qNew = QuatMult(qInv, QuatMult(qRot, q))
    
    return qNew

def QuatDeriv(q, w):
    q = QuatNormalize(q)

    w0, w1, w2 = w
        
    W = np.array([
        [0,  -w0, -w1, -w2], 
        [w0,  0,   w2, -w1],
        [w1, -w2,  0,   w0],
        [w2,  w1, -w0,  0 ]])
    
    qDot = 0.5 * W @ q
    
    return qDot


# def QuatDeriv(q, w):
#     q = QuatNormalize(q)
    
#     q0, q1, q2, q3 = q
        
#     Q = np.array([
#         [-q1, -q2, -q3], 
#         [ q0, -q3,  q2],
#         [ q3,  q0,  q1],
#         [-q2,  q1,  q0]])
    
#     qDot = 0.5 * Q @ w
    
#     return qDot

# Euler angles (3-2-1) to quaternion
def Euler2Quat(s):
    
    phi = s[0]
    theta = s[1]
    psi = s[2]
    
    q = np.zeros(4)
    q[0] = np.sin(phi/2)*np.sin(theta/2)*np.sin(psi/2) + np.cos(phi/2)*np.cos(theta/2)*np.cos(psi/2)
    q[1] = np.sin(phi/2)*np.cos(theta/2)*np.cos(psi/2) - np.cos(phi/2)*np.sin(theta/2)*np.sin(psi/2)
    q[2] = np.cos(phi/2)*np.sin(theta/2)*np.cos(psi/2) + np.sin(phi/2)*np.cos(theta/2)*np.sin(psi/2)
    q[3] = np.cos(phi/2)*np.cos(theta/2)*np.sin(psi/2) - np.sin(phi/2)*np.sin(theta/2)*np.cos(psi/2)

    return q

# Quaternion to Euler angles (3-2-1)
def Quat2Euler(q):
    q = QuatNormalize(q)
    
    T = Quat2DCM(q)
    
    phi = np.arctan2(T[1,2], T[2,2])
    theta = np.arcsin(-T[0,2])
    psi = np.arctan2(T[0,1], T[0,0])
    
    # Make psi be 0 to 2*pi, rather than -pi to pi
    psi = np.mod(psi, 2*np.pi)
    
    s = np.array([phi, theta, psi])
    
    return s

# Quaternion to DCM, S&L 3rd (1.8-18)
def Quat2DCM(q):
    T = np.zeros((3,3))
    
    T[0,0] = q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3]
    T[1,1] = q[0]*q[0] - q[1]*q[1] + q[2]*q[2] - q[3]*q[3]
    T[2,2] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]
    
    # T[0,0] = 2*(q[0]*q[0] + q[1]*q[1]) - 1
    # T[1,1] = 2*(q[0]*q[0] + q[2]*q[2]) - 1
    # T[2,2] = 2*(q[0]*q[0] + q[3]*q[3]) - 1
    
    T[0,1] = 2*(q[1]*q[2] + q[0]*q[3])
    T[0,2] = 2*(q[1]*q[3] - q[0]*q[2])
    
    T[1,0] = 2*(q[1]*q[2] - q[0]*q[3])
    T[1,2] = 2*(q[2]*q[3] + q[0]*q[1])
    
    T[2,0] = 2*(q[1]*q[3] + q[0]*q[2])
    T[2,1] = 2*(q[2]*q[3] - q[0]*q[1])
    
    return T