# """
# Author: Chris Regan
# """

import numpy as np

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