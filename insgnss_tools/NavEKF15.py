# -*- coding: utf-8 -*-
# Copyright (c) 2016 - 2020 Regents of the University of Minnesota and Bolder Flight Systems Inc.
# MIT License See LICENSE.md for complete details
# Adapted for RAPTRS: Brian Taylor and Chris Regan
# Adapted from prior versions
# Copyright 2011 Regents of the University of Minnesota. All rights reserved.
# Original Author: Adhika Lie, Gokhan Inalhan, Demoz Gebre, Jung Soon Jang
# Reference Frames and Coordinates from nav-functions()
# I - ECI (Earch Center Inertial): origin at Earth center
# E - ECEF (Earch Center Earth Fixed): origin at Earth center
# D - Geodetic: origin at Earth center, Uses earth ellisoid definition (example WGS84)
# G - Geocentric: origin at Earth center, Uses spheroid definition
# L - Local Level: origin at specified reference, [x- North, y- East, z- Down]
# B - Body: origin at Body CG, [x- Fwd, y- Starboard, z- Down]
# All units meters and radians
# "Acceleration" is actually "specific gravity", ie. gravity is removed.


import numpy as np
from . import Kinematics

# Model Constants
g_mps2 = 9.807 # Acceleration due to gravity
R_m = 6378137.0 # earth semi-major axis radius (m)

#%%
# Skew symmetric matrix from a given vector w
def Skew(w):
    C = np.array([
        [0.0, -w[2], w[1]],
        [w[1], 0.0, -w[0]],
        [-w[1], w[0], 0.0] ])

    return C

#%% Extended Kalman Filter for Navigation with IMU and GPS
# Identity matrices
I2 = np.identity(2)
I3 = np.identity(3)
I5 = np.identity(5)
I15 = np.identity(15)

# from scipy.spatial.transform import Rotation as R

class NavEKF15:
    def __init__(self):
        # Initialize flag
        self.initialized = False

        # Sensor variances (as standard deviation) and models (tau)
        self.aNoiseSigma_mps2 = 0.5 # Std dev of Accelerometer Wide Band Noise (m/s^2)
        self.aMarkovSigma_mps2 = 0.1 # Std dev of Accelerometer Markov Bias
        self.aMarkovTau_s = 100.0 # Correlation time or time constant

        self.wNoiseSigma_rps = 0.1 * d2r # Std dev of rotation rate output noise (rad/s)
        self.wMarkovSigma_rps = 0.015 * d2r # Std dev of correlated rotation rate bias (rad/s)
        self.wMarkovTau_s = 50.0 # Correlation time or time constant

        self.rNoiseSigma_NE_m = 3.0 # GPS measurement noise std dev (m)
        self.rNoiseSigma_D_m = 6.0 # GPS measurement noise std dev (m)

        self.vNoiseSigma_NE_mps = 0.5 # GPS measurement noise std dev (m/s)
        self.vNoiseSigma_D_mps = 1.0 # GPS measurement noise std dev (m/s)

        # Initial set of covariance
        self.pErrSigma_Init_m = 10.0 # Std dev of initial position error (m)
        self.vErrSigma_Init_mps = 1.0 # Std dev of initial velocity error (m/s)
        self.attErrSigma_Init_rad = 20 * d2r # Std dev of initial attitude (phi and theta) error (rad)
        self.hdgErrSigma_Init_rad = 90 * d2r # Std dev of initial Heading (psi) error (rad)
        self.aBiasSigma_Init_mps2 = 0.1 * g_mps2 # Std dev of initial acceleration bias (m/s^2)
        self.wBiasSigma_Init_rps = 1 * d2r # Std dev of initial rotation rate bias (rad/s)

        # Kalman Matrices
        self.H = np.zeros((6,15)) # Observation matrix
        self.R = np.zeros((6,6)) # Covariance of the Observation Noise (associated with MeasUpdate())
        self.Rw = np.zeros((12,12)) # Covariance of the Sensor Noise (associated with TimeUpdate())
        self.S = np.zeros((6,6)) # Innovation covariance
        self.P = np.zeros((15,15)) # Covariance estimate

        # Global variables
        self.aBias_mps2 = np.zeros((3)) # acceleration bias
        self.wBias_rps = np.zeros((3)) # rotation rate bias
        self.sEst_BL_rad = np.zeros((3)) # Euler angles - B wrt L (3-2-1) [phi, theta, psi]
        self.qEst_BL = np.zeros((4)) # Quaternion of B wrt L
        self.aEst_B_mps2 = np.zeros((3)) # Estimated acceleration in Body
        self.wEst_B_rps = np.zeros((3)) # Estimated rotation rate in Body
        self.vEst_L_mps = np.zeros((3)) # Estimated velocity in NED
        self.rEst_D_rrm = np.zeros((3)) # Estimated position in LLA (rad, rad, m)


    def Configure(self):
        # Observation matrix (H)
        self.H[0:5,0:5] = I5

        # Covariance of the Process Noise (associated with TimeUpdate())
        self.Rw[0:3,0:3] = self.aNoiseSigma_mps2**2 * I3
        self.Rw[3:6,3:6] = self.wNoiseSigma_rps**2 * I3
        self.Rw[6:9,6:9] = 2 / self.aMarkovTau_s * self.aMarkovSigma_mps2**2 * I3
        self.Rw[9:12,9:12] = 2 / self.wMarkovTau_s * self.wMarkovSigma_rps**2 * I3

        # Covariance of the Observation Noise (associated with MeasUpdate())
        self.R[0:2,0:2] = self.rNoiseSigma_NE_m**2 * I2
        self.R[2,2] = self.rNoiseSigma_D_m**2
        self.R[3:5,3:5] = self.vNoiseSigma_NE_mps**2 * I2
        self.R[5,5] = self.vNoiseSigma_D_mps**2

        # Initial Innovation Covariance Estimate (S)
        # Zeros

        # Initial Covariance Estimate (P)
        self.P[0:3,0:3] = self.pErrSigma_Init_m**2 * I3
        self.P[3:6,3:6] = self.vErrSigma_Init_mps**2 * I3
        self.P[6:8,6:8] = self.attErrSigma_Init_rad**2 * I2
        self.P[8,8] = self.hdgErrSigma_Init_rad**2
        self.P[9:12,9:12] = self.aBiasSigma_Init_mps2**2 * I3
        self.P[12:15,12:15] = self.wBiasSigma_Init_rps**2 * I3


    def InitAttitude(self, a0_B_mps2, v0_L_mps):
        # Initial attitude, roll and pitch
        aNorm_B_nd = a0_B_mps2 / np.linalg.norm(a0_B_mps2) # Normalize to remove the 1g sensitivity

        # Initial attitude
        s0_BL_rad = np.zeros((3))
        s0_BL_rad[1] = np.arcsin(aNorm_B_nd[0])
        s0_BL_rad[0] = -np.arcsin(aNorm_B_nd[1] / np.cos(s0_BL_rad[1]))

        # Estimate initial heading
        s0_BL_rad[2] = np.arctan2(v0_L_mps[1], v0_L_mps[0])
        s0_BL_rad[2] = np.mod(s0_BL_rad[2], 2*np.pi)

        return s0_BL_rad

    def Initialize(self, w0_B_rps, a0_B_mps2, r0_D_rrm, v0_L_mps, s0_BL_rad = None):
        # Initialize Position and Velocity
        self.rEst_D_rrm = r0_D_rrm # Position in LLA (rad, rad, m)
        self.vEst_L_mps = v0_L_mps # Velocity in NED

        # Specific forces (acceleration) and Rotation Rate
        self.aEst_B_mps2 = a0_B_mps2
        self.wEst_B_rps = w0_B_rps

        # Initialize sensor biases
        self.wBias_rps = np.zeros((3))
        self.aBias_mps2 = np.zeros((3))

        # Initial attitude
        if s0_BL_rad == None:
            s0_BL_rad = self.InitAttitude(a0_B_mps2, v0_L_mps)

        # Euler to quaternion
        self.qEst_BL = Kinematics.Euler2Quat(s0_BL_rad)

        # set initialized flag
        self.initialized = True


    def Update(self, updt, wMeas_B_rps, aMeas_B_mps2, rMeas_D_rrm, vMeas_L_mps, dt_s):
        # Catch large dt
        # if (dt_s > 1): dt_s = 1

        # A-priori accel and rotation rate estimate
        self.aEst_B_mps2 = aMeas_B_mps2 - self.aBias_mps2
        self.wEst_B_rps = wMeas_B_rps - self.wBias_rps

        # Kalman Time Update (Prediction)
        self.TimeUpdate(dt_s)

        # Gps measurement update, if TOW increased
        if updt:
            # Kalman Measurement Update
            self.MeasUpdate(rMeas_D_rrm, vMeas_L_mps)

            # Post-priori accel and rotation rate estimate, biases updated in MeasUpdate()
            self.aEst_B_mps2 = aMeas_B_mps2 - self.aBias_mps2
            self.wEst_B_rps = wMeas_B_rps - self.wBias_rps

        # Euler angles from quaternion
        self.sEst_BL_rad = Kinematics.Quat2Euler(self.qEst_BL)

        return self.aEst_B_mps2, self.wEst_B_rps, self.vEst_L_mps, self.sEst_BL_rad, self.rEst_D_rrm


    def TimeUpdate(self, dt_s):

        # Compute DCM (Body to/from NED) Transformations from Quaternion
        T_L2B = Kinematics.Quat2DCM(self.qEst_BL)
        T_B2L = T_L2B.T

        # Attitude Update
        qDelta_BL = np.hstack((1, 0.5*dt_s*self.wEst_B_rps))
        self.qEst_BL = Kinematics.QuatMult(self.qEst_BL, qDelta_BL)

        # Avoid quaternion flips sign
        if (self.qEst_BL[0] < 0):
            self.qEst_BL = -self.qEst_BL

        # Velocity Update
        aGrav_mps2 = np.array([0.0, 0.0, g_mps2])
        self.vEst_L_mps += dt_s * (T_B2L @ self.aEst_B_mps2 + aGrav_mps2)

        # Position Update
        rDot_D = Kinematics.L2D_Rate(self.vEst_L_mps, self.rEst_D_rrm)
        self.rEst_D_rrm += (dt_s * rDot_D)

        # Assemble the Jacobian (state update matrix)
        self.Fs = np.zeros((15,15))
        self.Fs[0:3,0:3] = I3
        self.Fs[5,2] = -2.0 * g_mps2 / R_m
        self.Fs[3:6,6:9] = -2.0 * T_B2L @ Skew(self.aEst_B_mps2)
        self.Fs[3:6,9:12] = -T_B2L
        self.Fs[6:9,6:9] = -Skew(self.wEst_B_rps)
        self.Fs[6:9,12:15] = -0.5 * I3
        self.Fs[9:12,9:12] = -1.0 / self.aMarkovTau_s * I3 # ... Accel Markov Bias
        self.Fs[12:15,12:15] = -1.0 / self.wMarkovTau_s * I3 # ... Rotation Rate Markov Bias

        # State Transition Matrix
        self.PHI = I15 + self.Fs * dt_s

        # Process Noise Covariance (Discrete approximation)
        self.Gs = np.zeros((15,12))
        self.Gs[3:6,0:3] = -T_B2L
        self.Gs[6:9,3:6] = -0.5 * I3
        self.Gs[9:12,6:9] = I3
        self.Gs[12:15,9:12] = I3

        # Discrete Process Noise
        self.Q = np.zeros((15,15))
        self.Q = (self.PHI * dt_s) @ self.Gs @ self.Rw @ self.Gs.T
        self.Q = 0.5 * (self.Q + self.Q.T)

        # Covariance Time Update
        self.P = self.PHI @ self.P @ self.PHI.T + self.Q
        self.P = 0.5 * (self.P + self.P.T)

    # Measurement Update
    def MeasUpdate(self, rMeas_D_rrm, vMeas_L_mps):
        # Position Error, converted to NED
        T_E2L = Kinematics.TransE2L(self.rEst_D_rrm) # Compute ECEF to NED
        pErr_L_m = T_E2L @ (Kinematics.D2E(rMeas_D_rrm) - Kinematics.D2E(self.rEst_D_rrm)) # Compute position error, apply transformation

        # Velocity Error
        vErr_L_mps = vMeas_L_mps - self.vEst_L_mps

        # Create measurement Y, as Error between Measures and Outputs
        y = np.zeros((6))
        y[0:3] = pErr_L_m
        y[3:6] = vErr_L_mps

        # Innovation covariance
        self.S = self.H @ self.P @ self.H.T + self.R

        # Kalman gain
        self.K = self.P @ self.H.T @ np.linalg.inv(self.S)

        # Covariance update, P = (I + K * H) * P * (I + K * H)' + K * R * K'
        I_KH = I15 - self.K @ self.H # temp
        self.P = I_KH @ self.P @ I_KH.T + self.K @ self.R @ self.K.T

        # State update, x = K * y
        x = self.K @ y

        # Pull apart x terms to update the Position, velocity, orientation, and sensor biases
        pDelta_D = x[0:3] # Position Deltas in LLA
        vDelta_L = x[3:6] # Velocity Deltas in NED
        quatDelta = x[6:9] # Quaternion Delta
        aBiasDelta = x[9:12] # Accel Bias Deltas
        wBiasDelta = x[12:15] # Rotation Rate Bias Deltas

        # Position update
        Rew, Rns = Kinematics.EarthRad(self.rEst_D_rrm[0])

        self.rEst_D_rrm[2] += -pDelta_D[2]
        self.rEst_D_rrm[0] += pDelta_D[0] / (Rew + self.rEst_D_rrm[2])
        self.rEst_D_rrm[1] += pDelta_D[1] / (Rns + self.rEst_D_rrm[2]) / np.cos(self.rEst_D_rrm[0])

        # Velocity update
        self.vEst_L_mps += vDelta_L

        # Attitude correction
        qDelta_BL = np.array([1.0, quatDelta[0], quatDelta[1], quatDelta[2]])
        self.qEst_BL = Kinematics.QuatMult(self.qEst_BL, qDelta_BL)

        # Update biases from states
        self.aBias_mps2 += aBiasDelta
        self.wBias_rps += wBiasDelta


#%%
d2r = np.pi / 180.0
r2d = 1 / d2r

def NavEKF15_All(time_s, rB_D_ddm, vB_L_mps, wB_B_rps, aB_B_mps2):
    navFilt = NavEKF15()
    navFilt.Configure()

    indxFinite = np.where(np.isfinite(rB_D_ddm[0]) & np.isfinite(vB_L_mps[0]))[0]

    ## Initialize, with first GPS data point
    w0_B_rps = wB_B_rps[:,indxFinite[0]].copy()
    a0_B_mps2 = aB_B_mps2[:,indxFinite[0]].copy()
    p0_D_rrm = rB_D_ddm[:,indxFinite[0]].copy() * np.asarray([d2r, d2r, 1])
    v0_L_mps = vB_L_mps[:,indxFinite[0]].copy()
    navFilt.Initialize(w0_B_rps, a0_B_mps2, p0_D_rrm, v0_L_mps)

    aEst_B_mps2 = np.nan * np.ones_like(wB_B_rps)
    wEst_B_rps = np.nan * np.ones_like(wB_B_rps)
    vEst_L_mps = np.nan * np.ones_like(wB_B_rps)
    sEst_BL_rad = np.nan * np.ones_like(wB_B_rps)
    rEst_D_ddm = np.nan * np.ones_like(wB_B_rps)
    for i in range(indxFinite[0], time_s.shape[-1]):

        wMeas_B_rps = wB_B_rps[:,i].copy()
        aMeas_B_mps2 = aB_B_mps2[:,i].copy()
        rMeas_D_rrm = rB_D_ddm[:,i].copy() * np.asarray([d2r, d2r, 1])
        vMeas_L_mps = vB_L_mps[:,i].copy()
        dt_s = time_s[i] - time_s[i-1]

        measUpdt = np.isfinite(rMeas_D_rrm[0]) & np.isfinite(vMeas_L_mps[0])
        timeUpdt = np.isfinite(wMeas_B_rps[0]) & np.isfinite(aMeas_B_mps2[0])
        if timeUpdt:
            aEst_B_mps2[:,i], wEst_B_rps[:,i], vEst_L_mps[:,i], sEst_BL_rad[:,i], rEst_D_rrm = navFilt.Update(measUpdt, wMeas_B_rps, aMeas_B_mps2, rMeas_D_rrm, vMeas_L_mps, dt_s)

            rEst_D_ddm[:,i] = rEst_D_rrm * np.asarray([r2d, r2d, 1])

    return aEst_B_mps2, wEst_B_rps, vEst_L_mps, sEst_BL_rad, rEst_D_ddm