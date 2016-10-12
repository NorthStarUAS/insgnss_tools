import math
import numpy as np
import os

# Import the structures
import cdefs
import pydefs

# Import these ctypes for proper declaration of cdefs.py structures
import ctypes
# Abbreviate these ctypes commands
POINTER = ctypes.POINTER
byref   = ctypes.byref

class filter():
    d2r = math.pi / 180.0
    r2d = 1 / d2r
    
    def __init__(self):
        # Load compilied `.so` file.
        self.sharedobj = ctypes.CDLL(os.path.abspath('../build/src/MadgwickAHRS/.libs/libMadgwickAHRS.so'))
        
        # Declare inputs to the init_nav function
        self.sharedobj.MadgwickAHRSupdate.argtypes = \
            [ctypes.c_double, # time
             ctypes.c_float, ctypes.c_float, ctypes.c_float, # gx, gy, gz (rad/sec)
             ctypes.c_float, ctypes.c_float, ctypes.c_float, # ax, ay, az
             ctypes.c_float, ctypes.c_float, ctypes.c_float] # mx, my, mz
        self.sharedobj.MadgwickAHRSupdate.restype = None # void return

    # Define a convenient function to access global float variables
    def get_quat(self):
        """ Returns q0, q1, q2, q3"""
        q0 = ctypes.c_float.in_dll(self.sharedobj, "q0").value
        q1 = ctypes.c_float.in_dll(self.sharedobj, "q1").value
        q2 = ctypes.c_float.in_dll(self.sharedobj, "q2").value
        q3 = ctypes.c_float.in_dll(self.sharedobj, "q3").value
        return [q0, q1, q2, q3]
    
    def quat2euler(self, q):
        #QUATERN2EULER Converts a quaternion orientation to ZYX Euler angles
        #
        #   q = quatern2euler(q)
        #
        #   Converts a quaternion orientation to ZYX Euler angles where phi is a
        #   rotation around X, theta around Y and psi around Z.
        #
        #   For more information see:
        #   http://www.x-io.co.uk/node/8#quaternions
        #
        #   Date          Author          Notes
        #   30/10/2015    Hamid Mokhtarzadeh    Migrate to Python
        #   27/09/2011    SOH Madgwick    Initial release
        #
        drl = len(q)
        R   = np.zeros((3,3))
        R[0,0] = 2.*q[0]**2-1+2.*q[1]**2;
        R[1,0] = 2.*(q[1]*q[2]-q[0]*q[3]);
        R[2,0] = 2.*(q[1]*q[3]+q[0]*q[2]);
        R[2,1] = 2.*(q[2]*q[3]-q[0]*q[1]);
        R[2,2] = 2.*q[0]**2-1+2*q[3]**2;

        phi = np.arctan2(R[2,1], R[2,2] );
        theta = -np.arctan(R[2,0] / np.sqrt(1-R[2,0]**2) );
        psi = np.arctan2(R[1,0], R[0,0] );

        euler = (phi, theta, psi)
        return euler

    def init(self, imu, gps):
        self.P = np.zeros(15)
        self.stateInnov = np.zeros(6)
        return self.update(imu, gps)

    def update(self, imu, gps):
        self.sharedobj.MadgwickAHRSupdate(imu.time,
                                         imu.p, -imu.q, -imu.r,
                                         imu.ax, -imu.ay, -imu.az,
                                         imu.hx, -imu.hy, -imu.hz)
        # self.sharedobj.MadgwickAHRSupdate(imu.time,
        #                                   imu.p, imu.q, -imu.r,
        #                                   imu.ax, imu.ay, -imu.az,
        #                                   0.0, 0.0, 0.0)
        q_b2n = np.array(self.get_quat())
        q_n2b = q_b2n.copy()
        q_n2b *= -1
        eul = self.quat2euler(q_n2b)

        nav = pydefs.INSGPS(1, # fixme: valid/init
                            imu.time,
                            [gps.lat*self.d2r, gps.lon*self.d2r, gps.alt],
                            [gps.vn, gps.ve, gps.vd],
                            [eul[2], eul[1], eul[0]],
                            [0.0, 0.0, 0.0], # accel biases
                            [0.0, 0.0, 0.0], # gyro biases
                            self.P,
                            self.stateInnov)
        return nav

    def close(self):
        pass
