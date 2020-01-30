import math
import numpy as np

import navpy

# values for our Talon
s = 6.62                        # square feet
L = 6.0                         # lbs (need to weigh with camera!)
d = 0.002308                    # density @ 1000' on a standard day

# values for our Mjolnir
s = 8.28                        # square feet
L = 16.0                        # lbs (need to weigh with camera!)
d = 0.002308                    # density @ 1000' on a standard day

r2d = 180.0 / math.pi
mps2kt = 1.94384
kt2mps = 1.0 / mps2kt
ft2m = 0.3048
m2ft = 1.0 / ft2m
g = -9.81

af = 0
lff = 0

cl_array = []
alpha_array = []

def update(navpt, airpt, imupt, wn, we):
    global af
    global lff
    global cl_array
    global a_array

    if not 'psi' in navpt:
        return
    
    C_N2B = navpy.angle2dcm(navpt['psi'], navpt['the'], navpt['phi'])
    vel = np.array([navpt['vn'] + wn, navpt['ve'] + we, navpt['vd']])
    # mag = np.linalg.norm(vel)
    # wind = np.linalg.norm(np.array([wn, we]))
    # print('vel:', vel, "(%.2f) %.2f %.2f" % (airpt.airspeed / mps2kt, mag, wind) )
    vel_body = C_N2B.dot(vel)
    beta = math.atan2(vel_body[1], vel_body[0]) * r2d
    bx = np.linalg.norm(np.array([vel_body[0], vel_body[1]]))
    alpha = math.atan2(vel_body[2], bx) * r2d
    # print(imupt.time, 'vel_body:', vel_body, "beta: %.2f alpha: %.2f" % (beta, alpha))

    v = airpt['airspeed'] * kt2mps * m2ft # feet per second
    lf = imupt['az'] / g
    CL = 2.0 * (L * lf) / (d * v*v * s) # Coefficient of Lift

    # test fit
    af = 0.99 * af + 0.01 * alpha
    lff = 0.99 * lff + 0.01 * lf
    
    CL_est = 0.000220868*af + 0.00131579
    tmp = lff / CL_est
    if tmp < 0: tmp = 0
    v_est = math.sqrt(tmp)
    if alpha >= 0 and CL > 0.0 and CL < 1.5:
        cl_array.append(CL)
        alpha_array.append(alpha)
        #print("alpha: %.2f lf: %.2f CL: %.06f v: %.1f (%.1f)" % (alpha, lf, CL, v, v_est))
        # print("%.3f, %.3f, %.06f, %.1f, %.1f" % (af, lff, CL, v, v_est))
    
    return alpha, beta, CL

def gen_stats():
    global cl_array
    global alpha_array
    if len(cl_array):
        cl_array = np.array(cl_array)
        alpha_array = np.array(alpha_array)
        cl_cal, res, _, _, _ = np.polyfit( alpha_array, cl_array, 2, full=True )
        print('alpha/CL function coeffs:', cl_cal)
 
