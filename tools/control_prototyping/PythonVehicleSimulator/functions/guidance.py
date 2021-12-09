# -*- coding: utf-8 -*-
"""
Guidance algorithms.

Reference: T. I. Fossen (2021). Handbook of Marine Craft Hydrodynamics and
Motion Control. 2nd. Edition, Wiley. 
URL: www.fossen.biz/wiley

Author:     Thor I. Fossen
Revisions: 7 dec 2021 added LPfilter method (Mohamed Magdy)
           6 dec 2021 added ILOS method     (Mohamed Magdy)
"""

import numpy as np
import math

# [x_d,v_d,a_d] = refModel3(x_d,v_d,a_d,r,wn_d,zeta_d,v_max,sampleTime) is a 3-order 
# reference  model for generation of a smooth desired position x_d, velocity |v_d| < v_max, 
# and acceleration a_d. Inputs are natural frequency wn_d and relative damping zeta_d.
def refModel3(x_d, v_d, a_d, r, wn_d, zeta_d, v_max, sampleTime):

    # Velocity saturation
    if (v_d > v_max):
        v_d = v_max
    elif (v_d < -v_max): 
        v_d = -v_max
    
    # desired "jerk"
    j_d = wn_d**3 * (r -x_d) - (2*zeta_d+1) * wn_d**2 * v_d - (2*zeta_d+1) * wn_d * a_d

   # Forward Euler integration
    x_d += sampleTime * v_d             # desired position
    v_d += sampleTime * a_d             # desired velocity
    a_d += sampleTime * j_d             # desired acceleration 
    
    return x_d, v_d, a_d

# eta_d = LPfiler(eta_reference, eta_d, T, sampleTime) computes smooth desired pose state
# using low-pass filter
# Inputs:
#   eta_ref,  reference state
#   eta_d,    desired state
#   T,        time contant vector for each state    
def LPfilter(eta_ref, eta_d, T, sampleTime):
    eta_d += sampleTime * (eta_ref - eta_d) / T
    return eta_d

# psi_d = ILOS(x,y,Delta,kappa,h,R_switch,wpt,psi) computes the desired 
# heading angle when the path is straight lines going through the waypoints
# (wpt.pos.x, wpt.pos.y). The desired heading angle psi_d is computed using 
# the ILOS guidance law by Børhaug et al. (2008)
# Inputs:   
#   (x,y), craft North-East positions (m)
#   Delta, positive look-ahead distance (m)
#   kappa, positive integral gain constant, Ki = kappa * Kp
#   U, speed of the craft (m/s)
#   h, sampling time (s)
#   R_switch, go to next waypoint when the along-track distance x_e 
#             is less than R_switch (m)
#   wpt.pos.x = [x1, x2,..,xn]' array of waypoints expressed in NED (m)
#   wpt.pos.y = [y1, y2,..,yn]' array of waypoints expressed in NED (m)
#   psi, current heading angle
#    y_int
# Output:  
#    psi_d: desired heading angle (rad)    
def ILOS(x,y,Delta,kappa,h,R_switch,wpt,psi,y_int):
    # integral state
    y_int = y_int
    # number of waypoints to follow
    n  = wpt.shape[1]
    xk = wpt[0][0]
    yk = wpt[1][0] 
    if n > 1:
        xk_next = wpt[0][1]
        yk_next = wpt[1][1]
    # Compute the desired course angle        
    pi_p = np.atan2(yk_next-yk,xk_next-xk);  # path-tangential angle w.r.t. to North        
    # along-track and cross-track errors (x_e, y_e) expressed in NED
    x_e =  (x-xk) * np.cos(pi_p) + (y-yk) * np.sin(pi_p)
    y_e = -(x-xk) * np.sin(pi_p) + (y-yk) * np.cos(pi_p)
    Kp = 1/Delta
    Ki = kappa * Kp
    psi_d = pi_p - np.atan(Kp * y_e + Ki * y_int)
    # kinematic differential equation
    Dy_int = Delta * y_e / ( Delta**2 + (y_e + kappa * y_int)**2)
    # Euler integration
    y_int = y_int + h * Dy_int;
    return psi_d , y_int
    