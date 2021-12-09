# -*- coding: utf-8 -*-
"""
Control methods.

Reference: T. I. Fossen (2021). Handbook of Marine Craft Hydrodynamics and
Motion Control. 2nd. Edition, Wiley. 
URL: www.fossen.biz/wiley

Author:     Thor I. Fossen
Revisions: 7 dec 2021 added DPpolePlacement6 method (Mohamed Magdy)
           8 dec 2021 added LQR method              (Mohamed Magdy)
"""

import numpy as np
import scipy.linalg
import math
from functions.guidance import refModel3, LPfilter
from functions.gnc import ssa, Rzyx, Tzyx, eulerang, Dmtx, m2c, gvect

# SISO PID pole placement
def PIDpolePlacement(e_int,e_x,e_v, \
    x_d,v_d,a_d,m,d,k,wn_d,zeta_d,wn,zeta,r,v_max,sampleTime):    
      
    # PID gains based on pole placement
    Kp = m * wn**2 - k
    Kd = m * 2 * zeta * wn - d
    Ki = (wn/10) * Kp

    # PID control law
    u = -Kp * e_x - Kd * e_v - Ki * e_int

    # Integral error, Euler's method
    e_int += sampleTime * e_x
    
    # 3rd-order reference model for smooth position, velocity and acceleration
    [x_d, v_d, a_d] = refModel3(x_d, v_d, a_d, r, wn_d, zeta_d, v_max, sampleTime)

    return u, e_int, x_d, v_d, a_d  


# MIMO nonlinear PID pole placement [3DOF] 
def DPpolePlacement(e_int,M3,D3,eta3,nu3, \
    x_d,y_d,psi_d,wn,zeta,eta_ref,sampleTime):  

    # PID gains based on pole placement
    Kp = wn @ wn @ M3
    Kd = 2.0 * zeta @ wn @ M3 - D3
    Ki = (1.0 / 10.0) * wn @ Kp
        
    # DP control law - setpoint regulation
    e = eta3 - eta_ref
    e[2] = ssa( e[2] )
    R = Rzyx( 0.0, 0.0, eta3[2] )
    tau = -np.matmul( (R.T @ Kp), e) \
          -np.matmul( (R.T @ Kd @ R), nu3) \
          -np.matmul( (R.T @ Ki), e_int)

    # Low-pass filters, Euler's method
    T = 5.0 * np.array([ 1/wn[0][0], 1/wn[1][1], 1/wn[2][2] ]) 
    x_d   += sampleTime  * ( eta_ref[0] - x_d )  / T[0]
    y_d   += sampleTime  * ( eta_ref[1] - y_d )  / T[1]
    psi_d += sampleTime * ( eta_ref[2] - psi_d ) / T[2]
    
    return tau, e_int, x_d, y_d, psi_d

# MIMO nonlinear PID pole placement [6DOF] 
def DPpolePlacement6(e_int,M,D,eta,nu, \
    eta_d,wn,zeta,eta_ref,sampleTime):  

    # PID gains based on pole placement
    Kp = wn @ wn @ M
    Kd = 2.0 * zeta @ wn @ M - D
    Ki = (1.0 / 10.0) * wn @ Kp
        
    # DP control law - setpoint regulation
    # Low-pass filters, Euler's method
    T = 5.0 * np.array([ 1/wn[0][0], 1/wn[1][1], 1/wn[2][2], 1/wn[3][3], 1/wn[4][4], 1/wn[5][5] ])
    eta_d = LPfilter(eta_ref, eta_d, T, sampleTime)

    e = eta - eta_d
    e[3] = ssa( e[3] )
    e[4] = ssa( e[4] )
    e[5] = ssa( e[5] )
    
    R = Rzyx( eta[3], eta[4], eta[5] )
    T = Tzyx(eta[3], eta[4])
    # Jacobi matrix for body to world transformation
    J = np.zeros(shape=(6,6), dtype=float)
    J[0:3, 0:3] = R
    J[3:6, 3:6] = T
    # Inverse of the Jacobi matrix for world to body transformation
    Jinv = np.zeros(shape=(6, 6), dtype=float)
    Jinv[0:3, 0:3] = R.T
    Jinv[3:6, 3:6] = np.linalg.inv(T)
    tau = -np.matmul( (Jinv @ Kp), e) \
          -np.matmul( (Jinv @ Kd @ J), nu) \
          -np.matmul( (Jinv @ Ki), e_int)
 


    # Integral error, Euler's method
    e_int +=  sampleTime * e
        
    return tau, e_int, eta_d

def LQR(M,Dlin,Dquad,r_bb,r_bg,weight,eta,nu, \
    eta_d,eta_ref,nu_d,sampleTime,wn,Q,R):
    """Solve the discrete time lqr controller.
 
    x[k+1] = A x[k] + B u[k]
 
    cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
    """
    # ref Bertsekas, p.151 
    # Constructing LTV model 
    # state x [eta, nu] 12x1 vector
    x = np.hstack((eta,nu)).reshape(-1,1)
    # Low-pass filters, Euler's method
    T = 5.0 * np.array([ 1/wn[0][0], 1/wn[1][1], 1/wn[2][2], 1/wn[3][3], 1/wn[4][4], 1/wn[5][5] ]) 
    eta_d = LPfilter(eta_ref, eta_d, T, sampleTime)
    # Compute desired state xd
    x_d = np.hstack((eta_d, np.zeros(6))).reshape(-1,1)
    

    # System matrix A 12x12
    A = np.zeros(shape=(12,12), dtype=float)
    C = m2c(M,nu)
    D = Dmtx(Dlin,Dquad,nu)
    J = eulerang(eta)
    A[0:6 , 6:12] = J
    A[6:12, 6:12] = -1 * np.linalg.inv(M) * (D + C)
    # Input matrix B 12x6
    B = np.zeros(shape=(12,6), dtype=float)
    B[6:12, 0:6] = np.linalg.inv(M)
    
    # first, try to solve the ricatti equation
    P = np.matrix(scipy.linalg.solve_continuous_are(A, B, Q, R)) 
    K = -1 * scipy.linalg.inv(R) @ B.T @ P
    # compute the LQR gain
    # eigVals, eigVecs = scipy.linalg.eig(A-B*K)
    # error state
    e = x - x_d
    # saturating
    e[3] = ssa( e[3] )
    e[4] = ssa( e[4] )
    e[5] = ssa( e[5] )    
    tau =  K * e
 
    return  tau,eta_d
