# -*- coding: utf-8 -*-
"""
otter.py: 
    Class for the Maritime Robotics Otter USV, www.maritimerobotics.com. 
    The length of the USV is L = 2.0 m. The constructors are:

    otter()                                          
        Step inputs for propeller revolutions n1 and n2
    otter('headingAutopilot',psi_d,V_current,beta_current,tau_X)  
       Heading autopilot with options:
          psi_d: desired yaw angle (deg)
          V_current: current speed (m/s)
          beta_c: current direction (deg)
          tau_X: surge force, pilot input (N)
        
Methods:
    
[nu,u_actual] = dynamics(eta,nu,u_actual,u_control,sampleTime) returns 
    nu[k+1] and u_actual[k+1] using Euler's method. The control inputs are:

    u_control = [ n1 n2 ]' where 
      n1: propeller shaft speed, left (rad/s)
      n2: propeller shaft speed, right (rad/s)

u = headingAutopilot(eta,nu,sampleTime) 
    PID controller for automatic heading control based on pole placement.

u = stepInput(t) generates propeller step inputs.

[n1, n2] = controlAllocation(tau_X, tau_N)     
    Control allocation algorithm.
    
References: 
  T. I. Fossen (2021). Handbook of Marine Craft Hydrodynamics and Motion 
     Control. 2nd. Edition, Wiley. 
     URL: www.fossen.biz/wiley            

Author:     Thor I. Fossen
"""
import numpy as np
import math
from functions.control import PIDpolePlacement
from functions.gnc import Smtrx,Hmtrx,m2c,crossFlowDrag,sat

# Class Vehicle
class otter:
    """
    otter()                          Step inputs for n1 and n2
    otter('headingAutopilot',psi_d)  Heading autopilot, desired yaw angle (deg)
    """        
    def __init__(self, controlSystem = 'stepInput', 
                 r = 0, V_current = 0, beta_current = 0, tau_X = 120):         
                
        if (controlSystem == 'headingAutopilot'):
            self.controlDescription = 'Heading autopilot, psi_d = ' \
            + str(r) + ' (deg)'       
        else:  
            self.controlDescription = 'Step inputs for n1 and n2'
            controlSystem = 'stepInput'  
      
        self.ref = r
        self.V_c = V_current
        self.beta_c = beta_current
        self.controlMode = controlSystem
        self.tauX = tau_X                       # surge force (N)
                    
        # Initialize the Otter USV model
        self.T_n = 1.0                          # propeller time constants (s)          
        self.L = 2.0                            # Length (m)
        self.B = 1.08                           # beam (m)           
        self.nu = np.array([0, 0, 0, 0, 0, 0], float) # velocity vector    
        self.u_actual = np.array([0, 0], float)   # propeller revolution states
        self.name = "Otter USV"     
        
        self.controls = [ 
            'Left propeller shaft speed (rad/s)', 
             'Right propeller shaft speed (rad/s)' ]
        self.dimU = len(self.controls)  

        # Constants   
        g   = 9.81                            # acceleration of gravity (m/s^2)
        rho = 1025                            # density of water
        
        m = 55.0                              # mass (kg)
        mp = 25.0                             # Payload (kg) 
        self.m_total = m + mp
        rp = np.array([0,   0, -0.35],float)  # location of payload (m)        
        rg = np.array([0.2, 0, -0.2],float)   # CG for hull only (m)  
        rg = (m*rg + mp*rp) / (m + mp)        # CG corrected for payload 
        self.S_rg = Smtrx(rg) 
        self.H_rg = Hmtrx(rg)
        self.S_rp = Smtrx(rp)        

        R44 = 0.4 * self.B                    # radii of gyration (m)
        R55 = 0.25 * self.L
        R66 = 0.25 * self.L
        T_yaw = 1.0                            # time constant in yaw (s)
        Umax = 6 * 0.5144                     # max forward speed (m/s)

        # Data for one pontoon
        self.B_pont  = 0.25  # beam of one pontoon (m)
        y_pont  = 0.395      # distance from centerline to waterline centroid (m)
        Cw_pont = 0.75       # waterline area coefficient (-)
        Cb_pont = 0.4        # block coefficient, computed from m = 55 kg
        
        # Inertia dyadic, volume displacement and draft
        nabla = (m + mp) / rho                                    # volume
        self.T = nabla / (2 * Cb_pont * self.B_pont*self.L)       # draft      
        Ig_CG = m * np.diag( np.array( [R44**2, R55**2, R66**2]) )
        self.Ig = Ig_CG - m * self.S_rg @ self.S_rg - mp * self.S_rp @ self.S_rp  

        # Experimental propeller data including lever arms
        self.l1 = -y_pont                    # lever arm, left propeller (m)
        self.l2 = y_pont                     # lever arm, right propeller (m)
        self.k_pos = 0.02216/2               # Positive Bollard, one propeller 
        self.k_neg = 0.01289/2               # Negative Bollard, one propeller 
        self.n_max =  math.sqrt((0.5*24.4 * g)/self.k_pos) # max. prop. rev.
        self.n_min = -math.sqrt((0.5*13.6 * g)/self.k_neg) # min. prop. rev. 

        # MRB_CG = [ (m+mp) * I3  O3      (Fossen 2021, Chapter 3)
        #               O3       Ig ]        
        MRB_CG = np.zeros( (6,6) )
        MRB_CG[0:3, 0:3] = (m + mp) * np.identity(3) 
        MRB_CG[3:6, 3:6] = self.Ig
        MRB = self.H_rg.T @ MRB_CG @ self.H_rg

        # Hydrodynamic added mass (best practice)  
        Xudot = -0.1 * m   
        Yvdot = -1.5 * m
        Zwdot = -1.0 * m
        Kpdot = -0.2 * self.Ig[0,0]
        Mqdot = -0.8 * self.Ig[1,1]
        Nrdot = -1.7 * self.Ig[2,2]        
        
        self.MA = -np.diag([Xudot, Yvdot, Zwdot, Kpdot, Mqdot, Nrdot]) 
        
        # System mass matrix
        self.M = MRB + self.MA
        self.Minv = np.linalg.inv(self.M)
        
        # Hydrostatic quantities (Fossen 2021, Chapter 4)
        Aw_pont = Cw_pont * self.L * self.B_pont # waterline area, one pontoon 
        I_T = 2 * (1/12)*self.L * self.B_pont**3 * \
            (6*Cw_pont**3/((1+Cw_pont)*(1+2*Cw_pont))) + 2 * Aw_pont * y_pont**2
        I_L = 0.8 * 2 * (1/12) * self.B_pont * self.L**3
        KB = (1/3) * ( 5 * self.T / 2 - 0.5 * nabla / (self.L * self.B_pont) )
        BM_T = I_T/nabla        # BM values
        BM_L = I_L/nabla
        KM_T = KB + BM_T        # KM values
        KM_L = KB + BM_L
        KG = self.T - rg[2]
        GM_T = KM_T - KG        # GM values
        GM_L = KM_L - KG

        G33 = rho * g * (2 * Aw_pont)      # spring stiffness
        G44 = rho * g * nabla * GM_T
        G55 = rho * g * nabla * GM_L
        G_CF = np.diag([0, 0, G33, G44, G55, 0])  # spring stiff. matrix in CF
        LCF = -0.2
        H = Hmtrx(np.array([LCF, 0.0, 0.0]))    # transform G_CF from CF to CO 
        self.G = H.T @ G_CF @ H

        # Natural frequencies
        w3 = math.sqrt( G33 / self.M[2,2] )       
        w4 = math.sqrt( G44 / self.M[3,3] )
        w5 = math.sqrt( G55 / self.M[4,4] )

        # Linear damping terms (hydrodynamic derivatives)
        Xu = -24.4 * g / Umax             # specified using the maximum speed        
        Yv = 0
        Zw = -2 * 0.3 * w3 * self.M[2,2]  # specified using relative damping
        Kp = -2 * 0.2 * w4 * self.M[3,3]
        Mq = -2 * 0.4 * w5 * self.M[4,4]
        Nr = -self.M[5,5] / T_yaw         #specified by the time constant T_yaw       
        
        self.D = -np.diag([Xu, Yv, Zw, Kp, Mq, Nr])   
  
        # Trim: theta = -7.5 deg corresponds to 13.5 cm less height aft 
        self.trim_moment = 0
        self.trim_setpoint = 280;   
        
        # Propeller configuration/input matrix 
        B = self.k_pos * \
            np.array([
                [1, 1],
                [-self.l1, -self.l2] ])
        self.Binv = np.linalg.inv(B)
    
        # Heading autopilot
        self.e_int = 0           # integral state   
        self.wn = 1.2            # PID pole placement
        self.zeta = 0.8
        
        # Reference model
        self.r_max = 10 * math.pi / 180   # maximum yaw rate 
        self.psi_d = 0           # angle, angular rate and angular acc. states
        self.r_d = 0
        self.a_d = 0
        self.wn_d = self.wn / 5  # desired natural frequency in yaw
        self.zeta_d = 1          # desired relative damping ratio
        

    def dynamics(self,eta,nu,u_actual,u_control,sampleTime):
        """
        [nu,u_actual] = dynamics(eta,nu,u_actual,u_control,sampleTime) integrates 
        the Otter USV equations of motion using Euler's method.
        """   

        # Input vector
        n = np.array( [u_actual[0], u_actual[1]] )  

        # Current velocities
        u_c = self.V_c * math.cos(self.beta_c - eta[5])  # current surge vel.
        v_c = self.V_c * math.sin(self.beta_c - eta[5])  # current sway vel.

        nu_c = np.array([u_c,v_c,0,0,0,0],float)     # current velocity vector
        nu_r = nu - nu_c                             # relative velocity vector
        
        # Rigid body and added mass Coriolis and centripetal matrices 
        # CRB_CG = [ (m+mp) * Smtrx(nu2)          O3   (Fossen 2021, Chapter 6)
        #              O3                   -Smtrx(Ig*nu2)  ] 
        CRB_CG = np.zeros( (6,6) )
        CRB_CG[0:3, 0:3] =  self.m_total * Smtrx(nu[3:6])
        CRB_CG[3:6, 3:6] = -Smtrx( np.matmul(self.Ig,nu[3:6]) )    
        CRB = self.H_rg.T @ CRB_CG @ self.H_rg  # transform CRB from CG to CO 
 
        CA = m2c(self.MA, nu_r)
        CA[5,0] = 0 # assume that the Munk moment in yaw can be neglected
        CA[5,1] = 0 # if nonzero, must be balanced by adding nonlinear damping

        C = CRB + CA      
        
        # Ballast
        g_0 = np.array([0.0, 0.0, 0.0, 0.0, self.trim_moment, 0.0])   

        # Control forces and moments - with propeller revolution saturation 
        thrust = np.zeros(2)
        for i in range(0,2):
            
            n[i] = sat(n[i],self.n_min,self.n_max) # saturation, physical limits

            if n[i] > 0:                                    # positive thrust              
                thrust[i] = self.k_pos * n[i] * abs(n[i])  
            else:                                           # negative thrust 
                thrust[i] = self.k_neg * n[i] * abs(n[i]) 

        # Control forces and moments
        tau = np.array( [thrust[0] + thrust[1], 0, 0, 0, 0, 
                       -self.l1 * thrust[0] - self.l2 * thrust[1] ])

        # Hydrodynamic linear damping + nonlinear yaw damping
        tau_damp = -np.matmul(self.D, nu_r)  
        tau_damp[5] = tau_damp[5] - 10 * self.D[5,5] * abs(nu_r[5]) * nu_r[5]

        # State derivatives (with dimension)
        tau_crossflow = crossFlowDrag(self.L,self.B_pont,self.T,nu_r)
        sum_tau = tau + tau_damp + tau_crossflow - np.matmul(C,nu_r) \
            - np.matmul(self.G,eta) - g_0
       
        nu_dot = np.matmul(self.Minv, sum_tau)   # USV dynamics
        n_dot = (u_control - n) / self.T_n       # propeller dynamics
        trim_dot = (self.trim_setpoint - self.trim_moment) / 5 # trim dynamics

        # Forward Euler integration [k+1]
        nu = nu + sampleTime * nu_dot
        n  = n  + sampleTime * n_dot
        self.trim_moment = self.trim_moment + sampleTime * trim_dot

        u_actual = np.array(n,float)          
        
        return nu, u_actual        
    

    def controlAllocation(self,tau_X, tau_N):
        """
        [n1, n2] = controlAllocation(tau_X, tau_N) 
        """
        tau = np.array([tau_X, tau_N])            # tau = B * u_alloc
        u_alloc = np.matmul(self.Binv, tau)       # u_alloc = inv(B) * tau
        
        # u_alloc = abs(n) * n --> n = sign(u_alloc) * sqrt(u_alloc)
        n1 = np.sign(u_alloc[0]) * math.sqrt( abs(u_alloc[0]) )
        n2 = np.sign(u_alloc[1]) * math.sqrt( abs(u_alloc[1]) )
        
        return n1, n2

    
    def headingAutopilot(self,eta,nu,sampleTime):
        """
        u = headingAutopilot(eta,nu,sampleTime) is a PID controller 
        for automatic heading control based on pole placement.
        
        tau_N = (T/K) * a_d + (1/K) * rd 
               - Kp * ( ssa( psi-psi_d ) + Td * (r - r_d) + (1/Ti) * z )
        
        """                  
        psi = eta[5]                # yaw angle
        r = nu[5]                   # yaw rate
        e_psi = psi - self.psi_d    # yaw angle tracking error
        e_r   = r - self.r_d        # yaw rate tracking error
        psi_ref = self.ref * math.pi / 180  # yaw angle setpoint
    
        wn = self.wn                # PID natural frequency
        zeta = self.zeta            # PID natural relative damping factor
        wn_d = self.wn_d            # reference model natural frequency
        zeta_d = self.zeta_d        # reference model relative damping factor

        m = 41.4             # moment of inertia in yaw including added mass
        T = 1
        K = T/m   
        d = 1/K   
        k = 0

        # PID feedback controller with 3rd-order reference model
        tau_X = self.tauX
                
        [tau_N, self.e_int, self.psi_d, self.r_d, self.a_d] = \
            PIDpolePlacement( self.e_int, e_psi, e_r, self.psi_d, self.r_d, self.a_d, \
            m, d, k, wn_d, zeta_d, wn, zeta, psi_ref, self.r_max, sampleTime )

        [n1, n2] = self.controlAllocation(tau_X, tau_N)
        u_control = np.array([n1, n2],float)   
         
        return u_control     
    
    def stepInput(self,t):
        """
        u = stepInput(t) generates propeller step inputs.
        """          
        n1 = 100        # rad/s
        n2 = 80
        
        if (t > 30 and t < 100):
            n1 = 80
            n2 = 120
        else: 
            n1 = 0
            n2 = 0
  
        u_control = np.array([n1, n2],float)   
         
        return u_control              
    
        

    