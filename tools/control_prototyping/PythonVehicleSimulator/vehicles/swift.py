# -*- coding: utf-8 -*-
"""
Created on Sun Dec  5 03:55:21 2021

@author: mohamedd
"""
"""
   swift.py 
        Class for underwater vehicle swift

    swift()                                      
        Step inputs for propeller revolutions n1, n2, n3, n4, n5, n6, n7 and n8 (RPM)
    swift('DPcontrol',x_d,y_d,psi_d,V_c,beta_c) DP control system
        x_d: desired x position (m)
        y_d: desired y position (m)
        psi_d: desired yaw angle (deg)
        V_c: current speed (m/s)
        beta_c: current direction (deg)
        
Methods:
    
[nu,u_actual] = dynamics(eta,nu,u_actual,u_control,sampleTime)
    returns nu[k+1] and u_actual[k+1] using Euler's method. 
    The control inputs are:

    u_control = n  (RPM)
    
    Indicies of the vertical thrusters are as follows:
                         
           | +x-axis
           |
   n2 o    |        o n1
           |
           + -------------------->
                                 +y-axis
   n4 o             o n3
   
    Indicies of the horizontal thrusters are as follows:
        
              | +x-axis
           |
   n6 o    |        o n5
           |
           + -------------------->
                                 +y-axis
   n8 o             o n7
   
   
   
    n = [ #1 vertical   thruster (RPM)
          #2 vertical   thruster (RPM)             
          #3 vertical   thruster (RPM)
          #4 vertical   thruster (RPM)            
          #5 horizontal thruster (RPM)
          #6 horizontal thruster (RPM)
          #7 horizontal thruster (RPM)
          #8 horizontal thruster (RPM)

u_alloc = controlAllocation(tau)
    Control allocation based on the pseudoinverse                 

n = DPcontrol(eta,nu,sampleTime)
    Nonlinear PID controller for DP based on pole placement.    

n = stepInput(t) generates propellers step inputs.
    
Reference: 
  T. I. Fossen (2021). Handbook of Marine Craft Hydrodynamics and Motion 
     Control. 2nd. Edition, Wiley. 
     URL: www.fossen.biz/wiley            

Author:     Thor I. Fossen
"""
import numpy as np
import math
from functions.control import DPpolePlacement6, LQR
from functions.gnc import sat, m2c, gvect, Dmtx

# Class Vehicle
class swift:
    """
    swift()                                      Step inputs tau_X, tay_Y, tau_N
    swift('DPcontrol',x_d,y_d,z_d,phi_d,theta_d,psi_d,V_c,beta_c)  DP control system
    """        
    def __init__(self, controlSystem = 'stepInput', 
                 r_x = 0, r_y = 0, r_z = 0, r_roll = 0, r_pitch = 0, r_yaw = 0, V_current = 0, beta_current = 0):         
                
        if (controlSystem == 'DPcontrol1'):
            self.controlDescription = 'Nonlinear PID DP control (x_d, y_d,zd, phi_d, theta_d psi_d) = (' \
            + str(r_x) + ' m, ' + str(r_y) + ' m, ' + str(r_z) + ' m, ' \
            + str(r_roll) + ' deg, ' + str(r_pitch) + ' deg, ' + str(r_yaw) + ' deg)'
             
        elif (controlSystem == 'DPcontrol2'):
            self.controlDescription = 'Optimal DP control (x_d, y_d,zd, phi_d, theta_d psi_d) = (' \
            + str(r_x) + ' m, ' + str(r_y) + ' m, ' + str(r_z) + ' m, ' \
            + str(r_roll) + ' deg, ' + str(r_pitch) + ' deg, ' + str(r_yaw) + ' deg)'
            
        else:  
            self.controlDescription = 'Step inputs for n = [n1, n2, n3, n4, n5, n6, n7, n8]'
            controlSystem = 'stepInput'  
      
        self.ref = np.array([ r_x, r_y, r_z, (math.pi/180) * r_roll, (math.pi/180) * r_pitch, (math.pi/180) * r_yaw], float)
        self.V_c = V_current
        self.beta_c = beta_current
        self.controlMode = controlSystem
                    
        # Initialize the semisub model 
        self.L = 0.75                # Length (m) 
        self.volume = 0.0364 
        self.mass = 35.5              
        self.T_n = 0.2               # propeller revolution time constants  
        self.n_max = np.array([ 
            3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000],float)      # RPM saturation limits (N)                                                   
        self.nu = np.array([0, 0, 0, 0, 0, 0], float)  # velocity vector    
        self.u_actual = np.array([0, 0, 0, 0, 0, 0, 0, 0], float)  # RPM inputs
        self.name = "swift"           

        self.controls = [ 
            '#1 Vertical thruster (RPM)', 
            '#2 Vertical thruster (RPM)',             
            '#3 Vertical thruster (RPM)',
            '#4 Vertical thruster (RPM)',            
            '#5 Horizontal thruster (RPM)',
            '#6 Horizontal thruster (RPM)',
            '#7 Horizontal thruster (RPM)',
            '#8 Horizontal thruster (RPM)']
        self.dimU = len(self.controls)  

        # Swift model
        # Rigid-body mass matrix MRB
        # MRB = [Mass*I3  0
        #        0        Ib]
        # where Ib is the inertia matrix 3x3
        MRB  = np.array([
            [ 35.5,       0,      0,       0,       0,        0],
            [ 0,       35.5,      0,       0,       0,        0],
            [ 0,          0,   35.5,       0,       0,        0],
            [ 0,          0,      0,  0.8061, -0.0059,   0.0005],
            [ 0,          0,      0, -0.0059,     0.8,  -0.0113],
            [ 0,          0,      0,  0.0005, -0.0113,  1.5599 ] ], float)
        
        # Added-mass matrix MA
        MA =  np.diag([13, 30, 45, 12.4, 12.2, 5.5])
        # Linear damping matrix
        self.Dlin = np.diag([-22.8, -30.95, -50.26, -16.05, -16.73, -5.13])
        # Quadratic damping matrix
        self.Dquad = np.diag([-28.43, -55.98, -137.5, -0.0, -0.0, -0.00])
        # Total damping matrix
        self.D = Dmtx(self.Dlin, self.Dquad, self.nu)
        # r_bg = [x_g y_g z_g]: location of the CG with respect to the CO
        self.r_bg = np.array([0,0,0], float)
        # r_bb = [x_b y_b z_b]: location of the CB with respect to th CO 
        self.r_bb = np.array([0,0,-0.12], float)
        # Weight force
        self.Weight = self.mass * 9.806
        # Buoyancy force
        self.Buoyancy = self.volume * 1000 * 9.806 

        # System-inertia matrix
        self.M = MRB + MA
        
        self.Minv = np.linalg.inv(self.M)     
        
        # Thrust configuration matrix for swift AUV      
        self.T = np.array([
            [0.00000, 0.00000, 0.00000, 0.00000, 0.70710, 0.70710,-0.7071,-0.7071],
            [0.00000, 0.00000, 0.00000, 0.00000, 0.70710,-0.70710,-0.7071, 0.7071],
            [1.00000, 1.00000, 1.00000, 1.00000, 0.00000, 0.00000, 0.0000, 0.0000],
            [0.19279,-0.18721, 0.19279,-0.18721,-0.00000, 0.00000,-0.0000, 0.0000],
            [0.19936, 0.19858,-0.26364,-0.26442,-0.00000,-0.00000, 0.0000, 0.0000],
            [0.00000,-0.00000, 0.00000,-0.00000, 0.29320,-0.28925,-0.3993, 0.39541]], float)
        # Thrust coefficient matrix 
        self.K = 0.000004 * np.identity(8, float)
        self.B = self.T @ self.K

        # DP control system
        self.e_int = np.array([0, 0, 0, 0, 0, 0],float)    # integral states 
        self.eta_d = np.zeros(6)
        self.nu_d  = np.zeros(6)
        self.wn = np.diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])        # PID pole placement
        self.zeta = np.diag([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        self.Q = np.eye(12)                                   # LQR 
        self.R = 0.001 * np.eye(6)
             

    def dynamics(self,eta,nu,u_actual,u_control,sampleTime):
        """
        [nu,u_actual] = dynamics(eta,nu,u_actual,u_control,sampleTime) integrates the  
        swift equations of motion using Euler's method.
        """   

        # Input vector
        n = u_actual 

        # Current velocities
        u_c = self.V_c * math.cos(self.beta_c - eta[5])      # current surge velocity
        v_c = self.V_c * math.sin(self.beta_c - eta[5])      # current sway velocity

        nu_c = np.array([u_c,v_c,0,0,0,0],float)        # current velocity vector
        nu_r = nu - nu_c                                # relative velocity vector
        
        # Control forces and moments with propeller saturation
        n_squared = np.zeros(self.dimU)
        for i in range(0,self.dimU):        
            n[i] = sat(n[i],-self.n_max[i],self.n_max[i])  # saturation limits
            n_squared[i] = abs( n[i] ) * n[i]   

        tau = np.matmul(self.B, n_squared)
        # Total damping matrix
        self.D = Dmtx(self.Dlin, self.Dquad, nu_r)
        # Coriolis and centripetal matrix C
        self.C = m2c(self.M, nu_r)
        # Restoring forces and moments vector
        self.g = gvect(self.Weight, self.Weight, eta[4], eta[3], self.r_bg, self.r_bb)
        tau_resistive = - np.matmul(self.D, nu_r) - np.matmul(self.C, nu_r) - self.g
        # 6-DOF AUV model
        nu_dot = np.matmul(self.Minv, tau + tau_resistive)
        n_dot = (u_control - u_actual) / self.T_n

        # Forward Euler integration
        nu = nu + sampleTime * nu_dot
        n  = n  + sampleTime * n_dot    

        u_actual = np.array(n,float) 
        
        return nu, u_actual     
            

    def controlAllocation(self,tau6):
        """
        u_alloc  = controlAllocation(tau6),  tau6 = [tau_X, tau_Y, tau_Z, tau_Roll, tau_Pitch, tau_Yaw]'
        u_alloc = B' * inv( B * B' ) * tau6
        """
        # Moore-Penrose pseudo-inverse
        B_pseudoInv = self.B.T @  np.linalg.inv( self.B @ self.B.T )
        # Eight thrusters [N]
        u_alloc = np.matmul(B_pseudoInv, tau6)      
        
        return u_alloc


    def DPcontrol(self,eta,nu,sampleTime):
        """
        u = DPcontrol(eta,nu,sampleTime) is the dynamics positioning method:
        """
        if (self.controlMode == 'DPcontrol1') :
            # MIMO nonlinear PID controller for DP based on pole placement
            # tau = -R' Kp (eta-r) - R' Kd R nu - R' Ki int(eta-r) 
            [tau,self.e_int,self.eta_d] = DPpolePlacement6(
            self.e_int,self.M,self.D,eta,nu,self.eta_d,
            self.wn,self.zeta,self.ref,sampleTime)

        elif (self.controlMode == 'DPcontrol2'):
            # Linear Quadratic Regulator for optimal DP
            # tau = K e, where K is the optimal feedback gain 
            # e is the error state vector 
            [tau,self.eta_d] = LQR(self.M,self.Dlin,self.Dquad,self.r_bb,self.r_bg,
            self.Weight,eta,nu,self.eta_d,self.ref,self.nu_d,sampleTime,self.wn,
            self.Q,self.R)

        # From generlized forces tau to each thruster squared speed (RPM^2)
        u_alloc = self.controlAllocation(tau)
        # u_alloc = abs(n) * n --> n = sign(u_alloc) * sqrt(u_alloc)
        n = np.zeros(self.dimU)
        for i in range(0, self.dimU):
            n[i] = np.sign(u_alloc[i]) * math.sqrt( abs(u_alloc[i]) )
        # Commanded 8 propeller speeds (RPM) 
        u_control = n 

        return u_control


    def stepInput(self,t):
        """
        u = stepInput(t) generates force step inputs.
        """          
        n = np.array([0, 0, 100, 100],float) 

        if t > 30:
            n = np.array([50, 50, 50, 50],float)       
        if t > 70:
            n = np.array([0, 0, 0, 0],float)        

        u_control = n
         
        return u_control              
    
        

    