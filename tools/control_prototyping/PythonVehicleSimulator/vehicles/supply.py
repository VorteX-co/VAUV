# -*- coding: utf-8 -*-
"""
supply.py: 
    Class for an offshore supply vessel length L = 76.2. 
    The constructors are:

    supply()                                      
        Step inputs for propeller revolutions n1, n2, n3 and n4
    supply('DPcontrol',x_d,y_d,psi_d,V_c,beta_c)  DP control system
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
    n = [ #1 Bow thruster (RPM) 
          #2 Bow thruster (RPM)
          #3 Right main propeller (RPM)
          #4 Left main propeller (RPM) ]

u_alloc = controlAllocation(tau)
    Control allocation based on the pseudoinverse                 

n = DPcontrol(eta,nu,sampleTime)
    Nonlinear PID controller for DP based on pole placement.    

n = stepInput(t) generates propellers step inputs.
    
References: 
  T. I. Fossen, S. I. Sagatun and A. J. Sorensen (1996)
     Identification of Dynamically Positioned Ships
     Journal of Control Engineering Practice CEP-4(3):369-376
  T. I. Fossen (2021). Handbook of Marine Craft Hydrodynamics and Motion 
     Control. 2nd. Edition, Wiley. 
     URL: www.fossen.biz/wiley            

Author:     Thor I. Fossen
"""
import numpy as np
import math
from functions.control import DPpolePlacement
from functions.gnc import sat,ssa

# Class Vehicle
class supply:
    """
    supply()                                      Step inputs n1,n2,n3 and n4
    supply('DPcontrol',x_d,y_d,psi_d,V_c,beta_c)  DP control system
    """        
    def __init__(self, controlSystem = 'stepInput', 
                 r_x = 0, r_y = 0, r_n = 0, V_current = 0, beta_current = 0):         
                
        if (controlSystem == 'DPcontrol'):
            self.controlDescription = 'Nonlinear DP control (x_d, y_d, psi_d) = (' \
            + str(r_x) + ' m, ' + str(r_y) + ' m, ' + str(r_n) + ' deg)'
             
        else:  
            self.controlDescription = 'Step inputs for n = [n1, n2, n3, n4]'
            controlSystem = 'stepInput'  
      
        self.ref = np.array([ r_x, r_y, (math.pi/180) * r_n ], float)
        self.V_c = V_current
        self.beta_c = beta_current
        self.controlMode = controlSystem
                    
        # Initialize the supply vessel model 
        self.L = 76.2                           # Length (m)   
        self.T_n = 1.0                          # prop. rev. time constant (s)          
        self.n_max = np.array([                 # RPM saturation limits (N) 
            250, 250, 160, 160 ],float)                                     
        self.nu = np.array([0, 0, 0, 0, 0, 0], float)  # velocity vector    
        self.u_actual = np.array([0, 0, 0, 0], float)  # RPM inputs
        self.name = 'Offshore supply vessel'     
        
        # Two tunnel thrusters in the bow, no. 1 and 2
        # Two main propellers aft, no. 3 and 4        
        self.controls = [ 
            '#1 Bow thruster (RPM)', 
            '#2 Bow thruster (RPM)',
            '#3 Right main propeller (RPM)',
            '#4 Left main propeller (RPM)' ]
        self.dimU = len(self.controls)  

        # Constants   
        g   = 9.81                   # acceleration of gravity (m/s^2)
        rho = 1025                   # density of water        
        m = 6000.0e3                 # mass (kg)

        # Thrust coefficient and configuration matrices (Fossen 2021, Ch. 11.2)
        K = np.diag( [ 2.4, 2.4, 17.6, 17.6 ])
        T = np.array([
            [0, 0, 1, 1],
            [1, 1, 0, 0],
            [30, 22, -self.L/2, self.L/2] ],float) 
        self.B = T @ K           

        # Tbis = np.diag( [1, 1, 1 / self.L],float)
        Tbis_inv = np.diag( [1.0, 1.0, self.L])

        # 3-DOF model matrices - bis scaling (Fossen 2021, App. D) 
        Mbis = np.array([
            [ 1.1274,         0,          0],
            [ 0,         1.8902,    -0.0744],
            [ 0,        -0.0744,     0.1278] ], float)

        Dbis = np.array([
            [ 0.0358,        0,        0 ],
            [ 0,        0.1183,  -0.0124 ],
            [ 0,       -0.0041,   0.0308 ] ], float)       

        self.M3 = m * Tbis_inv @ Mbis @ Tbis_inv
        self.M3inv = np.linalg.inv(self.M3)
        self.D3 = m * math.sqrt( g / self.L ) * Tbis_inv @ Dbis @ Tbis_inv

        # DP control system
        self.e_int = np.array([ 0, 0, 0 ],float)    # integral states 
        self.x_d = 0.0                              # setpoints
        self.y_d = 0.0
        self.psi_d = 0.0 
        self.wn = np.diag([ 0.3, 0.3, 0.1 ])        # PID pole placement
        self.zeta = np.diag([ 1.0, 1.0, 1.0 ])
             

    def dynamics(self,eta,nu,u_actual,u_control,sampleTime):
        """
        [nu,u_actual] = dynamics(eta,nu,u_actual,u_control,sampleTime) integrates the  
        supply vessel equations of motion using Euler's method.
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
            n[i] = sat(n[i],-self.n_max[i],self.n_max[i])  # saturation, physical limits
            n_squared[i] = abs( n[i] ) * n[i]   

        tau3 = np.matmul(self.B, n_squared)

        # 3-DOF dynamics
        nu3_r = np.array([ nu_r[0], nu_r[1], nu_r[5] ])         
        nu3_dot = np.matmul( self.M3inv, tau3 - np.matmul(self.D3, nu3_r) ) 

        # 6-DOF ship model
        nu_dot = np.array([ nu3_dot[0],nu3_dot[1],0,0,0,nu3_dot[2] ])  
        n_dot = (u_control - u_actual) / self.T_n

        # Forward Euler integration
        nu = nu + sampleTime * nu_dot
        n  = n  + sampleTime * n_dot    

        u_actual = np.array(n,float)   
        
        return nu, u_actual     
            

    def controlAllocation(self,tau3):
        """
        u_alloc  = controlAllocation(tau3),  tau3 = [tau_X, tau_Y, tau_N]'
        u_alloc = B' * inv( B * B' ) * tau3
        """
        B_pseudoInv = self.B.T @  np.linalg.inv( self.B @ self.B.T )
        u_alloc = np.matmul(B_pseudoInv, tau3)      
        
        return u_alloc


    def DPcontrol(self,eta,nu,sampleTime):
        """
        u = DPcontrol(eta,nu,sampleTime) is a nonlinear PID controller
        for DP based on pole placement:
        
        tau = -R' Kp (eta-r) - R' Kd R nu - R' Ki int(eta-r)
        u = B_pseudoinverse * tau 
        """     
        eta3 = np.array([ eta[0], eta[1], eta[5] ])  
        nu3  = np.array([ nu[0], nu[1], nu[5] ]) 

        [tau3,self.e_int,self.x_d,self.y_d,self.psi_d] = DPpolePlacement( \
            self.e_int,self.M3,self.D3,eta3,nu3, \
            self.x_d,self.y_d,self.psi_d, \
            self.wn,self.zeta,self.ref,sampleTime)

        u_alloc = self.controlAllocation(tau3)
        
        # u_alloc = abs(n) * n --> n = sign(u_alloc) * sqrt(u_alloc)
        n = np.zeros(self.dimU)
        for i in range(0, self.dimU):
            n[i] = np.sign(u_alloc[i]) * math.sqrt( abs(u_alloc[i]) )

        u_control = n 
    
        return u_control 


    def stepInput(self,t):
        """
        u = stepInput(t) generates propeller step inputs (RPM).
        """          
        n = np.array([0, 0, 100, 100],float) 
        
        if t > 30:
            n = np.array([50, 50, 50, 50],float)       
        if t > 70:
            n = np.array([0, 0, 0, 0],float)        
  
        u_control = n
         
        return u_control              
    
        

    