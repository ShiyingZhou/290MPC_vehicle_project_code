#!/usr/bin/env python
# Liting Sun 04/25/2016
# ---------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for 
# education or reserach purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link 
# to http://barc-project.com
#
# Attibution Information: The barc project ROS code-base was developed
# at UC Berkeley in the Model Predictive Control (MPC) lab by Jon Gonzales
# (jon.gonzales@berkeley.edu). The cloud services integation with ROS was developed
# by Kiet Lam  (kiet.lam@berkeley.edu). The web-server app Dator was 
# based on an open source project by Bruce Wootton
# ---------------------------------------------------------------------------
import rospy
import time
from numpy import pi, cos, sin, eye, array, zeros, eye, matrix


y = f(x, *args)
    
    jac = zeros( (y.size,x.size) )
    eps = 1e-5
    xp = copy(x)
    
    for i in range(x.size):
        xp[i] = x[i] + eps/2.0
        yhi = f(xp, *args)
        xp[i] = x[i] - eps/2.0
        ylo = f(xp, *args)
        xp[i] = x[i]
        jac[:,i] = (yhi - ylo) / eps
    return jac


class GLOBAL_X_EST:
    def __init__(self, p_x_int = 0.0, p_y_int = 0.0, yaw_init = 0.0, Integrator_max=500, Integrator_min=-500):
        
        self.p_x            = p_x_int;
        self.p_y            = p_y_int;
        self.p_yaw          = yaw_init;
        self.int_x_max      = Integrator_max
        self.int_y_max      = Integrator_max
        self.int_x_min      = Integrator_min
        self.int_y_min      = Integrator_min
        # rotation matrix from body frame to world frame
        self.rotationT      = eye(2);             
        self.rotationT[0,0] = cos(p_yaw);
        self.rotationT[0,1] = -sin(p_yaw);
        self.rotationT[1,0] = sin(p_yaw);
        self.rotationT[1,1] = cos(p_yaw);

    def update(self,current_value, dt):
        p_x_t           = self.rotationT[0,0]*current_value[0] + self.rotationT[0,1]*current_value[1];
        p_y_t           = self.rotationT[1,0]*current_value[0] + self.rotationT[1,1]*current_value[1];
        p_yaw           = current_value[2]
        
        self.p_x  = self.p_x + p_x_t * dt
        self.p_y  = self.p_y + p_y_t * dt
        if self.p_x > self.int_x_max:
            self.p_x = self.int_x_max
        elif self.p_x < self.int_x_min:
            self.p_x = self.int_x_min

        if self.p_y > self.int_y_max:
            self.p_y = self.int_y_max
        elif self.p_y < self.int_y_min:
            self.p_y = self.int_y_min

        return array([p_x,p_y, p_yaw]);

