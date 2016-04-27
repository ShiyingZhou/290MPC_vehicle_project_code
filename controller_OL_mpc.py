#!/usr/bin/env python
# Shiying, Liting, ME 290J
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
from barc.msg import ECU
from data_service.msg import TimeData
from geometry_msgs.msg import Vector3
from math import pi,sin,tan,arctan
import time
import serial
from numpy import zeros, hstack, cos, array, dot, arctan, sign
from numpy import unwrap
from input_map import angle_2_servo, servo_2_angle
from manuevers import TestSettings, CircularTest, Straight, SineSweep, DoubleLaneChange, CoastDown
from pid import PID

######### Julia code ##############


#from car_info_mpc import MPC

# v_x_enc=0
# d_f=0
# t0=0
# n_FL_prev=0
# n_FR_prev=0
# def enc_callback(data):
#     global v_x_enc, d_f, t0
#     global n_FL, n_FR, n_FL_prev, n_FR_prev

#     n_FL = data.x
#     n_FR = data.y

#     # compute time elapsed
#     tf = time.time()
#     dt = tf - t0
    
#     # if enough time elapse has elapsed, estimate v_x
#     dt_min = 0.20
#     if dt >= dt_min:
#         # compute speed :  speed = distance / time
#         dx_magnets=4
#         v_FL = float(n_FL- n_FL_prev)*dx_magnets/dt
#         v_FR = float(n_FR- n_FR_prev)*dx_magnets/dt

#         # update encoder v_x, v_y measurements
#         # only valid for small slip angles, still valid for drift?
#         v_x_enc     = (v_FL + v_FR)/2.0*cos(d_f)

#         # update old data
#         n_FL_prev   = n_FL
#         n_FR_prev   = n_FR
#         t0          = time.time()


#############################################################
def main_auto():
    # initialize ROS node
    
    rospy.init_node('auto_mode', anonymous=True)
    #rospy.Subscriber('encoder', Vector3, enc_callback)
    nh = rospy.Publisher('ecu', ECU, queue_size = 10)
    
	# set node rate
    rateHz  = 50
    dt      = 1.0 / rateHz
    rate 	= rospy.Rate(rateHz)
    t_i     = 0
    t0              = time.time()
    # read the control sequence from data file u_vec 2*60, dt=0.2 and u_vec[1,:] for acceleration and 2 for steering
    u_vec=zeros(2,30)
    # data 1
    u_vec[0,:]=[1.0 1.0 1.0 1.0 0.0 -0.0 -0.0 -0.0 -0.0 -0.0 0.0 -0.0 -0.177 -1.0 -1.0 -1.0 -1.0 -0.823 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0]
    u_vec[1,:]=[-0.239 -0.214 -0.155 -0.095 -0.062 -0.058 -0.064 0.524 -0.015 -0.01 -0.031 -0.015 -0.029 
    -0.025 0.155 0.524 0.524 0.524 0.017 0.018 0.018 0.019 0.019 0.019 0.02 0.02 0.02 0.02 0.02 0.02]
    # data 2
    u_vec[0,:]=[1.0 1.0 1.0 1.0 0.0 -0.0 -0.0 -0.0 -0.0 -0.0 0.0 -0.0 -0.027 -1.0 -1.0 -1.0 -1.0 -0.973 0.0 -0.0 -0.0 -0.0 -0.0 0.0 0.0 0.0 0.0 -0.0 0.0 -0.0]
    u_vec[1,:]=[-0.477 -0.397 -0.235 -0.13 -0.01 -0.284 0.443 0.524 -0.159 0.054 0.339 -0.177 0.039 
    -0.171 0.454 0.524 0.524 0.524 0.032 0.034 0.034 0.035 0.036 0.036 0.037 0.037 0.037 0.037 0.037 0.037]
    


    # main loop
    while not rospy.is_shutdown():
        # get steering wheel command
        
        for i in range(60)
            a_cmd=u_vec[0,i]
            steer_ang=u_vec[1,i]
            servoCMD=angle_2_servo(steer_ang) 
            # from lab8
            a0   = rospy.get_param("ol_mpc/a0")
            b0   = rospy.get_param("ol_mpc/b0")
            c0   = rospy.get_param("ol_mpc/c0")
            #lmd   = rospy.get_param("ol_mpc/lmd")
            # a0=0.1721
            # b0=0.1174
            # c0=0.6263
            # lmd=1 # controller coeeficient, for bandwidth
            #motorCMD=(-lmd*(V_cmd-v_x_enc)+c0*V_cmd^2+b0)/a0+90
            motorCMD=(a_cmd+c0*v_x_enc**2+b0)/a0+90
            motorCMD=96
            ecu_cmd = ECU(motorCMD, servoCMD)
            for j in range(10) # hold the command for 10*0.02=0.2, since the open loop sequence is generated with delta_t=0.2
                nh.publish(ecu_cmd) # there is delay between publishing and the vehicle actually execute the cmd
                t_i+=1
                rate.sleep()
            

#############################################################
if __name__ == '__main__':
	try:
		
		main_auto()
	except rospy.ROSInterruptException:
		pass
