#!/usr/bin/env python

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
# Liting Sun 04/25/2016
import rospy
import time
import os
import json
from numpy import pi, cos, sin, eye, array
from geometry_msgs.msg import Vector3
from GlobalSignal.msg import GlobalSignal
from input_map import angle_2_servo, servo_2_angle
from data_service.srv import *
from data_service.msg import *
from filtering import filteredSignal
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
import numpy as np

# input variables
d_f 	    = 0
servo_pwm   = 0
motor_pwm   = 0

# raw measurement variables
# from IMU
roll    = 0
pitch   = 0
yaw 	= 0
w_x 	= 0
w_y 	= 0
w_z 	= 0
a_x 	= 0
a_y 	= 0
a_z 	= 0

# from encoder
v_x_enc 	= 0
t0 	        = time.time()
n_FL	    = 0                 # counts in the front left tire
n_FR 	    = 0                 # counts in the front right tire
n_FL_prev 	= 0
n_FR_prev 	= 0
r_tire 		= 0.0319            # radius from tire center to perimeter along magnets
dx_magnets 	= 2.0*pi*r_tire/4.0     # distance between magnets

ignoreEncoder = 0

# ecu command update
def ecu_callback(data):
	global servo_pwm, motor_pwm, d_f
	motor_pwm	    = data.x
	servo_pwm       = data.y
	d_f 		    = data.z

# imu measurement update
def imu_callback(data):
	global roll, pitch, yaw, a_x, a_y, a_z, w_x, w_y, w_z
	(roll, pitch, yaw, a_x, a_y, a_z, w_x, w_y, w_z) = data.value

# encoder measurement update
def enc_callback(data):
	global v_x_enc, d_f, t0
	global n_FL, n_FR, n_FL_prev, n_FR_prev

	n_FL = data.x
	n_FR = data.y

	# compute time elapsed
	tf = time.time()
	dt = tf - t0
	
	# if enough time elapse has elapsed, estimate v_x
	dt_min = 0.20
	if dt >= dt_min:
		# compute speed :  speed = distance / time
		v_FL = float(n_FL- n_FL_prev)*dx_magnets/dt
		v_FR = float(n_FR- n_FR_prev)*dx_magnets/dt

		# update encoder v_x, v_y measurements
		# only valid for small slip angles, still valid for drift?
		v_x_enc 	= (v_FL + v_FR)/2.0*cos(d_f)

		# update old data
		n_FL_prev   = n_FL
		n_FR_prev   = n_FR
		t0 	        = time.time()


# global position estimation node
def global_estimation_XY():
	  # initialize node
    rospy.init_node('state_estimation_XY', anonymous=True)

    # topic subscriptions / publications
    rospy.Subscriber('imu', TimeData, imu_callback)
    rospy.Subscriber('encoder', Vector3, enc_callback)
    rospy.Subscriber('ecu', Vector3, ecu_callback)


    position_pub 	= rospy.Publisher('global_estimate', GlobalSignal, queue_size = 10)

	  # get system parameters
    username = rospy.get_param("controller/user")
    
	  # get vehicle dimension parameters
    # note, the imu is installed at the front axel
    L_a = rospy.get_param("state_estimation/L_a")       # distance from CoG to front axel
    L_b = rospy.get_param("state_estimation/L_b")       # distance from CoG to rear axel
    m   = rospy.get_param("state_estimation/m")         # mass of vehicle
    I_z = rospy.get_param("state_estimation/I_z")       # moment of inertia about z-axis
    vhMdl   = (L_a, L_b, m, I_z)

    # get encoder parameters
    dt_vx   = rospy.get_param("state_estimation/dt_vx")     # time interval to compute v_x
    v_x_min     = rospy.get_param("state_estimation/v_x_min")  # minimum velociy before using EKF


	# set node rate
    loop_rate 	= 50
    dt 		    = 1.0 / loop_rate
    rate 		= rospy.Rate(loop_rate)
    t0 		    = time.time()

    # global position vector
    p_xy           = GLOBAL_X_EST(0.0,0.0,yaw)
    p_global       = array(array([0.0,0.0]),yaw)

    # filtered signal for longitudinal velocity
    p_filter    = rospy.get_param("state_estimation/p_filter")
    v_x_filt    = filteredSignal(a = p_filter, method='lp')   # low pass filter

    while not rospy.is_shutdown():
	    # signals from inertial measurement unit, encoder, and control module
        global roll, pitch, yaw, w_x, w_y, w_z, a_x, a_y, a_z
        global n_FL, n_FR, v_x_enc
        global motor_pwm, servo_pwm, d_f
        global ignoreEncoder p_global

		# publish state estimate
        (p_x, p_y, yaw) = p_global          

        # publish information
        position_pub.publish(GlobalSignal(p_x, p_y, yaw,v_x_filt))
        # update filtered signal
        if not ignoreEncoder:
            v_x_filt.update(v_x_enc) # always use encoder Liting Sun 04/25/2016
        else:
            v_x_filt.update(v_LQR_min)
        v_x_est = v_x_filt.getFilteredSignal() # estimated velocity through encoder, treat this one as v_x for global position x estimation

        if v_x_est >= v_x_min:
            p_global = p_xy.update(array([v_x_est, 0.0, yaw]) ,dt) # always treat v_y as 0
        else:
            v_x_est = v_x_min
            p_global = p_xy.update(array([v_x_est, 0.0, yaw]) ,dt) # need to test whether v_x_min or zero should be used


		# wait
        rate.sleep()

if __name__ == '__main__':
	try:
	   global_estimation_XY()
	except rospy.ROSInterruptException:
		pass
