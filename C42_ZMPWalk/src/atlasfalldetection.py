#! /usr/bin/env python
import roslib; roslib.load_manifest('AtlasFallDetection')
import roslib; roslib.load_manifest('C42_ZMPWalk')
import math, rospy, os, rosparam
import tf
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from numpy import zeros, array, linspace, arange
import numpy as np
from math import *
import yaml
from copy import copy
import time
from atlas_msgs.msg import ForceTorqueSensors
#Enumerate States
class State(object):
    Safe, Unstable, Fallen = range(3)

class Fall_detection(object):
	# A class for building a state machine responsible for detecting high probability of falling
	# or other dangerous orientations/angular velocities of the robot.
	# In normal operation mode, the state machine is in Safe mode, scanning and analyzing the
	# incoming odometry or imu data, while transmitting all is in order.
	# At the moment an unstable state is recognized, the machine
	# outputs a warning contatining the appropriate information on its specified topic.
	# If and when, a fall has been detected to a high certainty, the state machine will output this.
	# If and when the robot resumes normal activity in safe parameters, the state machine will
	# go back to safe mode.
	# 
	#	States list:
	# 1) Safe mode
	# 2) Unstable position detected
	# 3) Fall detected
	#
	#	Proposed criteria for safety:
	# 1) |Angular Velocity| < AngularVelocity Threshold
	# 2) Orientation |x|,|y| < Respective thresholds
	#
	#	TO DO:
	#	1) Decide on criteria for stability, non_stability and fallen state 
    #	2) I will create detection for forward or backward falling, later on will generalize
    #   3) Notics enumeration of states is outside of class, check how to move it inside of the class
    

    def __init__(self):
        #Define parameters
        self.AngVelThreshold = 1.0	# Needs to be calibrated
        self.state = State.Safe 	# Maybe needs an entry state instead of assuming starting in safe mode
        
        self.P = 0
        self.R = 0
        self.K = 0
        self.r_force_z=self.l_force_z=400

        if rospy.get_name() == '/unnamed':
            rospy.init_node('Fall_Detection',anonymous=True)
        
        rospy.Subscriber('atlas/imu', Imu, self.updateImu)
        rospy.Subscriber('atlas/force_torque_sensors',ForceTorqueSensors,self.updateforce)
        # rospy.publish('',,)	#Publish StateMachine output, custum message.
	
	##########################################
    ### State model ###
    # CheckForStateChange()
    # ActAccordingToState()
    ##########################################
    ### Safe mode ###
    def SafeState(self):
    	if (self.CheckUnstable()):
    		self.state = State.Unstable
    		return
    	if (self.CheckFall()):
    		self.state = State.Fallen
    		return
    	self.ActionSafe()
    
    def CheckSafe(self):
    	# if (self.AngVelR < self.AngVelThreshold)
    		# return True
    	# else
    		# return False
        if (self.K < 0.3): #((self.r_force_z+self.l_force_z)>200)
            return True
        else:
            return False

    def ActionSafe(self):
    	rospy.loginfo('Safe and stable operation.')
    ##########################################
    ### Unstable mode ###
    def UnstableState(self):
    	if (self.CheckSafe()):
    		self.state = State.Safe
    		return
    	if (self.CheckFall()):
    		self.state = State.Fallen
    		return
    	self.ActionUnstable()
    
    def CheckUnstable(self):
    	# if (self.AngVelR > self.AngVelThreshold):
    	if (self.K > 0.3 and self.K < 0.8):
            return True
        else:
            return False

    def ActionUnstable(self):
    	rospy.loginfo('Unstable!')
    ##########################################
    ### Fallen mode ###
    def FallState(self):
    	if (self.CheckSafe()):
    		self.state = State.Safe
    		return
    	if (self.CheckUnstable()):
    		self.state = State.Unstable
    		return
    	self.ActionFall()

    def CheckFall(self):
    	if (self.K > 0.8) : #or ((self.r_force_z+self.l_force_z)<700) :
            return True
        else:
            return False

    def ActionFall(self):
    	rospy.loginfo('Atlas fell and is on ground.')
        time = rospy.get_time()
        fall_data.write('%s  \n'  %(time))
    ##########################################
   	### Master ###
    def Act(self):
        if self.state == State.Safe:
            self.SafeState()
        if self.state == State.Unstable:
            self.UnstableState()
        if self.state == State.Fallen:
            self.FallState()
   	##########################################
    def updateImu(self, msg):
    	# self.imu = msg
    	# Extract angular velocity absolute value and direction in xy plane
    	self.AngVelX = msg.angular_velocity.x
    	self.AngVelY = msg.angular_velocity.y
    	self.AngVelTheta   = np.math.atan2(self.AngVelX,self.AngVelY)
        self.AngVelR       = np.math.sqrt(self.AngVelX**2 + self.AngVelY**2)	# Note sqrt(), maybe delete
        # Extract orientation (R, P, Y)
        (self.R, self.P, self.Y) = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y,
                                                              msg.orientation.z, msg.orientation.w])
        self.K = math.sqrt(self.R**2+self.P**2)
	##########################################
    def updateforce(self,msg):
        self.r_force_z = msg.r_foot.force.z
        self.l_force_z = msg.l_foot.force.z
        
        #rospy.loginfo(self.l_force_z)
        #rospy.loginfo(self.r_force_z)
        

################# Actual program #############
detector = Fall_detection()
fall_data = open('fall_data.txt','w')

while not rospy.is_shutdown():
    detector.Act()
    
