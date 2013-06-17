#! /usr/bin/env python

import roslib; roslib.load_manifest('C48_StandUp')
import rospy
from RobilTaskPy import *
from Controller import Controller
from atlas_msgs.msg import AtlasCommand
from atlas_msgs.msg import AtlasSimInterfaceCommand, AtlasSimInterfaceState, AtlasState, AtlasBehaviorStepData

import numpy as np
class StandUpServer(RobilTask):
	def __init__(self):
		print "StandUp Server Started!"
		RobilTask.__init__(self, "StandUp")

	def task(self, name, uid, parameters):
		print "Start standing up"
		# setting up a controller for sending commands
		self._controller = Controller()
		roll, pitch, yaw = self._controller.getRPY()
		# keep trying until robot stands up
		while pitch > 1.2 or pitch < -1.2 or roll > 1.4 or roll < -1.4:
			
			# in case it fell on its back roll down		
			while pitch < -1:
				
				self.rollDown()
				roll, pitch, yaw = self._controller.getRPY()
			# stand up
			self.standUp()
			roll, pitch, yaw = self._controller.getRPY()

		# task succeeded
		return RTResult_SUCCESSED("Finished in Success")

	def rollDown(self):
		# raise one arm and bring the other close to the torso
		pos = [0, 0, 0, 0,
				0, 0, -0.02, 0.04, -0.02, 0,
				0, 0, -0.02, 0.04, -0.02, 0,
				0, 1.5, 0, 0, 0, 0,
				0, 1.5, 0, 0, 0, 0]
		self._controller.publish(pos, 0.4)
		# rotate the torso arm to push the ground and turn around
		# add hip, legs rotation to aid the process
		pos = [-0.7, 0, 0, 0,
				0, 0, 0.7, 0.04, -0.02, 0,
				0, 0, -0.6, 0.04, -0.02, 0,
				0, 1.5, 0, 0, 0, 0,
				2, 1.2, 0, 0, 0, 0]
		self._controller.publish(pos, 0.8)
		# move arm forward to shift weight further and ensure a succesful turn
		pos = [0.7, 0, 0, 0,
				0, 0, 0.7, 0.04, -0.02, 0,
				0, 0, -0.6, 0.04, -0.02, 0,
				0, 1.5, 0, 0, 0, 0,
				-1, 1.5, 0, 0, 0, 0]
		self._controller.publish(pos, 0.4)
		# return to ground position
		pos = [0, 0, 0, 0,
				0, 0, -0.02, 0.04, -0.02, 0,
				0, 0, -0.02, 0.04, -0.02, 0,
				0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0]
		self._controller.publish(pos, 0.6)

	def standUp(self):
		# return to ground position
		pos = [0, 0, 0, 0,
				0, 0, -0.02, 0.04, -0.02, 0,
				0, 0, -0.02, 0.04, -0.02, 0,
				0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0]
		stand_pos = [-0.0, 0.001, 0.0, -0.611, 
					-0.005, 0.062, -0.231, 0.518, -0.277, -0.062, 
					0.005, -0.062, -0.235, 0.52, -0.276, 0.062, 
					0.3, -1.303, 2.001, 0.499, 0.0, -0.003, 
					0.3, 1.303, 2.001, -0.499, 0.0, 0.003]
		self._controller.publish(pos, 0.6)
		time = 1.5
		# bend elbows and feet
		self.doPose(AU = 0, AP = 0, EB = 1.5, DF = 2.1, KB = 0, HF = 0, PF = 0, MF = 0, dt = time/5.0)
		# flex into fetal position
		self.doPose(AU = -0.3, AP = 1.7, EB = 1.5, DF = 2.1, KB = 2, HF = 2, PF = 0.8, MF = 0, dt = time/1.5)
		# place body weight on feet only
		self.doPose(AU = -0.8, AP = 1.7, EB = 0, DF = 2.6, KB = 2.8, HF = 2.2, PF = 1.1, MF = 1.5, dt = time)
		rospy.sleep(1)
		# straighten step #1
		self.doPose(AU = -0.8, AP = 1.1, EB = 0, DF = 2.6, KB = 2.8, HF = 2, PF = 0.9, MF = 0, dt = time)

		self.doPose(AU = -0.8, AP = 0.8, EB = 0, DF = 2.6, KB = 2.8, HF = 2, PF = 0.5, MF = 0, dt = time)
		# straighten step #2
		self._controller.publish(stand_pos,3*time)
		# self.doPose(AU = -0.8, AP = 0.15, EB = 0, DF = 0.12, KB = 0.38, HF = 0.35, PF = 0, MF = 0, dt = 3*time)
		# get back to standing up position
		# self.doPose(AU = -1.2, AP = 0, EB = 0, DF = 0.0, KB = 0.04, HF = 0.02, PF = 0, MF = 0, dt = 1.5)
		#self.doPose(AU = 0, AP = 0, EB = 0, DF = 0.02, KB = 0.04, HF = 0.02, PF = 0, WZ=-1, WD=0, dt = 1.5)

	        stand_prep_msg = AtlasCommand()
	        stand_prep_msg.header.stamp = rospy.Time.now()
	        ac_pub.publish(stand_prep_msg)
	
		asi_command = rospy.Publisher('/atlas/atlas_sim_interface_command', AtlasSimInterfaceCommand, None, False, True, None)
		self.ac_pub = rospy.Publisher('atlas/atlas_command', AtlasCommand)

		#k_effort = [0] * 28
#	 	stand = AtlasSimInterfaceCommand(None,AtlasSimInterfaceCommand.STAND, None, None, None, None, None)
		command = AtlasSimInterfaceCommand()
                command.behavior = AtlasSimInterfaceCommand.WALK
                
        	rospy.sleep(0.3)
        
        	asi_command.publish(command)


	def doPose(self, AU, AP, EB, DF, KB, HF, PF, MF, dt):
		# AU - Arms up, AP - Arms push, EB = Elbow bend
		# DF - Dorsi flexion, KB - Knee bend
		# HF - Hip flex, PF - Pelvis flex

		# setting command position
		pos = [
			0, PF, 0, 0,
			-0.005, 0.062, -HF, KB, -DF, -0.062,
			0.005, -0.062, -HF, KB, -DF, 0.062,
			-AP, AU, 2, EB, 0, MF,
			-AP, -AU, 2, -EB, 0, -MF]
		# publishing the command position
		self._controller.publish(pos, dt)
 
if __name__ == '__main__':
	rospy.init_node('C48_StandUp')
	
	controller = Controller()
	pos = [0, 0, 0, 0,
			0, 0, -0.02, 0.04, -0.02, 0,
			0, 0, -0.02, 0.04, -0.02, 0,
			0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0]

	orientation = np.random.uniform(-3.1, 3.1, size=1)
	if np.abs(orientation) > 1.6:
		pos[8] = -0.2 * np.cos(orientation)
		pos[8+6] = -0.2 * np.cos(orientation)
	else:
		pos[8] = -0.15 * np.cos(orientation)
		pos[8+6] = -0.15 * np.cos(orientation)
	pos[9] = 0.3 * np.sin(orientation)
	pos[9+6] = 0.3 * np.sin(orientation)
	controller.publish(pos, 0.4)
	
#	ac_pub = rospy.Publisher('atlas/atlas_command', AtlasCommand)
	StandUpServer()	
#        stand_prep_msg = AtlasCommand()
#        stand_prep_msg.header.stamp = rospy.Time.now()
#        ac_pub.publish(stand_prep_msg)

	
	rospy.spin()
