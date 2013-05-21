#! /usr/bin/env python

import roslib; roslib.load_manifest('C48_StandUp')
import rospy
from RobilTaskPy import *
from Controller import Controller

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
		while pitch > 1.4 or pitch < -1.4:
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
		time = 1.5
		# bend elbows and feet
		self.doPose(AU = 0, AP = 0, EB = 1.5, DF = 2.1, KB = 0, HF = 0, PF = 0, dt = time/5.0)
		# flex into fetal position
		self.doPose(AU = -0.3, AP = 1.7, EB = 1.5, DF = 2.1, KB = 2, HF = 2, PF = 0.8, dt = time/1.5)
		# place body weight on feet only
		self.doPose(AU = -0.8, AP = 1.4, EB = 0, DF = 2.6, KB = 2.8, HF = 2.2, PF = 1.1, dt = time)
		rospy.sleep(1)
		# straighten step #1
		self.doPose(AU = -0.8, AP = 1.1, EB = 0, DF = 2.6, KB = 2.8, HF = 2, PF = 0.9, dt = time)
		# straighten step #2
		self.doPose(AU = -0.4, AP = 0.15, EB = 0, DF = 0.12, KB = 0.38, HF = 0.35, PF = 0, dt = 3*time)
		# get back to standing up position
		self.doPose(AU = 0, AP = 0, EB = 0, DF = 0.02, KB = 0.04, HF = 0.02, PF = 0, dt = 1.5)
		#self.doPose(AU = 0, AP = 0, EB = 0, DF = 0.02, KB = 0.04, HF = 0.02, PF = 0, WZ=-1, WD=0, dt = 1.5)

	def doPose(self, AU, AP, EB, DF, KB, HF, PF, dt):
		# AU - Arms up, AP - Arms push, EB = Elbow bend
		# DF - Dorsi flexion, KB - Knee bend
		# HF - Hip flex, PF - Pelvis flex

		# setting command position
		pos = [
			0, PF, 0, 0,
			0, 0, -HF, KB, -DF, 0,
			0, 0, -HF, KB, -DF, 0,
			-AP, AU, 0, EB, 0, 0,
			-AP, -AU, 0, -EB, 0, 0]
		# publishing the command position
		self._controller.publish(pos, dt)

if __name__ == '__main__':
	rospy.init_node('C48_StandUp')
	StandUpServer()
	rospy.spin()
