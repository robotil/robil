#!/usr/bin/env python

import roslib;roslib.load_manifest('C42_DynamicLocomotion')
import rospy
from JointController import JointCommands_msg_handler
from robot_state import robot_state
class stand_up_controller(object):
	def __init__(self,JC,RS,Tf):
		self.JC = JC
		self.RS = RS
		self.Tf = Tf
	def extend(self):
		pass
	def rs_cb(self,msg):
		self.RS.UpdateState(msg)

if __name__ == '__main__':
	robot_name = "atlas"
	jnt_names = ['back_lbz', 'back_mby',' back_ubx', 'neck_ay', #3
	                   'l_leg_uhz', 'l_leg_mhx', 'l_leg_lhy', 'l_leg_kny', 'l_leg_uay', 'l_leg_lax', #9
	                   'r_leg_uhz', 'r_leg_mhx', 'r_leg_lhy', 'r_leg_kny', 'r_leg_uay', 'r_leg_lax', #15
	                   'l_arm_usy', 'l_arm_shx', 'l_arm_ely', 'l_arm_elx', 'l_arm_uwy', 'l_arm_mwx', #21
	                   'r_arm_usy', 'r_arm_shx', 'r_arm_ely', 'r_arm_elx', 'r_arm_uwy', 'r_arm_mwx'] #27

	# Initialize joint commands handler
	JC = JointCommands_msg_handler(robot_name,self._jnt_names)