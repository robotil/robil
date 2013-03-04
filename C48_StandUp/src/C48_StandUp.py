#! /usr/bin/env python

import roslib; roslib.load_manifest('C48_StandUp')
import rospy, yaml, sys
import actionlib
from RobilTaskPy import *
from osrf_msgs.msg import JointCommands
from sensor_msgs.msg import JointState
from numpy import zeros, array, linspace
from math import ceil
from geometry_msgs.msg import Pose
from std_msgs.msg import String

atlasJointNames = [
  'atlas::back_lbz', 'atlas::back_mby', 'atlas::back_ubx', 'atlas::neck_ay',
  'atlas::l_leg_uhz', 'atlas::l_leg_mhx', 'atlas::l_leg_lhy', 'atlas::l_leg_kny', 'atlas::l_leg_uay', 'atlas::l_leg_lax',
  'atlas::r_leg_uhz', 'atlas::r_leg_mhx', 'atlas::r_leg_lhy', 'atlas::r_leg_kny', 'atlas::r_leg_uay', 'atlas::r_leg_lax',
  'atlas::l_arm_usy', 'atlas::l_arm_shx', 'atlas::l_arm_ely', 'atlas::l_arm_elx', 'atlas::l_arm_uwy', 'atlas::l_arm_mwx',
  'atlas::r_arm_usy', 'atlas::r_arm_shx', 'atlas::r_arm_ely', 'atlas::r_arm_elx', 'atlas::r_arm_uwy', 'atlas::r_arm_mwx']

currentJointState = JointState()
def jointStatesCallback(msg):
  global currentJointState
  currentJointState = msg

class StandUpServer(RobilTask):
	def __init__(self):
		print "StandUp Server Started!"
		#Initiating the action server
		RobilTask.__init__(self, "StandUp")

	def task(self, name, uid, parameters):
		print "Start standing up"

		#Here should be the code for standing up

		#Setup subscriber to atlas states
		rospy.Subscriber("/atlas/joint_states", JointState, jointStatesCallback)

		#Initialize JointCommands message
		command = JointCommands()
		command.name = list(atlasJointNames)
		n = len(command.name)
		command.position     = zeros(n)
		command.velocity     = zeros(n)
		command.effort       = zeros(n)
		command.kp_position  = zeros(n)
		command.ki_position  = zeros(n)
		command.kd_position  = zeros(n)
		command.kp_velocity  = zeros(n)
		command.i_effort_min = zeros(n)
		command.i_effort_max = zeros(n)

		#Now get gains from parameter server
		for i in xrange(len(command.name)):
			name = command.name[i]
			command.kp_position[i]  = rospy.get_param('atlas_controller/gains/' + name[7::] + '/p')
			command.ki_position[i]  = rospy.get_param('atlas_controller/gains/' + name[7::] + '/i')
			command.kd_position[i]  = rospy.get_param('atlas_controller/gains/' + name[7::] + '/d')
			command.i_effort_max[i] = rospy.get_param('atlas_controller/gains/' + name[7::] + '/i_clamp')
			command.i_effort_min[i] = -command.i_effort_max[i]

		#Set up the publisher
		pub = rospy.Publisher('/atlas/joint_commands', JointCommands)
		#Get initial joint positions
		initialPosition = array(currentJointState.position)
		#Set init poisitions
		y = [2, ' 0 0 0 0    0 0 0 0 0 0     0 0 0 0 0 0     0 -1.4 0 0 0 0        0 1.4 0 0 0 0']
		#First value is time duration
		dt = float(y[0])
		#Subsequent values are desired joint positions
		commandPosition = array([ float(x) for x in y[1].split() ])
		#Desired publish interval
		dtPublish = 0.02
		n = ceil(dt / dtPublish)
		for ratio in linspace(0, 1, n):
			interpCommand = (1-ratio)*initialPosition + ratio * commandPosition
			command.position = [ float(x) for x in interpCommand ]
			pub.publish(command)
			rospy.sleep(dt / float(n))

		#Setting the position
		pose = rospy.Publisher('/atlas/set_pose', Pose)
		mode = rospy.Publisher('/atlas/mode', String)
		p = Pose()
		p.position.x = 0
		p.position.y = 0
		p.position.z = 0.93
		p.orientation.w = 1
		p.orientation.x = 0
		p.orientation.y = 0
		p.orientation.z = 0
		rospy.sleep(3)
		pose.publish(p)
		rospy.sleep(0.1)
		mode.publish("pinned")
		rospy.sleep(2)
		mode.publish("nominal")

		#Task succeeded
		return RTResult_SUCCESSED("Finished in Success")

if __name__ == '__main__':
	rospy.init_node('C48_StandUp')
	StandUpServer()
	rospy.spin()
