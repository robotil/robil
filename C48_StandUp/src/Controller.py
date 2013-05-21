#! /usr/bin/env python

import roslib; roslib.load_manifest('C48_StandUp')
import rospy
import tf
from atlas_msgs.msg import AtlasCommand
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from numpy import zeros, array, linspace
from math import ceil

class Controller:
	def __init__(self):
		# initialize atlas joint names
		self._atlasJointNames = [
			'atlas::back_lbz', 'atlas::back_mby', 'atlas::back_ubx', 'atlas::neck_ay',
			'atlas::l_leg_uhz', 'atlas::l_leg_mhx', 'atlas::l_leg_lhy', 'atlas::l_leg_kny', 'atlas::l_leg_uay', 'atlas::l_leg_lax',
			'atlas::r_leg_uhz', 'atlas::r_leg_mhx', 'atlas::r_leg_lhy', 'atlas::r_leg_kny', 'atlas::r_leg_uay', 'atlas::r_leg_lax',
			'atlas::l_arm_usy', 'atlas::l_arm_shx', 'atlas::l_arm_ely', 'atlas::l_arm_elx', 'atlas::l_arm_uwy', 'atlas::l_arm_mwx',
			'atlas::r_arm_usy', 'atlas::r_arm_shx', 'atlas::r_arm_ely', 'atlas::r_arm_elx', 'atlas::r_arm_uwy', 'atlas::r_arm_mwx']
		# initialize new constants
		self._kp_position = [
			800, 4000, 2000, 20,
			1000, 1200, 1100, 1000, 1200, 600,
			1000, 1200, 1100, 1000, 1200, 600,
			2000, 1000, 30, 700, 50, 100,
			2000, 1000, 30, 700, 50, 100]
		self._kd_position = [
			0, 2, 1, 1,
			0, 0, 0, 10, 0, 0,
			0, 0, 0, 10, 0, 0,
			3, 10, 2, 3, 0.1, 0.2,
			3, 10, 2, 3, 0.1, 0.2]
		# joint states and odometry subsribers
		self._currentJointState = JointState()
		self._currentOdometry = Odometry()

		# initialize command
		self._command = AtlasCommand()
		#self._command.name = list(self._atlasJointNames)
		n = len(self._atlasJointNames)
		self._command.velocity     = zeros(n)
		self._command.effort       = zeros(n)
		self._command.kp_position  = zeros(n)
		self._command.ki_position  = zeros(n)
		self._command.kd_position  = zeros(n)
		self._command.kp_velocity  = zeros(n)
		self._command.i_effort_min = zeros(n)
		self._command.i_effort_max = zeros(n)
		# Set k_effort to 255 to indicate that we want PID control of each joint
  		self._command.k_effort = [255] * n
		for i in xrange(len(self._atlasJointNames)):
			name = self._atlasJointNames[i]
			self._command.kp_position[i] = self._kp_position[i]
			self._command.ki_position[i]  = rospy.get_param('atlas_controller/gains/' + name[7::] + '/i')
			self._command.kd_position[i] = self._kd_position[i]
			self._command.i_effort_max[i] = rospy.get_param('atlas_controller/gains/' + name[7::] + '/i_clamp')
			self._command.i_effort_min[i] = -self._command.i_effort_max[i]

		rospy.Subscriber("/atlas/joint_states", JointState, self._jointStatesCallback)
		rospy.Subscriber("/ground_truth_odom", Odometry, self._OdometryCallback)
		self._pub = rospy.Publisher('/atlas/atlas_command', AtlasCommand)
		rospy.sleep(0.5)

	# update joint states
	def _jointStatesCallback(self, msg):
		self._currentJointState = msg

	# update odometry
	def _OdometryCallback(self, msg):
		self._currentOdometry = msg

	# publish a command
	def publish(self, position, dt):
		initialPosition = array(self._currentJointState.position)
		commandPosition = array(position)
		dtPublish = 0.01
		n = ceil(dt / dtPublish)
		for ratio in linspace(0, 1, n):
			interpCommand = ((1 - ratio) * initialPosition) + (ratio * commandPosition)
			self._command.position = [ float(x) for x in interpCommand ]
			self._pub.publish(self._command)
			rospy.sleep(dt / float(n))

	# get the odometry position
	def getPose(self):
		return self._currentOdometry.pose.pose.position

	def getYPR(self):
		quat = self._currentOdometry.pose.pose.orientation
		(r, p, y) = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
		return (y, p, r)
