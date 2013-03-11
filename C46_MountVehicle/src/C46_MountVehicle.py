#! /usr/bin/env python

import roslib; roslib.load_manifest('C46_MountVehicle')
import rospy
import actionlib
from RobilTaskPy import *
from geometry_msgs.msg import Pose

class MountVehicleServer(RobilTask):
	def __init__(self):
		print "MountVehicle Server Started!"
		#Initiating the action server
		RobilTask.__init__(self, "MountVehicle")

	def task(self, name, uid, parameters):
		print "Start mounting the vehicle"

		#Here should be the code for entering the car
		pub = rospy.Publisher('/drc_world/robot_enter_car', Pose)
		rospy.sleep(1)
		pub.publish()

		#Task succeeded
		return RTResult_SUCCESSED("Finished in Success")

if __name__ == '__main__':
	rospy.init_node('C46_MountVehicle')
	MountVehicleServer()
	rospy.spin()