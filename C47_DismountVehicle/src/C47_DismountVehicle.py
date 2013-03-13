#! /usr/bin/env python

import roslib; roslib.load_manifest('C47_DismountVehicle')
import rospy
import actionlib
from RobilTaskPy import *
from geometry_msgs.msg import Pose

class DismountVehicleServer(RobilTask):
	def __init__(self):
		print "DismountVehicle Server Started!"
		#Initiating the action server
		RobilTask.__init__(self, "DismountVehicle")

	def task(self, name, uid, parameters):
		print "Start dismounting the vehicle"

		#Here should be the code for exiting the car
		pub = rospy.Publisher('/drc_world/robot_exit_car', Pose)
		p = Pose()
		p.position.x = -0.2
		p.position.y = -0.5
		p.orientation.w = 0.707
		p.orientation.z = 0.707
		rospy.sleep(1)
		pub.publish(p)

		#Task succeeded
		return RTResult_SUCCESSED("Finished in Success")

if __name__ == '__main__':
	rospy.init_node('C47_DismountVehicle')
	DismountVehicleServer()
	rospy.spin()
