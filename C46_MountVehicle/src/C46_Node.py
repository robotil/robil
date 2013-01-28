#!/usr/bin/env python

import roslib; roslib.load_manifest('C46_MountVehicle')
import rospy

import actionlib
from MountVehicleServer import MountVehicleServer

if __name__ == '__main__':
	rospy.init_node('C46_MountVehicle')
	MountVehicleServer()
	rospy.spin()
