#!/usr/bin/env python

import roslib; roslib.load_manifest('C48_StandUp')
import rospy

import actionlib
from StandUpServer import StandUpServer

if __name__ == '__main__':
	rospy.init_node('C48_StandUp')
	StandUpServer()
	rospy.spin()
