#! /usr/bin/env python

import roslib; roslib.load_manifest('C41_BodyControl')
import rospy, math

import actionlib
from RobotBodyServer import RobotBodyServer
from std_msgs.msg import Float64

if __name__ == '__main__':
  rospy.init_node('C41_BodyControl')
  RobotBodyServer()
  rospy.spin()

