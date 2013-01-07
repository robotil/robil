#!/usr/bin/env python

import roslib; roslib.load_manifest('C47_DismountVehicle')
import rospy, math

import actionlib
from C47_Dismount_server import DismountVehicleServer
from std_msgs.msg import Float64

if __name__ == '__main__':
  rospy.init_node('C47_Dismount')
  DismountVehicleServer()
  rospy.spin()
