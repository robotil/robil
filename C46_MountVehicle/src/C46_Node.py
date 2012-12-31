#!/usr/bin/env python

import roslib; roslib.load_manifest('C46_MountVehicle')
import rospy, math

import actionlib
from C46_Mount_server import MountVehicleServer
from std_msgs.msg import Float64

if __name__ == '__main__':
  rospy.init_node('C46_Mount')
  MountVehicleServer()
  rospy.spin()
