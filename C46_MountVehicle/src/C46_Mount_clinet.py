#! /usr/bin/env python

import roslib; roslib.load_manifest('C46_MountVehicle')
import rospy
import actionlib

from C46_MountVehicle.msg import *

if __name__ == '__main__':
    rospy.init_node('C46_MountVehicle_Clinet')
    client = actionlib.SimpleActionClient('Mount_Vehicle', MountAction)
    client.wait_for_server()

    goal = "EnterVehicle"
    # Fill in the goal here
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))
