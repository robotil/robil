#! /usr/bin/env python

import roslib; roslib.load_manifest('C47_DismountVehicle')
import rospy
import actionlib

from C47_DismountVehicle.msg import *

if __name__ == '__main__':
    rospy.init_node('C47_DismountVehicle_Clinet')
    client = actionlib.SimpleActionClient('Dismount_Vehicle', DismountAction)
    client.wait_for_server()

    goal = "GetOutVehicle"
    # Fill in the goal here
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))
