#! /usr/bin/env python

import roslib; roslib.load_manifest('C47_DismountVehicle')
import rospy
import actionlib
from C0_RobilTask.msg import *

def TestDismountVehicle():
    # Creating the action client
    client = actionlib.SimpleActionClient('/DismountVehicle', RobilTaskAction)
    client.wait_for_server()

    # Creating a goal to send to the action server
    goal = RobilTaskGoal()
    goal.name = "DismountVehicle"
	
	# Sending the goal
    client.send_goal(goal)

	# Returning the result
    client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializing a rospy node so the SimpleActionClient can publish and subscribe over ROS
        rospy.init_node('TestDismountVehicle')
        result = TestDismountVehicle()
        print result
    except rospy.ROSInterruptException:
        print "Program interrupted before completion"
