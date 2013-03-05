#! /usr/bin/env python

import roslib; roslib.load_manifest('C46_MountVehicle')
import rospy
import actionlib
from C0_RobilTask.msg import *

def TestMountVehicle():
    # Creating the action client
    client = actionlib.SimpleActionClient('/MountVehicle', RobilTaskAction)
    client.wait_for_server()

    # Creating a goal to send to the action server
    goal = RobilTaskGoal()
    goal.name = "MountVehicle"
	
	# Sending the goal
    client.send_goal(goal)

	# Returning the result
    client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializing a rospy node so the SimpleActionClient can publish and subscribe over ROS
        rospy.init_node('TestMountVehicle')
        result = TestMountVehicle()
        print result
    except rospy.ROSInterruptException:
        print "Program interrupted before completion"
