#! /usr/bin/env python

import roslib; roslib.load_manifest('C48_StandUp')
import rospy
import actionlib
from C0_RobilTask.msg import *

def TestStandUp():
    # Creating the action client
    client = actionlib.SimpleActionClient('/StandUp', RobilTaskAction)
    client.wait_for_server()

    # Creating a goal to send to the action server
    goal = RobilTaskGoal()
    goal.name = "StandUp"
	
	# Sending the goal
    client.send_goal(goal)

	# Returning the result
    client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializing a rospy node so the SimpleActionClient can publish and subscribe over ROS
        rospy.init_node('TestStandUp')
        result = TestStandUp()
        print result
    except rospy.ROSInterruptException:
        print "Program interrupted before completion"
