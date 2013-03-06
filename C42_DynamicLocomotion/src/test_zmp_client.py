#! /usr/bin/env python

import roslib; roslib.load_manifest('C42_DynamicLocomotion')
import rospy
import actionlib
from C42_DynamicLocomotion.msg import *
from C31_PathPlanner.srv  import *
from std_msgs.msg import Float64, Int32

def zmp_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('/ZmpWalk', C42_ZmpWlkAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.

    goal = C42_ZmpWlkGoal()
    #rospy.wait_for_service("C31_GetPath_srv")
    #goal.goal_pos.theta = 0
    #goal.tol = 0.1
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    rospy.loginfo("server has finished the task")
    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        result = C42_DynamicLocomotion.msg.C42_ZmpWlkResult()
        rospy.init_node('zmp_test_client_py')
        result = zmp_client()

        print result
        #print "Target reached, Position: x = ", result.res_pos.x,"y = ",result.res_pos.y
        print "distance from goal:",result.dis
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
