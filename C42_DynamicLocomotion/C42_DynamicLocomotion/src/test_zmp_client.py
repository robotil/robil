#! /usr/bin/env python

import roslib; roslib.load_manifest('C42_DynamicLocomotion')
import rospy
import actionlib
from C42_DynamicLocomotion.msg import *

import C31_PathPlanner.msg
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
    
    Pth = rospy.ServiceProxy("C31_GetPath", srv)
    pth = Pth()

    pos.x = pth.path.points[0].x#2
    pos.y = pth.path.points[0].y#0
    
    #goal.goal_pos.theta = 0
    #goal.tol = 0.1


    # Sends the goal to the action server.
    #client.send_goal(goal)
    client.send_goal()
    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('zmp_test_client_py')
        result = zmp_client()
        print "Target reached, Position: x = ", result.res_pos.x,"y = ",result.res_pos.y
        print "distance from goal:",result.dis
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
