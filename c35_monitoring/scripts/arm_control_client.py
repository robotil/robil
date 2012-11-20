 
#! /usr/bin/env python

import roslib; roslib.load_manifest('c35_monitoring')
import rospy, math

# Brings in the SimpleActionClient
import actionlib
from std_msgs.msg import Float64
# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import c35_monitoring.msg


def arm_control_client():
    # Creates the SimpleActionClient, passing the type of the action (arm_controlAction) to the constructor.
    client = actionlib.SimpleActionClient('c35_monitoring', c35_monitoring.msg.arm_controlAction)
    
    #get input from user
    hand = int(raw_input("please choose which hand to activate, 1 for right, 2 for left, 3 for both  "))
    
    # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = c35_monitoring.msg.arm_controlGoal(hand)
   
    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result(rospy.Duration.from_sec(5.0))
    
    
    
    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('arm_control_client_py')
        result = arm_control_client()
        print "result: %f" %result.finalLocation
    except rospy.ROSInterruptException:
        print "program interrupted before completion"