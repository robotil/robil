#! /usr/bin/env python

import roslib; roslib.load_manifest('c35_monitoring')
import rospy

# Brings in the SimpleActionClient

import actionlib

import c35_monitoring.msg

def sitting_client():
  
    # Creates the SimpleActionClient, passing the type of the action
    # (finger_grabAction) to the constructor.
    client = actionlib.SimpleActionClient('sitting', c35_monitoring.msg.sittingAction)
    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    
    # Creates a goal to send to the action server.
    goal = c35_monitoring.msg.sittingGoal(positionNum=1)
    
    
    # Sends the goal to the action server.
    client.send_goal(goal)
    
    # Waits for the server to finish performing the action.
    client.wait_for_result(rospy.Duration.from_sec(5.0))
    
    # Prints out the result of executing the action
    return client.get_result()  






if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('sitting_client_py')
        result = sitting_client()
        print "Result: %f" % (result.location)
    except rospy.ROSInterruptException:
        print "program interrupted before completion" 