#!/usr/bin/env python
import roslib; roslib.load_manifest('C25_AtlasFell')
import rospy
import actionlib
import C25_AtlasFell.msg

def Fell_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    rospy.loginfo('C25_AtlasFell client Online!')
    client = actionlib.SimpleActionClient('Fell', C25_AtlasFell.msg.FallenAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = C25_AtlasFell.msg.FallenGoal(name='StillStanding?',uid='5',parameters='xyz')

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('FellClient')
        result =  Fell_client()
        if result:
            rospy.loginfo('Atlas Fell down!!!')
    except rospy.ROSInterruptException:
        print "Initialization interrupted before completion"
