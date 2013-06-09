#!/usr/bin/env python
import roslib; roslib.load_manifest('C51_CarOperation')
import rospy
import actionlib
import C0_RobilTask.msg

def Switch_client(dir):
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('GripGearStick', C0_RobilTask.msg.RobilTaskAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    if dir==1:
        dz=0.1
    else:
        dz=0.12
    goal = C0_RobilTask.msg.RobilTaskGoal(name='Switch',uid='dz=0.1',parameters='operation=%d,dz=%f'%(dir, dz))#

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
        rospy.init_node('InitDrive_client')
        result =  Switch_client(1)
        if result:
            print 'Car is initialized and ready to go - hand brake released and brake pedal is pressed!'
    except rospy.ROSInterruptException:
        print "Initialization interrupted before completion"
