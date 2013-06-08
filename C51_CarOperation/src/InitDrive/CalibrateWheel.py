#!/usr/bin/env python
import roslib; roslib.load_manifest('C51_CarOperation')
import rospy
import actionlib
import C0_RobilTask.msg
hand="right"
def WheelCalibrate_client(str):
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('CalibrateWheel', C0_RobilTask.msg.RobilTaskAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    goal = C0_RobilTask.msg.RobilTaskGoal(name='Wheel',uid='dz=0.1',parameters=str)#
        
    # Creates a goal to send to the action server.
    

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()
    #rospy.sleep(0.5)

    # Prints out the result of executing the action
    return client.get_result()  
def runCalibrate():
        result = WheelCalibrate_client("operation=2,angle=0.3")
        result =  WheelCalibrate_client("operation=2,angle=1.57")
        result =  WheelCalibrate_client("operation=1,angle=1.57")
        return result
if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('Calibrate_client')
        result = run()
        if result:
            print 'Car is initialized and ready to go - hand brake released and brake pedal is pressed!'
    except rospy.ROSInterruptException:
        print "Initialization interrupted before completion"
