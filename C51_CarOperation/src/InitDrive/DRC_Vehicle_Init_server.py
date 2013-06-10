#!/usr/bin/env python
import roslib; roslib.load_manifest('C51_CarOperation')
import rospy
import actionlib
import C51_CarOperation.msg
from geometry_msgs.msg import Twist
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from std_msgs.msg import  Float64
from geometry_msgs.msg import Pose
from directionClient import Switch_client
from HBClient import HB_client
from CalibrateWheel import WheelCalibrate_client, runCalibrate
car='drc_vehicle'#'golf_cart'#

class InitDrive(object):
    _feedback = C51_CarOperation.msg.DriveFeedback()
    _result   = C51_CarOperation.msg.DriveResult()
    
    def __init__(self, name):    
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, C51_CarOperation.msg.DriveAction, execute_cb=self.InitDriveCallback, auto_start=False)
        self._as.start()
    
    def InitDriveCallback(self, goal):
        # helper variables
        success = True
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            success = False
            
        if success:
            #DRC Vehicles controllers online - should be replaced with C67 module
            #Switch_client(1)
            HB_client(1, 0)         #handbrake online
            gasP=Gas() #gas pedal online
            brakeP=Brake() #gas pedal online
            result = runCalibrate() #Initialize Steering wheel  
            #Steer=SW()      #steering wheel online
            # - Press on brake - and release handbrake
            #brakeP.brake(1)
            #gasP.gas(0)

            self._feedback.complete = 50
            self._feedback.complete = 100
            self._result.success = 0
            self._result.description = "init car"
            self._result.plan = "init car"
            
            self._as.publish_feedback(self._feedback)
        if success:
            rospy.loginfo('Car is initialized and ready to go - hand brake released and brake pedal is pressed!')
            self._as.set_succeeded(self._result)
        #----------------hand brake released --------------------------#


class Gas:
    status=-11
    pub=0
    sub=0
    def __init__(self):
        self.pub = rospy.Publisher(car+'/gas_pedal/cmd', Float64)
        self.sub = rospy.Subscriber('/'+car+'/gas_pedal/state', Float64, self.gasCallback)
    def gasCallback(self, data):
        self.status=data.data

    def gas(self, num):
        self.pub.publish(num)

class Brake:
    status=-11
    pub=0
    sub=0
    def __init__(self):
        self.pub = rospy.Publisher(car+'/brake_pedal/cmd', Float64)
        self.sub = rospy.Subscriber('/'+car+'/brake_pedal/state', Float64, self.brakeCallback)
    def brakeCallback(self, data):
        self.status=data.data

    def brake(self, num):
        self.pub.publish(num)

class SW:
    status=0
    pub=0
    sub=0
    def __init__(self):
        self.pub = rospy.Publisher(car+'/hand_wheel/cmd', Float64)
        self.sub = rospy.Subscriber('/'+car+'/hand_wheel/state', Float64, self.SWCallback)
    def SWCallback(self, data):
        self.status=data.data

    def turn(self, num):
        self.pub.publish(num)

if __name__ == '__main__':
    try:
        rospy.init_node('InitDrive')
        InitDrive(rospy.get_name())   
        rospy.spin()
    except rospy.ROSInterruptException: 
        print "Did not initialized vehicle"


