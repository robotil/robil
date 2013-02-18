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


class InitDrive(object):
    _feedback = C51_CarOperation.msg.InitDriveFeedback()
    _result   = C51_CarOperation.msg.InitDriveResult()
    
    def __init__(self, name):    
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, C51_CarOperation.msg.InitDriveAction, execute_cb=self.InitDriveCallback, auto_start=False)
        self._as.start()
    
    def InitDriveCallback(self, goal):
        # helper variables
        success = True
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            success = False
            
        if goal.PerformJob:
            #DRC Vehicles controllers online - should be replaced with C67 module
            hb=handB()         #handbrake online
            gasP=Gas() #gas pedal online
            brakeP=Brake() #gas pedal online
            Steer=SW()      #steering wheel online
            # - Press on brake - and release handbrake
            brakeP.brake(0)
            hb.releaseHB()     #release handbrake
            self._feedback.HandBrake = 1
            self._feedback.SteeringWheel = 1
            self._feedback.GasPedal = 1
            self._feedback.BrakePedal = 1
            self._result.Success = 1
            
            self._as.publish_feedback(self._feedback)
        if success:
            rospy.loginfo('Car is initialized and ready to go - hand brake released and brake pedal is pressed!')
            self._as.set_succeeded(self._result)
        #----------------hand brake released --------------------------#

class handB:

    handbrake=1
    pub=0
    sub=0
    def __init__(self):
        self.pub = rospy.Publisher('drc_vehicle/hand_brake/cmd', Float64)
        self.sub = rospy.Subscriber('/drc_vehicle/hand_brake/state', Float64, self.handbrakeCallback)
    def handbrakeCallback(self, data):
        self.handbrake=data.data

    def releaseHB(self):
        while self.handbrake>0.5 :
            self.pub.publish(0.0)

    def pullHB(self):
        self.pub.publish(1.0)
        #while self.handbrake<0.5 :


class Gas:
    status=-11
    pub=0
    sub=0
    def __init__(self):
        self.pub = rospy.Publisher('drc_vehicle/gas_pedal/cmd', Float64)
        self.sub = rospy.Subscriber('/drc_vehicle/gas_pedal/state', Float64, self.gasCallback)
    def gasCallback(self, data):
        self.status=data.data

    def gas(self, num):
        self.pub.publish(num)

class Brake:
    status=-11
    pub=0
    sub=0
    def __init__(self):
        self.pub = rospy.Publisher('drc_vehicle/brake_pedal/cmd', Float64)
        self.sub = rospy.Subscriber('/drc_vehicle/brake_pedal/state', Float64, self.brakeCallback)
    def brakeCallback(self, data):
        self.status=data.data

    def brake(self, num):
        self.pub.publish(num)

class SW:
    status=0
    pub=0
    sub=0
    def __init__(self):
        self.pub = rospy.Publisher('drc_vehicle/hand_wheel/cmd', Float64)
        self.sub = rospy.Subscriber('/drc_vehicle/hand_wheel/state', Float64, self.SWCallback)
    def SWCallback(self, data):
        self.status=data.data

    def turn(self, num):
        self.pub.publish(num)

if __name__ == '__main__':
    try:
        rospy.init_node('InitDrive_server')
        InitDrive(rospy.get_name())   
        rospy.spin()
    except rospy.ROSInterruptException: 
        print "Did not initialized vehicle"

