#!/usr/bin/env python

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

import time
import math
import rospy
import sys
import copy
import PyKDL
import roslib
roslib.load_manifest('C42_DynamicLocomotion')

from Abstractions.WalkingMode import *
from Abstractions.Odometer import *
from QS_PathPlanner import *

from tf_conversions import posemath
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from atlas_msgs.msg import AtlasCommand, AtlasSimInterfaceCommand, AtlasSimInterfaceState, AtlasState, AtlasBehaviorStepData
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from std_msgs.msg import String

class QS_WalkingMode(WalkingMode):
    def __init__(self):
        self._LPP = QS_PathPlanner()
        WalkingMode.__init__(self,self._LPP)
        self.step_index_for_reset = 0
        # Initialize atlas atlas_sim_interface_command publisher       
        self.asi_command = rospy.Publisher('/atlas/atlas_sim_interface_command', AtlasSimInterfaceCommand, None, False, True, None)
        self._Odometer = Odometer()
        self._bDone = False
        self._bIsSwaying = False
        
    def Initialize(self):
        WalkingMode.Initialize(self)
        # Subscribers:
        self._odom_sub = rospy.Subscriber('/ground_truth_odom',Odometry,self._odom_cb)
        self.asi_state = rospy.Subscriber('/atlas/atlas_sim_interface_state', AtlasSimInterfaceState, self.asi_state_cb)
        self._atlas_imu_sub = rospy.Subscriber('/atlas/imu', Imu, self._get_imu)
        
        # Subscribe to some other topic / service
        #self._path_sub = rospy.Subscriber('/path',C31_Waypoints,self._path_cb)
        rospy.sleep(0.3)
    
        k_effort = [0] * 28
        self._bDone = False
        self._bIsSwaying = False
    
    def StartWalking(self):
        self._bDone = False
    
    def Walk(self):
        WalkingMode.Walk(self)
    
    def EmergencyStop(self):
        WalkingMode.Stop(self)

    def IsDone(self):
        return self._bDone
    
    def GetCommand(self):
        command = AtlasSimInterfaceCommand()
        command.behavior = AtlasSimInterfaceCommand.STEP
        command.step_params = self._LPP.GetNextStep()
        # Not sure why such a magic number
        command.step_params.desired_step.duration = 0.63
        # Apparently this next line is a must
        command.step_params.desired_step.step_index = 1
        return command
    
    def HandleStateMsg(self,state):
        command = 0
        if ("Idle" == self._WalkingModeStateMachine.GetCurrentState().Name):
            pass
        elif ("Wait" == self._WalkingModeStateMachine.GetCurrentState().Name):
            self._Odometer.SetPosition(state.pos_est.position.x,state.pos_est.position.y)
            self._WalkingModeStateMachine.PerformTransition("Go")
        elif ("Walking" == self._WalkingModeStateMachine.GetCurrentState().Name):
            if (QS_PathPlannerEnum.Active == self._LPP.State):
                command = self.GetCommand()
        elif ("Done" == self._WalkingModeStateMachine.GetCurrentState().Name):
            self._bDone
        else:
            raise Exception("QS_WalkingModeStateMachine::Bad State Name")
    
        return command
    
###################################################################################
#--------------------------- CallBacks --------------------------------------------
###################################################################################

    def _path_cb(self,path):
        rospy.loginfo('got path %s',path)
        p = []
        for wp in path.points:
            p.append(Waypoint(wp.x,wp.y))
        self.SetPath(p)

    # /atlas/atlas_sim_interface_state callback. Before publishing a walk command, we need
    # the current robot position   
    def asi_state_cb(self, state):
        command = self.HandleStateMsg(state)
        if (0 !=command):
            self.asi_command.publish(command)

    def _odom_cb(self,odom):
        self._LPP.UpdatePosition(odom.pose.pose.position.x,odom.pose.pose.position.y)
 
    def _get_imu(self,msg):  #listen to /atlas/imu/pose/pose/orientation
        roll, pitch, yaw = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self._Odometer.SetYaw(yaw)


###############################################################################################


