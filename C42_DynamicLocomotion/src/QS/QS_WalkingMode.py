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

from C42_DynamicLocomotion.srv import *
from C42_DynamicLocomotion.msg import Foot_Placement_data

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
        self._command = 0
        
    def Initialize(self):
        WalkingMode.Initialize(self)
        # Subscribers:
        self._odom_sub = rospy.Subscriber('/ground_truth_odom',Odometry,self._odom_cb)
        self.asi_state = rospy.Subscriber('/atlas/atlas_sim_interface_state', AtlasSimInterfaceState, self.asi_state_cb)
        self._atlas_imu_sub = rospy.Subscriber('/atlas/imu', Imu, self._get_imu)

        rospy.wait_for_service('foot_placement_path')
        self._foot_placement_client = rospy.ServiceProxy('foot_placement_path', FootPlacement_Service)       
        self._RequestFootPlacements()
        rospy.sleep(0.3)
    
        k_effort = [0] * 28
        self._bDone = False
        self._bIsSwaying = False
        
        self._command = 0
    
    def StartWalking(self):
        self._bDone = False
    
    def Walk(self):
        WalkingMode.Walk(self)
        self._command = self.GetCommand()
        self.asi_command.publish(self._command)
        #print(command)
        self._WalkingModeStateMachine.PerformTransition("Go")
        self._bIsSwaying = True
    
    def EmergencyStop(self):
        WalkingMode.Stop(self)

    def IsDone(self):
        return self._bDone
    
    def GetCommand(self):
        command = AtlasSimInterfaceCommand()
        command.behavior = AtlasSimInterfaceCommand.STEP
        command.k_effort = [0] * 28
        command.step_params.desired_step = self._LPP.GetNextStep()
        if(0 != command.step_params.desired_step):
            # Not sure why such a magic number
            command.step_params.desired_step.duration = 0.63
            # Apparently this next line is a must
            command.step_params.desired_step.step_index = 1
        else:
            command = 0
        return command
    
    def HandleStateMsg(self,state):
        command = 0
        if ("Idle" == self._WalkingModeStateMachine.GetCurrentState().Name):
            self._Odometer.SetPosition(state.pos_est.position.x,state.pos_est.position.y)
        elif ("Wait" == self._WalkingModeStateMachine.GetCurrentState().Name):
            self._Odometer.SetPosition(state.pos_est.position.x,state.pos_est.position.y)
            print("Odometer Updated")
            #print(2)
            self._WalkingModeStateMachine.PerformTransition("Go")
        elif ("Walking" == self._WalkingModeStateMachine.GetCurrentState().Name):
            #print(3)
            if (QS_PathPlannerEnum.Active == self._LPP.State):
                command = self.GetCommand()
            elif(QS_PathPlannerEnum.Waiting == self._LPP.State):
                self._RequestFootPlacements()
        elif ("Done" == self._WalkingModeStateMachine.GetCurrentState().Name):
            #print(4)
            self._bDone = True
        else:
            raise Exception("QS_WalkingModeStateMachine::Bad State Name")
    
        return command
    
    def _RequestFootPlacements(self):
        print("Request Foot Placements")
        # Perform a service request from FP
        try:
            # Handle preemption?
                # if received a "End of mission" sort of message from FP
            resp = self._foot_placement_client(0)
            if 1 == resp.done:
                self._WalkingModeStateMachine.PerformTransition("Finished")
            else:
                listSteps = []
                for desired in resp.foot_placement_path:
                    #print("Desired: ",desired)
                    command = AtlasSimInterfaceCommand()
                    step = command.step_params.desired_step
                    step.foot_index = desired.foot_index
                    step.swing_height = desired.clearance_height#0.4
                    step.pose.position.x = desired.pose.position.x
                    step.pose.position.y = desired.pose.position.y
                    step.pose.position.z = desired.pose.position.z
                    Q = quaternion_from_euler(desired.pose.ang_euler.x, desired.pose.ang_euler.y, desired.pose.ang_euler.z)
                    step.pose.orientation.x = Q[0]
                    step.pose.orientation.y = Q[1]
                    step.pose.orientation.z = Q[2]
                    step.pose.orientation.w = Q[3]
                    listSteps.append(step)
                self._LPP.SetPath(listSteps)
                #print("listSteps: ",listSteps)
        except rospy.ServiceException, e:
            print "Foot Placement Service call failed: %s"%e
    
    
###################################################################################
#--------------------------- CallBacks --------------------------------------------
###################################################################################

    # /atlas/atlas_sim_interface_state callback. Before publishing a walk command, we need
    # the current robot position   
    def asi_state_cb(self, state):
        command = 0
        #print("Current: ",state.current_behavior,"Desired: ",state.desired_behavior)
        # When the robot status_flags are 1 (SWAYING), you can publish the next step command.
        if (state.step_feedback.status_flags == 1 and not self._bIsSwaying):
            command = self.HandleStateMsg(state)
        elif (state.step_feedback.status_flags == 2 and self._bIsSwaying):
            self._bIsSwaying = False
            print("step done")
        if (0 !=command):
            self._command = command
            self._bIsSwaying = True
        
        if(0 == state.current_behavior and 0 != self._command):
            self.asi_command.publish(self._command)
            self._command = 0
            print("step start")
            

    def _odom_cb(self,odom):
        self._LPP.UpdatePosition(odom.pose.pose.position.x,odom.pose.pose.position.y)
 
    def _get_imu(self,msg):  #listen to /atlas/imu/pose/pose/orientation
        roll, pitch, yaw = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self._Odometer.SetYaw(yaw)


###############################################################################################


