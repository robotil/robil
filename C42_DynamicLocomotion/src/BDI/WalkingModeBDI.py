#!/usr/bin/env python
import roslib
roslib.load_manifest('C42_DynamicLocomotion')
from Abstractions.WalkingMode import *
import time
from atlas_msgs.msg import AtlasCommand, AtlasSimInterfaceCommand, AtlasSimInterfaceState, AtlasState, AtlasBehaviorStepData
from sensor_msgs.msg import Imu
import PyKDL
from tf_conversions import posemath
from atlas_msgs.msg import AtlasSimInterfaceCommand, AtlasSimInterfaceState, AtlasState
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from sensor_msgs.msg import Imu

from BDI_Odometer import *
from BDI_WalkingModeStateMachine import *

import math
import rospy
import sys

from nav_msgs.msg import Odometry

from geometry_msgs.msg import Pose
from BDI_Strategies import *
from Abstractions.StepQueue import *

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

class WalkingModeBDI(WalkingMode):
    def __init__(self,localPathPlanner):
        self.step_index_for_reset = 0
        # Initialize atlas mode and atlas_sim_interface_command publishers        
        self.asi_command = rospy.Publisher('/atlas/atlas_sim_interface_command', AtlasSimInterfaceCommand, None, False, True, None)
        self._Odometer = BDI_Odometer()
        ##############################
        self._StrategyForward = BDI_StrategyForward(self._Odometer)
        self._StepQueue = StepQueue()
        ##############################
        self._LPP = localPathPlanner
        self._StateMachine = BDI_WalkingModeStateMachine(self._Odometer,self._LPP)
        #self._odom_position = Pose()
        self._odom_sub = rospy.Subscriber('/ground_truth_odom',Odometry,self._odom_cb)
        self._bDone = False
        self.asi_state = rospy.Subscriber('/atlas/atlas_sim_interface_state', AtlasSimInterfaceState, self.asi_state_cb)
        self._atlas_imu_sub = rospy.Subscriber('/atlas/imu', Imu, self._get_imu)
        rospy.sleep(0.3)

    def Initialize(self):
        self._bDone = False
        # # Puts robot into freeze behavior, all joints controlled
        # # Put the robot into a known state
        k_effort = [0] * 28
        # freeze = AtlasSimInterfaceCommand(None,AtlasSimInterfaceCommand.FREEZE, None, None, None, None, k_effort )
        # self.asi_command.publish(freeze)
        
        # # Puts robot into stand_prep behavior, a joint configuration suitable
        # # to go into stand mode
        # stand_prep = AtlasSimInterfaceCommand(None,AtlasSimInterfaceCommand.STAND_PREP, None, None, None, None, k_effort)
        # self.asi_command.publish(stand_prep)

        # rospy.sleep(2.0)
        # self.mode.publish("nominal")
        
        # Put robot into stand position
        stand = AtlasSimInterfaceCommand(None,AtlasSimInterfaceCommand.STAND, None, None, None, None, k_effort)
                
        rospy.sleep(0.3)
        
        self.asi_command.publish(stand)
        
        # Initialize some variables before starting.

        self.is_swaying = False
        self._StateMachine.Initialize(self.step_index_for_reset)
    
    def StartWalking(self):
        self._bDone = False
        return 0.3
    
    def Walk(self):
      self._yaw = 0
      self._StateMachine.Walk(self._yaw)
    
    def Stop(self):
      self._StateMachine.Stop()
    
    def EmergencyStop(self):
        # Puts robot into freeze behavior, all joints controlled
        # Put the robot into a known state
        k_effort = [0] * 28
        # freeze = AtlasSimInterfaceCommand(None,AtlasSimInterfaceCommand.FREEZE, None, None, None, None, k_effort )
        # self.asi_command.publish(freeze)
        # rospy.sleep(0.3)
        
        # Put robot into stand position
        stand = AtlasSimInterfaceCommand(None,AtlasSimInterfaceCommand.STAND, None, None, None, None, k_effort)

    def IsDone(self):
        return self._bDone

###################################################################################
#--------------------------- CallBacks --------------------------------------------
###################################################################################

    # /atlas/atlas_sim_interface_state callback. Before publishing a walk command, we need
    # the current robot position   
    def asi_state_cb(self, state):
        if (14.8 < self._LPP.GetPos().GetX()) and (self._LPP.GetPos().GetX() < 20):
            if (True == self._setStep):
                self.step_index = state.behavior_feedback.walk_feedback.next_step_index_needed -1
                self._StepQueue.Initialize(self._StrategyForward.GetStepData(self.step_index+1),self._StrategyForward.GetStepData(self.step_index+2),self._StrategyForward.GetStepData(self.step_index+3),self._StrategyForward.GetStepData(self.step_index+4))
                self._setStep = False
            command = self.SteppingStoneCommand(state);
            #print(command)
        else:
            command = self._StateMachine.HandleStateMsg(state)
            self._bDone = self._StateMachine.IsDone()
            ######################
            self._setStep = True

        # command = self._StateMachine.HandleStateMsg(state)
        # self._bDone = self._StateMachine.IsDone()
        if (0 !=command):
            self.asi_command.publish(command)

    def _odom_cb(self,odom):
        # SHOULD USE:
        self._LPP.UpdatePosition(odom.pose.pose.position.x,odom.pose.pose.position.y)
        #self._odom_position = odom.pose.pose
 
    def _get_imu(self,msg):  #listen to /atlas/imu/pose/pose/orientation
        roll, pitch, self._yaw = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])


###############################################################################################


    def SteppingStoneCommand(self,state):
        command = 0 
        nextindex = state.behavior_feedback.walk_feedback.next_step_index_needed
        if (nextindex>self.step_index):
            self.step_index += 1
            command = self.GetCommand()
            # Instead of generating new steps, just pop a predefined queue
            stepData = self._StepQueue.Push(self._StrategyForward.GetStepData(self.step_index+4))
            print("SteppingStoneCommand:: index: ",self.step_index)
        return command

    def SteppingStoneCommandStatic(self, state):
        
        # When the robot status_flags are 1 (SWAYING), you can publish the next step command.
        if (state.behavior_feedback.step_feedback.status_flags == 1 and self._setStep):
            self.step_index += 1
            self._setStep = False
            print("Step " + str(self.step_index))
        elif (state.behavior_feedback.step_feedback.status_flags == 2):
            self._setStep = True
            print("No Step")
        
        is_right_foot = self.step_index % 2
        
        command = AtlasSimInterfaceCommand()
        command.behavior = AtlasSimInterfaceCommand.STEP

        # k_effort is all 0s for full bdi control of all joints
        command.k_effort = [0] * 28
        
        # step_index should always be one for a step command
        command.step_params.desired_step.step_index = 1
        command.step_params.desired_step.foot_index = is_right_foot
        
        # duration has far as I can tell is not observed
        command.step_params.desired_step.duration = 0.63
        
        # swing_height is not observed
        command.step_params.desired_step.swing_height = 0.1

        #if self.step_index > 30:
            #print(str(self.calculate_pose(self.step_index)))
        # Determine pose of the next step based on the number of steps we have taken
        command.step_params.desired_step.pose = self.calculate_pose(self.step_index,state)
        
        return command

    # This method is used to calculate a pose of step based on the step_index
    # The step poses just cause the robot to walk in a circle
    def calculate_pose(self, step_index,state):
        # Right foot occurs on even steps, left on odd
        is_right_foot = step_index % 2
        is_left_foot = 1 - is_right_foot
                      
        # Calculate orientation quaternion
        Q = quaternion_from_euler(0, 0, 0)
        pose = Pose()
        pose.position.x = state.pos_est.position.x - 3
        pose.position.y = state.pos_est.position.y + 0.15*is_left_foot
        
        # The z position is observed for static walking, but the foot
        # will be placed onto the ground if the ground is lower than z
        pose.position.z = 0
        
        pose.orientation.x = Q[0]
        pose.orientation.y = Q[1]
        pose.orientation.z = Q[2]
        pose.orientation.w = Q[3]

        return pose

    def GetCommand(self):
        command = AtlasSimInterfaceCommand()
        command.behavior = AtlasSimInterfaceCommand.WALK
        for i in range(4):
            command.walk_params.step_data[i] = self._StepQueue.Peek(i)
        return command