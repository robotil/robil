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
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from BDI_StateMachine import *

import math
import rospy
import sys

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

class WalkingModeBDI(WalkingMode):

    def __init__(self,localPathPlanner):
        # Initialize atlas mode and atlas_sim_interface_command publishers        
        self.mode = rospy.Publisher('/atlas/mode', String, None, False, True, None)
        self.asi_command = rospy.Publisher('/atlas/atlas_sim_interface_command', AtlasSimInterfaceCommand, None, False, True, None)
        self._StateMachine = BDI_StateMachine()
        self._LPP = localPathPlanner
        self._yaw = 0.0
                
    def Initialize(self):
        # Puts robot into freeze behavior, all joints controlled
        # Put the robot into a known state
        k_effort = [0] * 28
        freeze = AtlasSimInterfaceCommand(None,AtlasSimInterfaceCommand.FREEZE, None, None, None, None, k_effort )
        self.asi_command.publish(freeze)
        
        # Puts robot into stand_prep behavior, a joint configuration suitable
        # to go into stand mode
        stand_prep = AtlasSimInterfaceCommand(None,AtlasSimInterfaceCommand.STAND_PREP, None, None, None, None, k_effort)
        self.asi_command.publish(stand_prep)

        rospy.sleep(2.0)
        self.mode.publish("nominal")
        
        # Put robot into stand position
        stand = AtlasSimInterfaceCommand(None,AtlasSimInterfaceCommand.STAND, None, None, None, None, k_effort)
                
        rospy.sleep(0.3)
        
        self.asi_command.publish(stand)
        
        # Initialize some variables before starting.
        self.step_index = 0
        self.is_swaying = False
    
    def StartWalking(self):
        return 0.3
    
    def Walk(self):
        # Subscribe to atlas_state and atlas_sim_interface_state topics.
        self.asi_state = rospy.Subscriber('/atlas/atlas_sim_interface_state', AtlasSimInterfaceState, self.asi_state_cb)
        self.atlas_state = rospy.Subscriber('/atlas/atlas_state', AtlasState, self.atlas_state_cb)

        self._StateMachine.GoForward()

        while not rospy.is_shutdown():
            rospy.spin()
    
    def Stop(self):
        pass
    
    def EmergencyStop(self):
        pass    

###################################################################################
#--------------------------- CallBacks --------------------------------------------
###################################################################################

    # /atlas/atlas_sim_interface_state callback. Before publishing a walk command, we need
    # the current robot position   
    def asi_state_cb(self, state):
        if (self._LPP.IsActive()):
            try:
                x = self.robot_position.x
            except AttributeError:            
                self.robot_position = Point()
                self.robot_position.x = state.pos_est.position.x
                self.robot_position.y = state.pos_est.position.y
                self.robot_position.z = state.pos_est.position.z  
            
            command = AtlasSimInterfaceCommand()

            command.behavior = AtlasSimInterfaceCommand.WALK
            
            # k_effort is all 0s for full BDI controll of all joints.
            command.k_effort = [0] * 28
            
            # Observe next_step_index_needed to determine when to switch steps.
            self.step_index = state.behavior_feedback.walk_feedback.next_step_index_needed

            self._LPP.UpdatePosition(self.robot_position.x,self.robot_position.y)

            targetYaw = self._LPP.GetTargetYaw()
            delatYaw = targetYaw - self._yaw

            print(delatYaw)
            if (delatYaw > 0.025):
                self._StateMachine.TurnLeft()
                print("Left")
            elif (delatYaw < -0.025):
                self._StateMachine.TurnRight()
                print("Right")
            else:
                self._StateMachine.GoForward()
            
            # A walk behavior command needs to know three additional steps beyond the current step needed to plan
            # for the best balance
            for i in range(4):
                step_index = self.step_index + i
                is_right_foot = step_index % 2
                
                command.walk_params.step_data[i].step_index = step_index
                command.walk_params.step_data[i].foot_index = is_right_foot
                
                # A duration of 0.63s is a good default value
                command.walk_params.step_data[i].duration = 0.63
                
                # As far as I can tell, swing_height has yet to be implemented
                command.walk_params.step_data[i].swing_height = 0.2

                # Determine pose of the next step based on the step_index
                self._StateMachine.GetCurrentState().calculate_pose(step_index,command.walk_params.step_data[i].pose,self.robot_position)
            
            # Publish this command every time we have a new state message
            self.asi_command.publish(command)
 
    # /atlas/atlas_state callback. This message provides the orientation of the robot from the torso IMU
    # This will be important if you need to transform your step commands from the robot's local frame to world frame
    def atlas_state_cb(self, state):
        # If you don't reset to harnessed, then you need to get the current orientation
        roll, pitch, yaw = euler_from_quaternion([state.orientation.x, state.orientation.y, state.orientation.z, state.orientation.w])
        self._yaw = yaw
        return True
