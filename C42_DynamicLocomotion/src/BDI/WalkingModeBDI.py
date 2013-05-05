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
from BDI_Odometer import *

import math
import rospy
import sys

from nav_msgs.msg import Odometry

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

class WalkingModeBDI(WalkingMode):

    def __init__(self,localPathPlanner):
        self.step_index_for_reset = 0
        # Initialize atlas mode and atlas_sim_interface_command publishers        
        self.mode = rospy.Publisher('/atlas/mode', String, None, False, True, None)
        self.asi_command = rospy.Publisher('/atlas/atlas_sim_interface_command', AtlasSimInterfaceCommand, None, False, True, None)
        self._Odometer = BDI_Odometer()
        self._StateMachine = BDI_StateMachine(self._Odometer)
        self._LPP = localPathPlanner
        #self._odom_position = Pose()
        self._odom_sub = rospy.Subscriber('/ground_truth_odom',Odometry,self._odom_cb)

    def Initialize(self):
        self._bDone = False
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

        self.is_swaying = False
        self._StateMachine.Initialize(self.step_index_for_reset)
    
    def StartWalking(self):
        self._bDone = False
        return 0.3
    
    def Walk(self):
        # Subscribe to atlas_state and atlas_sim_interface_state topics.
        self.asi_state = rospy.Subscriber('/atlas/atlas_sim_interface_state', AtlasSimInterfaceState, self.asi_state_cb)
    
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
        if (not self._bDone):
          #if (self._LPP.IsActive()):
              #print(state)
          self.step_index_for_reset = state.behavior_feedback.walk_feedback.next_step_index_needed - 1
          # This weird little piece of code is supposed to initialize the odometer
          try:
              x = self.robot_position.x
          except AttributeError:            
              self.robot_position = Point()
              self.robot_position.x = state.pos_est.position.x
              self.robot_position.y = state.pos_est.position.y
              self.robot_position.z = state.pos_est.position.z
              self._Odometer.SetPosition(state.pos_est.position.x,state.pos_est.position.y)
              self._StateMachine.Initialize(self.step_index_for_reset)
              self._StateMachine.GoForward()
          
          if (self._LPP.IsActive()):
              # x,y = self._Odometer.GetGlobalPosition()
              # self._LPP.UpdatePosition(x,y)
              self._StateMachine.SetPathError(self._LPP.GetPathError())
          
              targetYaw = self._LPP.GetTargetYaw()
              delatYaw = targetYaw - self._Odometer.GetYaw()
              
              debug_transition_cmd = "NoCommand"
              if (math.sin(delatYaw) > 0.6):
                #print("Sin(Delta)",math.sin(delatYaw), "Left")
                debug_transition_cmd = "TurnLeft"
                self._StateMachine.TurnLeft(targetYaw)
              elif (math.sin(delatYaw) < -0.6):
                #print("Sin(Delta)",math.sin(delatYaw), "Right")
                debug_transition_cmd = "TurnRight"
                self._StateMachine.TurnRight(targetYaw)
          else:
              debug_transition_cmd = "Stop"
              self._StateMachine.Stop()
              
          self._bDone = self._StateMachine.IsDone()

          command = self._StateMachine.Step(state.behavior_feedback.walk_feedback.next_step_index_needed)
          
          if (0 !=command):
              self.asi_command.publish(command)
              rospy.loginfo("WalkingModeBDI, asi_state_cb: State Machine Transition Cmd = %s" % (debug_transition_cmd) )

    def _odom_cb(self,odom):
        # SHOULD USE:
        self._LPP.UpdatePosition(odom.pose.pose.position.x,odom.pose.pose.position.y)
        #self._odom_position = odom.pose.pose
 