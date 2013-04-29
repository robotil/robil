#!/usr/bin/env python

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

from Abstractions.StateMachine import *
import math
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from BDI_StepStateMachine import *

###################################################################################
#--------------------------- Strategies -------------------------------------------
###################################################################################

class BDI_Strategy(object):
    """
    """
    def __init__(self):
        self._k_effort = [0] * 28

class BDI_StrategyIdleLeft(BDI_Strategy):
    """
    """
    def __init__(self):
        pass

class BDI_StrategyIdleRight(BDI_Strategy):
    """
    """
    def __init__(self):
        pass

    def GetAtlasSimInterfaceCommand(self):
        command = AtlasSimInterfaceCommand()

        command.behavior = AtlasSimInterfaceCommand.WALK
        
        # k_effort is all 0s for full BDI controll of all joints.
        command.k_effort = self._k_effort
        
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

###################################################################################
#--------------------------- States -----------------------------------------------
###################################################################################

class BDI_State(State):
    """
        The StepState class is intended to be used with the BDI_StateMachine class
    """    
    def __init__(self,strStateName):
    	State.__init__(self,strStateName)

class BDI_Idle(BDI_State):
    """
        The BDI_Idle class is intended to be used when not walking
    """
    def __init__(self):
        BDI_State.__init__(self,"Idle")

    def calculate_pose(self, step_index,pose,robot_position):
        i = step_index
        # Right foot occurs on even steps, left on odd
        pose.position.x = robot_position.x
        # Step 0.15m to either side of center, alternating with feet
        pose.position.y = 0.15 if (i%2==0) else -0.15
        
        # The z position is observed for static walking, but the foot
        # will be placed onto the ground if the ground is lower than z
        pose.position.z = 0.0
        
        # Point those feet straight ahead
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0

        return pose

class BDI_Forward(BDI_State):
    """
        The BDI_Idle class is intended to be used when not walking
    """
    def __init__(self):
    	BDI_State.__init__(self,"Forward")

    def calculate_pose(self, step_index,pose,robot_position):
        i = step_index
        # Right foot occurs on even steps, left on odd
        pose.position.x = robot_position.x + 0.3*i
        # Step 0.15m to either side of center, alternating with feet
        pose.position.y = 0.15 if (i%2==0) else -0.15
        
        # The z position is observed for static walking, but the foot
        # will be placed onto the ground if the ground is lower than z
        pose.position.z = 0.0
        
        # Point those feet straight ahead
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0

        return pose

class BDI_TrunLeft(BDI_State):
    """
        The BDI_Idle class is intended to be used when not walking
    """
    def __init__(self):
    	BDI_State.__init__(self,"Turn_Left")

    # This method is used to calculate a pose of step based on the step_index
    # The step poses just cause the robot to walk in a circle
    def calculate_pose(self, step_index,pose,robot_position):
        # Right foot occurs on even steps, left on odd
        is_right_foot = step_index % 2
        is_left_foot = 1 - is_right_foot
        
        # There will be 60 steps to a circle, and so our position along the circle is current_step
        current_step = step_index % 4
        
        # yaw angle of robot around circle
        theta = current_step * math.pi / 450
           
        R = 2 # Radius of turn
        W = 0.3 # Width of stride
        
        # Negative for inside foot, positive for outside foot
        offset_dir = 1 - 2 * is_right_foot

        # Radius from center of circle to foot
        R_foot = R + offset_dir * W/2
        
        # X, Y position of foot
        X = R_foot * math.sin(theta)
        Y = (R - R_foot*math.cos(theta))
        
        # Calculate orientation quaternion
        Q = quaternion_from_euler(0, 0, theta)
        pose.position.x = robot_position.x + X
        pose.position.y = robot_position.y + Y
        
        # The z position is observed for static walking, but the foot
        # will be placed onto the ground if the ground is lower than z
        pose.position.z = 0
        
        pose.orientation.x = Q[0]
        pose.orientation.y = Q[1]
        pose.orientation.z = Q[2]
        pose.orientation.w = Q[3]

        return pose

class BDI_TrunRight(BDI_State):
    """
        The BDI_Idle class is intended to be used when not walking
    """
    def __init__(self):
    	BDI_State.__init__(self,"Turn_Right")

    # This method is used to calculate a pose of step based on the step_index
    # The step poses just cause the robot to walk in a circle
    def calculate_pose(self, step_index,pose,robot_position):
        # Right foot occurs on even steps, left on odd
        is_right_foot = step_index % 2
        is_left_foot = 1 - is_right_foot
        
        # There will be 60 steps to a circle, and so our position along the circle is current_step
        current_step = step_index % 4
        
        # yaw angle of robot around circle
        theta = current_step * math.pi / 450
           
        R = 2 # Radius of turn
        W = 0.3 # Width of stride
        
        # Negative for inside foot, positive for outside foot
        offset_dir = 1 - 2 * is_left_foot

        # Radius from center of circle to foot
        R_foot = R + offset_dir * W/2
        
        # X, Y position of foot
        X = R_foot * math.sin(theta)
        Y = (R - R_foot*math.cos(theta))
        
        # Calculate orientation quaternion
        Q = quaternion_from_euler(0, 0, theta)
        pose.position.x = robot_position.x + X
        pose.position.y = robot_position.y + Y
        
        # The z position is observed for static walking, but the foot
        # will be placed onto the ground if the ground is lower than z
        pose.position.z = 0
        
        pose.orientation.x = Q[0]
        pose.orientation.y = Q[1]
        pose.orientation.z = Q[2]
        pose.orientation.w = Q[3]

        return pose

###################################################################################
#--------------------------- State Machine ----------------------------------------
###################################################################################

class BDI_StateMachine(StateMachine):
    """
        The StepStateMachine class is a finite state machine that handles the stepping logic for the ZMP module
    """
    
    def __init__(self):
        StateMachine.__init__(self,BDI_Idle())
        self._StepStateMachine = BDI_StepStateMachine()
        # Add states
        StateMachine.AddState(self,BDI_Forward())
        StateMachine.AddState(self,BDI_TrunLeft())
        StateMachine.AddState(self,BDI_TrunRight())
        
        # Add transitions
        StateMachine.AddTransition(self,"Idle",         "TurnRight",        "Turn_Right")
        StateMachine.AddTransition(self,"Idle",         "GoForward",        "Forward")
        StateMachine.AddTransition(self,"Idle",         "TurnLeft",         "Turn_Left")
        StateMachine.AddTransition(self,"Forward",      "Stop",         	"Idle")
        StateMachine.AddTransition(self,"Forward",      "TurnRight",     	"Turn_Right")
        StateMachine.AddTransition(self,"Forward",      "TurnLeft",         "Turn_Left")
        StateMachine.AddTransition(self,"Turn Right",   "Stop",         	"Idle")
        StateMachine.AddTransition(self,"Turn Right",   "GoForward",        "Forward")
        StateMachine.AddTransition(self,"Turn Right",   "TurnLeft",         "Forward")
        StateMachine.AddTransition(self,"Turn_Left",    "Stop",         	"Idle")
        StateMachine.AddTransition(self,"Turn_Left",    "GoForward",        "Forward")
        StateMachine.AddTransition(self,"Turn_Left",    "TurnRight",        "Forward")

    def GoForward(self):
        StateMachine.PerformTransition(self,"GoForward")

    def TurnRight(self):
        StateMachine.PerformTransition(self,"TurnRight")

    def TurnLeft(self):
        StateMachine.PerformTransition(self,"TurnLeft")


