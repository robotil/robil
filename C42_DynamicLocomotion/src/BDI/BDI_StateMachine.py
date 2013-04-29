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

from atlas_msgs.msg import AtlasCommand, AtlasSimInterfaceCommand, AtlasSimInterfaceState, AtlasState, AtlasBehaviorStepData

###################################################################################
#--------------------------- Strategies -------------------------------------------
###################################################################################

class BDI_Strategy(object):
    """
    """
    def __init__(self):
        self._k_effort = [0] * 28

        self._command = AtlasSimInterfaceCommand()
        self._command.behavior = AtlasSimInterfaceCommand.WALK

        for i in range(4):
            step_index = i
            self._command.walk_params.step_data[i].step_index = step_index

            self._command.walk_params.step_data[i].foot_index = step_index%2
            
            # A duration of 0.63s is a good default value
            self._command.walk_params.step_data[i].duration = 0.63
            
            # As far as I can tell, swing_height has yet to be implemented
            self._command.walk_params.step_data[i].swing_height = 0.2

            # Determine pose of the next step based on the step_index
            # Right foot occurs on even steps, left on odd
            self._command.walk_params.step_data[i].pose.position.x = 0
            # Step 0.15m to either side of center, alternating with feet
            self._command.walk_params.step_data[i].pose.position.y = 0.15 if (step_index%2==0) else -0.15
            
            # The z position is observed for static walking, but the foot
            # will be placed onto the ground if the ground is lower than z
            self._command.walk_params.step_data[i].pose.position.z = 0.0
            
            # Point those feet straight ahead
            self._command.walk_params.step_data[i].pose.orientation.x = 0.0
            self._command.walk_params.step_data[i].pose.orientation.y = 0.0
            self._command.walk_params.step_data[i].pose.orientation.z = 0.0
            self._command.walk_params.step_data[i].pose.orientation.w = 1.0

    def GetAtlasSimInterfaceCommand(self,index,robot_position):       
        # A walk behavior command needs to know three additional steps beyond the current step needed to plan
        # for the best balance
        for i in range(4):
            step_index = index + i
            is_right_foot = step_index % 2
            
            self._command.walk_params.step_data[i].step_index = step_index
            self._command.walk_params.step_data[i].foot_index = step_index%2

            # Determine pose of the next step based on the step_index
            # Right foot occurs on even steps, left on odd
            self._command.walk_params.step_data[i].pose.position.x = robot_position.x
            # Step 0.15m to either side of center, alternating with feet
            self._command.walk_params.step_data[i].pose.position.y = 0.15 if (step_index%2==0) else -0.15

        return self._command

class BDI_StrategyIdle(BDI_Strategy):
    """
    """
    def __init__(self):
        BDI_Strategy.__init__(self)
        self._command.behavior = AtlasSimInterfaceCommand.STAND

class BDI_StrategyForward(BDI_Strategy):
    """
    """
    def __init__(self):
        BDI_Strategy.__init__(self)

    def GetAtlasSimInterfaceCommand(self,index,robot_position):       
        # A walk behavior command needs to know three additional steps beyond the current step needed to plan
        # for the best balance
        print(robot_position.x)
        for i in range(4):
            step_index = index + i
            is_right_foot = step_index % 2
            
            self._command.walk_params.step_data[i].step_index = step_index
            self._command.walk_params.step_data[i].foot_index = step_index%2

            # Determine pose of the next step based on the step_index
            # Right foot occurs on even steps, left on odd
            self._command.walk_params.step_data[i].pose.position.x = robot_position.x + 0.4*step_index
            # Step 0.15m to either side of center, alternating with feet
            self._command.walk_params.step_data[i].pose.position.y = 0.1 if (step_index%2==0) else -0.1
        return self._command


###################################################################################
#--------------------------- States -----------------------------------------------
###################################################################################

class BDI_State(State):
    """
        The StepState class is intended to be used with the BDI_StateMachine class
    """    
    def __init__(self,strStateName):
    	State.__init__(self,strStateName)
        self._Strategy = BDI_StrategyIdle()

class BDI_Idle(BDI_State):
    """
        The BDI_Idle class is intended to be used when not walking
    """
    def __init__(self):
        BDI_State.__init__(self,"Idle")

    def calculate_pose(self, step_index,pose,robot_position):



        return pose

class BDI_Forward(BDI_State):
    """
        The BDI_Idle class is intended to be used when not walking
    """
    def __init__(self):
    	BDI_State.__init__(self,"Forward")

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
        self._StrategyIdle = BDI_StrategyIdle()
        self._StrategyForward = BDI_StrategyForward()
        self._StepStateMachine = BDI_StepStateMachine()
        self._StepStateMachine.SetStrategy(self._StrategyIdle,self._StrategyIdle)
        self._index = 0

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
        self._StepStateMachine.SetStrategy(self._StrategyForward,self._StrategyForward)

    def TurnRight(self):
        StateMachine.PerformTransition(self,"TurnRight")
        self._StepStateMachine.SetStrategy(self._StrategyIdle,self._StrategyIdle)

    def TurnLeft(self):
        StateMachine.PerformTransition(self,"TurnLeft")
        self._StepStateMachine.SetStrategy(self._StrategyIdle,self._StrategyIdle)

    def Step(self,robot_position,nextindex):
        if (nextindex>self._index):
            self._StepStateMachine.Step()
        return self._StepStateMachine.GetCommand(robot_position,nextindex)


