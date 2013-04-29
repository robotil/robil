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
    def __init__(self,odometer):

        self._Odometer = odometer

        self._k_effort = [0] * 28
        self._StepLength = 0.0
        self._StepWidth = 0.15
        self._alpha = 0.0

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
            x = self._StepLength
            y = self._StepWidth if (step_index%2==0) else -self._StepWidth
            x,y = self._Odometer.GetInGlobalCoordinates(x,y)
            self._command.walk_params.step_data[i].pose.position.x = x
            # Step 0.15m to either side of center, alternating with feet
            self._command.walk_params.step_data[i].pose.position.y = y
            
            # The z position is observed for static walking, but the foot
            # will be placed onto the ground if the ground is lower than z
            self._command.walk_params.step_data[i].pose.position.z = 0.0
            
            # Point those feet straight ahead
            self._command.walk_params.step_data[i].pose.orientation.x = 0.0
            self._command.walk_params.step_data[i].pose.orientation.y = 0.0
            self._command.walk_params.step_data[i].pose.orientation.z = 0.0
            self._command.walk_params.step_data[i].pose.orientation.w = 1.0

    def GetAtlasSimInterfaceCommand(self,index):       
        # A walk behavior command needs to know three additional steps beyond the current step needed to plan
        # for the best balance
        for i in range(4):
            step_index = index + i
            is_right_foot = step_index % 2
            
            self._command.walk_params.step_data[i].step_index = step_index
            self._command.walk_params.step_data[i].foot_index = step_index%2

            x = self._StepLength
            y = self._StepWidth if (step_index%2==0) else -self._StepWidth
            x,y = self._Odometer.GetInGlobalCoordinates(x,y)
            self._command.walk_params.step_data[i].pose.position.x = x
            # Step 0.15m to either side of center, alternating with feet
            self._command.walk_params.step_data[i].pose.position.y = y

        return self._command

    def GetStepSize(self):
        return self._StepLength,self._StepWidth,self._alpha

class BDI_StrategyIdle(BDI_Strategy):
    """
    """
    def __init__(self,odometer):
        BDI_Strategy.__init__(self,odometer)
        self._command.behavior = AtlasSimInterfaceCommand.STAND

class BDI_StrategyForward(BDI_Strategy):
    """
    """
    def __init__(self,odometer):
        BDI_Strategy.__init__(self,odometer)
        self._StepLength = 0.3
        self._StepWidth = 0.15

    def GetAtlasSimInterfaceCommand(self,index):       
        # A walk behavior command needs to know three additional steps beyond the current step needed to plan
        # for the best balance
        for i in range(4):
            step_index = index + i
            is_right_foot = step_index % 2
            
            self._command.walk_params.step_data[i].step_index = step_index
            self._command.walk_params.step_data[i].foot_index = step_index%2

            x = self._StepLength*(i+1)
            y = self._StepWidth if (step_index%2==0) else -self._StepWidth
            x,y = self._Odometer.GetInGlobalCoordinates(x,y)
            self._command.walk_params.step_data[i].pose.position.x = x
            # Step 0.15m to either side of center, alternating with feet
            self._command.walk_params.step_data[i].pose.position.y = y

        return self._command

class BDI_StrategyLeft(BDI_Strategy):
    """
    """
    def __init__(self,odometer):
        BDI_Strategy.__init__(self,odometer)
        self._StepLength = 0.3
        self._StepWidth = 0.15

        self._TurnRadius = 3.25

        self._InnerRadius = self._TurnRadius - self._StepWidth
        self._OuterRadius = self._TurnRadius + self._StepWidth
        #self._alpha = self._StepLength/self._TurnRadius
        self._alpha = math.acos(1-self._StepLength/(2*self._TurnRadius))

    def GetAtlasSimInterfaceCommand(self,index): 
        print("stepLeft")      
        for i in range(4):
            step_index = index + i
            is_right_foot = step_index % 2
            
            self._command.walk_params.step_data[i].step_index = step_index
            self._command.walk_params.step_data[i].foot_index = step_index%2

            # a left step is in the inner circle
            r = self._InnerRadius if (step_index%2==0) else self._OuterRadius

            theta = self._alpha*(i+1)
            print(theta)

            # (Translate(dot)Rotate(dot)Translate)
            x = r*math.sin(theta)
            y = self._TurnRadius - r*math.cos(theta)

            print(x,y)
            x,y = self._Odometer.GetInGlobalCoordinates(x,y)
            print(x,y)
            self._command.walk_params.step_data[i].pose.position.x = x
            self._command.walk_params.step_data[i].pose.position.y = y

            # Calculate orientation quaternion
            Q = quaternion_from_euler(0, 0, self._Odometer.GetYaw()+theta)

            self._command.walk_params.step_data[i].pose.orientation.x = Q[0]
            self._command.walk_params.step_data[i].pose.orientation.y = Q[1]
            self._command.walk_params.step_data[i].pose.orientation.z = Q[2]
            self._command.walk_params.step_data[i].pose.orientation.w = Q[3]

        return self._command

class BDI_StrategyRight(BDI_Strategy):
    """
    """
    def __init__(self,odometer):
        BDI_Strategy.__init__(self,odometer)
        self._StepLength = 0.3
        self._StepWidth = 0.15

        self._TurnRadius = 3.25

        self._InnerRadius = self._TurnRadius - self._StepWidth
        self._OuterRadius = self._TurnRadius + self._StepWidth
        self._alpha = -self._StepLength/self._TurnRadius

    def GetAtlasSimInterfaceCommand(self,index):       
        for i in range(4):
            step_index = index + i
            is_right_foot = step_index % 2
            
            self._command.walk_params.step_data[i].step_index = step_index
            self._command.walk_params.step_data[i].foot_index = step_index%2

            # a left step is in the inner circle
            r = self._InnerRadius if (step_index%2==0) else self._OuterRadius

            r = r
            R = -self._TurnRadius
            theta = self._alpha*i

            # (Translate(dot)Rotate(dot)Translate)
            x = R*math.sin(theta)
            y = R*math.cos(theta) + r
            x,y = self._Odometer.GetInGlobalCoordinates(x,y)
            self._command.walk_params.step_data[i].pose.position.x = x
            # Step 0.15m to either side of center, alternating with feet
            self._command.walk_params.step_data[i].pose.position.y = y

            # Calculate orientation quaternion
            Q = quaternion_from_euler(0, 0, theta)

            self._command.walk_params.step_data[i].pose.orientation.x = Q[0]
            self._command.walk_params.step_data[i].pose.orientation.y = Q[1]
            self._command.walk_params.step_data[i].pose.orientation.z = Q[2]
            self._command.walk_params.step_data[i].pose.orientation.w = Q[3]

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

class BDI_Idle(BDI_State):
    """
        The BDI_Idle class is intended to be used when not walking
    """
    def __init__(self):
        BDI_State.__init__(self,"Idle")

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

class BDI_TrunRight(BDI_State):
    """
        The BDI_Idle class is intended to be used when not walking
    """
    def __init__(self):
    	BDI_State.__init__(self,"Turn_Right")

###################################################################################
#--------------------------- State Machine ----------------------------------------
###################################################################################

class BDI_StateMachine(StateMachine):
    """
        The StepStateMachine class is a finite state machine that handles the stepping logic for the ZMP module
    """
    
    def __init__(self,odometer):
        StateMachine.__init__(self,BDI_Idle())
        self._Odometer = odometer
        self._StrategyIdle = BDI_StrategyIdle(odometer)
        self._StrategyForward = BDI_StrategyForward(odometer)
        self._StrategyRight = BDI_StrategyRight(odometer)
        self._StrategyLeft = BDI_StrategyLeft(odometer)
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
        self._StepStateMachine.SetStrategy(self._StrategyRight,self._StrategyRight)

    def TurnLeft(self):
        StateMachine.PerformTransition(self,"TurnLeft")
        self._StepStateMachine.SetStrategy(self._StrategyLeft,self._StrategyLeft)

    def Step(self,nextindex):
        result = 0 
        if (nextindex>self._index):
            self._index += 1
            x,y,alpha = self._StepStateMachine.GetStepSize()
            self._Odometer.AddYaw(alpha)
            self._Odometer.AddLocalPosition(x,0)
            self._StepStateMachine.Step()
            result = self._StepStateMachine.GetCommand(nextindex)
        return result


