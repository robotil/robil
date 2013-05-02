#!/usr/bin/env python

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

from Abstractions.StateMachine import *
from BDI_StepStateMachine import *
from BDI_Strategies import *
from Abstractions.StepQueue import *

###################################################################################
#--------------------------- States -----------------------------------------------
###################################################################################

class BDI_State(State):
    """
        The StepState class is intended to be used with the BDI_StateMachine class
    """    
    def __init__(self,strStateName,strategy):
    	State.__init__(self,strStateName)
        self._Strategy = strategy

    def OnEnter(self):
        self._Strategy.Initialize()

    def GetStepData(self,index):
        return self._Strategy.GetStepData(index)


###################################################################################
#--------------------------- State Machine ----------------------------------------
###################################################################################

class BDI_StateMachine(StateMachine):
    """
        The StepStateMachine class is a finite state machine that handles the stepping logic for the ZMP module
    """
    
    def __init__(self,odometer):
        self._StepQueue = StepQueue()
        #Strategies
        self._StrategyIdle = BDI_StrategyIdle(odometer)
        self._StrategyForward = BDI_StrategyForward(odometer)
        self._StrategyRight = BDI_StrategyRight(odometer)
        self._StrategyLeft = BDI_StrategyLeft(odometer)
        #StateMachine
        StateMachine.__init__(self,BDI_State("Idle",self._StrategyIdle))
        self._index = 0
        self._PathError = 0.0

        # Add states
        StateMachine.AddState(self,BDI_State("Forward",self._StrategyForward))
        StateMachine.AddState(self,BDI_State("Turn_Left",self._StrategyLeft))
        StateMachine.AddState(self,BDI_State("Turn_Right",self._StrategyRight))
        
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

    def GoForward(self,targetDistance):
        if (StateMachine.PerformTransition(self,"GoForward")):
            self._StrategyRight.SetTarget(targetDistance)
            print("GoForward")

    def TurnRight(self,targetYaw):
        if (StateMachine.PerformTransition(self,"TurnRight")):
            self._StrategyRight.SetTarget(targetYaw)
            print("TurnRight")

    def TurnLeft(self,targetYaw):
        if (StateMachine.PerformTransition(self,"TurnLeft")):
            self._StrategyRight.SetTarget(targetYaw)
            print("TurnLeft")

    def Stop(self):
        if (StateMachine.PerformTransition(self,"Stop")):
            print("Stop")

    def Step(self,nextindex):
        command = 0 
        if (nextindex>self._index):
            self._index += 1
            self._StrategyForward.SetPathError(self._PathError)
            print("Error: ",self._PathError)
            command = self.GetCommand()
            stepData = self._StepQueue.Push(StateMachine.GetCurrentState(self).GetStepData(self._index+4))
            #print("Step ",self._index ,command.walk_params.step_data)
        return command

    def SetPathError(self,pathError):
        self._PathError = pathError

    def GetCommand(self):
        command = AtlasSimInterfaceCommand()
        command.behavior = AtlasSimInterfaceCommand.WALK
        for i in range(4):
            command.walk_params.step_data[i] = self._StepQueue.Peek(i)
        return command

    def Initialize(self):
        self._StepQueue.Initialize(self._StrategyIdle.GetStepData(1),self._StrategyIdle.GetStepData(2),self._StrategyIdle.GetStepData(3),self._StrategyIdle.GetStepData(4))

