#!/usr/bin/env python

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

from Abstractions.StateMachine import *
from BDI_StepStateMachine import *
from BDI_Strategies import *

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
    def __init__(self,strategy):
        BDI_State.__init__(self,"Idle")
        self._StrategyIdle = strategy

    def OnEnter(self):
        self._StrategyIdle.Initialize()
        

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

class BDI_TransitionRight(BDI_State):
    """
        The BDI_Idle class is intended to be used when not walking
    """
    def __init__(self):
        BDI_State.__init__(self,"Trans_Right")

class BDI_TransitionLeft(BDI_State):
    """
        The BDI_Idle class is intended to be used when not walking
    """
    def __init__(self):
        BDI_State.__init__(self,"Trans_Left")

class BDI_TransitionForward(BDI_State):
    """
        The BDI_Idle class is intended to be used when not walking
    """
    def __init__(self):
        BDI_State.__init__(self,"Trans_Forward")

class BDI_TransitionStop(BDI_State):
    """
        The BDI_Idle class is intended to be used when not walking
    """
    def __init__(self):
        BDI_State.__init__(self,"Trans_Stop")

###################################################################################
#--------------------------- State Machine ----------------------------------------
###################################################################################

class BDI_StateMachine(StateMachine):
    """
        The StepStateMachine class is a finite state machine that handles the stepping logic for the ZMP module
    """
    
    def __init__(self,odometer):
        self._Odometer = odometer
        self._StrategyIdle = BDI_StrategyIdle(odometer)
        self._StrategyForward = BDI_StrategyForward(odometer)
        self._StrategyRight = BDI_StrategyRight(odometer)
        self._StrategyLeft = BDI_StrategyLeft(odometer)
        StateMachine.__init__(self,BDI_Idle(self._StrategyIdle))
        self._StepStateMachine = BDI_StepStateMachine()
        self._StepStateMachine.SetStrategy(self._StrategyIdle,self._StrategyIdle)
        self._index = 0
        self._PathError = 0.0
        self._NextStrategy = self._StrategyIdle

        # Add states
        StateMachine.AddState(self,BDI_Forward())
        StateMachine.AddState(self,BDI_TrunLeft())
        StateMachine.AddState(self,BDI_TrunRight())
        StateMachine.AddState(self,BDI_TransitionRight())
        StateMachine.AddState(self,BDI_TransitionLeft())
        StateMachine.AddState(self,BDI_TransitionForward())
        StateMachine.AddState(self,BDI_TransitionStop())

        
        # Add transitions
        StateMachine.AddTransition(self,"Idle",         "TurnRight",        "Trans_Right")
        StateMachine.AddTransition(self,"Idle",         "GoForward",        "Trans_Forward")
        StateMachine.AddTransition(self,"Idle",         "TurnLeft",         "Trans_Left")
        StateMachine.AddTransition(self,"Forward",      "Stop",         	"Trans_Stop")
        StateMachine.AddTransition(self,"Forward",      "TurnRight",     	"Trans_Right")
        StateMachine.AddTransition(self,"Forward",      "TurnLeft",         "Trans_Left")
        StateMachine.AddTransition(self,"Turn Right",   "Stop",         	"Trans_Stop")
        StateMachine.AddTransition(self,"Turn Right",   "GoForward",        "Trans_Forward")
        StateMachine.AddTransition(self,"Turn Right",   "TurnLeft",         "Trans_Forward")
        StateMachine.AddTransition(self,"Turn_Left",    "Stop",         	"Trans_Stop")
        StateMachine.AddTransition(self,"Turn_Left",    "GoForward",        "Trans_Forward")
        StateMachine.AddTransition(self,"Turn_Left",    "TurnRight",        "Trans_Forward")
        StateMachine.AddTransition(self,"Trans_Stop",   "NextStep",         "Idle")
        StateMachine.AddTransition(self,"Trans_Right",  "NextStep",         "Turn_Right")
        StateMachine.AddTransition(self,"Trans_Left",   "NextStep",         "Turn_Left")
        StateMachine.AddTransition(self,"Trans_Forward","NextStep",         "Forward")

    def GoForward(self):
        if (StateMachine.PerformTransition(self,"GoForward")):
            self._NextStrategy = self._StrategyForward

    def TurnRight(self):
        if (StateMachine.PerformTransition(self,"TurnRight")):
            self._NextStrategy = self._StrategyRight

    def TurnLeft(self):
        if (StateMachine.PerformTransition(self,"TurnLeft")):
            self._NextStrategy = self._StrategyLeft

    def Stop(self):
        if (StateMachine.PerformTransition(self,"Stop")):
            self._NextStrategy = self._StrategyIdle

    def Step(self,nextindex):
        result = 0 
        if (nextindex>self._index):
            self._index += 1
            self._StepStateMachine.Step()
            self.NextStep()
            self._StrategyForward.SetPathError(self._PathError)
            print("Error: ",self._PathError)
            result = self._StepStateMachine.GetCommand(self._index)
        return result

    def SetPathError(self,pathError):
        self._PathError = pathError

    def NextStep(self):
        if(StateMachine.PerformTransition(self,"NextStep")):
            self._StepStateMachine.SetStrategy(self._NextStrategy,self._NextStrategy)

