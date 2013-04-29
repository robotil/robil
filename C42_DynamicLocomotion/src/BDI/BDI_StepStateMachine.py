#!/usr/bin/env python

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

from Abstractions.StateMachine import *
import math
from tf.transformations import quaternion_from_euler, euler_from_quaternion

###################################################################################
#--------------------------- States -----------------------------------------------
###################################################################################

class BDI_StepState(State):
    """
        The StepState class is intended to be used with the StepStateMachine class
    """    
    def __init__(self,strStateName):
    	State.__init__(self,strStateName)

    def SetStrategy(self,strategy):
        self._Strategy = strategy

class BDI_StepLeft(BDI_StepState):
    """
        The BDI_Idle class is intended to be used when not walking
    """
    def __init__(self):
        BDI_State.__init__(self,"Left")


class BDI_StepRight(BDI_StepState):
    """
        The BDI_Idle class is intended to be used when not walking
    """
    def __init__(self):
        BDI_State.__init__(self,"Right")


###################################################################################
#--------------------------- State Machine ----------------------------------------
###################################################################################

class BDI_StepStateMachine(StateMachine):
    """
        The StepStateMachine class is a finite state machine that handles the stepping logic for the ZMP module
    """
    
    def __init__(self):
        StateMachine.__init__(self,BDI_StepLeft())
        # Add states
        StateMachine.AddState(self,BDI_StepRight())
        
        # Add transitions
        StateMachine.AddTransition(self,"Left",         "Step",        "Right")
        StateMachine.AddTransition(self,"Right",        "Step",        "Left")

    def Step(self):
        StateMachine.PerformTransition(self,"Step")

    def SetStrategy(self,LeftStrategy,RightStrategy):
        StateMAchine.GetStateAtTransition(self,"Left","Step").SetStrategy(RightStrategy)
        StateMAchine.GetStateAtTransition(self,"Right","Step").SetStrategy(LeftStrategy)

