#!/usr/bin/env python

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

from StateMachine import *

###################################################################################
#--------------------------- States -----------------------------------------------
###################################################################################

class WalkingModeState(State):
    """
        The StepState class is intended to be used with the WalkingModeStateMachine class
    """    
    def __init__(self,strStateName):
    	State.__init__(self,strStateName)

###################################################################################
#--------------------------- State Machine ----------------------------------------
###################################################################################

class WalkingModeStateMachine(StateMachine):
    """
        The WalkingModeStateMachine class is a finite state machine that manages WalkingMode phases of operation
    """
    def __init__(self):
        #StateMachine
        StateMachine.__init__(self,WalkingModeState("Idle"))

        # Add states
        StateMachine.AddState(self,WalkingModeState("Wait"))
        StateMachine.AddState(self,WalkingModeState("Walking"))
        StateMachine.AddState(self,WalkingModeState("Done"))
        
        # Add transitions
        StateMachine.AddTransition(self,"Idle",         "Walk",        "Wait")
        StateMachine.AddTransition(self,"Wait",         "Go",        "Walking")
        StateMachine.AddTransition(self,"Walking",      "Finished",        "Done")
        StateMachine.AddTransition(self,"Done",       "Stop",        "Idle")
        StateMachine.AddTransition(self,"Wait",       "Stop",        "Idle")
        StateMachine.AddTransition(self,"Walking",       "Stop",        "Idle")

    def IsDone(self):
        return ("Done" == StateMachine.GetCurrentState(self).Name)
