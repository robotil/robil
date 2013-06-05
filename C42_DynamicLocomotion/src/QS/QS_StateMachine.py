#!/usr/bin/env python

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

from Abstractions.StateMachine import *

###################################################################################
#--------------------------- State Machine ----------------------------------------
###################################################################################

class QS_StateMachine(StateMachine):
    """
        The WalkingModeStateMachine class is a finite state machine that manages WalkingMode phases of operation
    """
    def __init__(self):
        #StateMachine
        StateMachine.__init__(self,State("Empty"))

        # Add states
        StateMachine.AddState(self,WalkingModeState("Active"))
        StateMachine.AddState(self,WalkingModeState("Waiting"))
        
        # Add transitions
        StateMachine.AddTransition(self,"Empty",         "Fill",        "Active")
        StateMachine.AddTransition(self,"Active",        "Stop",        "Empty")
        StateMachine.AddTransition(self,"Active",        "Empty",       "Waiting")
        StateMachine.AddTransition(self,"Waiting",       "Fill",        "Active")
        StateMachine.AddTransition(self,"Waiting",       "Stop",        "Empty")

