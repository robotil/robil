#!/usr/bin/env python

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

from Abstractions.StateMachine import *
from BDI_StateMachine import *

import rospy

###################################################################################
#--------------------------- States -----------------------------------------------
###################################################################################

class BDI_WalkingModeState(State):
    """
        The StepState class is intended to be used with the BDI_WalkingModeStateMachine class
    """    
    def __init__(self,strStateName):
    	State.__init__(self,strStateName)

###################################################################################
#--------------------------- State Machine ----------------------------------------
###################################################################################

class BDI_WalkingModeStateMachine(StateMachine):
    """
        The WalkingModeStateMachine class is a finite state machine that handles the atlas_sim_interface_state logic for the BDI module
    """
    def __init__(self,odometer,lpp):
        self._Odometer = odometer
        self._LPP = lpp
        #StateMachine
        StateMachine.__init__(self,BDI_WalkingModeState("Idle"))

        self._BDI_StateMachine = BDI_StateMachine(self._Odometer)

        # Add states
        StateMachine.AddState(self,BDI_WalkingModeState("Wait"))
        StateMachine.AddState(self,BDI_WalkingModeState("Walking"))
        StateMachine.AddState(self,BDI_WalkingModeState("Done"))
        
        # Add transitions
        StateMachine.AddTransition(self,"Idle",         "Walk",        "Wait")
        StateMachine.AddTransition(self,"Wait",         "Go",        "Walking")
        StateMachine.AddTransition(self,"Walking",      "Finished",        "Done")
        StateMachine.AddTransition(self,"Done",       "Stop",        "Idle")
        StateMachine.AddTransition(self,"Wait",       "Stop",        "Idle")
        StateMachine.AddTransition(self,"Walking",       "Stop",        "Idle")

    def Initialize(self,step_index_for_reset):
        self._BDI_StateMachine.Initialize(step_index_for_reset)

    def Walk(self,yaw):
        StateMachine.PerformTransition(self,"Walk")
        self._Odometer.SetYaw(yaw)

    def Stop(self):
        if(StateMachine.PerformTransition(self,"Stop")):
            self._BDI_StateMachine.Stop()

    def HandleStateMsg(self,state):
        command = 0
        if ("Idle" == StateMachine.GetCurrentState(self).Name):
            pass
        elif ("Wait" == StateMachine.GetCurrentState(self).Name):
            self.step_index_for_reset = state.behavior_feedback.walk_feedback.next_step_index_needed - 1
            self._Odometer.SetPosition(state.pos_est.position.x,state.pos_est.position.y)
            self._BDI_StateMachine.Initialize(self.step_index_for_reset)
            self._BDI_StateMachine.GoForward()
            StateMachine.PerformTransition(self,"Go")

            #yaw?
        elif ("Walking" == StateMachine.GetCurrentState(self).Name):
            if (self._LPP.IsActive()):
                # x,y = self._Odometer.GetGlobalPosition()
                # self._LPP.UpdatePosition(x,y)
                self._BDI_StateMachine.SetPathError(self._LPP.GetPathError())
          
                targetYaw = self._LPP.GetTargetYaw()
                delatYaw = targetYaw - self._Odometer.GetYaw()
              
                debug_transition_cmd = "NoCommand"
                if (math.sin(delatYaw) > 0.6):
                    #print("Sin(Delta)",math.sin(delatYaw), "Left")
                    debug_transition_cmd = "TurnLeft"
                    self._BDI_StateMachine.TurnLeft(targetYaw)
                elif (math.sin(delatYaw) < -0.6):
                    #print("Sin(Delta)",math.sin(delatYaw), "Right")
                    debug_transition_cmd = "TurnRight"
                    self._BDI_StateMachine.TurnRight(targetYaw)
            else:
                debug_transition_cmd = "Stop"
                self._BDI_StateMachine.Stop()
            if (self._BDI_StateMachine.IsDone()):
                StateMachine.PerformTransition(self,"Finished")
            command = self._BDI_StateMachine.Step(state.behavior_feedback.walk_feedback.next_step_index_needed)

            if (0 !=command):
                rospy.loginfo("WalkingModeBDI, asi_state_cb: State Machine Transition Cmd = %s" % (debug_transition_cmd) )
        elif ("Done" == StateMachine.GetCurrentState(self).Name):
            pass
        else:
            raise StateMachineError("BDI_WalkingModeStateMachine::Bad State Name")

        return command

    def IsDone(self):
        return ("Done" == StateMachine.GetCurrentState(self).Name)
