#!/usr/bin/env python

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

from WalkingModeStateMachine import *

class WalkingMode(object):
    def __init__(self,localPathPlanner):
        self._LPP = localPathPlanner
        self._WalkingModeStateMachine = WalkingModeStateMachine()
        
    def Initialize(self):
        self._WalkingModeStateMachine = WalkingModeStateMachine()
        self._LPP.Stop()

    def StartWalking(self):
        pass
    
    def Walk(self):
        return self._WalkingModeStateMachine.PerformTransition("Walk")
    
    def Stop(self):
        self._LPP.Stop()
        return self._WalkingModeStateMachine.PerformTransition("Stop")
    
    def EmergencyStop(self):
        pass    
    
    def SetPath(self,Path):
        self._LPP.SetPath(Path)
        
    def GetPath(self):
        return self._LPP.GetPath()
        
    def IsDone(self):
        return False

    def IsReady(self):
        return self._LPP.IsActive()
    
    def Fitness(self,path):
        return True
