#!/usr/bin/env python

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

import rospy
from WalkingModeStateMachine import *

class WalkingMode(object):
    def __init__(self,localPathPlanner):
        self._LPP = localPathPlanner
        self._WalkingModeStateMachine = WalkingModeStateMachine()
        self._Subscribers = dict()
        self._interval = rospy.Rate(0.6)
        
    def Initialize(self,parameters):
        self._WalkingModeStateMachine = WalkingModeStateMachine()
        self._LPP.Stop()

    def StartWalking(self):
        pass
    
    def Walk(self):
        return self._WalkingModeStateMachine.PerformTransition("Walk")
    
    def Stop(self):
        self._LPP.Stop()
        for subscriber in self._Subscribers.itervalues():
            subscriber.unregister()
        self._Subscribers.clear()
        return self._WalkingModeStateMachine.PerformTransition("Stop")
    
    def EmergencyStop(self):
        pass    
    
    def SetPath(self,Path):
        print("WalkingMode: SetPath()")
        self._LPP.SetPath(Path)
        
    def GetPath(self):
        return self._LPP.GetPath()
        
    def IsDone(self):
        return False

    def IsReady(self):
        return self._LPP.IsActive()
    
    def Fitness(self,path):
        return True

    def Sleep(self):
        self._interval.sleep()


