#!/usr/bin/env python
import rospy
from WalkingMode import *
from WalkingModeChooserInterface import *

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
################################################################################### 

class DynamicWalker(object):
    
    def __init__(self,walkingModeChooser):
        self._ModeChooser = walkingModeChooser

    def Initialize(self):
        self._WalkingMode = self._ModeChooser.GetRecommendedMode()
        self._WalkingMode.Initialize()
        self._interval = rospy.Rate(self._WalkingMode.StartWalking())
    
    def WaitForPath(self):
        pass
    
    def Start(self):
        pass
    
    def Walk(self):
        #while self._WalkingMode.Walk():
        #     if self.isPreepted():
        #         self._WalkingMode.EmergencyStop()
        #         return false
        #     self._interval.sleep()
        self._WalkingMode.Walk()
        return True
    
    def Stop(self):
        self._WalkingMode.Stop()



