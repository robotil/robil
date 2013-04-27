#!/usr/bin/env python

from DynamicWalker import *
import rospy

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

class DynamicLocomotion(object):
    
    def __init__(self):
        self._ModeChooser = WalkingModeChooser()
        self._DynamicWalker = DynamicWalker(self._ModeChooser)
        
    def DynamicLocomotionTask(self):
        
        self._DynamicWalker.Initialize()
        self._DynamicWalker.Start()
        self._DynamicWalker.WaitForPath()
        if self._DynamicWalker.Walk():
            self._DynamicWalker.Stop()
            return RTResult_SUCCESSED("Finished in Success")
        else:
            return RTResult_PREEPTED()


