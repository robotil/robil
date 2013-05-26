#!/usr/bin/env python

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
################################################################################### 

import rospy
from WalkingMode import *
from WalkingModeChooserInterface import *

###################################################################################
#--------------------------- Walker -----------------------------------------------
###################################################################################

class Walker(object):
    
    def __init__(self,walkingModeChooser):
        self._ModeChooser = walkingModeChooser

    # Commands:
    def Initialize(self):
        self._WalkingMode = self._ModeChooser.GetRecommendedMode()
        self._WalkingMode.Initialize()
        
    def Start(self):
        pass
    
    def Walk(self):
        self._WalkingMode.StartWalking()
        self._WalkingMode.Walk()
        return True
    
    def Stop(self):
        self._WalkingMode.Stop()

    def SetPath(self,Path):
        self._WalkingMode.SetPath(Path)

    # Status:
    def IsDone(self):
        """
        Return True if the walker is done walking.
        """
        bIsDone = False
        if self._ModeChooser.IsCurrentModeAppropriate():
            bIsDone = self._WalkingMode.IsDone()
        else:
            RecommendedWalkingMode = self._ModeChooser.GetRecommendedMode()
            if (False != RecommendedWalkingMode):
                self._WalkingMode = RecommendedWalkingMode
                self._WalkingMode.Initialize()
                self._WalkingMode.StartWalking()
                self._WalkingMode.Walk()
        return bIsDone

    def IsReady(self):
        return self._WalkingMode.IsReady()




