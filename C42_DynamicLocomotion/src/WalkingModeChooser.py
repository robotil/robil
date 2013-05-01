#!/usr/bin/env python

from Abstractions.WalkingModeChooserInterface import *
from BDI.WalkingModeBDI import *

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

class WalkingModeChooser(WalkingModeChooserInterface):

    def __init__(self,localPathPlanner):
        self._Modes = {'BDI':WalkingModeBDI(localPathPlanner)}
        self._CurrentMode = self._Modes['BDI']
        self._Recommended = self._Modes['BDI']
        
    def IsCurrentModeAppropriate(self):
        return True;
    
    def GetRecommendedMode(self):
        return self._Recommended


