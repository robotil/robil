#!/usr/bin/env python

from WalkingModeChooserInterface import *
from WalkingModeBDI import *

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

class WalkingModeChooser(WalkingModeChooserInterface):

    def __init__(self):
        self._Modes = {'BDI':WalkingModeBDI()}
        self._CurrentMode = self._Modes['BDI']
        self._Recommended = self._Modes['BDI']
        
    def IsCurrentModeAppropriate(self):
        return true;
    
    def GetRecommendedMode(self):
        return self._Recommended


