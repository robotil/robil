#!/usr/bin/env python

from Abstractions.WalkingModeChooserInterface import *
from BDI.WalkingModeBDI import *
from QS.QS_WalkingMode import *

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

class WalkingModeChooser(WalkingModeChooserInterface):

    def __init__(self,prefferedMode,bIsDoingQual):
        
        # Once we clean bIsDoingQual out of the code, we can allow for the lpp to be known only to the concrete walking
        # mode
        lpp = LocalPathPlanner()
        lpp.SetDoingQual(bIsDoingQual)
        
        self._Modes = {'BDI':WalkingModeBDI(lpp),'QS':QS_WalkingMode()}
        self._CurrentMode = self._Modes[prefferedMode]
        self._Recommended = self._Modes[prefferedMode]
        
    def IsCurrentModeAppropriate(self):
        return True;
    
    def GetRecommendedMode(self):
        return self._Recommended


