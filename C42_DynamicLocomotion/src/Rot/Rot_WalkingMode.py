#!/usr/bin/env python

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

from DirectControl.DC_WalkingMode import *

class Rot_WalkingMode(DC_WalkingMode):
    def __init__(self,iTf):
        DC_WalkingMode.__init__(self,iTf)
 
    def Initialize(self,generalParameters):
        DC_WalkingMode.Initialize(self)      
        
        specificParameters = {}
        self._rotationAngle = 0.0
        if ((None != generalParameters) and ('Rotate' in generalParameters)):
            self._rotationAngle = float(generalParameters['Rotate'])
        else:
            self._rotationAngle = 0.0
        
        if ('AP' == self._CurrentStandingMode):
            specificParameters['turn_in_place_Yaw'] = self._rotationAngle
        elif ('DW' == self._CurrentStandingMode):
            pass
        else:
            raise Exception("Rot_WalkingMode::Initialize: Unexpected _CurrentStandingMode")
            
        self._WalkingModes[self._CurrentStandingMode].Initialize(specificParameters)
        
    def Walk(self):
        if('AP' == self._CurrentStandingMode):
            DC_WalkingMode.Walk(self)
        elif('DW' == self._CurrentStandingMode):
            self._WalkingModes[self._CurrentStandingMode]._WalkingModeStateMachine.PerformTransition("Walk")
            y,p,r = self._WalkingModes[self._CurrentStandingMode]._Controller.current_ypr()
            self._WalkingModes[self._CurrentStandingMode]._Controller.RotateToOri(y+self._rotationAngle)
            self._WalkingModes[self._CurrentStandingMode]._bDone = True
        else:
            raise Exception("Rot_WalkingMode::Walk: Unexpected _CurrentStandingMode")

    def IsDone(self):
        result = self._WalkingModes[self._CurrentStandingMode]._bDone
        if (False == result):
            result = DC_WalkingMode.IsDone(self)
        return result

