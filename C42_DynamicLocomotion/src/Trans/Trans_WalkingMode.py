#!/usr/bin/env python

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

from DirectControl.DC_WalkingMode import *

class Trans_WalkingMode(DC_WalkingMode):
    def __init__(self,iTf):
        DC_WalkingMode.__init__(self,iTf)
 
    def Initialize(self,generalParameters):
        DC_WalkingMode.Initialize(self)      
        
        specificParameters = {}
        self._translationY = 0.0
        if ((None != generalParameters) and ('Translate_X' in generalParameters)):
            self._translationX = float(generalParameters['Translate_X'])
        else:
            self._translationX = 0.0

        if ((None != generalParameters) and ('Translate_Y' in generalParameters)):
            self._translationY = float(generalParameters['Translate_Y'])
        else:
            self._translationY = 0.0
        
        if ('AP' == self._CurrentStandingMode):
            specificParameters['Xmovement'] = self._translationX
            specificParameters['Ymovement'] = self._translationY
        elif ('DW' == self._CurrentStandingMode):
            pass
        else:
            raise Exception("Trans_WalkingMode::Initialize: Unexpected _CurrentStandingMode")
            
        self._WalkingModes[self._CurrentStandingMode].Initialize(specificParameters)
        
    def Walk(self):
        if('AP' == self._CurrentStandingMode):
            DC_WalkingMode.Walk(self)
        elif('DW' == self._CurrentStandingMode):
            self._WalkingModes[self._CurrentStandingMode]._WalkingModeStateMachine.PerformTransition("Walk")

            p = []
            direction = "fwd"
            yaw,pitch,roll = self._WalkingModes[self._CurrentStandingMode]._Controller.current_ypr()
            x = self._WalkingModes[self._CurrentStandingMode]._Controller.GlobalPos.x
            y = self._WalkingModes[self._CurrentStandingMode]._Controller.GlobalPos.y
            p.append([x,y,direction])
            p.append([x+self._translationX*math.cos(yaw)+self._translationY*math.cos(yaw),\
                y+self._translationX*math.sin(yaw)+self._translationY*math.sin(yaw),direction])
            self._WalkingModes[self._CurrentStandingMode].SetPath(p)
            
            DC_WalkingMode.Walk(self)
        else:
            raise Exception("Trans_WalkingMode::Walk: Unexpected _CurrentStandingMode")
