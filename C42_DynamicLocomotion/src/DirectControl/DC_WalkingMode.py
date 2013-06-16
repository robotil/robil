#!/usr/bin/env python

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

import rospy
from Abstractions.WalkingMode import *
from AlinePose.AP_WalkingMode import *
from AlinePose.AP_PathPlanner import *
from DW.DW_WalkingMode import *
from DW.DW_PathPlanner import *
from C42_State.srv import *
from C42_State.msg import StandingPosition

class DC_WalkingMode(WalkingMode):
    def __init__(self,iTf):
        self._LPPs = {'DW':DW_PathPlanner(),'AP':AP_PathPlanner()}
        WalkingMode.__init__(self,self._LPPs['AP'])
        self._WalkingModes = {'DW':DW_WalkingMode(iTf),'AP':AP_WalkingMode(iTf)}
               
        rospy.wait_for_service("/motion_state/info/standing_position")
        self._srv_StandingPosition = rospy.ServiceProxy("/motion_state/info/standing_position", StandingPositionInfo)
 
        self._SetRequiredWalkingMode()
        
        self._CurrentStandingMode = self._RequiredStandingMode
 
    def Initialize(self):
        parameters = None
        WalkingMode.Initialize(self,parameters)        
        
        self._SetRequiredWalkingMode()
        
        if(self._CurrentStandingMode != self._RequiredStandingMode):
            self._WalkingModes[self._RequiredStandingMode].SetPath(self._WalkingModes[self._CurrentStandingMode].GetPath())
            self._LPP = self._LPPs[_RequiredStandingMode]
        
    def StartWalking(self):
        self._WalkingModes[self._CurrentStandingMode].StartWalking()
    
    def Walk(self):
        WalkingMode.Walk(self)
        self._WalkingModes[self._CurrentStandingMode].Walk()
    
    def Stop(self):
        WalkingMode.Stop(self)
        self._WalkingModes[self._CurrentStandingMode].Stop()
    
    def EmergencyStop(self):
        WalkingMode.Stop(self)
        self._WalkingModes[self._CurrentStandingMode].EmergencyStop()
    
    def SetPath(self,Path):
        self._WalkingModes[self._CurrentStandingMode].SetPath(Path)
        self._LPP.SetPath(Path)
        
    def GetPath(self):
        return self._WalkingModes[self._CurrentStandingMode].GetPath()
        
    def IsDone(self):
        return self._WalkingModes[self._CurrentStandingMode].IsDone()

    def IsReady(self):
        return True
    
    def Fitness(self,path):
        return True

    def Sleep(self):
        self._WalkingModes[self._CurrentStandingMode].Sleep()

################################################
#                  "private"                   #
################################################

    def _SetRequiredWalkingMode(self):
        standingPosition = self._srv_StandingPosition().info.state
        if (StandingPosition.state_standing == standingPosition):
            self._RequiredStandingMode = 'AP'
        elif (StandingPosition.state_sitting == standingPosition):
            self._RequiredStandingMode = 'DW'
