#!/usr/bin/env python

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

from Abstractions.PathPlanner import *
from collections import deque

import math

###################################################################################
#--------------------------------- QS Path Planner  -------------------------------
###################################################################################

class DW_PathPlannerEnum:
    Empty,Active,Waiting = range(3)

class DW_PathPlanner(PathPlanner):
    """
        The DW_PathPlanner is an PathPlanner to be used by the DD_WalkingMode
    """
    
    def __init__(self):
        PathPlanner.__init__(self)
        self._Queue = deque([])
        self.State = DW_PathPlannerEnum.Empty
        self._PathYaw = 0.0

    def SetPath(self,waypointList):
        PathPlanner.SetPath(self,waypointList)
        self._Queue = deque([])
        self._Queue.extend(waypointList)
        if(0 < len(self._Queue)):
            #self._SetPathYaw()
            #print ("Setting PathYaw:", self._PathYaw)
            self.State = DW_PathPlannerEnum.Active
        #print ("SetPath",self._Queue)
            
    def GetPath(self):
        return self._Queue

    def Stop(self):
        PathPlanner.Stop(self)
        self.State = DW_PathPlannerEnum.Empty

    def _SetPathYaw(self):
        """
            Returns the yaw in radians - in Front Left Up Coordinates, with the origin set at system init
        """
        u = [self._Queue[1][0] - self._Queue[0][0],self._Queue[1][1] - self._Queue[0][1]] # [deltaX,deltaY]
        _u_ = math.sqrt(u[0]**2+u[1]**2)
        if (_u_>0):
            u = [u[0]/_u_, u[1]/_u_]
        else:
            u = [1,0]

        #rospy.loginfo('GetYaw: u_norm = %f; u_x = %f, u_y = %f' %(_u_,u[0],u[1] ) )
        self._PathYaw = math.atan2(u[1],u[0])

    def GetPathYaw(self):
        return self._PathYaw

