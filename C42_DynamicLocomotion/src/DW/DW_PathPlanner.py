#!/usr/bin/env python

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

from Abstractions.PathPlanner import *
from collections import deque

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
        
    def SetPath(self,waypointList):
        PathPlanner.SetPath(self,waypointList)
        self._Queue.extend(waypointList)
        if(0 < len(self._Queue)):
            self.State = DW_PathPlannerEnum.Active
        #print ("SetPath",self._Queue)
            
    def GetPath(self):
        return self._Queue

    def Stop(self):
        PathPlanner.Stop(self)
        self.State = DW_PathPlannerEnum.Empty

