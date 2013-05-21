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

class QS_PathPlannerEnum:
    Empty,Active,Waiting = range(3)

class QS_PathPlanner(PathPlanner):
    """
        The QS_PathPlanner is an PathPlanner to be used by the QS_WalkingMode
    """
    
    def __init__(self):
        PathPlanner.__init__(self)
        self._Queue = deque([])
        self.State = QS_PathPlannerEnum.Empty
        
    def SetPath(self,waypointList):
        PathPlanner.SetPath(self,waypointList)
        self._Queue.extend(waypointList)
        if(0 < len(self._Queue)):
            self.State = QS_PathPlannerEnum.Active
        
    def GetNextStep(self):
        step_params = 0
        # Zero is a magic number too
        if(0 < len(self._Queue)):
            step_params = self._Queue.popleft()
        else:
            self.State = QS_PathPlannerEnum.Waiting

    def Stop(self):
        PathPlanner.Stop(self,waypointList)
        self.State = QS_PathPlannerEnum.Empty

