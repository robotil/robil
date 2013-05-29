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

class DD_PathPlannerEnum:
    Empty,Active,Waiting = range(3)

class DD_PathPlanner(PathPlanner):
    """
        The DD_PathPlanner is an PathPlanner to be used by the DD_WalkingMode
    """
    
    def __init__(self):
        PathPlanner.__init__(self)
        self._Queue = deque([])
        self.State = DD_PathPlannerEnum.Empty
        self._countEnteries = 0
        
    def SetPath(self,waypointList):
        PathPlanner.SetPath(self,waypointList)
        self._Queue.extend(waypointList)
        if(0 < len(self._Queue)):
            self.State = DD_PathPlannerEnum.Active
            
    def GetPath(self):
        return self._Queue
        
    def GetNextStep(self):
        step_queue = 0
        if(3 < len(self._Queue)):            
            step_queue = []
            #print self._Queue
            for i in range(4):
                #print("GetNextStep ",i,self._Queue[i])
                step_queue.append(self._Queue[i])
            self._Queue.popleft()
        elif(3 == len(self._Queue)):
            step_queue = []
            #print self._Queue
            for i in range(3):
                #print("GetNextStep ",i,self._Queue[i])
                step_queue.append(self._Queue[i])
            self._Queue.popleft()
            step_queue.append(self._Queue[i-1])
            self._countEnteries = 0
        else:
            step_queue = []
            zero_or_one = self._countEnteries%2
            one_or_zero = (self._countEnteries+1)%2
            step_queue.append(self._Queue[zero_or_one])
            step_queue.append(self._Queue[one_or_zero])
            step_queue.append(self._Queue[zero_or_one])
            step_queue.append(self._Queue[one_or_zero])
            self._countEnteries += 1
            self.State = DD_PathPlannerEnum.Waiting
        print step_queue
        return step_queue

    def Stop(self):
        PathPlanner.Stop(self)
        self.State = DD_PathPlannerEnum.Empty

