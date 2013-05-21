#!/usr/bin/env python

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

###################################################################################
#--------------------------- Path Planner Abstraction -----------------------------
###################################################################################

class PathPlanner(object):
    """
        The PathPlanner is an abstraction to be used by the WalkingMode
    """
    
    def __init__(self):
        self._PathReady = False
        
    def SetPath(self,waypointList):
        self._PathReady = True
                      
    def Stop(self):
        self._PathReady = False 

    def IsActive(self):
        return self._PathReady

    def UpdatePosition(self,CoordinateX,CoordinateY,CoordinateZ = 0.0):
        pass
