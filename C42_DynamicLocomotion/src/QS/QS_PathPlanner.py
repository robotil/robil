#!/usr/bin/env python

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

from Abstractions.PathPlanner import *

###################################################################################
#--------------------------------- QS Path Planner  -------------------------------
###################################################################################

class QS_PathPlanner(PathPlanner):
    """
        The QS_PathPlanner is an PathPlanner to be used by the QS_WalkingMode
    """
    
    def __init__(self):
        PathPlanner.__init__(self)
        
    def SetPath(self,waypointList):
        PathPlanner.SetPath(self,waypointList)


