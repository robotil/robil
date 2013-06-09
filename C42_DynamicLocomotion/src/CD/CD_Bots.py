#!/usr/bin/env python

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

from Abstractions.PathPlanner import *
from Abstractions.Odometer import *


###################################################################################
#----------------------------------- CD Bot ---------------------------------------
###################################################################################

class CD_Robot(object):
    """
        The CD_ActualRobot represents the real actual robot as it walks on the path
    """
    def __init__(self,pathPlanner,stepQueue):
        self._LPP = pathPlanner
        self._StepQueue = stepQueue

    def Initialize(self,stepQueue):
        self._StepQueue = stepQueue

    def SetPosition(self,x,y):
        pass

    def SetYaw(self,yaw):
        pass

    def SetPath(self,path):
        pass

###################################################################################
#-------------------------------- Actual Bot --------------------------------------
###################################################################################

class CD_ActualRobot(CD_Robot):
    """
        The CD_ActualRobot represents the real actual robot as it walks on the path
    """
    def __init__(self,pathPlanner,stepQueue):
        CD_Robot.__init__(self,pathPlanner,stepQueue)

    def Step(self):
        pass

    def EndOfPath(self):
        pass

###################################################################################
#-------------------------------- Phantom Bot -------------------------------------
###################################################################################

class CD_PhantomRobot(CD_Robot):
    """
        The CD_PhantomRobot represents the future robot as it walks on the path in the future
    """
    def __init__(self,pathPlanner,odometer,stepQueue):
        CD_Robot.__init__(self,pathPlanner,stepQueue)
        self._Odometer = odometer
        self._index = 0

    def Initialize(self,stepQueue,index):
        CD_Robot.Initialize(self,stepQueue)
        self._index = index

    def Step(self):
        pass

    def EndOfSegment(self):
        pass

    def PrepareNextSegment(self):
        pass

    def GetIndex(self):
        pass
