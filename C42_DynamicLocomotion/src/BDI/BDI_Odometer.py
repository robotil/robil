#!/usr/bin/env python

import math

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

class BDI_Odometer(object):
    def __init__(self):
        self._X = 0.0
        self._Y = 0.0
        self._Yaw = 0.0

    def SetPosition(self,x,y):
        self._X = x
        self._Y = y

    def GetGlobalPosition(self):
        return self._X,self._Y

    def GetInGlobalCoordinates(self,LocalX,LocalY):
        x = self._X + LocalX*math.cos(-self._Yaw) + LocalY*math.sin(-self._Yaw)
        y = self._Y + LocalY*math.cos(-self._Yaw) - LocalX*math.sin(-self._Yaw)
        return x,y

    def AddGlobalPosition(self,x,y):
        self._X += x
        self._Y += y

    def AddLocalPosition(self,x,y):
        self._X += x*math.cos(-self._Yaw) + y*math.sin(-self._Yaw)
        self._Y += y*math.cos(-self._Yaw) - x*math.sin(-self._Yaw)

    def AddYaw(self,yaw):
        self._Yaw += yaw

    def SetYaw(self,yaw):
        self._Yaw = yaw

    def GetYaw(self):
        return self._Yaw
