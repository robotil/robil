#!/usr/bin/env python

from collections import deque
import math

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

###################################################################################
#---------------------------------- Waypoint --------------------------------------
###################################################################################

class Waypoint(object):
    """
        The Waypoint class is the basic building block of paths
    """   
    def __init__(self,CoordinateX = 0.0,CoordinateY = 0.0):
        self._fX = CoordinateX
        self._fY = CoordinateY
        
    def GetX(self):
        return self._fX
    
    def SetX(self,X):
        self._fX = X
  
    def GetY(self):
        return self._fY
    
    def SetY(self,Y):
        self._fY = Y      
        
    def GetDistanceFrom(self,otherWaypoint):
        dX = otherWaypoint._fX - self._fX
        dY = otherWaypoint._fY - self._fY
        return math.sqrt(dX*dX+dY*dY)
        
###################################################################################
#---------------------------------- Segment --------------------------------------
###################################################################################
       
class Segment(object):
    """
        The Segment class represents a part in a path between two waypoints: Source to Target
    """  
    def __init__(self,waypointSource,waypointTarget):
        self._Source = waypointSource
        self._Target = waypointTarget
        
    def SetSource(self,waypointSource):
        self._Source = waypointSource
    
    def SetTarget(self,waypointTarget):
        self._Target = waypointTarget
        
    def GetSource(self):
        return self._Source
    
    def GetTarget(self):
        return self._Target
        
    def GetDistanceFrom(self,waypoint):
        u = [self._Target.GetX() - self._Source.GetX(),self._Target.GetY() - self._Source.GetY()]
        v = [waypoint.GetX() - self._Source.GetX(),waypoint.GetY() - self._Source.GetY()]
        
        _u_ = math.sqrt(u[0]**2+u[1]**2)
        # a X b = _a_*_b_*sin(alpha) => _b_*sin(alpha) = a X b /_a_
        if (_u_>0):
            vectorDistance = (u[0]*v[1]-u[1]*v[0])/_u_
        else:
            vectorDistance = 0.0
 
        return vectorDistance
    
    def GetYaw(self):
        """
            Returns the yaw in radians  - TODO - What a=is the coordinate system?
        """
        u = [self._Target.GetX() - self._Source.GetX(),self._Target.GetY() - self._Source.GetY()]
        _u_ = math.sqrt(u[0]**2+u[1]**2)
        
        if (_u_>0):
            u = [u[0]/_u_, u[1]/_u_]
        else:
            u = [1,0]
        
        return math.acos(u[0])

###################################################################################
#--------------------------- Local Path Planner -----------------------------------
###################################################################################

class LocalPathPlanner(object):
    """
        The LocalPathPlanner class plans the next few steps for the ZMP module
    """
    
    def __init__(self):
        self._Path = deque([])
        self._Position = Waypoint()
        self._CurrentSegment = Segment(self._Position,self._Position)
        self._CloseEnoughToTarget = 0.2 #0.5       
        
    def SetPath(self,waypointList):
        self._Path = deque(waypointList)
        if (len(self._Path)<2):
            if(len(self._Path)<1):
                self._CurrentSegment.SetSource(self._Position)
                self._CurrentSegment.SetTarget(self._Position)
            else:
                self._CurrentSegment.SetSource(self._Position)
                self._CurrentSegment.SetTarget(self._Path[0])
        else:
            self._CurrentSegment.SetSource(self._Path.popleft())
            self._CurrentSegment.SetTarget(self._Path[0])    
        
    def GetPathError(self):
        return self._CurrentSegment.GetDistanceFrom(self._Position)
        
    def GetPos(self):
		return self._Position
    
    def GetTargetYaw(self):
        return self._CurrentSegment.GetYaw()
        
    def GetCurrentSegment(self):
		return self._CurrentSegment
    
    def UpdatePosition(self,CoordinateX,CoordinateY):
        """
            Updates the position, returns true if at end of current path, false otherwise
        """
        self._Position.SetX(CoordinateX)
        self._Position.SetY(CoordinateY)
        bStop = False
        if (self._CurrentSegment.GetTarget().GetDistanceFrom(self._Position) < self._CloseEnoughToTarget):
            self._CurrentSegment.SetSource(self._CurrentSegment.GetTarget())
            if(len(self._Path)<1):
                bStop = True
            else:
                self._CurrentSegment.SetTarget(self._Path.popleft())
        return bStop
        
  

###################################################################################
# a little testing script
###################################################################################

if __name__ == "__main__":
    pass


