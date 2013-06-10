#!/usr/bin/env python

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

from Abstractions.WalkingMode import *
from DogWormVRC3 import *
from DW_PathPlanner import *
import math

import roslib;roslib.load_manifest('C42_DynamicLocomotion')
from C31_PathPlanner.msg import C31_Waypoints
from C25_GlobalPosition.msg import C25C0_ROP
from atlas_msgs.msg import AtlasCommand, AtlasSimInterfaceCommand, AtlasSimInterfaceState, AtlasState, AtlasBehaviorStepData
class DW_WalkingMode(WalkingMode):
    def __init__(self,iTf):
        WalkingMode.__init__(self,DW_PathPlanner())
        self._Controller = DW_Controller(iTf)
        
    def Initialize(self):
        WalkingMode.Initialize(self)
        self._Controller.Initialize(Terrain = "MUD") #How to get the true terrain type?
        self._bDone = False
        
        self._Subscribers["Path"] = rospy.Subscriber('/path',C31_Waypoints,self._path_cb)
        self._Subscribers["Odometry"] = rospy.Subscriber('/C25/publish',C25C0_ROP,self._Controller.Odom_cb)
        #self._Subscribers["Odometry"] = rospy.Subscriber('/ground_truth_odom',Odometry,self._Controller.Odom_cb)
        self._Subscribers["AtlasState"] = rospy.Subscriber('/atlas/atlas_state',AtlasState,self._Controller.RS_cb)
        rospy.sleep(0.3)

    def StartWalking(self):
        self._Controller.LHC.set_all_pos(self._Controller.BaseHandPose)
        self._Controller.RHC.set_all_pos(self._Controller.BaseHandPose)
        self._Controller.LHC.send_command()
        self._Controller.RHC.send_command()
        self._Controller.Sit(1.5)
        rospy.sleep(0.5)
    
    def Walk(self):
        WalkingMode.Walk(self)
        self._Controller.DoPath(self._LPP.GetPath())
        self._Controller.RotateToOri( self._LPP.GetPathYaw() - math.pi/2 )
        # self._Controller.DynStandUp()
        self._bDone = True
    
    def EmergencyStop(self):
        WalkingMode.Stop(self)    
        
    def IsDone(self):
        return self._bDone

###################################################################################
#--------------------------- CallBacks --------------------------------------------
###################################################################################

    def _path_cb(self,path):
        rospy.loginfo('got path %s',path)
        p = []
        i = 0
        for wp in path.points:
            if 0 < i: # ignor first way-point (current position) 
                if 1 == i%2:
                    direction = "fwd"
                else:
                    direction = "bwd"
                p.append([wp.x,wp.y,direction])
            i = i+1
        print p
        self.SetPath(p)

    
