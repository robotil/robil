#!/usr/bin/env python

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

from Abstractions.WalkingMode import *
from DogWormVRC3 import *
from DW_PathPlanner import *

import roslib;roslib.load_manifest('C42_DynamicLocomotion')
from C31_PathPlanner.msg import C31_Waypoints

class DW_WalkingMode(WalkingMode):
    def __init__(self,iTf):
        WalkingMode.__init__(self,DW_PathPlanner())
        self._Controller = DW_Controller(iTf)
        
    def Initialize(self):
        WalkingMode.Initialize(self)
        self._Controller.Initialize()
        self._bDone = False
        
        self._Subscribers["Path"] = rospy.Subscriber('/path',C31_Waypoints,self._path_cb)
        self._Subscribers["Odometry"] = rospy.Subscriber('/ground_truth_odom',Odometry,self._Controller.Odom_cb)
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
        self._Controller.DynStandUp()
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
            if 0 == i%2:
                direction = "fwd"
            else:
                direction = "bwd"
            i = i+1
            print p
            p.append([wp.x,wp.y,direction])
        self.SetPath(p)
    
