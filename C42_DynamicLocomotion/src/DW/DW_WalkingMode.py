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
from std_msgs.msg import Int32
from C31_PathPlanner.msg import C31_Waypoints
from C25_GlobalPosition.msg import C25C0_ROP
from C25_GlobalPosition.srv import *
from atlas_msgs.msg import AtlasCommand, AtlasSimInterfaceCommand, AtlasSimInterfaceState, AtlasState, AtlasBehaviorStepData
from C42_State.srv import *
from C42_State.msg import StandingPosition

class DW_WalkingMode(WalkingMode):
    def __init__(self,iTf):
        WalkingMode.__init__(self,DW_PathPlanner())
        self._Controller = DW_Controller(iTf)

        rospy.wait_for_service("/motion_state/info/standing_position")
        self._srv_StandingPosition = rospy.ServiceProxy("/motion_state/info/standing_position", StandingPositionInfo)

        self.terrain = ''
        
    def Initialize(self,parameters):
        WalkingMode.Initialize(self,parameters)

        if ((None != parameters) and ('Terrain' in parameters)):
            self.terrain = parameters['Terrain']
        else:
            self.terrain="HILLS"

        self._Controller.Initialize(Terrain = self.terrain)
        self._bDone = False

        self._BDIswitch_client = rospy.ServiceProxy('C25/BDIswitch',C25BDI)
        state = Int32()
        state.data = 0
        resp_switched_to_BDI_odom = self._BDIswitch_client(state)
        print "Using ROBIL odom"
        
        self._Subscribers["Path"] = rospy.Subscriber('/path',C31_Waypoints,self._path_cb)
        self._Subscribers["Odometry"] = rospy.Subscriber('/C25/publish',C25C0_ROP,self._Controller.Odom_cb)
        #self._Subscribers["Odometry"] = rospy.Subscriber('/ground_truth_odom',Odometry,self._Controller.Odom_cb)
        self._Subscribers["AtlasState"] = rospy.Subscriber('/atlas/atlas_state',AtlasState,self._Controller.RS_cb)
        rospy.sleep(0.3)
        self._Controller.JC.set_all_pos(self._Controller.RS.GetJointPos())
        self._Controller.JC.send_command()
        # Move neck to view position:
        self._Controller.move_neck()
        
    def StartWalking(self):
        self._Controller.LHC.set_all_pos(self._Controller.BaseHandPose)
        self._Controller.RHC.set_all_pos(self._Controller.BaseHandPose)
        self._Controller.LHC.send_command()
        self._Controller.RHC.send_command()
        standingPosition = self._srv_StandingPosition().info.state
        if (StandingPosition.state_standing == standingPosition):
            self._Controller.Sit(1.5)
        else:
            self._Controller.CheckTipping()
        rospy.sleep(0.5)
    
    def Walk(self):
        print("DW_WalkingMode::Walk: path",self._LPP.GetPath())
        WalkingMode.Walk(self)
        path = self._LPP.GetPath()
        self._Controller.GoToPoint(path[0])
    
    def EmergencyStop(self):
        WalkingMode.Stop(self)    
        
    def IsDone(self):
        print("DW_WalkingMode::IsDone: path",self._LPP.GetPath())
        if(self._Controller.PerformStep()):
            if(1 < len(self._LPP.GetPath())):
                self._LPP.GetPath().popleft()
                path = self._LPP.GetPath()
                self._Controller.GoToPoint(path[0])
            else:
                self._bDone = True
        return self._bDone

    def Sleep(self):
        pass

###################################################################################
#--------------------------- CallBacks --------------------------------------------
###################################################################################

    def _path_cb(self,path):
        rospy.loginfo('got path %s',path)
        p = []
        i = 0
        for wp in path.points:
            if 0 < i: # ignore first way-point (current position) 
                if 1 == i%2:
                    if self.terrain=="MUD":
                        direction = "fwd"
                    elif  self.terrain=="HILLS":
                        direction = "bwd"
                else:
                    direction = "bwd"
                p.append([wp.x,wp.y,direction])
            i = i+1
        print p
        self.SetPath(p)

    
