#!/usr/bin/env python

import roslib;roslib.load_manifest('C42_DynamicLocomotion')
import rospy
from Abstractions.Interface_tf import *
from RobilTaskPy import *
import actionlib
from std_msgs.msg import Int32
from DW.DogWormVRC3 import *
import math

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

class DWTurn(RobilTask):
    
    def __init__(self,name):
        RobilTask.__init__(self, name)
        self._iTf = Interface_tf()
        self._Controller = DW_Controller(self._iTf)
        self._interval = rospy.Rate(2)

    def task(self, name, uid, parameters):
        #initialize values:
        Bearing = 0.5*math.pi
        self._Controller.Initialize(Terrain = "HILLS")
        ## TOPIC setup:
        odom_sub = rospy.Subscriber('/C25/publish',C25C0_ROP,self._Controller.Odom_cb)
        rs_sub = rospy.Subscriber('/atlas/atlas_state',AtlasState,self._Controller.RS_cb)
        rospy.sleep(0.3)
        self._Controller.JC.set_all_pos(self._Controller.RS.GetJointPos())
        # self._debug_cmd_sub = rospy.Subscriber('walker_command',Int32,self._debug_command)

        rospy.loginfo("DynamicLocomotion, task: %s" % ("DW Turn") )
        
        if self._Controller.RotateToOri(Bearing):
            print("SUCCESS!!")
            return RTResult_SUCCESSED("Finished in Success")
        else:
            print("FAIL")
            return RTResult(RobilTask_FAULT, "", "Did not end in success", False) 
        
        odom_sub.unregister()
        rs_sub.unregister()

###################################################################################
#---------------------------  a little testing script -----------------------------
###################################################################################
if __name__ == '__main__':
    rospy.init_node('DynamicLocomotion')
    node = DWTurn("DWTurn")
    rospy.spin()
    print "C42_DynamicLocomotion node Closed"