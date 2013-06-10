#!/usr/bin/env python
import roslib; roslib.load_manifest('C42_DynamicLocomotion')

from C42_DynamicLocomotion.srv import *
from C42_DynamicLocomotion.msg import Foot_Placement_data
import rospy
import math

########################################################################
### To test Service:                                                 ###
### 1) Run Service:                                                  ###
###    $ rosrun C42_DynamicLocomotion Clone_C23_orient.py            ###
### 2) Call Service:                                                 ###
###    $ rosservice call /foot_aline_pose "rotation"                  ###
########################################################################

class Nasmpace: pass
ns = Nasmpace()
ns._bSent_path_once = False

def Get_target_pose(req):
    FP_res = C23_orientResponse()
    if "rotation" == req.target:
        FP_res.x = 0.0
        FP_res.y = 0.0
        FP_res.z = 0.0
        FP_res.R = 0.0
        FP_res.P = 0.0
        FP_res.Y = -math.pi/2
    else:
        FP_res.x = 0.3
        FP_res.y = 0.0
        FP_res.z = 0.0
        FP_res.R = 0.0
        FP_res.P = 0.0
        FP_res.Y = 0.0

    if ns._bSent_path_once: # to avoid sending the same target pose a few times
        FP_res.x = 0.0
        FP_res.y = 0.0
        FP_res.z = 0.0
        FP_res.R = 0.0
        FP_res.P = 0.0
        FP_res.Y = 0.0

    print "Returning target pose: %s"%(FP_res)
    ns._bSent_path_once = True
    return FP_res

def deg2r(deg):
    return (deg*math.pi/180.0)



def C23_orient_server():
    rospy.init_node('foot_AlinePose_server')
    s = rospy.Service('foot_aline_pose', C23_orient, Get_target_pose)
    print "Ready to get target pose."
    rospy.spin()

if __name__ == "__main__":    
    C23_orient_server()
