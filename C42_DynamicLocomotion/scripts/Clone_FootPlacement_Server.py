#!/usr/bin/env python
import roslib; roslib.load_manifest('C42_DynamicLocomotion')

from C42_DynamicLocomotion.srv import *
from C42_DynamicLocomotion.msg import Foot_Placement_data
import rospy
import math

########################################################################
### To test Service:                                                 ###
### 1) Run Service:                                                  ###
###    $ rosrun C42_DynamicLocomotion Clone_FootPlacement_Server.py  ###
### 2) Call Service:                                                 ###
###    $ rosservice call /foot_placement_path 0                      ###
########################################################################

def Get_foot_placement_path(req):
    FP_res = FootPlacement_ServiceResponse()
    FP_res.done = 0
    # assumption: placements are relative to left foot which is at position (0,0,0), in world frame 
    FP_res.foot_placement_path = [FP_data(1,[0.3,0.25,0.0],[0.0,0.0,0.0],0.0),FP_data(0,[0.6,0.0,0.0],[0.0,0.0,0.0],0.0),FP_data(1,[0.9,0.25,0.0],[0.0,0.0,0.0],0.0),\
                                  FP_data(0,[1.2,0.0,0.0],[0.0,0.0,0.0],0.0),FP_data(1,[1.5,0.25,0.0],[0.0,0.0,0.0],0.0)]
    print "Returning foot placement path: %s"%(FP_res.foot_placement_path)
    return FP_res

def FP_data(foot_index,position,euler_angle_deg,clearance_height):
    res = Foot_Placement_data()
    res.foot_index = foot_index
    res.pose.position.x = position[0]
    res.pose.position.y = position[1]
    res.pose.position.z = position[2]
    res.pose.ang_euler.x = deg2r(euler_angle_deg[0]) # roll
    res.pose.ang_euler.y = deg2r(euler_angle_deg[1]) # pitch
    res.pose.ang_euler.z = deg2r(euler_angle_deg[2]) # yaw
    res.clearance_height = clearance_height
    return res

def deg2r(deg):
    return (deg*math.pi/180.0)



def foot_placement_path_server():
    rospy.init_node('foot_placement_server')
    s = rospy.Service('foot_placement_path', FootPlacement_Service, Get_foot_placement_path)
    print "Ready to get foot placement path."
    rospy.spin()

if __name__ == "__main__":
    foot_placement_path_server()
