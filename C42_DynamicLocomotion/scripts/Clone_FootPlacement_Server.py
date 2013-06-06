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

class Nasmpace: pass
ns = Nasmpace()
ns._bSent_path_once = False

def Get_foot_placement_path(req):
    FP_res = FootPlacement_ServiceResponse()
    FP_res.done = 0
    # assumption: placements from foot model measurments (add off-set to ground in function FP_data)
    # FP_res.foot_placement_path = [FP_data(1,[-2.78,-38.1,0.1],[0.0,0.0,0.0],0.1),FP_data(0,[-2.50,-37.9,0.1],[0.0,0.0,0.0],0.1),FP_data(1,[-2.25,-38.1,0.1],[0.0,0.0,0.0],0.1),\
    #                               FP_data(0,[-2.00,-37.9,0.1],[0.0,0.0,0.0],0.1),FP_data(1,[-1.75,-38.1,0.1],[0.0,0.0,0.0],0.1),FP_data(0,[-1.50,-37.9,0.1],[0.0,0.0,0.0],0.1),\
    #                               FP_data(1,[-1.25,-38.1,0.1],[0.0,0.0,0.0],0.1),FP_data(0,[-1.1,-37.9,0.1],[0.0,0.0,0.0],0.1),FP_data(1,[-0.84,-38.1,0.1],[0.0,0.0,0.0],0.1),\
    #                               FP_data(0,[-0.84,-37.9,0.1],[0.0,0.0,0.40],0.1),FP_data(1,[-0.84,-38.2,0.1],[0.0,0.0,-0.4],0.1)]#,FP_data(0,[-0.84,-37.9,0.1],[0.0,0.0,-0.4],0.1)] # doesn't respond to yaw = -0.80 # FP_data(0,[-0.63,-37.87,0.1],[0.0,0.0,-1.579],0.1)
    FP_res.foot_placement_path = [FP_data(1,[5.751,6.721,0.099],[0.0,0.0,1.7598],0.1),FP_data(0,[5.483,6.8963,0.0988],[0.0,0.0,1.821],0.1),FP_data(1,[5.643,7.109,0.0988],[0.0,0.0,1.9597],0.1),\
                                  FP_data(0,[5.376,7.173,0.0988],[0.0,0.0,1.821],0.1),FP_data(1,[5.567,7.447,0.0988],[0.0,0.0,1.7198],0.1),FP_data(0,[5.3288,-7.613,0.0988],[0.0,0.0,1.661],0.1),\
                                  FP_data(1,[5.588,7.803,0.0988],[0.0,0.0,1.5198],0.1),FP_data(0,[5.334,8.033,0.0988],[0.0,0.0,1.5411],0.1)]#,FP_data(1,[-0.84,-38.1,0.1],[0.0,0.0,0.0],0.1),\
                                 # FP_data(0,[-0.84,-37.9,0.1],[0.0,0.0,0.40],0.1),FP_data(1,[-0.84,-38.2,0.1],[0.0,0.0,-0.4],0.1)]#,FP_data(0,[-0.84,-37.9,0.1],[0.0,0.0,-0.4],0.1)] # doesn't respond to yaw = -0.80 # FP_data(0,[-0.63,-37.87,0.1],[0.0,0.0,-1.579],0.1)

    print req.start_pose.pose.position
    if PositionsDistance(FP_res.foot_placement_path[0].pose.position, req.start_pose.pose.position) >= 0.5 or ns._bSent_path_once: # to avoid sending the same path a few times
    	FP_res.foot_placement_path = []

    print "Returning foot placement path: %s"%(FP_res.foot_placement_path)
    ns._bSent_path_once = True
    return FP_res

def FP_data(foot_index,position,euler_angle,clearance_height):
    res = Foot_Placement_data()
    foot_off_set = (0.06, 0.0, -0.085) # off-set from foot frame ('l_foot') to center of foot on ground  
    res.foot_index = foot_index
    res.pose.position.x = position[0] + foot_off_set[0]
    res.pose.position.y = position[1] + foot_off_set[1]
    res.pose.position.z = position[2] + foot_off_set[2]
    res.pose.ang_euler.x = euler_angle[0] # deg2r(euler_angle_deg[0]) # roll
    res.pose.ang_euler.y = euler_angle[1] # deg2r(euler_angle_deg[1]) # pitch
    res.pose.ang_euler.z = euler_angle[2] # deg2r(euler_angle_deg[2]) # yaw
    res.clearance_height = clearance_height
    return res

def PositionsDistance(position1,position2):
	return ( (position2.x-position1.x)**2 + (position2.y-position1.y)**2 )**0.5 #+ (position2.z-position1.z)**2

def deg2r(deg):
    return (deg*math.pi/180.0)



def foot_placement_path_server():
    rospy.init_node('foot_placement_server')
    s = rospy.Service('foot_placement_path', FootPlacement_Service, Get_foot_placement_path)
    print "Ready to get foot placement path."
    rospy.spin()

if __name__ == "__main__":    
    foot_placement_path_server()
