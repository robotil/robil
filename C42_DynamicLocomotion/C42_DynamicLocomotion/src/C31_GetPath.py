#!/usr/bin/env python
import roslib; roslib.load_manifest('C42_DynamicLocomotion')
 
from C31_PathPlanner.srv import *
from C31_PathPlanner.msg import C31_Location

import rospy
 
def pathis():
     wp = C31_GetPathResponse()
     wp.path.points = [C31_Location(k,k) for k in xrange(5)] 
     #print "Path_x is [%s] Path_y is [%s]"%(wp.path.points.x[0], points.y[0])
     return wp
 
def C31_GetPath():
     rospy.init_node('C31_GetPath_node')
     path = rospy.Service('C31_GetPath_srv', C31_GetPath, pathis)
     rospy.spin()
 
if __name__ == "__main__":
     C31_GetPath()


