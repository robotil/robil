#! /usr/bin/env python
import roslib; roslib.load_manifest('C42_ZMPWalk')
import rospy
from C31_PathPlanner.msg import C31_Waypoints, C31_Location
rospy.init_node('C31_clone')
pub = rospy.Publisher('/path',C31_Waypoints)
rospy.sleep(1)
wp = C31_Waypoints()
wp.points = [C31_Location(-7.0,-15.0),C31_Location(-0.5,-15.0),C31_Location(-1.0,-7.5),C31_Location(-2.0,-5.1)]
pub.publish(wp)
rospy.spin()
