#! /usr/bin/env python
import roslib; roslib.load_manifest('C42_ZMPWalk')
import rospy
from C31_PathPlanner.msg import C31_Waypoints, C31_Location
rospy.init_node('C31_clone')
pub = rospy.Publisher('/path',C31_Waypoints)
rospy.sleep(1)
wp = C31_Waypoints()
wp.points = [C31_Location(0.5,-10.0),C31_Location(0.6,-10.02),C31_Location(7.3,-7.9),C31_Location(10.37,-4.84),C31_Location(10.54,-2),C31_Location(6.0,6.46)]
pub.publish(wp)
rospy.spin()
