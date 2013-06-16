#! /usr/bin/env python
import roslib; roslib.load_manifest('C42_ZMPWalk')
import rospy
from C31_PathPlanner.msg import C31_Waypoints, C31_Location
rospy.init_node('C31_clone')
pub = rospy.Publisher('/path',C31_Waypoints)
rospy.sleep(1)
wp = C31_Waypoints()
#wp.points = [C31_Location(6.0,6.46),C31_Location(6.157,6.556),C31_Location(6.185,18.324)]
wp.points = [C31_Location(9,44.5),C31_Location(9,58)]
pub.publish(wp)
rospy.spin()
