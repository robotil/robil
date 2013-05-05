#! /usr/bin/env python
import roslib; roslib.load_manifest('C42_ZMPWalk')
import rospy
from C31_PathPlanner.msg import C31_Waypoints, C31_Location
rospy.init_node('C31_clone2')
pub = rospy.Publisher('/path',C31_Waypoints)
rospy.sleep(1)
wp = C31_Waypoints()
wp.points = [C31_Location(16.7,8.0),C31_Location(17.4,8.0),C31_Location(17.9,8.5),C31_Location(18.6,8.5),C31_Location(19.1,8.0),C31_Location(20.0,8.0)]
pub.publish(wp)
rospy.spin()
