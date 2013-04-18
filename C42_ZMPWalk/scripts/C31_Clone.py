#! /usr/bin/env python
import roslib; roslib.load_manifest('C42_ZMPWalk')
import rospy
from C31_PathPlanner.msg import C31_Waypoints, C31_Location
rospy.init_node('C31_clone')
pub = rospy.Publisher('/C31PathPlanner',C31_Waypoints)
rospy.sleep(1)
wp = C31_Waypoints()
wp.points = [C31_Location(-0.5,0),C31_Location(1,0),C31_Location(6,0),C31_Location(10,4),C31_Location(14,8),C31_Location(16.7,8)]
#wp.points = [C31_Location(-0.5,0),C31_Location(1,0),C31_Location(6,0),C31_Location(10,0),C31_Location(10,4),C31_Location(10,8),C31_Location(14,8),C31_Location(16.7,8)]
pub.publish(wp)
rospy.spin()
