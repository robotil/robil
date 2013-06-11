#! /usr/bin/env python
import roslib; roslib.load_manifest('C42_ZMPWalk')
import rospy
from C31_PathPlanner.msg import C31_Waypoints, C31_Location
rospy.init_node('C31_clone')
pub = rospy.Publisher('/path',C31_Waypoints)
rospy.sleep(1)
wp = C31_Waypoints()
#wp.points = [C31_Location(0.0,-22.5),C31_Location(0.0,-16.0),C31_Location(0.5,-10.0)]
#wp.points = [C31_Location(0.0,0.0),C31_Location(6.0,0.0),C31_Location(12.0,0.0)]
wp.points = [C31_Location(3.0,16.0),C31_Location(3.0,22.5),C31_Location(3.0,28.5)]
pub.publish(wp)
rospy.spin()
