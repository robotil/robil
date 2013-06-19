#! /usr/bin/env python
import roslib; roslib.load_manifest('C42_ZMPWalk')
import rospy
from C31_PathPlanner.msg import C31_Waypoints, C31_Location
rospy.init_node('C31_clone')
pub = rospy.Publisher('/path',C31_Waypoints)
rospy.sleep(1)
wp = C31_Waypoints()
wp.points = [C31_Location(-3.0,-38.0),C31_Location(1.0,-38.0)]
#wp.points = [C31_Location(-3.0,-38.0),C31_Location(4.0,-38.0),C31_Location(4.0,-30.0),C31_Location(0.0,-27.0),C31_Location(0.0,-22.5)]
#wp.points = [C31_Location(0.0,0.0),C31_Location(7.0,0.0),C31_Location(7.0,8.0),C31_Location(3.0,11.0),C31_Location(3.0,15.5)]
pub.publish(wp)
rospy.spin()
