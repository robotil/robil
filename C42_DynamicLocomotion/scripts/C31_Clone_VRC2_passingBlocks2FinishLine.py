#! /usr/bin/env python
import roslib; roslib.load_manifest('C42_ZMPWalk')
import rospy
from C31_PathPlanner.msg import C31_Waypoints, C31_Location
rospy.init_node('C31_clone')
pub = rospy.Publisher('/path',C31_Waypoints)
rospy.sleep(1)
wp = C31_Waypoints()
wp.points = [C31_Location(6.0,6.5),C31_Location(7.0,7.5),C31_Location(6.5,8.2),C31_Location(5.5,9.2),C31_Location(7.2,11.2),C31_Location(6.0,13.0),C31_Location(7.2,13.7),C31_Location(6.6,15.5),C31_Location(6.6,18.5)]
#wp.points = [C31_Location(4.3,6.5),C31_Location(4.0,7.4),C31_Location(4.4,8.5),C31_Location(5.6,9.0),C31_Location(7.2,11.2),C31_Location(5.9,12.3),C31_Location(7.2,13.7),C31_Location(6.6,15.5),C31_Location(6.6,18.5)]
pub.publish(wp)
rospy.spin()
