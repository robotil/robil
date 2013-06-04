#! /usr/bin/env python
import roslib; roslib.load_manifest('C42_DynamicLocomotion')
import rospy
from C25_GlobalPosition.msg import C25C0_ROP
from nav_msgs.msg import Odometry

################# To Run Clone type: ###########################
## $ rosrun C42_DynamicLocomotion C25_GlobalPosition_Clone.py ##
################################################################

def _odom_cb(odom):
        GlobalPosition = C25C0_ROP()
        GlobalPosition.header = odom.header
        GlobalPosition.pose = odom
        pub.publish(GlobalPosition)

if __name__ == '__main__':
    rospy.init_node('C25_GlobalPosition_clone')
    pub = rospy.Publisher('/C25/publish',C25C0_ROP)
    rospy.sleep(1)
    sub = rospy.Subscriber('/ground_truth_odom',Odometry,_odom_cb)
    print ("Created C25_GlobalPosition clone of /ground_truth_odom")
    rospy.spin()
