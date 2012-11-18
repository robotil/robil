#!/usr/bin/env python
import roslib; roslib.load_manifest('c35_monitoring')
import rospy
import std_msgs.msg
import c35_monitoring.msg

def callbackArmControlFeedback(data):
    rospy.loginfo(rospy.get_name()+"arm control action percent complete: %f",data.feedback.progress)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/c35_monitoring/feedback", c35_monitoring.msg.arm_controlActionFeedback, callbackArmControlFeedback)
    rospy.spin()

if __name__ == '__main__':
    listener()


