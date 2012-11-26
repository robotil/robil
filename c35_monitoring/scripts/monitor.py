#!/usr/bin/env python
import roslib; roslib.load_manifest('c35_monitoring')
import rospy
import std_msgs.msg
import c35_monitoring.msg

def callback_finger_grab(data):
    rospy.loginfo(rospy.get_name()+"Finger grab action percent complete: %i",data.feedback.percent_done)

def callbackArmControlFeedback(data):
    rospy.loginfo(rospy.get_name()+"arm control action percent complete: %f",data.feedback.progress)
   
def callbackSittingControlFeedback(data)
    rospy.loginfo(rospy.get_name()+"sitting action percent complete: %i",data.feedback.percent_done)
    
    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/finger_grab/feedback", c35_monitoring.msg.finger_grabActionFeedback, callback_finger_grab)
    rospy.Subscriber("/arm_control/feedback", c35_monitoring.msg.arm_controlActionFeedback, callbackArmControlFeedback)
    rospy.Subscriber("/sitting/feedback",c35_monitoring.msg.sittingFeedback,callbackSittingControlFeedback)
    rospy.spin()

if __name__ == '__main__':
    listener() 


