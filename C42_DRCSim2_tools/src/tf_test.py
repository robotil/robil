#! /usr/bin/env python
import roslib; roslib.load_manifest('DRCSim2_tools')
import drc2_tools
import rospy
RL = drc2_tools.robot_listner()
rospy.sleep(.5)
for k in xrange(10):
	(trans,rot,T) = RL.get_transform('/l_foot','/pelvis')
	Delay = rospy.Time.now() - T
	rospy.loginfo('tf time: %s, delay(sec): %s',T.to_sec(),Delay.to_sec())
	rospy.sleep(0.01)
