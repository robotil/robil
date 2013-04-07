#! /usr/bin/env python

import roslib; roslib.load_manifest('C35_Monitoring')
import rospy
from RobilTaskPy import *
from std_msgs.msg import Int8	#Boolean value indicating the robot has fallen.
		
class StabilityMonitor(RobilTask):
	robot_stability_measure = 5	#maximal stability measure
	
	def callback(self, data):
		if data.data != StabilityMonitor.robot_stability_measure:
			print "In callback, detected change of stability to " + str(data.data) + "."
			StabilityMonitor.robot_stability_measure = data.data
	
	def __init__(self, name):
		print "Initializing falling monitoring Node"
		rospy.Subscriber("/C35/stability", Int8, self.callback)
		StabilityMonitor.robot_stability_measure = 5
		RobilTask.__init__(self, name)
	
	def task(self, name, uid, parameters):
		print "Start Monitoring the stability of the robot." 
		d = rospy.Duration(1,0)
		while StabilityMonitor.robot_stability_measure > 1:	
			if self.isPreepted():
				print "Preempt stability monitoring task"
				return RTResult_PREEPTED()
			rospy.sleep(d)
		print "Robot is dangerously unstable! EXITING..."
		StabilityMonitor.robot_stability_measure = 5
		error_code = RobilTask_FAULT + 1
		return RTResult_ABORT(error_code,"Stability Monitor detected a dangerous change of stability. "+str(error_code));
		#return RTResult_SUCCESSED("Finished in Success")
      
if __name__ == '__main__':
	print "Start StabilityMonitor Node"
	rospy.init_node('StabilityMonitor')
	StabilityMonitor("StabilityMonitor")
	rospy.spin()
	print "Stability monitor Node Closed" 
