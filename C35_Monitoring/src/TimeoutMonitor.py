#! /usr/bin/env python

import roslib; roslib.load_manifest('C35_Monitoring')
import rospy
import actionlib
import time
from RobilTaskPy import *
		
class TimeoutMonitor(RobilTask):
	def __init__(self, name):
		print "Init timeout monitoring Node"
		RobilTask.__init__(self, name)
	
	def task(self, name, uid, parameters):
		print "Start Monitoring task " + name + " with parameters: " 
		print parameters
		r = rospy.Rate(1)
		start_time = time.time()
		threshold = int(parameters['threshold'])
		while time.time() - start_time < threshold:			
			if self.isPreepted():
				print "Preempt monitoring task"
				return RTResult_PREEPTED()
			r.sleep()
		print "Passed threhold time of " + str(threshold) + " seconds. EXITING..."
		error_code = RobilTask_FAULT + 1
		return RTResult_ABORT(error_code,"Timeout Monitor passed threshold time. "+str(error_code));
		#return RTResult_SUCCESSED("Finished in Success")
      
if __name__ == '__main__':
	print "Start TimeoutMonitor Node"
	rospy.init_node('TimeoutMonitor')
	TimeoutMonitor("TimeoutMonitor")
	rospy.spin()
	print "Timeour monitor Node Closed" 
