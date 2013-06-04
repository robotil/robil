#! /usr/bin/env python

import roslib; roslib.load_manifest('C35_Monitoring')
import rospy
import C0_RobilTask.msg
from std_msgs.msg import String
from RobilTaskPy import *

		
class AnomalyDetector(RobilTask):
	runtimes_success = {}
	tasks_start_time = {}
	
	def callback(self, msg):
		runtimes_success["johny"].
	
	def __init__(self, name):
		print "Initializing AnomalyDetector Node"
		rospy.Subscriber("/executer/stack_stream", String, self.callback)
		RobilTask.__init__(self, name)
	
	def task(self, name, uid, parameters):
		print "Start Anomaly Detection." 
		#print parameters
		d = rospy.Duration(1,0)
		
		DrivingMonitor.init_time = rospy.get_time()
		DrivingMonitor.started_task = True
		while not DrivingMonitor.detected_problem:	
			#print DrivingMonitor.detected_problem
			if self.isPreepted():
				print "Preempt driving monitoring task"
				return RTResult_PREEPTED()
			rospy.sleep(d)
		print "detected problem in driving module! EXITING..."
		DrivingMonitor.detected_problem = False
		error_code = RobilTask_FAULT + 1
		return RTResult_ABORT(error_code,"Driving Monitor detected a problem. "+str(error_code));
		#return RTResult_SUCCESSED("Finished in Success")
      
if __name__ == '__main__':
	print "Start AnomalyDetector Node"
	rospy.init_node('AnomalyDetector')
	AnomalyDetector("AnomalyDetector")
	rospy.spin()
	print "AnomalyDetector Node Closed" 
