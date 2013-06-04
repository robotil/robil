#! /usr/bin/env python

import roslib; roslib.load_manifest('C35_Monitoring')
import rospy
from RobilTaskPy import *
from std_msgs.msg import Bool	#Boolean value indicating the robot has fallen.
#from nav_msgs.msg import Odometry
from C25_GlobalPosition.msg import C25C0_ROP
import math
		
class DrivingMonitor(RobilTask):
	detected_problem = False
	init_x = -1
	init_y = -1
	init_time = -1
	started_task = False
	
	def callback(self, msg):
		if DrivingMonitor.started_task:
		  DrivingMonitor.started_task = False
		  DrivingMonitor.init_x = msg.pose.pose.pose.position.x
		  DrivingMonitor.init_y = msg.pose.pose.pose.position.y
		  print "Driving monitor got first message."
		else:  
		  delta_x = msg.pose.pose.pose.position.x - DrivingMonitor.init_x
		  delta_y = msg.pose.pose.pose.position.y - DrivingMonitor.init_y
		  elapsed_time = rospy.get_time() - DrivingMonitor.init_time
		  
		  if DrivingMonitor.init_x != -1 and DrivingMonitor.init_y != -1 and elapsed_time > 10 and math.fabs(delta_x) < 1 and math.fabs(delta_y) < 1:
			  DrivingMonitor.detected_problem = True
	
	def __init__(self, name):
		print "Initializing driving monitoring Node"
		#rospy.Subscriber("/ground_truth_odom", Odometry, self.callback)
		rospy.Subscriber('/C25/publish',C25C0_ROP,self.callback) 
		DrivingMonitor.detected_problem = False
		RobilTask.__init__(self, name)
	
	def task(self, name, uid, parameters):
		print "Start Monitoring driving of the robot." 
		#print parameters
		d = rospy.Duration(1,0)
		
		DrivingMonitor.init_time = rospy.get_time()
		DrivingMonitor.started_task = True
		print "Driving monitor received first message."
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
	print "Start DrivingMonitor Node"
	rospy.init_node('DrivingMonitor')
	DrivingMonitor("DrivingMonitor")
	rospy.spin()
	print "Driving monitor Node Closed" 
