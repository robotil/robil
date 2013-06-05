#! /usr/bin/env python

import roslib; roslib.load_manifest('C35_Monitoring')
import rospy
from RobilTaskPy import *
from std_msgs.msg import Bool	#Boolean value indicating the robot has fallen.
#from nav_msgs.msg import Odometry
from C25_GlobalPosition.msg import C25C0_ROP

import math
		
class WalkingMonitor(RobilTask):
	detected_problem = False
	init_x = -1
	init_y = -1
	init_time = -1
	started_task = False
	
	def callback(self, msg):
		if WalkingMonitor.started_task:
		  WalkingMonitor.started_task = False
		  WalkingMonitor.init_x = msg.pose.pose.pose.position.x
		  WalkingMonitor.init_y = msg.pose.pose.pose.position.y
		  print "Walking monitor got first message."
		else:  
		  delta_x = msg.pose.pose.pose.position.x - WalkingMonitor.init_x
		  delta_y = msg.pose.pose.pose.position.y - WalkingMonitor.init_y
		  elapsed_time = rospy.get_time() - WalkingMonitor.init_time
		  #print "x" ,msg.pose.pose.pose.position.x, WalkingMonitor.init_x
		  #print "y" ,msg.pose.pose.pose.position.y, WalkingMonitor.init_y
		  print "time" ,elapsed_time
		  print "delta" ,delta_x, delta_y
		  
		  if WalkingMonitor.init_x != -1 and WalkingMonitor.init_y != -1 and elapsed_time > 20 and math.fabs(delta_x) < 0.2 and math.fabs(delta_y) < 0.2:
			  WalkingMonitor.detected_problem = True
	
	def __init__(self, name):
		print "Initializing walking monitoring Node"
		rospy.Subscriber('/C25/publish',C25C0_ROP,self.callback) 
		#rospy.Subscriber("/ground_truth_odom", Odometry, self.callback)
		WalkingMonitor.detected_problem = False
		RobilTask.__init__(self, name)
	
	def task(self, name, uid, parameters):
		print "Start Monitoring walking of the robot." 
		#print parameters
		d = rospy.Duration(1,0)
		
		WalkingMonitor.init_time = rospy.get_time()
		WalkingMonitor.started_task = True
		print "Walking monitor received first message."
		while not WalkingMonitor.detected_problem:	
			#print WalkingMonitor.detected_problem
			if self.isPreepted():
				print "Preempt walking monitoring task"
				return RTResult_PREEPTED()
			rospy.sleep(d)
		print "detected problem in walking module! EXITING..."
		WalkingMonitor.detected_problem = False
		error_code = RobilTask_FAULT + 1
		return RTResult_ABORT(error_code,"Walking Monitor detected a problem. "+str(error_code));
		#return RTResult_SUCCESSED("Finished in Success")
      
if __name__ == '__main__':
	print "Start WalkingMonitor Node"
	rospy.init_node('WalkingMonitor')
	WalkingMonitor("WalkingMonitor")
	rospy.spin()
	print "Walking monitor Node Closed" 
