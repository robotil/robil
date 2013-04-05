#! /usr/bin/env python

import roslib; roslib.load_manifest('C35_Monitoring')
import rospy
from RobilTaskPy import *
from std_msgs.msg import Bool	#Boolean value indicating the robot has fallen.
		
class FallingMonitor(RobilTask):
	robot_has_fallen = False
	
	def callback(self, data):
		if data.data:
			print "In callback, detected fall message."
			FallingMonitor.robot_has_fallen = True
	
	def __init__(self, name):
		print "Initializing falling monitoring Node"
		rospy.Subscriber("/C35/falling", Bool, self.callback)
		FallingMonitor.robot_has_fallen = False
		RobilTask.__init__(self, name)
	
	def task(self, name, uid, parameters):
		print "Start Monitoring falling of the robot, parameters: " 
		print parameters
		d = rospy.Duration(1,0)
		while not FallingMonitor.robot_has_fallen:	
			print FallingMonitor.robot_has_fallen
			if self.isPreepted():
				print "Preempt fall monitoring task"
				return RTResult_PREEPTED()
			rospy.sleep(d)
		print "Robot has fallen! EXITING..."
		FallingMonitor.robot_has_fallen = False
		error_code = RobilTask_FAULT + 1
		return RTResult_ABORT(error_code,"Falling Monitor detected a fall. "+str(error_code));
		#return RTResult_SUCCESSED("Finished in Success")
      
if __name__ == '__main__':
	print "Start FallingMonitor Node"
	rospy.init_node('FallingMonitor')
	FallingMonitor("FallingMonitor")
	rospy.spin()
	print "Falling monitor Node Closed" 
