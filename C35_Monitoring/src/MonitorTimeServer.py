#! /usr/bin/env python
 
import roslib; roslib.load_manifest('C35_Monitoring')#pkg name
import rospy
import actionlib
import time
import sys
import C0_RobilTask.msg
from std_msgs.msg import String
from tree import xmlTree


TASK_RESULT_REJECT=1
TASK_RESULT_OK=0
TASK_RESULT_PLAN=-1

def callback(data):
	#rospy.loginfo(rospy.get_name() + ": I heard %s" % data.data)
	code = data.data[data.data.find("code=")+5]
	node_id = data.data[data.data.find("[id=")+4 : data.data.find("]", data.data.find("[id=")+4)]
	if code == "1":		#This means a task is over
		print "Node with id=%s has ended." % node_id
		if MonitorTimeServer._monitored_node_id == node_id:
			print "The node we are monitoring has ended! Time:%f" % (time.time() - MonitorTimeServer._start_time)
			MonitorTimeServer._monitored_task_finished_on_time = True
	else:			#This means a task has started
		print "Node with id=%s has begun." % node_id
		if MonitorTimeServer._monitored_node_id == node_id:
			MonitorTimeServer._start_time = time.time()
	

class MonitorTimeServer(object):
	# create messages that are used to publish feedback/result
	_feedback = C0_RobilTask.msg.RobilTaskFeedback()
	_result   = C0_RobilTask.msg.RobilTaskResult()
	_start_time = 0.0
	_monitored_node_id = ""
	_monitored_task_finished_on_time = False
	_offline_computed_BT = None
      
     
     

     	def __init__(self,event_file):
		print "TIME MONITOR SERVER STARTED"
		#self.listOfTimes=[]

		print "Behavior tree taken from: %s" % sys.argv[0][0:-11]+event_file		
		MonitorTimeServer._offline_computed_BT = xmlTree(sys.argv[0][0:-11]+event_file)

 
		self._action_name = "MonitorTime"
		self._as = actionlib.SimpleActionServer(self._action_name, C0_RobilTask.msg.RobilTaskAction, execute_cb=self.task)
		self._as.start()
		
		
		
	
	def finishTask(self, task_success, task_result, task_plan):
		print "FINISH TASK : ",task_success, task_result, task_plan
		if task_success:
			self._result.success = task_result;
			rospy.loginfo("%s: Succeeded", self._action_name);
			if task_result == TASK_RESULT_PLAN:
				ROS_INFO("%s: New plan", self._action_name);
				self._result.plan = task_plan;
			self._as.set_succeeded(self._result);
		else:
			rospy.loginfo("%s: Aborted", self._action_name);
			self._as.set_aborted(self._result);

	def task(self, goal):
		print "START MONITORING OF TASK"
		MonitorTimeServer._start_time = time.time()
		print MonitorTimeServer._start_time
		#### GET TASK PARAMETERS ####
		rospy.loginfo("%s: Start: task name = %s",self._action_name, goal.name);
		rospy.loginfo("%s: Start: task id = %s", self._action_name, goal.uid);
		rospy.loginfo("%s: Start: task params = %s", self._action_name, goal.parameters);
    		MonitorTimeServer._monitored_node_id = goal.parameters.split("=")[1]

		#MonitorTimeServer._monitored_node_id  is the id of the task we must monitor

#		task_success = True
#		task_result = TASK_RESULT_OK
		task_plan = ""

		#TODO - get the average completion time of the node we need to monitor 
		
		
		#rospy.init_node('stack_stream_listener', anonymous=True)
		rospy.Subscriber("/executer/stack_stream", String, callback)
    		
		threshold_time = 10	#TODO - get the average time for the monitored node!!!
		while not rospy.is_shutdown() and time.time() - MonitorTimeServer._start_time  < threshold_time and not MonitorTimeServer._monitored_task_finished_on_time:
	            rospy.sleep(0.2)
		if MonitorTimeServer._monitored_task_finished_on_time:
			print "Monitored node finished on time! Finishing monitoring task with SUCCESS."
			self.finishTask(True, TASK_RESULT_OK, task_plan)
		else:
			print "Passed threshold time!!! EXITING WITH ALERT"
			self.finishTask(False, TASK_RESULT_REJECT, task_plan)
 
