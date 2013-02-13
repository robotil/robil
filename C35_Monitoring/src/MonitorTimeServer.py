#! /usr/bin/env python
 
import roslib; roslib.load_manifest('C35_Monitoring')#pkg name
import rospy
import actionlib
import time
import sys
import C0_RobilTask.msg
from std_msgs.msg import String
from tree import xmlTree
from Node import node


TASK_RESULT_REJECT=1
TASK_RESULT_OK=0
TASK_RESULT_PLAN=-1
DEFAULT_THRESHOLD_TIME = 10000	#Should not be used!


	

class MonitorTimeServer(object):
	# create messages that are used to publish feedback/result
	_feedback = C0_RobilTask.msg.RobilTaskFeedback()
	_result   = C0_RobilTask.msg.RobilTaskResult()
	#These are used per instance
	_start_time = 0.0
	_monitored_node_id = ""
	_monitored_task_finished_on_time = False
	#These are used per Class
	_offline_computed_BT = None
	_node_start_times_by_id = {}
	_node_execution_times_by_id = {}
      	_threshold_time = DEFAULT_THRESHOLD_TIME
     
	def callback(self, data):
#		print "------------------------In Callback------------------------"
		current_time = time.time()
		finished_execution_tree = data.data.find("node=Unknown()") > -1
		if finished_execution_tree:
#			print "Finished execution tree... Exiting"
			return

		code = data.data[data.data.find("code=")+5]
		node_id = data.data[data.data.find("[id=")+4 : data.data.find("]", data.data.find("[id=")+4)]
		node_success = data.data.find(":OK$") > -1

#		print code, node_id, node_success
		if code == "1":		#This means a task is over
			start_time = MonitorTimeServer._node_start_times_by_id.get(node_id, self._start_time)     #this assigns start_time with default value of the monitor's start time if the start message for this node did not arrive.
			MonitorTimeServer._node_execution_times_by_id[node_id] = current_time - start_time

			#updating debug information in BTree
			finished_node = MonitorTimeServer._offline_computed_BT.getWrappedNode(node_id)
			finished_node.setDebug(str(node_success) + " " + str(current_time - start_time))
#			print "Added debug info: %s" % finished_node.getDebug()
			#TODO - Run Monte-Carlo sampling in debug mode to update tree
			#TODO - currently, adding the debug information and re-running MC doesn't change the average succ time although it should.
			#TODO - consult Adi about this, probably has to do with her methods.
#			node.debugMode = True
#			root = MonitorTimeServer._offline_computed_BT.getRoot()
#   			for i in range(100):
#        			root.runPlan(0)
#			if self._monitored_node_id:
#				self._threshold_time = MonitorTimeServer._offline_computed_BT.getWrappedNode(self._monitored_node_id).getAverageSuccTime(0) * 1.3
#				print "Updated threshold time of monitored node to:%f" % self._threshold_time

#			print "Node with id=%s has ended after %f seconds" % (node_id, MonitorTimeServer._node_execution_times_by_id[node_id])
			if self._monitored_node_id == node_id:    
				rospy.loginfo("The node we are monitoring has ended! Time:%f, Node_id:%s",MonitorTimeServer._node_execution_times_by_id.get(node_id),self._monitored_node_id)
				self._monitored_task_finished_on_time = True
		else:			#This means a task has started
#			print "Node with id=%s has begun." % node_id
			MonitorTimeServer._node_start_times_by_id[node_id] = time.time()
			if self._monitored_node_id == node_id: 
				rospy.loginfo("The node we are monitoring has started! Time:%f, Node_id:%s",MonitorTimeServer._node_start_times_by_id.get(node_id),self._monitored_node_id)
#		print MonitorTimeServer._node_start_times_by_id
#		print MonitorTimeServer._node_execution_times_by_id
#		print "------------------------Done Callback------------------------"
  
  
     	def __init__(self,event_file):
		print "TIME MONITOR SERVER STARTED"
		rospy.Subscriber("/executer/stack_stream", String, self.callback)
		
		event_file_full_path = sys.argv[0][0:-11]+event_file
		tsk_attributes_full_path = event_file_full_path[0:-4] + "_tsk_attribs.xml"
		print "Computing statistics for behavior tree taken from: %s" % sys.argv[0][0:-11]+event_file
		print "Task attributes taken from: %s" % tsk_attributes_full_path

		#creating tree and computing statistics
		MonitorTimeServer._offline_computed_BT = xmlTree(event_file_full_path, None,tsk_attributes_full_path)
		root = MonitorTimeServer._offline_computed_BT.getRoot()
    		node.parmetersInTheWorld = 1	#TODO - refactor this into the global variables of MonitorTimeServer
		node.debugMode = False		#TODO - refactor this into the global variables of MonitorTimeServer
    		for i in range(1000):
        		root.runPlan(0)
  
          	# I added this function- makes a map key- id/name, value- pointer to the wrapped node.
          	# example- revice name as unique id- we can also write  MonitorTimeServer._offline_computed_BT.createWrapperTreeMap("id")-Adi.
        	MonitorTimeServer._offline_computed_BT.createWrapperTreeMap("id") 

 
		self._action_name = "MonitorTime"
		self._as = actionlib.SimpleActionServer(self._action_name, C0_RobilTask.msg.RobilTaskAction, execute_cb=self.task)
		self._as.start()
		
		
		
	
	def finishTask(self, task_success, task_result, task_plan):
		print "FINISH TASK : ",task_success, task_result, task_plan

		#resetting public (per instance) variables.
		self._start_time = 0.0
		self._monitored_node_id = ""
		self._monitored_task_finished_on_time = False
		
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
		self._start_time = time.time()
		#rospy.Subscriber("/executer/stack_stream", String, self.callback)
		#print self._start_time
		#### GET TASK PARAMETERS ####
		rospy.loginfo("%s: Start: task name = %s",self._action_name, goal.name);
		rospy.loginfo("%s: Start: task id = %s", self._action_name, goal.uid);
		rospy.loginfo("%s: Start: task params = %s", self._action_name, goal.parameters);
    		self._monitored_node_id = goal.parameters.split("=")[1]

           	#from the xml tree get a wrappedNode by it's id/ name or a number (whatever we choose earlier in the constructor)
		MonitoredNode = MonitorTimeServer._offline_computed_BT.getWrappedNode(self._monitored_node_id)

           	# if the node existes-- suppose to be always true- unless we're not consistent with the event- get the time.      
           	if MonitoredNode!=None:
			#TODO - we are currently calling getAverageSuccTime with index = 0. Change to handle parameters of the world more robustly.
			#TODO - Currently using 130% of average time as threshold.  
			self._threshold_time = MonitoredNode.getAverageSuccTime(0) * 1.3
			rospy.loginfo("Monitored Node is:%s, with threshold time:%f",MonitoredNode.getAttrib("name"),self._threshold_time)
		else:
			rospy.loginfo("ERROR: Can't find monitored node with id:%s", self._monitored_node_id)


    		
		
           	while not rospy.is_shutdown() and time.time() - MonitorTimeServer._node_start_times_by_id.get(self._monitored_node_id, self._start_time) < self._threshold_time and not self._monitored_task_finished_on_time:
	            	rospy.sleep(0.2)
		if self._monitored_task_finished_on_time:
			rospy.loginfo("Monitored node finished on time! Finishing monitoring task with SUCCESS.")
			self.finishTask(True, TASK_RESULT_OK, "")
		else:
			rospy.loginfo("Monitored node passed threshold time!!! EXITING WITH ALERT")
			self.finishTask(False, TASK_RESULT_REJECT, "")
 
