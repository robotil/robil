#! /usr/bin/env python
 
import roslib; roslib.load_manifest('C35_Monitoring')#pkg name
import rospy
import time
import sys
import C0_RobilTask.msg
from std_msgs.msg import String
from computeTree import *
from RobilTaskPy import *



DEFAULT_THRESHOLD_TIME = 10000	#Should not be used!
PARAM = 0


	

class MonitorTimeServer(RobilTask):
    _startTime = 0.0
    _monitoredNodeId = ""
    _monitoredTaskFinishedOnTime = False
    #These are used per Class
    _nodeStartTimesById = {}
    _nodeExecutionTimesById = {}
    _thresholdTime = DEFAULT_THRESHOLD_TIME
 

    def __init__(self,event_file,name):
        print "TIME MONITOR SERVER STARTED"
        rospy.Subscriber("/executer/stack_stream", String, self.callback)
        constructTree(event_file, PARAM)		
        RobilTask.__init__(self, name)


  
    def task(self, name, uid, parameters):
        print "START MONITORING OF TASK"
        self._startTime = time.time()
        rospy.loginfo("%s: Start: task name = %s",self._action_name, name);
        rospy.loginfo("%s: Start: task id = %s", self._action_name, uid);
        rospy.loginfo("%s: Start: task params = %s", self._action_name, parameters);
        self._monitoredNodeId = parameters['param']
        
        #calc self._monitoredNodeId attributs (prob, sd, E) from the ORIGINAL tree - NOT DEBUG MODE 
        (prob, sd, E) = getNodeInfo(self._monitoredNodeId, PARAM)
   	  # if the node existes--> it has attrib E-> suppose to be always true- unless we're not consistent with the event- get the time.      
        if E!=None:
		self._thresholdTime = E * 1.3
		rospy.loginfo("Monitored Node is:%s, with threshold time:%f",self._monitoredNodeId,self._thresholdTime)
        else:
		rospy.loginfo("ERROR: Can't find monitored node with id:%s", self._monitoredNodeId)
        
        # the monitor loop - run as long as: self._monitoredNodeId didn't pass self._thresholdTime
        while ((not rospy.is_shutdown()) 
        and ((time.time() - self._nodeStartTimesById.get(self._monitoredNodeId, self._startTime)) < self._thresholdTime) 
        and (not self._monitoredTaskFinishedOnTime)):
            rospy.sleep(0.2)
            
            
        if self._monitoredTaskFinishedOnTime:
		rospy.loginfo("Monitored node finished on time! Finishing monitoring task with SUCCESS.")
        else:
		rospy.loginfo("Monitored node passed threshold time!!! EXITING WITH ALERT")
  
        if self.isPreepted():
                print "Preempt monitoring task"
                return RTResult_PREEPTED()
        return RTResult_SUCCESSED("Finished in Success")
 
  
    def callback(self, data):

        #parsing msg from executor      
        current_time = time.time()
        finished_execution_tree = data.data.find("node=Unknown()") > -1
        if finished_execution_tree:
		return
        code = data.data[data.data.find("code=")+5]
        node_id = data.data[data.data.find("[id=")+4 : data.data.find("]", data.data.find("[id=")+4)]
        node_success = data.data.find("OK") > -1
#	  print code, node_id, node_success        
        #-------------------------------------

        #if task is over
        if code == "1":		#This means a task is over
            #this assigns start_time with default value of the monitor's start time if the start message for this node did not arrive.                        
            start_time = self._nodeStartTimesById.get(node_id, self._startTime)     
            #this assigns Execution time as the current_time - start_time            
            self._nodeExecutionTimesById[node_id] = current_time - start_time
            #local var -temp
            nodeTime = current_time - start_time

            print "Added debug info: ***[%s]***" % (str(node_success) + " " + str(nodeTime))
            #calc self._monitoredNodeId attributs (prob, sd, E) from the tree IN DEBUG MODE, AFTER UPDATING THE REAL TIME
            #print nodeTime,node_success,node_id, self._monitoredNodeId
            (prob, sd, E) = nodeDataInDebugMode(nodeTime,node_success,node_id, self._monitoredNodeId, 100 ,PARAM )
            
            if self._monitoredNodeId:
			self._thresholdTime = E * 1.3
			print "Updated threshold time of monitored node to:%f" % self._thresholdTime

#			print "Node with id=%s has ended after %f seconds" % (node_id, self._node_execution_times_by_id[node_id])
            if self._monitoredNodeId == node_id:    
			rospy.loginfo("The node we are monitoring has ended! Time:%f, Node_id:%s",self._nodeExecutionTimesById.get(node_id),self._monitoredNodeId)
			self._monitoredTaskFinishedOnTime = True
   
        #This means a task has started
        else:			
#			print "Node with id=%s has begun." % node_id
		self._nodeStartTimesById[node_id] = time.time()
		print "started some node"
		if self._monitoredNodeId == node_id: 
			rospy.loginfo("The node we are monitoring has started! Time:%f, Node_id:%s",self._nodeStartTimesById.get(node_id),self._monitoredNodeId)

  
  
 
		
		
		
	
	
    
