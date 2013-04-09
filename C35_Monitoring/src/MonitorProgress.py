#! /usr/bin/env python
 
import roslib; roslib.load_manifest('C35_Monitoring')#pkg name
import rospy
import time
import sys
import C0_RobilTask.msg
from std_msgs.msg import String
from computeTree import *
from RobilTaskPy import *
from std_msgs.msg import  Float32
from std_msgs.msg import Int32MultiArray
from C35_Monitoring.msg import progress


DEFAULT_THRESHOLD_TIME = 10000	#Should not be used!
DEADLINE = 30
PARAM = 0


	

class MonitorProgressServer(RobilTask):
    _startTime = 0.0
    _monitoredNodeId = ""
    _monitoredTaskFinishedOnTime = False
    #These are used per Class
    _nodeStartTimesById = {}
    _nodeExecutionTimesById = {}
    _thresholdTime = DEFAULT_THRESHOLD_TIME
    _priorE = 0.0
 

    def __init__(self,event_file,name):
        print "PROGRESS MONITOR SERVER STARTED"
        rospy.Subscriber("/executer/stack_stream", String, self.callback)
        constructTree(event_file, PARAM)		
        RobilTask.__init__(self, name)
        self.resultPublisher = rospy.Publisher("C35/Progress",progress)


  
    def task(self, name, uid, parameters):
        print "START MONITORING OF TASK"
        self._startTime = time.time()
        rospy.loginfo("%s: Start: task name = %s",self._action_name, name);
        rospy.loginfo("%s: Start: task id = %s", self._action_name, uid);
        rospy.loginfo("%s: Start: task params = %s", self._action_name, parameters);
        
        #calc self._monitoredNodeId attributs (prob, sd, E) from the ORIGINAL tree - NOT DEBUG MODE 
        (prob, std, E) = getNodeInfo(self._monitoredNodeId, PARAM)
        self._priorE = E
   	  # if the node existes--> it has attrib E-> suppose to be always true- unless we're not consistent with the event- get the time.      
        if E!=None:
            self._thresholdTime = E * 1.3
            rospy.loginfo("Monitored Node is:%s, with threshold time:%f",self._monitoredNodeId,self._thresholdTime)
            self.progressEstimation(std, E, DEADLINE, DEADLINE)
        else:
            rospy.loginfo("ERROR: Can't find monitored node with id:%s", self._monitoredNodeId)
        
        # the monitor loop - run as long as: self._monitoredNodeId didn't pass self._thresholdTime
        while ((not rospy.is_shutdown()) 
#        and ((time.time() - self._nodeStartTimesById.get(self._monitoredNodeId, self._startTime)) < self._thresholdTime) 
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
#            print "self._nodeStartTimesById" 
#            print self._nodeStartTimesById
#            print "self._nodeExecutionTimesById"
#            print self._nodeExecutionTimesById
            #calc self._monitoredNodeId attributs (prob, sd, E) from the tree IN DEBUG MODE, AFTER UPDATING THE REAL TIME
            
            (prob, std, E) = nodeDataInDebugMode(nodeTime,node_success,node_id, self._monitoredNodeId, 100, PARAM )
#            print "************************"
#            print nodeTime,node_success,node_id
#            print prob, std, E
#            print "************************"
            self._thresholdTime = E * 1.3
            self.progressEstimation(std, E, DEADLINE, self._priorE)
#            if self._monitoredNodeId:
#			self._thresholdTime = E * 1.3
#			print "Updated threshold time of monitored node to:%f" % self._thresholdTime
                
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

  
  
 
    def progressEstimation (self, std, E, deadLine, PriorE ):
#       print ("bla%f,%f" %(std, E))
       K = 1
       dangerZone=[]
       #Current expected completion time + K*std >= deadline : we are highly likely not to make it
       if ((E+K*std)>=deadLine):
           print "we are highly likely not to make it.."
           dangerZone.append(1)
       #Current expected completion time >= deadline: only 50% that we make it    
       elif (E>= deadLine):
           print "only 50% that we make it.."
           dangerZone.append(2)
       #Current expected completion time - K*std >= deadline: there is a reasonable chance that we will not make it    
       elif ((E-K*std)>=deadLine):    
           print "there is a reasonable chance that we will not make it.."
           dangerZone.append(3)
       else:
           print "everything is OK!"
           dangerZone.append(4)
       
       
       #Current expected completion time > Prior expected completion time + K*std: we are progressing slower than expected
       if (E > (PriorE + K*std)):
           print "we are progressing slower than expected"
           dangerZone.append(1)
       #Current expected completion time < Prior expected completion time - K*std: we are progressing faster than expected    
       elif (E > (PriorE - K*std)):
           print "we are progressing faster than expected"
           dangerZone.append(-1)
           
       self.resultPublisher.publish(dangerZone)    
	
	
    
