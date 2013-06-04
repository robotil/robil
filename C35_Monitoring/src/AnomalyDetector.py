#! /usr/bin/env python

import roslib; roslib.load_manifest('C35_Monitoring')
import rospy
import C0_RobilTask.msg
from std_msgs.msg import String
from RobilTaskPy import *

		
class AnomalyDetector(RobilTask):
	runtimes_success = {}	#holds for every node, all the execution times when succeeded.
	runtimes_failure = {}	#holds for every node, all the execution times when failed.
	tasks_start_time = {}	#holds for every node, the current start time of that node.
	
def callback(self, msg):
    current_time = rospy.get_time()
    finished_execution_tree = data.data.find("node=Unknown()") > -1
    if finished_execution_tree:
        return
    code = data.data[data.data.find("code=")+5]
    node_id = data.data[data.data.find("[id=")+4 : data.data.find("]", data.data.find("[id=")+4)]
    node_success = data.data.find("OK") > -1
    print "-------------------------------------"
    print code, node_id, node_success        
    print "-------------------------------------"
    if code == "0":		#task has begun
        AnomalyDetector.tasks_start_time[node_id] = current_time
    else:
        runtime = current_time-AnomalyDetector.tasks_start_time[node_id]
        if not AnomalyDetector.tasks_start_time.has_key(node_id):                   
            return
        else:
            if node_success:
                if AnomalyDetector.runtimes_success.has_key(node_id):
                    AnomalyDetector.runtimes_success[node_id].append(runtime)
                else:
                    AnomalyDetector.runtimes_success[node_id] = [runtime]
            else:
                if AnomalyDetector.runtimes_failure.has_key(node_id):
                    AnomalyDetector.runtimes_failure[node_id].append(runtime)
                else:
                    AnomalyDetector.runtimes_failure[node_id] = [runtime]
 

	
	def __init__(self, name):
		print "Initializing AnomalyDetector Node"
		rospy.Subscriber("/executer/stack_stream", String, self.callback)
		RobilTask.__init__(self, name)
	
	def task(self, name, uid, parameters):
		print "Start Anomaly Detection." 
		#print parameters
		d = rospy.Duration(1,0)
		while True:	
			if self.isPreepted():
				print "Preempt Anomaly detector task"
				return RTResult_PREEPTED()
			rospy.sleep(d)
		print "EXITING anomaly detector task..."
		error_code = RobilTask_FAULT + 1
		return RTResult_ABORT(error_code,"Anomaly detector is exiting. "+str(error_code));
		#return RTResult_SUCCESSED("Finished in Success")
      
if __name__ == '__main__':
	print "Start AnomalyDetector Node"
	rospy.init_node('AnomalyDetector')
	AnomalyDetector("AnomalyDetector")
	rospy.spin()
	print "AnomalyDetector Node Closed" 
