#! /usr/bin/env python
 
import roslib; roslib.load_manifest('C35_Monitoring')#pkg name
import rospy
import actionlib
import time
import sys
import C0_RobilTask.msg
from std_msgs.msg import String
import math
import C51_CarOperation.msg
from C51_CarOperation.msg import Monitoring
import C0_RobilTask.msg
from std_msgs.msg import  Float32
from std_msgs.msg import String
from RobilTaskPy import *

TASK_RESULT_OK=0
TASK_RESULT_PLAN=-1
NUM_OF_ARGS_TO_AVG = 5
THRESHOLD = 6


	

class MonitorWayPointServer(RobilTask):
    # create messages that are used to publish feedback/result
    _feedback = C0_RobilTask.msg.RobilTaskFeedback()
    _result   = C0_RobilTask.msg.RobilTaskResult()
    #those DS are used for saving way points
    _wayPoints=[]
    _passedWayPoints=[]
    _dists=[]
    #avgMsg = C35_Monitoring.msg.statusResult()
                       
     
    def __init__(self, name):
        print "WAY POINTS MONITOR SERVER STARTED"
        rospy.Subscriber("/C51/m_feedback", Monitoring, self.callback)
        RobilTask.__init__(self, name)
        self.resultPublisher = rospy.Publisher("C35/MonitorWayPointResult",Float32)
  
    def callback(self, data):
        print "In callback"
        self._wayPoints.append(data.LastWPpassed) 
        self._passedWayPoints.append(data.MyLoc4LastWP)        
        size  = len(self._wayPoints)
        self._dists.append(self.dist(self._wayPoints[size-1], self._passedWayPoints[size-1]))
        avg = 0
        if size>=NUM_OF_ARGS_TO_AVG:
            avg = self.avgLast5Points(size-NUM_OF_ARGS_TO_AVG, size)
        if avg == 0:
            rospy.loginfo("Did not receive enough waypoints yet.")
        elif avg < THRESHOLD:
            rospy.loginfo("Monitored node did not pass threshold distance!")
        else:
            rospy.loginfo("Monitored node passed threshold distance!!! SEND AN ALERT")
        print "avg dist:"
        print avg  
        self.resultPublisher.publish(avg)
		
		
    def task(self, name, uid, parameters):
        print "MONITORING WAYPOINTS:"
        while not rospy.is_shutdown():
            rospy.sleep(1)
        if self.isPreepted():
                print "Preempt monitoring task"
                return RTResult_PREEPTED()
        return RTResult_SUCCESSED("Finished in Success")
	  
	
	  
        #if the average distance in the last 5 waypoints is > 3 meters
    def avgLast5Points(self, low, up):	
        sumDist=0
	#res = True
        for i in range(low,up):
#            sumDist = sumDist+self.dist(self._wayPoints[i], self._passedWayPoints[i])
            sumDist = sumDist + self._dists[i]
        return (sumDist/(up-low))
      
	
    def dist(self, p1, p2):	
        return math.sqrt((p2[0]-p1[0])**2+(p2[1]-p1[1])**2)
	
	
	