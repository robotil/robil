#! /usr/bin/env python
  
import roslib; roslib.load_manifest('C51_CarOperation')
import rospy
import actionlib
import time
import sys
import C51_CarOperation.msg
from C51_CarOperation.msg import Monitoring
#import C0_RobilTask.msg
from std_msgs.msg import  Float32
from std_msgs.msg import String



#TASK_RESULT_REJECT=1
#TASK_RESULT_OK=0
#TASK_RESULT_PLAN=-1
#DEFAULT_THRESHOLD_TIME = 10000	#Should not be used!
    
    	
    
class MonitorWayPointServer(object):
    # create messages that are used to publish feedback/result
    #_feedback = C0_RobilTask.msg.RobilTaskFeedback()
    #_result   = C0_RobilTask.msg.RobilTaskResult()
    _wayPoints=[]
    _passedWayPoints=[]

    def __init__(self, name):
        print "WAY POINTS MONITOR SERVER STARTED"
        rospy.Subscriber("/C51/m_feedback", Monitoring, self.callback)
        #temp = rospy.Subscriber("C51_CarOperation/DriveActionFeedback", C51_CarOperation.msg.DriveFeedback, self.callback)
        #print temp
        self._action_name = "MonitorWayPointServer"
        self._as = actionlib.SimpleActionServer(self._action_name, C51_CarOperation.msg.DriveAction, execute_cb=self.monitor, auto_start=False)
        self._as.start() 

      
    def callback(self, data):
        print "In callback"
        self._wayPoints.append(data.LastWPpassed) 
        self._passedWayPoints.append(data.MyLoc4LastWP)


    def monitor(self, goal):
        print "MONITORING:"
        #if the average distance in the last 5 waypoints is > 3 meters


if __name__ == '__main__':
  try:
    rospy.init_node('C35_MonitoringWayPoint')
    MonitorWayPointServer("MonitorWayPointServer") 
    #while not rospy.is_shutdown():
    #  rospy.sleep(0.2)
    rospy.spin()
  except rospy.ROSInterruptException: 
    print "something with init went bad.."
