#! /usr/bin/env python
 
import roslib; roslib.load_manifest('C35_Monitoring')#pkg name
import rospy
import actionlib
import time

from chores.msg import *

class MonitorTimeServer:
     # create messages that are used to publish feedback/result
    _feedback = C35_Monitoring.msg.monitorTimeFeedback()
    _result   = C35_Monitoring.msg.monitorTimeResult()
        
    
    def __init__(self):
      self.listOfTimes=[]
      #self.start = time.time()
      self.server = actionlib.SimpleActionServer('monitor_time', C35_Monitoring.msg.monitorTimeAction, self.execute, False)
      self.server.start()
    
    
    def execute(self, goal):
      # Do lots of awesome groundbreaking robot stuff here-or just count time :)
      code = getCode(goal)
      if code == 0:
          listOfTimes.append(time.time())
      else: #code == 1
          endTime = time.time()
          startTime = 0
          if len(listOfTime) > 0: #list is not empty
              startTime = listOfTimes.pop()
          #updateExcuteTime
          executeTime = endTime-startTime
          #update result msg/feedback with executeTime
      
      
      success = True

      r.sleep()
      
      self.server.publish_feedback(self._feedback)
      if success:
         #rospy.loginfo('%s: Succeeded' % node_name)
         self._as.set_succeeded(self._result)
    
    
    def getCode(self,msg):#parse the msg /goal
        pass #get 0- node start, 1 node ends
        
    def getNodeName(self,msg):
        pass
    
    
    
    
    if __name__ == '__main__':
        rospy.init_node('monitor_time')
        server = MonitorTimeServer()
        rospy.spin()

