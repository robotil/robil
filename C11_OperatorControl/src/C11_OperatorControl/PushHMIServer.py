
#! /usr/bin/env python

import roslib; 
#roslib.load_manifest('NAME OF PROJECT')

import rospy, math
import actionlib

import RobilTask.msg
from std_msgs.msg import Float64

TASK_RESULT_REJECT=0
TASK_RESULT_OK=1
TASK_RESULT_PLAN=2

class PushHMIServer(object):
  # create messages that are used to publish feedback/result
  _feedback = RobilTask.msg.RobilTaskFeedback()
  _result   = RobilTask.msg.RobilTaskResult()
  
    
  def __init__(self):
    self._action_name = "/PushHMI"
    self._as = actionlib.SimpleActionServer(self._action_name, RobilTask.msg.RobilTaskAction, execute_cb=self.task)
    self._as.start()

  def task(self, goal):
    task_success = True
    task_result = TASK_RESULT_OK
    task_plan = ""

    # start executing the action
    #### GET TASK PARAMETERS ####
    rospy.loginfo("%s: Start: task name = %s",self._action_name, goal.name);
    rospy.loginfo("%s: Start: task id = %s", self._action_name, goal.uid);
    rospy.loginfo("%s: Start: task params = %s", self._action_name, goal.parameters);

    #### HERE PROCESS TASK PARAMETERS ####

    #### DEFINE SLEEP DURATION BETWEEN TASK LOOP ITERATIONS ####
    r = rospy.Rate(100)

    #### SET NUMBER OF TASK LOOP ITERATIONS ####
    for i in xrange(1000): 
        if self._as.is_preempt_requested() or rospy.is_shutdown():
        
            #### HERE PROICESS PREEMTION OR INTERAPT #####
    
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            task_success = False
            break
            
            #### HERE PROCESS TASK ####
            
        r.sleep()
  
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
      
