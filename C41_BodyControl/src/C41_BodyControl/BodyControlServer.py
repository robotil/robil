#! /usr/bin/env python

import roslib, yaml; 
roslib.load_manifest('C41_BodyControl')

import rospy, math
import actionlib

import C0_RobilTask.msg
from std_msgs.msg import Float64

TASK_RESULT_REJECT=0
TASK_RESULT_OK=1
TASK_RESULT_PLAN=2

class BodyControlServer(object):
  # create messages that are used to publish feedback/result
  _feedback = C0_RobilTask.msg.RobilTaskFeedback()
  _result   = C0_RobilTask.msg.RobilTaskResult()
  
    
  def __init__(self):
    print "SERVER STARTED"
    self._action_name = "/BodyControl"
    self._as = actionlib.SimpleActionServer(self._action_name, C0_RobilTask.msg.RobilTaskAction, execute_cb=self.task)
    self._as.start()

  def task(self, goal):
    print "TASK STARTED"
    task_success = True
    task_result = TASK_RESULT_OK
    task_plan = ""

    # start executing the action

    #### GET TASK PARAMETERS ####
    rospy.loginfo("%s: Start: task name = %s",self._action_name, goal.name);
    rospy.loginfo("%s: Start: task id = %s", self._action_name, goal.uid);
    rospy.loginfo("%s: Start: task params = %s", self._action_name, goal.parameters);

    #### HERE PROCESS TASK PARAMETERS ####
    if goal.parameters == "param=MoveArm" :

      print "PARAM FAUND"

      _arm = rospy.Publisher('/r_arm_ely_position_controller/command', Float64)
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
	print "TASK PROCESS"
   	t = 6 * rospy.get_time()
    	next_pos =  0.4 + 0.4 * math.sin(t)
    	_arm.publish(next_pos)
	     
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
    print "TASK FINISHED"

