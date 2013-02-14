#! /usr/bin/env python

import roslib; roslib.load_manifest('C42_DynamicLocomotion')
import numpy as np
import rospy
import actionlib
from nav_msgs.msg import Odometry
import C42_DynamicLocomotion.msg
from std_msgs.msg import Float64
import geometry_msgs.msg as gm

class CpgWlkServer(object):
  # create messages that are used to publish feedback/result
  _feedback = C42_DynamicLocomotion.msg.C42_CpgWlkFeedback()
  _result   = C42_DynamicLocomotion.msg.C42_CpgWlkResult()
    
  def __init__(self):
    self._action_name = "/CGPWalk"
    self._as = actionlib.SimpleActionServer(self._action_name, C42_DynamicLocomotion.msg.C42_CpgWlkAction, execute_cb=self.task)
    self._as.start()
    #getting position and orientation
    self._pos_sub = rospy.Subscriber("/ground_truth_odom",Odometry,self.get_pos)
    self._pos = gm.Point()
    self._or = gm.Quaternion()
    #tolerance for deviation from goal
    self._tol = 0.1
    #ensure we enter the main loop at least once
    self._dis_from_goal = self._tol + 1

  def get_pos(self,odom):
    #get robot position, currently from /ground_truth_odom, later from navigation node
    self._pos = odom.pose.pose.position
    self._or = odom.pose.pose.orientation

  def _get_path(self,goal,path_planner = "C31_Path_Planner",srv="C31PathPlan"):
    #client for path planning service from C31
    rospy.loginfo("waiting for path planing service")
    rospy.wait_for_service(path_planner)
    try:
      pth_pln = rospy.ServiceProxy(path_planner, srv)
      pth = pth_pln(goal)
      return resp1
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e
    pass

  def _plan_pref(self,path):
    #plan ref ZMP based on given path
    pass

  def _zmp_control(self,p_ref):
    #control legs based on ref ZMP
    pass

  def task(self, goal):

    # start executing the action

    #### LOG TASK PARAMETERS ####
    rospy.loginfo("started CGPWalk")
    rospy.loginfo("Target position: x:%s y:%s", goal.goal_pos.x, goal.goal_pos.y)
    task_success = True
    self._tol = goal.tol

    #### TASK ####
    while self._dis_from_goal > self._tol:
      #calculate distance from goal
      pos = np.array([self._pos.x,self._pos.y])
      gl = np.array([goal.goal_pos.x, goal.goal_pos.y])
      self._dis_from_goal = np.linalg.norm(gl-pos)
      self._feedback.dis_to_goal = self._dis_from_goal
      self._as.publish_feedback(self._feedback)
      rospy.sleep(1)
      if self._as.is_preempt_requested() or rospy.is_shutdown():
        #### HERE PROICESS PREEMTION OR INTERUPT #####
        rospy.loginfo('%s: Preempted' % self._action_name)
        self._as.set_preempted()
        task_success = False
        break

    if task_success:
      self._result.res_pos.x = self._pos.x
      self._result.res_pos.y = self._pos.y
      self._result.res_pos.theta = self._or.z
      self._result.dis = self._dis_from_goal      
      rospy.loginfo('%s: Succeeded' % self._action_name)
      self._as.set_succeeded(self._result)


if __name__ == '__main__':
  rospy.init_node('CPG_server')
  CpgWlkServer()
  rospy.spin()