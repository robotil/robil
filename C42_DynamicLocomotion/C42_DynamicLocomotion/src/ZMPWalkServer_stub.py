#! /usr/bin/env python

import roslib; roslib.load_manifest('C42_DynamicLocomotion')
import numpy as np
import rospy
import actionlib
from nav_msgs.msg import Odometry
import C42_DynamicLocomotion.msg
from C31_PathPlanner.srv import *
from C31_PathPlanner.msg import C31_Location
from std_msgs.msg import Float64, Int32
import geometry_msgs.msg as gm


class ZmpWlkServer(object):
  # create messages that are used to publish feedback/result
  _feedback = C42_DynamicLocomotion.msg.C42_ZmpWlkFeedback()
  _result   = C42_DynamicLocomotion.msg.C42_ZmpWlkResult()
    
  def __init__(self):
    rospy.loginfo('zmp initialized')
    self.walk_pub = rospy.Publisher('/atlas/cmd_vel',gm.Twist)
    self._action_name = "/ZmpWalk"
    self._as = actionlib.SimpleActionServer(self._action_name, C42_DynamicLocomotion.msg.C42_ZmpWlkAction, execute_cb=self.task)
    self._as.start()
    #getting position and orientation
    self._pos_sub = rospy.Subscriber("/ground_truth_odom",Odometry,self.get_pos)
    self._pos = gm.Point()
    self._or = gm.Quaternion()
    #tolerance for deviation from goal
    self._tol = 0.1
    self._nxt_wp = C31_Location()
    #ensure we enter the main loop at least once

  def get_pos(self,odom):
    #get robot position, currently from /ground_truth_odom, later from navigation node
    self._pos = odom.pose.pose.position
    self._or = odom.pose.pose.orientation

  def _get_path(self,path_planner = 'getPath'):
    pth = C31_Location()
    #client for path planning service from C31
    rospy.loginfo("waiting for path planing service")
    rospy.wait_for_service(path_planner)
    try:
      pth_pln = rospy.ServiceProxy(path_planner, C31_GetPath)
      pth = pth_pln()
      return pth
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e
    pass

  def _plan_pref(self,path):
    #plan ref ZMP based on given path
    pass

  def _zmp_control(self,p_ref):
    #control legs based on ref ZMP
    pass

  #def task(self, goal):
  def task(self, goal):
    #init_zmp.init_pose_with_trajectory_controllers()
    rospy.loginfo("started ZMPwalk")
    task_success = True
    rospy.sleep(2)
    Vel = gm.Twist()
    pth = self._get_path()
    # start executing the action
    #### LOG TASK PARAMETERS ####
    for point in pth.path.points:
      self._nxt_wp.x = point.x#2
      self._nxt_wp.y = point.y#0
      rospy.loginfo("next way point: x:%s y:%s", point.x, point.y)
      pos = np.array([self._pos.x,self._pos.y])
      gl = np.array([point.x, point.y])
      self._dis_from_goal = np.linalg.norm(gl-pos)
      V = (gl-pos)/np.linalg.norm(gl-pos)
      Vel.linear.x = V[0]
      Vel.linear.y = V[1]
      self.walk_pub.publish(Vel)
      print Vel

      while self._dis_from_goal > self._tol:
        #calculate distance from goal
        pos = np.array([self._pos.x,self._pos.y])
        gl = np.array([point.x, point.y])
        self._dis_from_goal = np.linalg.norm(gl-pos)
        rospy.sleep(0.1)
        if self._as.is_preempt_requested() or rospy.is_shutdown():
          #### HERE PROICESS PREEMTION OR INTERUPT #####
          rospy.loginfo('%s: Preempted' % self._action_name)
          self._as.set_preempted()
          task_success = False
          break
    Vel = gm.Twist()
    self.walk_pub.publish(Vel)
    if task_success:
      self._result.res_pos.x = self._pos.x
      self._result.res_pos.y = self._pos.y
      self._result.dis = self._dis_from_goal      
      rospy.loginfo('%s: Succeeded' % self._action_name)
      self._as.set_succeeded(self._result)


if __name__ == '__main__':
  rospy.init_node('ZMP_server')
  ZmpWlkServer()
  rospy.spin()
