#! /usr/bin/env python

import roslib; roslib.load_manifest('c35_monitoring')
import rospy

import c35_monitoring.msg

import actionlib

from std_msgs.msg import Float64




class sittingAction(object):
  # create messages that are used to publish feedback/result
  _feedback = c35_monitoring.msg.sittingFeedback()
  _result   = c35_monitoring.msg.sittingResult()

  def __init__(self, name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, c35_monitoring.msg.sittingAction, execute_cb=self.execute_cb)
    self._as.start()
    
  #def reachTheGoal(self,goal):
   # if self._feedback.percent_done == 100 :
    #  return True
   # return False
    
  def execute_cb(self, goal):
    # helper variables
    r = rospy.Rate(1)
    success = True
    
    l_leg_kny = rospy.Publisher('/l_leg_kny_position_controller/command', Float64)
    #left paw - left and right
    l_leg_lax = rospy.Publisher('/l_leg_lax_position_controller/command', Float64)
    #move the leg forward and backward
    l_leg_lhy = rospy.Publisher('/l_leg_lhy_position_controller/command', Float64)
    #move the leg left and right
    l_leg_mhx = rospy.Publisher('/l_leg_mhx_position_controller/command', Float64)
    #move leg paw forward and backward
    l_leg_uay = rospy.Publisher('/l_leg_uay_position_controller/command', Float64)
    #turn leg - open the crotch
    l_leg_uhz = rospy.Publisher('/l_leg_uhzu_position_controller/command', Float64)  

    
    r_leg_kny = rospy.Publisher('/r_leg_kny_position_controller/command', Float64)
    #left paw - left and right
    r_leg_lax = rospy.Publisher('/r_leg_lax_position_controller/command', Float64)
    #move the leg forward and backward
    r_leg_lhy = rospy.Publisher('/r_leg_lhy_position_controller/command', Float64)
    #move the leg left and right
    r_leg_mhx = rospy.Publisher('/r_leg_mhx_position_controller/command', Float64)
    #move leg paw forward and backward
    r_leg_uay = rospy.Publisher('/r_leg_uay_position_controller/command', Float64)
    #turn leg - open the crotch
    r_leg_uhz = rospy.Publisher('/r_leg_uhzu_position_controller/command', Float64) 
    

    # check that preempt has not been requested by the client
    if self._as.is_preempt_requested():
      rospy.loginfo('%s: Preempted' % self._action_name)
      self._as.set_preempted()
      success = False
      
    

    l_leg_kny.publish(1.5)
    #left paw - left and right
    l_leg_lax.publish(0)
    #move the leg forward and backward
    l_leg_lhy.publish(-1.5)
    #move the leg left and right
    l_leg_mhx.publish(0)
    #move leg paw forward and backward
    l_leg_uay.publish(0)
    #turn leg - open the crotch
    l_leg_uhz.publish(0) 

    
    r_leg_kny.publish(1.5)
    #left paw - left and right
    r_leg_lax.publish(0)
    #move the leg forward and backward
    r_leg_lhy.publish(-1.5)
    #move the leg left and right
    r_leg_mhx.publish(0)
    #move leg paw forward and backward
    r_leg_uay.publish(0)
    #turn leg - open the crotch
    r_leg_uhz.publish(0) 
     
    # publish the feedback
    self._feedback.percent_done=100
    self._as.publish_feedback(self._feedback)    # append the seeds for the legFeedback sequence
    #self._feedback.sequence = []
    #self._feedback.sequence.append(l_leg_kny)
      # check that preempt has not been requested by the client
    #self._feedback.sequence.append(l_leg_lax)
    #self._feedback.sequence.append(l_leg_lhy)
    #self._feedback.sequence.append(l_leg_mhx)
    #self._feedback.sequence.append(l_leg_uay)
    #self._feedback.sequence.append(l_leg_uhz)

      
    if success:
      self._result.location = self._feedback.percent_done
      rospy.loginfo('%s: Succeeded' % self._action_name)
      self._as.set_succeeded(self._result)
      
    return
      
if __name__ == '__main__':
  rospy.init_node('sitting')
  sittingAction(rospy.get_name())
  rospy.spin()