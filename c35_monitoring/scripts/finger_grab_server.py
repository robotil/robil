#! /usr/bin/env python

import roslib; roslib.load_manifest('c35_monitoring')
import rospy

import c35_monitoring.msg

import actionlib

from std_msgs.msg import Float64

class FingerGrabAction(object):
  # create messages that are used to publish feedback/result
  _feedback = c35_monitoring.msg.finger_grabFeedback()
  _result   = c35_monitoring.msg.finger_grabResult()

  def __init__(self, name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, c35_monitoring.msg.finger_grabAction, execute_cb=self.execute_cb)
    self._as.start()
    
  def execute_cb(self, goal):
    # helper variables
    r = rospy.Rate(1)
    success = True
    
    # append the seeds for the fibonacci sequence
    #self._feedback.sequence = []
    #self._feedback.sequence.append(0)
    #self._feedback.sequence.append(1)
    
    self._feedback.percent_done = 0
    
    # publish info to the console for the user
    rospy.loginfo('%s: Executing, making fingers of hand %i grab something. Percent complete %i' % (self._action_name, goal.which_hand, self._feedback.percent_done))
    
    r_f0_j0 = rospy.Publisher('/r_f0_j0_position_controller/command', Float64)
    r_f0_j1 = rospy.Publisher('/r_f0_j1_position_controller/command', Float64)
    r_f0_j2 = rospy.Publisher('/r_f0_j2_position_controller/command', Float64)
    r_f1_j0 = rospy.Publisher('/r_f1_j0_position_controller/command', Float64)
    r_f1_j1 = rospy.Publisher('/r_f1_j1_position_controller/command', Float64)
    r_f1_j2 = rospy.Publisher('/r_f1_j2_position_controller/command', Float64)
    r_f2_j0 = rospy.Publisher('/r_f2_j0_position_controller/command', Float64)
    r_f2_j1 = rospy.Publisher('/r_f2_j1_position_controller/command', Float64)
    r_f2_j2 = rospy.Publisher('/r_f2_j2_position_controller/command', Float64)
    r_f3_j0 = rospy.Publisher('/r_f3_j0_position_controller/command', Float64)
    r_f3_j1 = rospy.Publisher('/r_f3_j1_position_controller/command', Float64)
    r_f3_j2 = rospy.Publisher('/r_f3_j2_position_controller/command', Float64)
    
    # Initialize the node
    #rospy.init_node('joint_control')

    #Initial pose
    r_f0_j0.publish(-0.25)
    r_f0_j1.publish(0)
    r_f0_j2.publish(0)
    r_f1_j0.publish(-0.24)
    r_f1_j1.publish(0)
    r_f1_j2.publish(0)
    r_f2_j0.publish(-0.22)
    r_f2_j1.publish(0)
    r_f2_j2.publish(0)
    r_f3_j0.publish(0.64)
    r_f3_j1.publish(0)
    r_f3_j2.publish(0)

    j1_pos = 0.0
    j2_pos = 0.0
    
    
    # start executing the action
    for i in xrange(1, 3500):
    # check that preempt has not been requested by the client
      if self._as.is_preempt_requested():
	rospy.loginfo('%s: Preempted' % self._action_name)
	self._as.set_preempted()
	success = False
	break
      if i < 200:
	j1_pos = j1_pos + 0.001
      else:
	j2_pos = j2_pos + 0.001
      r_f0_j1.publish(j1_pos)
      r_f0_j2.publish(j2_pos)
      #r_f1_j0.publish(-0.24)
      r_f1_j1.publish(j1_pos)
      r_f1_j2.publish(j2_pos)
      #r_f2_j0.publish(-0.22)
      r_f2_j1.publish(j1_pos)
      r_f2_j2.publish(j2_pos)
      #r_f3_j0.publish(0.64)
      r_f3_j1.publish(j1_pos)
      r_f3_j2.publish(j2_pos)

      self._feedback.percent_done = (3500 / i) * 100
      # publish the feedback
      self._as.publish_feedback(self._feedback)
	
      if success:
	self._result.final_location = j1_pos
	rospy.loginfo('%s: Succeeded' % self._action_name)
	self._as.set_succeeded(self._result)
      
if __name__ == '__main__':
  rospy.init_node('finger_grab')
  FingerGrabAction(rospy.get_name())
  rospy.spin() 
