 
#! /usr/bin/env python

import roslib; roslib.load_manifest('arm_control')
import rospy, math

import actionlib

import c35_monitoring.msg
from std_msgs.msg import Float64

class ArmControlAction(object):
  # create messages that are used to publish feedback/result
  _feedback = c35_monitoring.msg.arm_controlFeedback()
  _result   = c35_monitoring.msg.arm_controlResult()

  def __init__(self, name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, c35_monitoring.msg.arm_controlAction, execute_cb=self.execute_cb)
    self._as.start()
    
  def moveToLoc(self, rng, topic, newLoc, percent):
    
    for i in range(int(math.fabs(rng*10))):
      newLoc = newLoc + math.copysign(0.1, rng)
      topic.publish(newLoc)
      ## check that preempt has not been requested by the client
      if self._as.is_preempt_requested():
        rospy.loginfo('%s: Preempted' % self._action_name)
        self._as.set_preempted()
        success = False
        break
        
       
      self._feedback.progress = self._feedback.progress + percent
      ## publish the feedback
      self._as.publish_feedback(self._feedback)
      rospy.sleep(0.001)
      rospy.loginfo('%f percent done' %(self._feedback.progress)) 
    
      
  def execute_cb(self, goal):
    # helper variables
    r = rospy.Rate(1)
    success = True
    
    right = True
    left = True
    
    # initial value for the feedback parameter
    self._feedback.progress = 0
    
    # publish info to the console for the user
    rospy.loginfo('%s: Executing, creating a movment  %i in this progress %i' % (self._action_name, goal.hand, self._feedback.progress))
    
    # right hand
    #albow x
    r_arm_elx = rospy.Publisher('/r_arm_elx_position_controller/command', Float64)
    #albow y
    r_arm_ely = rospy.Publisher('/r_arm_ely_position_controller/command', Float64)
    #wrist- SHORES CAF HAYAD
    r_arm_mwx = rospy.Publisher('/r_arm_mwx_position_controller/command', Float64)
    #shoulder
    r_arm_shx = rospy.Publisher('/r_arm_shx_position_controller/command', Float64)
    #shoulder - y (like moment around z axis)
    r_arm_usy = rospy.Publisher('/r_arm_usy_position_controller/command', Float64)
    #lower limb, between the elbow and the wrist - Rotation
    r_arm_uwy = rospy.Publisher('/r_arm_uwy_position_controller/command', Float64)
    
    #left hand
    l_arm_elx = rospy.Publisher('/l_arm_elx_position_controller/command', Float64)
    l_arm_ely = rospy.Publisher('/l_arm_ely_position_controller/command', Float64)
    l_arm_mwx = rospy.Publisher('/l_arm_mwx_position_controller/command', Float64)
    l_arm_shx = rospy.Publisher('/l_arm_shx_position_controller/command', Float64)
    l_arm_usy = rospy.Publisher('/l_arm_usy_position_controller/command', Float64)
    l_arm_uwy = rospy.Publisher('/l_arm_uwy_position_controller/command', Float64)

    # Initialize the node
    rospy.init_node('c35_monitoring')
    
    mov_arm_x= 0
    mov_arm_y= 0
    mov_arm_mwx= 0
    mov_arm_shx= 0
    mov_arm_usy= 0
    mov_arm_uwy= 0
    
    r_arm_elx.publish(mov_arm_x)
    r_arm_ely.publish(mov_arm_y)
    r_arm_mwx.publish(mov_arm_mwx)
    r_arm_shx.publish(mov_arm_shx)
    r_arm_usy.publish(mov_arm_usy)
    r_arm_uwy.publish(mov_arm_uwy)
    
    l_arm_elx.publish(mov_arm_x)
    l_arm_ely.publish(mov_arm_y)
    l_arm_mwx.publish(mov_arm_mwx)
    l_arm_shx.publish(mov_arm_shx)
    l_arm_usy.publish(mov_arm_usy)
    l_arm_uwy.publish(mov_arm_uwy)
    
    
    # start executing the action
    
    #right arm
    if goal.hand ==1:
      left = False
    if goal.hand ==2:
      right = False
      
    if right:
    #moveToLoc(self, rng, interval, topic, newLoc):
      arm_x= 5.600000
      arm_y= -6.100000
      arm_mwx= -0.100000
      arm_shx= -0.500000
      arm_usy= 1.300000
      arm_uwy= 1.500000
      totalTime = 56+61+1+5+13+15
      percent = (1.0/totalTime) * 100.0
      print "%f percent" %percent
      self.moveToLoc(arm_x, r_arm_elx, mov_arm_x, percent)
      self.moveToLoc(arm_y,  r_arm_ely, mov_arm_y, percent)
      self.moveToLoc(arm_mwx, r_arm_mwx, mov_arm_mwx, percent)
      self.moveToLoc(arm_shx, r_arm_shx, mov_arm_shx, percent)
      self.moveToLoc(arm_usy, r_arm_usy, mov_arm_usy, percent)
      self.moveToLoc(arm_uwy, r_arm_uwy, mov_arm_uwy, percent)
      
    #left arm
    if left:
      arm_x= 0.600000
      arm_y= 0.30000
      arm_mwx= 0.200000
      arm_shx= 0.600000
      arm_usy= -1.200000
      arm_uwy= 0.200000
      totalTime = 6+3+2+6+12+2
      percent = (1.0/totalTime) * 100.0
      self.moveToLoc(arm_x, l_arm_elx, mov_arm_x, percent)
      self.moveToLoc(arm_y, l_arm_ely, mov_arm_y, percent)
      self.moveToLoc(arm_mwx, l_arm_mwx, mov_arm_mwx, percent)
      self.moveToLoc(arm_shx, l_arm_shx, mov_arm_shx, percent)
      self.moveToLoc(arm_usy, l_arm_usy, mov_arm_usy, percent)
      self.moveToLoc(arm_uwy, l_arm_uwy, mov_arm_uwy, percent)
      
      
    if success:
      self._result.finalLocation = arm_x
      rospy.loginfo('%s: Succeeded' % self._action_name)
      self._as.set_succeeded(self._result)
      
if __name__ == '__main__':
  rospy.init_node('c35_monitoring')
  ArmControlAction(rospy.get_name())
  rospy.spin()
