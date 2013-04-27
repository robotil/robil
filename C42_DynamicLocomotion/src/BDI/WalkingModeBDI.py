#!/usr/bin/env python

from WalkingMode import *
import roslib
roslib.load_manifest('C42_DynamicLocomotion')
import rospy
import time
from atlas_msgs.msg import AtlasCommand, AtlasSimInterfaceCommand, AtlasSimInterfaceState, AtlasState, AtlasBehaviorStepData
from sensor_msgs.msg import Imu
import PyKDL
from tf_conversions import posemath

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

class WalkingModeBDI(WalkingMode):

    def __init__(self):
      pass
        
    def Initialize(self):
        pass
    
    def StartWalking(self):
        pass
    
    def Walk(self):
        # Step 4: Request BDI walk
        walk_msg = AtlasSimInterfaceCommand()
        # Always insert current time
        walk_msg.header.stamp = rospy.Time.now()
        # Tell it to walk
        walk_msg.behavior = walk_msg.WALK
        walk_msg.walk_params.use_demo_walk = False
        # Fill in some steps
        for i in range(4):
            step_data = AtlasBehaviorStepData()
            # Steps are indexed starting at 1
            step_data.step_index = i+1
            # 0 = left, 1 = right
            step_data.foot_index = i%2
            # 0.3 is a good number
            step_data.swing_height = 0.3
            # 0.63 is a good number
            step_data.duration = 0.63
            # We'll specify desired foot poses in ego-centric frame then
            # transform them into the robot's world frame.
            # Match feet so that we end with them together
            step_data.pose.position.x = (1+i/2)*0.25
            # Step 0.15m to either side of center, alternating with feet
            step_data.pose.position.y = 0.15 if (i%2==0) else -0.15
            step_data.pose.position.z = 0.0
            # Point those feet straight ahead
            step_data.pose.orientation.x = 0.0
            step_data.pose.orientation.y = 0.0
            step_data.pose.orientation.z = 0.0
            step_data.pose.orientation.w = 1.0
            # Transform this foot pose according to robot's
            # current estimated pose in the world, which is a combination of IMU
            # and internal position estimation.
            # http://www.ros.org/wiki/kdl/Tutorials/Frame%20transformations%20%28Python%29
            f1 = posemath.fromMsg(step_data.pose)
            f2 = PyKDL.Frame(PyKDL.Rotation.Quaternion(self.imu_msg.orientation.x,
                                                       self.imu_msg.orientation.y,
                                                       self.imu_msg.orientation.z,
                                                       self.imu_msg.orientation.w),
                             PyKDL.Vector(self.asis_msg.pos_est.position.x,
                                          self.asis_msg.pos_est.position.y,
                                          self.asis_msg.pos_est.position.z))
            f = f2 * f1
            step_data.pose = posemath.toMsg(f)
            walk_msg.walk_params.step_data[i] = step_data
        # Use the same k_effort from the last step, to retain user control over some
        # joints. BDI has control of the other joints.
        walk_msg.k_effort = slight_movement_msg.k_effort
        # Publish and give time to take effect
        print('[USER/BDI] Walking...')
        self.asic_pub.publish(walk_msg)
        time.sleep(6.0)
        
        return true
    
    def Stop(self):
        pass
    
    def EmergencyStop(self):
        pass    
    
 
