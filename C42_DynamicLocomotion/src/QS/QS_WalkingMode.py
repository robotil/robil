#!/usr/bin/env python

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

import time
import math
import rospy
import sys
import copy
import PyKDL
import roslib
roslib.load_manifest('C42_DynamicLocomotion')

from Abstractions.WalkingMode import *
from Abstractions.Odometer import *
from QS_PathPlanner import *

import tf
from tf_conversions import posemath
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from atlas_msgs.msg import AtlasCommand, AtlasSimInterfaceCommand, AtlasSimInterfaceState, AtlasState, AtlasBehaviorStepData
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from std_msgs.msg import String

from C42_DynamicLocomotion.srv import *
from C42_DynamicLocomotion.msg import Foot_Placement_data

class QS_WalkingMode(WalkingMode):
    def __init__(self):
        self._LPP = QS_PathPlanner()
        WalkingMode.__init__(self,self._LPP)
        self._listener = tf.TransformListener()
        self._tf_br = tf.TransformBroadcaster()
        # Initialize atlas atlas_sim_interface_command publisher       
        self.asi_command = rospy.Publisher('/atlas/atlas_sim_interface_command', AtlasSimInterfaceCommand, None, False, True, None)
        self._Odometer = Odometer()
        self._bDone = False
        self._bIsSwaying = False
        self._command = 0

        
        self._odom_sub = rospy.Subscriber('/ground_truth_odom',Odometry,self._odom_cb)

    def Initialize(self):
        WalkingMode.Initialize(self)
        self._command = 0
        # Subscribers:
        self.asi_state = rospy.Subscriber('/atlas/atlas_sim_interface_state', AtlasSimInterfaceState, self.asi_state_cb)
        self._atlas_imu_sub = rospy.Subscriber('/atlas/imu', Imu, self._get_imu)

        rospy.wait_for_service('foot_placement_path')
        self._foot_placement_client = rospy.ServiceProxy('foot_placement_path', FootPlacement_Service)       
        self._RequestFootPlacements()
        rospy.sleep(0.3)
    
        k_effort = [0] * 28
        self._bDone = False
        self._bIsSwaying = False
    
    def StartWalking(self):
        self._bDone = False
    
    def Walk(self):
        WalkingMode.Walk(self)
        self._command = self.GetCommand(self._BDI_state)
        self.asi_command.publish(self._command)
        #print(command)
        self._WalkingModeStateMachine.PerformTransition("Go")
        self._bIsSwaying = True
    
    def EmergencyStop(self):
        WalkingMode.Stop(self)

    def IsDone(self):
        return self._bDone
    
    def GetCommand(self,state):
        command = AtlasSimInterfaceCommand()
        command.behavior = AtlasSimInterfaceCommand.STEP
        command.k_effort = [0] * 28
        command.step_params.desired_step = self._LPP.GetNextStep()
        if(0 != command.step_params.desired_step):
            # Not sure why such a magic number
            command.step_params.desired_step.duration = 0.63
            # Apparently this next line is a must
            command.step_params.desired_step.step_index = 1
            command.step_params.desired_step = self._TransforFromGlobalToBDI(command.step_params.desired_step,state)
        else:
            command = 0
        return command
    
    def HandleStateMsg(self,state):
        command = 0
        if ("Idle" == self._WalkingModeStateMachine.GetCurrentState().Name):
            self._Odometer.SetPosition(state.pos_est.position.x,state.pos_est.position.y)
        elif ("Wait" == self._WalkingModeStateMachine.GetCurrentState().Name):
            self._Odometer.SetPosition(state.pos_est.position.x,state.pos_est.position.y)
            print("Odometer Updated")
            #print(2)
            self._WalkingModeStateMachine.PerformTransition("Go")
        elif ("Walking" == self._WalkingModeStateMachine.GetCurrentState().Name):
            #print(3)
            if (QS_PathPlannerEnum.Active == self._LPP.State):
                command = self.GetCommand(state)
            elif(QS_PathPlannerEnum.Waiting == self._LPP.State):
                self._RequestFootPlacements()
        elif ("Done" == self._WalkingModeStateMachine.GetCurrentState().Name):
            #print(4)
            self._bDone = True
        else:
            raise Exception("QS_WalkingModeStateMachine::Bad State Name")
    
        return command
    
    def _RequestFootPlacements(self):
        print("Request Foot Placements")
        # Perform a service request from FP
        try:
            # Handle preemption?
                # if received a "End of mission" sort of message from FP
            start_pose,other_foot_pose = self._GetStartingFootPose()
            resp = self._foot_placement_client(0,start_pose,other_foot_pose)
            if [] == resp.foot_placement_path: # 1 == resp.done:
                self._WalkingModeStateMachine.PerformTransition("Finished") # if array is empty need to finish with error (didn't reach goal)
            else:
                listSteps = []
                for desired in resp.foot_placement_path:
                    command = AtlasSimInterfaceCommand()
                    step = command.step_params.desired_step
                    step.foot_index = desired.foot_index
                    step.swing_height = desired.clearance_height
                    step.pose.position.x = desired.pose.position.x
                    step.pose.position.y = desired.pose.position.y
                    step.pose.position.z = desired.pose.position.z
                    Q = quaternion_from_euler(desired.pose.ang_euler.x, desired.pose.ang_euler.y, desired.pose.ang_euler.z)
                    step.pose.orientation.x = Q[0]
                    step.pose.orientation.y = Q[1]
                    step.pose.orientation.z = Q[2]
                    step.pose.orientation.w = Q[3]
                    listSteps.append(step)
                self._LPP.SetPath(listSteps)
                #print(listSteps)
        except rospy.ServiceException, e:
            print "Foot Placement Service call failed: %s"%e
    
    def _GetStartingFootPose(self): #*************** NEED TO CHANGE when not static *************#
        start_pose = Foot_Placement_data()
        other_foot_pose = Foot_Placement_data()
        
        trans, rot = self._GetTf('World','l_foot')
        start_pose.foot_index = 2 # both feet are static
        start_pose.pose.position.x = trans[0]
        start_pose.pose.position.y = trans[1]
        start_pose.pose.position.z = trans[2]
        start_pose.pose.ang_euler.x = rot[0]
        start_pose.pose.ang_euler.y = rot[1]
        start_pose.pose.ang_euler.z = rot[2]

        trans, rot = self._GetTf('World','r_foot')
        other_foot_pose.foot_index = 1 # right foot
        other_foot_pose.pose.position.x = trans[0]
        other_foot_pose.pose.position.y = trans[1]
        other_foot_pose.pose.position.z = trans[2]
        other_foot_pose.pose.ang_euler.x = rot[0]
        other_foot_pose.pose.ang_euler.y = rot[1]
        other_foot_pose.pose.ang_euler.z = rot[2]


        return start_pose,other_foot_pose
    
###################################################################################
#--------------------------- CallBacks --------------------------------------------
###################################################################################

    # /atlas/atlas_sim_interface_state callback. Before publishing a walk command, we need
    # the current robot position   
    def asi_state_cb(self, state):
        #self._Update_tf_BDI_odom(state)
        self._BDI_state = copy.copy(state)
        command = 0
        #print(state.step_feedback.status_flags)
        # When the robot status_flags are 1 (SWAYING), you can publish the next step command.
        if (state.step_feedback.status_flags == 1 and not self._bIsSwaying):
            command = self.HandleStateMsg(state)
        elif (state.step_feedback.status_flags == 2 and self._bIsSwaying):
            self._bIsSwaying = False
            #print("step done")
        if (0 !=command):
            self._command = command
            self._bIsSwaying = True
        
        if(0 == state.current_behavior and 0 != self._command):
            #print self._command
            self.asi_command.publish(self._command)
            self._command = 0
            print("step start")

    def _odom_cb(self,odom):
        self._LPP.UpdatePosition(odom.pose.pose.position.x,odom.pose.pose.position.y)
        # sendTransform(translation - tuple (x, y, z), rotation - tuple (x, y, z, w), time, child, parent)
        self._tf_br.sendTransform(vec2tuple(odom.pose.pose.position), vec2tuple(odom.pose.pose.orientation), odom.header.stamp, "pelvis", "World") # "World", "pelvis") 
 
    def _get_imu(self,msg):  #listen to /atlas/imu/pose/pose/orientation
        roll, pitch, yaw = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self._Odometer.SetYaw(yaw)


###############################################################################################

    # def _Update_tf_BDI_odom(self,state):
    #     self._tf_br.sendTransform( vec2tuple(state.pos_est.position), (0, 0, 0, 1), state.header.stamp, "BDI_pelvis", "World")
    #     self._tf_br.sendTransform( vec2tuple(state.foot_pos_est[0].position), vec2tuple(state.foot_pos_est[0].orientation),\
    #                  state.header.stamp, "BDI_l_foot", "World")
    #     self._tf_br.sendTransform( vec2tuple(state.foot_pos_est[1].position), vec2tuple(state.foot_pos_est[1].orientation),\
    #                  state.header.stamp, "BDI_r_foot", "World")
    #     # check foot index:
    #     if AtlasSimInterfaceCommand.STEP == state.current_behavior:
    #         static_foot_index = state.step_feedback.desired_step_saturated.foot_index
    #     elif AtlasSimInterfaceCommand.WALK == state.current_behavior:
    #         static_foot_index = state.step_feedback.desired_step_saturated.foot_index
    #     else:
    #         static_foot_index = 2
    #     # determine static foot: ??? if needed
    #     if 0 == static_foot_index: # Left foot is static
    #         self._tf_br.sendTransform( vec2tuple(state.foot_pos_est[0].position), vec2tuple(state.foot_pos_est[0].orientation),\
    #                  state.header.stamp, "BDI_static_foot", "World")
    #     elif 1 == static_foot_index: # Right foot is static
    #         self._tf_br.sendTransform( vec2tuple(state.foot_pos_est[1].position) , vec2tuple(state.foot_pos_est[1].orientation),\
    #                  state.header.stamp, "BDI_static_foot", "World")
    #     elif 2 == static_foot_index: # both feet are static
    #         self._tf_br.sendTransform( (0, 0, 0), (0, 0, 0, 1), state.header.stamp, "BDI_static_foot", "World")
    #     else: # problem
    #         self._tf_br.sendTransform( (0, 0, -10), (0, 0, 0, 1), state.header.stamp, "BDI_static_foot", "World")

    def _TransforFromGlobalToBDI(self,step_data,state):
        static_foot_index = (step_data.foot_index+1) % 2 # the foot that we arn't placing        
        if 0 == static_foot_index:
            static_foot_global_position , static_foot_global_rotation = self._GetTf('World','l_foot')
        else:
            static_foot_global_position , static_foot_global_rotation = self._GetTf('World','r_foot')
        # new_BDI_foot_pose = old_BDI_foot_pose + Global_pose_delta_between_feet 
        step_data.pose.position.x = state.foot_pos_est[static_foot_index].position.x + step_data.pose.position.x - static_foot_global_position[0]
        step_data.pose.position.y = state.foot_pos_est[static_foot_index].position.y + step_data.pose.position.y - static_foot_global_position[1]
        step_data.pose.position.z = state.foot_pos_est[static_foot_index].position.z + step_data.pose.position.z - static_foot_global_position[2]
        #TODO: add in rotation correction using euler angles
        rospy.loginfo("_TransforFromGlobalToBDI:: command position: x=%f, y=%f , z=%f" \
             % (step_data.pose.position.x, step_data.pose.position.y, step_data.pose.position.z) )
        return step_data

    def _GetTf(self,base_frame,get_frames):
        # waiting for transform to be avilable
        time_out = rospy.Duration(2)
        polling_sleep_duration = rospy.Duration(0.01)
        while self._listener.waitForTransform (base_frame, get_frames, rospy.Time(0), time_out, polling_sleep_duration) and not rospy.is_shutdown():
                rospy.loginfo("QS_WalkingMode - _GetTf:: Not ready for Global To BDI transform")
        try:
          (translation,rotation) = self._listener.lookupTransform(base_frame, get_frames, rospy.Time(0))  #  rospy.Time(0) to use latest availble transform 
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
          print ex
          rospy.loginfo("QS_WalkingMode - _GetTf:: tf exception")
          translation = [0,0,0]
          rotation = [0,0,0]
          #continue
        return translation,rotation


def vec2tuple(vector):
    res = ( vector.x, vector.y, vector.z )
    try:
        res = res + (vector.w,)
    except AttributeError:
        pass # print 'oops'
    return res