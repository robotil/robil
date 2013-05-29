#!/usr/bin/env python

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

import copy
import rospy
import roslib
roslib.load_manifest('C42_DynamicLocomotion')

from Abstractions.WalkingMode import *
from Abstractions.Odometer import *
from DD_PathPlanner import *

import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np

from atlas_msgs.msg import AtlasCommand, AtlasSimInterfaceCommand, AtlasSimInterfaceState, AtlasState, AtlasBehaviorStepData
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from std_msgs.msg import String, Int32

from C42_DynamicLocomotion.srv import *
from C42_DynamicLocomotion.msg import Foot_Placement_data

class DD_WalkingMode(WalkingMode):
    def __init__(self):
        self._LPP = DD_PathPlanner()
        WalkingMode.__init__(self,self._LPP)
        self.step_index_for_reset = 0
        # Initialize atlas atlas_sim_interface_command publisher       
        self.asi_command = rospy.Publisher('/atlas/atlas_sim_interface_command', AtlasSimInterfaceCommand, None, False, True, None)
        # for debug publish:
        self._debug_pub_StepIndex = rospy.Publisher('debug_DD_StepIndex',Int32)
        self._Odometer = Odometer()
        self._bDone = False
        self._StepIndex = 1
        self._command = 0
        
    def Initialize(self):
        WalkingMode.Initialize(self)
        self._bRobotIsStatic = True
        # Subscribers:
        self._listener = tf.TransformListener()
        self._tf_br = tf.TransformBroadcaster()

        self._odom_sub = rospy.Subscriber('/ground_truth_odom',Odometry,self._odom_cb)
        self.asi_state = rospy.Subscriber('/atlas/atlas_sim_interface_state', AtlasSimInterfaceState, self.asi_state_cb)
        self._atlas_imu_sub = rospy.Subscriber('/atlas/imu', Imu, self._get_imu)

        rospy.wait_for_service('foot_placement_path')
        self._foot_placement_client = rospy.ServiceProxy('foot_placement_path', FootPlacement_Service)       
        
        rospy.sleep(0.3)

        self._RequestFootPlacements()
        k_effort = [0] * 28
        self._bDone = False
        self._bIsSwaying = False
        #self._bRobotIsStatic = False
        self._GetOrientationDelta0Values() # Orientation difference between BDI odom and Global
    
    def StartWalking(self):
        self._bDone = False
    
    def Walk(self):
        WalkingMode.Walk(self)
        command = self.GetCommand()         
        #print ("Walk ",command)
        self.asi_command.publish(command)
        #print(command)
        self._WalkingModeStateMachine.PerformTransition("Go")
        self._StepIndex = self._StepIndex + 1
    
    def EmergencyStop(self):
        WalkingMode.Stop(self)

    def IsDone(self):
        return self._bDone
    
    def GetCommand(self):
        command = AtlasSimInterfaceCommand()
        command.behavior = AtlasSimInterfaceCommand.WALK
        command.k_effort = [0] * 28
        command.walk_params.step_queue = self._LPP.GetNextStep()
        if(0 == command.walk_params.step_queue):
            command = 0
        else:
            for i in range(4):
                command.walk_params.step_queue[i].step_index = self._StepIndex + i
                command.walk_params.step_queue[i].duration = 0.63
                #print("GetCommand",command.walk_params.step_queue)
                command.walk_params.step_queue[i] = self._TransforFromGlobalToBDI(command.walk_params.step_queue[i],i)
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
            if (DD_PathPlannerEnum.Active == self._LPP.State):
                command = self.GetCommand()
            elif(DD_PathPlannerEnum.Waiting == self._LPP.State):
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
                self._WalkingModeStateMachine.PerformTransition("Finished")
            else:
                listSteps = []
                for desired in resp.foot_placement_path:
                    #print("Desired: ",desired)
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
                #print("listSteps: ",listSteps)
        except rospy.ServiceException, e:
            print "Foot Placement Service call failed: %s"%e

    def _GetStartingFootPose(self): #*************** NEED TO CHANGE when not static *************#
        start_pose = Foot_Placement_data()
        other_foot_pose = Foot_Placement_data()
        
        t = self._listener.getLatestCommonTime('World','l_foot')
        trans, rot_q = self._GetTf('World','l_foot',t)
        start_pose.foot_index = 2 # both feet are static
        start_pose.pose.position.x = trans[0]
        start_pose.pose.position.y = trans[1]
        start_pose.pose.position.z = trans[2]
        rot_euler = euler_from_quaternion(rot_q)
        start_pose.pose.ang_euler.x = rot_euler[0]
        start_pose.pose.ang_euler.y = rot_euler[1]
        start_pose.pose.ang_euler.z = rot_euler[2]


        trans, rot_q = self._GetTf('World','r_foot',t)
        other_foot_pose.foot_index = 1 # right foot
        other_foot_pose.pose.position.x = trans[0]
        other_foot_pose.pose.position.y = trans[1]
        other_foot_pose.pose.position.z = trans[2]
        rot_euler = euler_from_quaternion(rot_q)
        other_foot_pose.pose.ang_euler.x = rot_euler[0]
        other_foot_pose.pose.ang_euler.y = rot_euler[1]
        other_foot_pose.pose.ang_euler.z = rot_euler[2]

        return start_pose,other_foot_pose

    def Fitness(self,path):
        stepList = self._LPP.GetPath()
        result = True
        AllowedDeltaZ = 0.1
        StartingZ = stepList[0].pose.position.z
        if(4 < len(stepList)):
            endRange = 5
        else:
            endRange = len(stepList)
        for i in range(1,endRange):
            if (AllowedDeltaZ < math.fabs(StartingZ-stepList[i].pose.position.z)):
                result = False
        return result
    
###################################################################################
#--------------------------- CallBacks --------------------------------------------
###################################################################################

    # /atlas/atlas_sim_interface_state callback. Before publishing a walk command, we need
    # the current robot position   
    def asi_state_cb(self, state):
        if self._bRobotIsStatic:
            self._BDI_Static_orientation_q = state.foot_pos_est[0].orientation
        self._Update_tf_BDI_odom(state)
        command = 0
        if(self._StepIndex < state.walk_feedback.next_step_index_needed):
            command = self.HandleStateMsg(state)
        if (0 != command):
            #print("Step",self._StepIndex,command)
            self._bRobotIsStatic = False
            self.asi_command.publish(command)
            self._StepIndex = self._StepIndex+1

        self._debug_pub_StepIndex.publish(self._StepIndex)

    def _odom_cb(self,odom):
        if self._bRobotIsStatic:
            self._Global_Static_orientation_q = odom.pose.pose.orientation
        self._LPP.UpdatePosition(odom.pose.pose.position.x,odom.pose.pose.position.y)
        self._tf_br.sendTransform(vec2tuple(odom.pose.pose.position), vec2tuple(odom.pose.pose.orientation), odom.header.stamp, "pelvis", "World")
 
    def _get_imu(self,msg):  #listen to /atlas/imu/pose/pose/orientation
        roll, pitch, yaw = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self._Odometer.SetYaw(yaw)


###############################################################################################

    def _Update_tf_BDI_odom(self,state):
        self._tf_br.sendTransform( vec2tuple(state.pos_est.position), vec2tuple(state.foot_pos_est[0].orientation), state.header.stamp, "BDI_pelvis", "World")
        self._tf_br.sendTransform( vec2tuple(state.foot_pos_est[0].position), vec2tuple(state.foot_pos_est[0].orientation),\
                     state.header.stamp, "BDI_l_foot", "World")
        self._tf_br.sendTransform( vec2tuple(state.foot_pos_est[1].position), vec2tuple(state.foot_pos_est[1].orientation),\
                     state.header.stamp, "BDI_r_foot", "World")
        # check foot index:
        if AtlasSimInterfaceCommand.STEP == state.current_behavior:
            static_foot_index = state.step_feedback.desired_step_saturated.foot_index
        elif AtlasSimInterfaceCommand.WALK == state.current_behavior:
            static_foot_index = state.step_feedback.desired_step_saturated.foot_index
        else:
            static_foot_index = 2

    def _TransforFromGlobalToBDI(self,step_data,queue_i):
        foot_off_set = (0.06, 0.0, -0.085) # off-set from foot frame ('l_foot') to center of foot on ground (step_data refrence point)
        static_foot_index = (step_data.foot_index+1) % 2 # the foot that we arn't placing
        
        ## !!!---- Transformation Calculation: -------!!!:
        ## 1) Orientation difference between Global and BDI coordinates is calculated when robot is static (on initialize).
        ##    To determine BDI orientation we add difference (_GetOrientationDelta0Values) to Global orientation.  
        ## 2) Translation vector from static foot to new FP is calculated in Global coordinates (glabal_trans_delta_vec).
        ##    To determine BDI position we rotate vector to BDI coord. system and add the BDI static foot position (using one homogeneous transformations).
        if 0 == queue_i and self._bRobotIsStatic:    # self._bRobotIsStatic: #  
            if 0 == static_foot_index:
                static_foot_global_position , static_foot_global_rotation_q = self._GetTf('World','l_foot')
                trans2BDI,rot2BDI_quat = self._GetTf('World','BDI_l_foot')
            else:
                static_foot_global_position , static_foot_global_rotation_q = self._GetTf('World','r_foot')
                trans2BDI,rot2BDI_quat = self._GetTf('World','BDI_r_foot')
        else:
            static_foot_global_position = copy.copy(self._previous_des_global_FP_position)
            static_foot_global_rotation_q = copy.copy(self._previous_des_global_FP_orientation_q)
            trans2BDI = copy.copy(self._previous_BDI_FP)

        # ##TODO: determine which foot is static (=>self._static_foot_index) 
        # t = self._listener.getLatestCommonTime('World','l_foot')
        # if 0 == self._static_foot_index:
        #     static_foot_global_position , static_foot_global_rotation_q = self._GetTf('World','l_foot',t)
        #     trans2BDI,rot2BDI_quat = self._GetTf('World','BDI_l_foot',t)
        # else:
        #     static_foot_global_position , static_foot_global_rotation_q = self._GetTf('World','r_foot',t)
        #     trans2BDI,rot2BDI_quat = self._GetTf('World','BDI_r_foot',t)

        static_foot_global_rotation_euler = euler_from_quaternion(static_foot_global_rotation_q)
        #rot2BDI_euler = euler_from_quaternion(rot2BDI_quat) # Problem that rot2BDI_quat (foot_pos_est) doesn't return foot orientation (returns pelvis ori.?)  
        
        # new_BDI_foot_pose = BDI_static_foot_pose + Global_pose_delta_between_feet
        des_global_FP_position = [step_data.pose.position.x-foot_off_set[0],step_data.pose.position.y-foot_off_set[1], step_data.pose.position.z-foot_off_set[2]]
        global_X_delta = des_global_FP_position[0] - static_foot_global_position[0] #pelvis_global_position[0] #
        global_Y_delta = des_global_FP_position[1] - static_foot_global_position[1] #pelvis_global_position[1] #
        global_Z_delta = des_global_FP_position[2] - static_foot_global_position[2] #pelvis_global_position[2] #
        glabal_trans_delta_vec = np.matrix([[global_X_delta],[global_Y_delta],[global_Z_delta],[1.0]]) # numpy matrix (vector)
        rospy.loginfo("_TransforFromGlobalToBDI:: queue_i=%i  global static foot: x=%f, y=%f; Global FP: x=%f, y=%f "\
                     % ( queue_i,static_foot_global_position[0], static_foot_global_position[1], step_data.pose.position.x,step_data.pose.position.y) )
        # rotation correction using euler angles:
        des_global_FP_orientation_q = [step_data.pose.orientation.x, step_data.pose.orientation.y, step_data.pose.orientation.z, step_data.pose.orientation.w]
        des_global_roll, des_global_pitch, des_global_yaw = euler_from_quaternion(des_global_FP_orientation_q)
        global_roll_delta = des_global_roll - static_foot_global_rotation_euler[0] #pelvis_global_rotation_euler[0] #
        global_pitch_delta = des_global_pitch - static_foot_global_rotation_euler[1] #pelvis_global_rotation_euler[0] #
        global_yaw_delta = des_global_yaw - static_foot_global_rotation_euler[2] #pelvis_global_rotation_euler[0] #

        # homogeneous transformations:                                   
        Global2BDI_q = quaternion_from_euler(self._roll_delta0, self._pitch_delta0, self._yaw_delta0)
        transform_world2BDI = self._listener.fromTranslationRotation(trans2BDI, Global2BDI_q) #state.step_feedback.desired_step_saturated.pose.orientation)# rot2BDI_quat) 
           # homogeneous trans. of static foot to BDI coord. (Returns a Numpy 4x4 matrix for a transform)
        BDI_new_FP = transform_world2BDI*glabal_trans_delta_vec


        step_data.pose.position.x = BDI_new_FP.item(0) #[state.foot_pos_est[static_foot_index].position.x + BDI_X_delta
        step_data.pose.position.y = BDI_new_FP.item(1) #state.foot_pos_est[static_foot_index].position.y + BDI_Y_delta
        step_data.pose.position.z = BDI_new_FP.item(2) #state.foot_pos_est[static_foot_index].position.z + BDI_Z_delta
                
        BDI_new_FP_q = quaternion_from_euler(self._roll_delta0 + des_global_roll, self._pitch_delta0 + des_global_pitch, self._yaw_delta0 + des_global_yaw)
        step_data.pose.orientation.x = BDI_new_FP_q[0]
        step_data.pose.orientation.y = BDI_new_FP_q[1]
        step_data.pose.orientation.z = BDI_new_FP_q[2]
        step_data.pose.orientation.w = BDI_new_FP_q[3]

        self._previous_des_global_FP_position = copy.copy(des_global_FP_position)
        self._previous_des_global_FP_orientation_q = copy.copy(des_global_FP_orientation_q)
        self._previous_BDI_FP = [BDI_new_FP.item(0),BDI_new_FP.item(1),BDI_new_FP.item(2)]

        rospy.loginfo("_TransforFromGlobalToBDI:: command relative to static foot: distanceXY=%f, z=%f, yaw=%f, Global FP: pitch=%f, roll=%f "\
                     % ( (global_X_delta**2+global_Y_delta**2)**0.5, global_Z_delta, global_yaw_delta,des_global_pitch,des_global_roll) )
        return step_data

    def _GetOrientationDelta0Values(self):
        # learn delta0 values on initialize:
        global_euler = euler_from_quaternion([self._Global_Static_orientation_q.x,self._Global_Static_orientation_q.y,self._Global_Static_orientation_q.z,self._Global_Static_orientation_q.w])
        BDI_euler = euler_from_quaternion([self._BDI_Static_orientation_q.x,self._BDI_Static_orientation_q.y,self._BDI_Static_orientation_q.z,self._BDI_Static_orientation_q.w])
        self._roll_delta0 = global_euler[0] - BDI_euler[0]  # [rad] initial orientation difference between BDI odom and Global
        self._pitch_delta0 = global_euler[1] - BDI_euler[1] # [rad] initial orientation difference between BDI odom and Global
        self._yaw_delta0 = global_euler[2] - BDI_euler[2] # [rad] initial orientation difference between BDI odom and Global  

    def _GetTf(self,base_frame,get_frames,time=rospy.Time(0)):
        # waiting for transform to be avilable
        time_out = rospy.Duration(2)
        polling_sleep_duration = rospy.Duration(0.01)
        while self._listener.waitForTransform (base_frame, get_frames, time, time_out, polling_sleep_duration) and not rospy.is_shutdown():
                rospy.loginfo("QS_WalkingMode - _GetTf:: Not ready for Global To BDI transform")
        try:
          (translation,rotation_q) = self._listener.lookupTransform(base_frame, get_frames, time)  #  rospy.Time(0) to use latest availble transform 
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
          print ex
          rospy.loginfo("QS_WalkingMode - _GetTf:: tf exception")
          translation = [0,0,0]
          rotation_q = [0,0,0,1]
          #continue
        return translation,rotation_q


def vec2tuple(vector):
    res = ( vector.x, vector.y, vector.z )
    try:
        res = res + (vector.w,)
    except AttributeError:
        pass # print 'oops'
    return res
