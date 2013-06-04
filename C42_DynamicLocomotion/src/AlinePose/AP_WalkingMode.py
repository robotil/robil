#!/usr/bin/env python

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

import copy
import roslib
roslib.load_manifest('C42_DynamicLocomotion')

from Abstractions.WalkingMode import *
from Abstractions.Odometer import *
from AP_PathPlanner import *

import tf
from tf_conversions import posemath
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np

from atlas_msgs.msg import AtlasCommand, AtlasSimInterfaceCommand, AtlasSimInterfaceState, AtlasState, AtlasBehaviorStepData
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion, Point
from std_msgs.msg import String

from C42_DynamicLocomotion.srv import *
from C42_DynamicLocomotion.msg import Foot_Placement_data

class AP_WalkingMode(WalkingMode):
    def __init__(self,iTf):
        self._LPP = AP_PathPlanner()
        WalkingMode.__init__(self,self._LPP)
        self._tf = iTf
        # Initialize atlas atlas_sim_interface_command publisher       
        self.asi_command = rospy.Publisher('/atlas/atlas_sim_interface_command', AtlasSimInterfaceCommand, None, False, True, None)       

        self._Odometer = Odometer()
        self._bDone = False
        self._bIsSwaying = False
        self._command = 0

    def Initialize(self):
        WalkingMode.Initialize(self)
        self._command = 0
        self._bRobotIsStatic = True
        self._BDI_Static_pose = Pose()
        self._DesiredObject = "rotation"

        rospy.wait_for_service('foot_aline_pose')
        self._foot_placement_client = rospy.ServiceProxy('foot_aline_pose', C23_orient)
        # Subscribers:
        #self._Subscribers["Odometry"] = rospy.Subscriber('/ground_truth_odom',Odometry,self._odom_cb)
        self._Subscribers["ASI_State"]  = rospy.Subscriber('/atlas/atlas_sim_interface_state', AtlasSimInterfaceState, self.asi_state_cb)
        self._Subscribers["IMU"]  = rospy.Subscriber('/atlas/imu', Imu, self._get_imu)

        rospy.sleep(0.3)
        
        self._RequestTargetPose(self._DesiredObject)
        k_effort = [0] * 28
        self._bDone = False
        self._bIsSwaying = False
        self._bRobotIsStatic = False
        #self._GetOrientationDelta0Values() # Orientation difference between BDI odom and Global
    
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

    def Stop(self):
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
            #command.step_params.desired_step = self._TransforFromGlobalToBDI(command.step_params.desired_step,state)
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
            if (AP_PathPlannerEnum.Active == self._LPP.State):
                command = self.GetCommand(state)
            elif(AP_PathPlannerEnum.Waiting == self._LPP.State):
                self._RequestTargetPose(self._DesiredObject)
        elif ("Done" == self._WalkingModeStateMachine.GetCurrentState().Name):
            #print(4)
            self._bDone = True
        else:
            raise Exception("AL_WalkingModeStateMachine::Bad State Name")
    
        return command
    
    def _RequestTargetPose(self,desired_object):
        err_rot = 0.02 # [rad]
        err_trans = 0.1 # [meters]
        print("Request Target Pose to ",desired_object)
        # Perform a service request from FP
        try:
            # Handle preemption?
                # if received a "End of mission" sort of message from FP
            #start_pose,other_foot_pose = self._GetStartingFootPose()
            resp_target_pose = self._foot_placement_client(desired_object)
            delta_yaw, delta_trans = self._GetDeltaToObject(desired_object,resp_target_pose)
            
            foot_placement_path = []
            if math.fabs(delta_yaw) > err_rot:
                foot_placement_path = foot_placement_path + self._GetRotationDeltaFP_Path(delta_yaw)
            # if self._DistanceXY(delta_trans) > err_trans:
            #     self._GetTranslationDeltaFP_Path(delta_yaw)

            # if (math.fabs(delta_yaw) <= err_rot) and (self._DistanceXY(delta_trans) <= err_trans): # finished task
            if [] == foot_placement_path: # 1 == resp.done:
                self._WalkingModeStateMachine.PerformTransition("Finished")
                # if big error need to finish with error (didn't reach goal)
            else:
                listSteps = []
                for desired in foot_placement_path:
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
                    print ("step command:",step)
                    listSteps.append(step)
                self._LPP.SetPath(listSteps)
                #print(listSteps)
        except rospy.ServiceException, e:
            print "Foot Placement Service call failed: %s"%e

    def _GetDeltaToObject(self,desired_object,target_pose):
        delta_trans = Point()
        if "rotation" == desired_object:
            delta_yaw = target_pose.Y - 0.0
            delta_trans.x = target_pose.x - 0.0
            delta_trans.y = target_pose.y - 0.0
        elif "fwOneMeter" == desired_object:
            delta_yaw = target_pose.Y - 0.0
            delta_trans.x = target_pose.x - 0.0
            delta_trans.y = target_pose.y - 0.0
        elif "leftOneMeter" == desired_object:
            delta_yaw = target_pose.Y - 0.0
            delta_trans.x = target_pose.x - 0.0
            delta_trans.y = target_pose.y - 0.0
        else:
            delta_yaw = 0.0
            delta_trans.x = target_pose.x - 0.0
            delta_trans.y = target_pose.y - 0.0

        return delta_yaw, delta_trans

    def _DistanceXY(self,position):
        return ( (pposition.x)**2 + (position.y)**2 )**0.5

    def _GetRotationDeltaFP_Path(self,delta_yaw):
        print ("_GetRotationDeltaFP_Path yaw:",delta_yaw)
        foot_placement_path= []
        start_position = copy.copy(self._BDI_Static_pose.position)
        start_orientation = euler_from_quaternion([self._BDI_Static_pose.orientation.x, self._BDI_Static_pose.orientation.y, self._BDI_Static_pose.orientation.z, self._BDI_Static_pose.orientation.w])
        
        theta_max = 0.4 # max turning angle per step
        W = 0.3 # Width of stride
        R = W/2 # Radius of turn, turn in place

        ### Turn in place (pivot):
        Num_seq = int(math.floor(math.fabs(delta_yaw)/theta_max))
        #theta_rem = math.fabs(delta_yaw)%theta_max
        if delta_yaw > 0.0: #turn left (CCW)
            LeftRight_step_seq = True # Turn first with left foot and then with right
        else: #turn right (CW)
            theta_max = -theta_max
            LeftRight_step_seq = False
            #theta_rem = -theta_rem

        ## Step in place: two first steps
        # yaw angle of robot
        theta0 = copy.copy(start_orientation[2])

        # X, Y position of foot
        X_l_foot = start_position.x - R * math.sin(theta0)
        Y_l_foot = start_position.y + R * math.cos(theta0)

        X_r_foot = start_position.x + R * math.sin(theta0)
        Y_r_foot = start_position.y - R * math.cos(theta0)

        euler_ori = [start_orientation[0], start_orientation[1], theta0 ]

        if LeftRight_step_seq:
            foot_placement_path.append( self._FP_data(0,[X_l_foot,Y_l_foot,start_position.z],euler_ori) )
            foot_placement_path.append( self._FP_data(1,[X_r_foot,Y_r_foot,start_position.z],euler_ori) )
        else:
            foot_placement_path.append( self._FP_data(1,[X_r_foot,Y_r_foot,start_position.z],euler_ori) )
            foot_placement_path.append( self._FP_data(0,[X_l_foot,Y_l_foot,start_position.z],euler_ori) )

        ## Turn Num_seq steps:
        for n in range(1,Num_seq+1):
            theta = n*theta_max + theta0
            # X, Y position of foot
            X_l_foot = start_position.x - R * math.sin(theta)
            Y_l_foot = start_position.y + R * math.cos(theta)

            X_r_foot = start_position.x + R * math.sin(theta)
            Y_r_foot = start_position.y - R * math.cos(theta)

            euler_ori = [start_orientation[0], start_orientation[1], theta ]

            if LeftRight_step_seq:
                foot_placement_path.append( self._FP_data(0,[X_l_foot,Y_l_foot,start_position.z],euler_ori) )
                foot_placement_path.append( self._FP_data(1,[X_r_foot,Y_r_foot,start_position.z],euler_ori) )
            else:
                foot_placement_path.append( self._FP_data(1,[X_r_foot,Y_r_foot,start_position.z],euler_ori) )
                foot_placement_path.append( self._FP_data(0,[X_l_foot,Y_l_foot,start_position.z],euler_ori) )
        
        theta = delta_yaw + theta0    

        # X, Y position of foot
        X_l_foot = start_position.x - R * math.sin(theta)
        Y_l_foot = start_position.y + R * math.cos(theta)

        X_r_foot = start_position.x + R * math.sin(theta)
        Y_r_foot = start_position.y - R * math.cos(theta)

        euler_ori = [start_orientation[0], start_orientation[1], theta ]

        if LeftRight_step_seq:
            foot_placement_path.append( self._FP_data(0,[X_l_foot,Y_l_foot,start_position.z],euler_ori) )
            foot_placement_path.append( self._FP_data(1,[X_r_foot,Y_r_foot,start_position.z],euler_ori) )
        else:
            foot_placement_path.append( self._FP_data(1,[X_r_foot,Y_r_foot,start_position.z],euler_ori) )
            foot_placement_path.append( self._FP_data(0,[X_l_foot,Y_l_foot,start_position.z],euler_ori) )
            
        # print ("FP data path:",foot_placement_path)

        return foot_placement_path

    def _FP_data(self,foot_index,position,euler_angle,swing_height=0.1):
        res = Foot_Placement_data()
        #foot_off_set = (0.06, 0.0, -0.085) # off-set from foot frame ('l_foot') to center of foot on ground  
        res.foot_index = foot_index
        res.pose.position.x = position[0] #+ foot_off_set[0]
        res.pose.position.y = position[1] #+ foot_off_set[1]
        res.pose.position.z = position[2] #+ foot_off_set[2]
        res.pose.ang_euler.x = euler_angle[0] # deg2r(euler_angle_deg[0]) # roll
        res.pose.ang_euler.y = euler_angle[1] # deg2r(euler_angle_deg[1]) # pitch
        res.pose.ang_euler.z = euler_angle[2] # deg2r(euler_angle_deg[2]) # yaw
        res.clearance_height = swing_height
        return res
    
    # def _GetStartingFootPose(self): #*************** NEED TO CHANGE when not static *************#
    #     start_pose = Foot_Placement_data()
    #     other_foot_pose = Foot_Placement_data()
        
    #     trans, rot_q = self._GetTf('World','l_foot')
    #     start_pose.foot_index = 2 # both feet are static
    #     start_pose.pose.position.x = trans[0]
    #     start_pose.pose.position.y = trans[1]
    #     start_pose.pose.position.z = trans[2]
    #     rot_euler = euler_from_quaternion(rot_q)
    #     start_pose.pose.ang_euler.x = rot_euler[0]
    #     start_pose.pose.ang_euler.y = rot_euler[1]
    #     start_pose.pose.ang_euler.z = rot_euler[2]
    #     trans, rot_q = self._GetTf('World','r_foot')
    #     other_foot_pose.foot_index = 1 # right foot
    #     other_foot_pose.pose.position.x = trans[0]
    #     other_foot_pose.pose.position.y = trans[1]
    #     other_foot_pose.pose.position.z = trans[2]
    #     rot_euler = euler_from_quaternion(rot_q)
    #     other_foot_pose.pose.ang_euler.x = rot_euler[0]
    #     other_foot_pose.pose.ang_euler.y = rot_euler[1]
    #     other_foot_pose.pose.ang_euler.z = rot_euler[2]
    #    return start_pose,other_foot_pose
    
###################################################################################
#--------------------------- CallBacks --------------------------------------------
###################################################################################

    # /atlas/atlas_sim_interface_state callback. Before publishing a walk command, we need
    # the current robot position   
    def asi_state_cb(self, state):
        if self._bRobotIsStatic:
            self._BDI_Static_pose.position = copy.copy(state.pos_est.position)
            self._BDI_Static_pose.position.z = copy.copy(state.foot_pos_est[0].position.z)
            self._BDI_Static_pose.orientation = copy.copy(state.foot_pos_est[0].orientation)
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
        if (0 != command):
            self._command = command
            self._bIsSwaying = True
        
        if(0 == state.current_behavior and 0 != self._command):
            #print self._command
            self.asi_command.publish(self._command)
            self._command = 0
            print("step start")

    # def _odom_cb(self,odom):
    #     if self._bRobotIsStatic:
    #         self._Global_Static_orientation_q = odom.pose.pose.orientation
    #     self._LPP.UpdatePosition(odom.pose.pose.position.x,odom.pose.pose.position.y)
    #     # sendTransform(translation - tuple (x, y, z), rotation - tuple (x, y, z, w), time, child, parent)
    #     self._tf.TransformBroadcaster().sendTransform(vec2tuple(odom.pose.pose.position), vec2tuple(odom.pose.pose.orientation), odom.header.stamp, "pelvis", "World") # "World", "pelvis") 
 
    def _get_imu(self,msg):  #listen to /atlas/imu/pose/pose/orientation
        roll, pitch, yaw = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self._Odometer.SetYaw(yaw)


###############################################################################################

    def _Update_tf_BDI_odom(self,state):
        self._tf.TransformBroadcaster().sendTransform( vec2tuple(state.pos_est.position), vec2tuple(state.foot_pos_est[0].orientation), state.header.stamp, "BDI_pelvis", "World")
        self._tf.TransformBroadcaster().sendTransform( vec2tuple(state.foot_pos_est[0].position), vec2tuple(state.foot_pos_est[0].orientation),\
                     state.header.stamp, "BDI_l_foot", "World")
        self._tf.TransformBroadcaster().sendTransform( vec2tuple(state.foot_pos_est[1].position), vec2tuple(state.foot_pos_est[1].orientation),\
                     state.header.stamp, "BDI_r_foot", "World")
        # check foot index:
        if AtlasSimInterfaceCommand.STEP == state.current_behavior:
            static_foot_index = state.step_feedback.desired_step_saturated.foot_index
        elif AtlasSimInterfaceCommand.WALK == state.current_behavior:
            static_foot_index = state.step_feedback.desired_step_saturated.foot_index
        else:
            static_foot_index = 2
        # # determine static foot: ??? if needed
        # if 0 == static_foot_index: # Left foot is static
        #     self._tf.TransformBroadcaster().sendTransform( vec2tuple(state.foot_pos_est[0].position), vec2tuple(state.foot_pos_est[0].orientation),\
        #              state.header.stamp, "BDI_static_foot", "World")
        # elif 1 == static_foot_index: # Right foot is static
        #     self._tf.TransformBroadcaster().sendTransform( vec2tuple(state.foot_pos_est[1].position) , vec2tuple(state.foot_pos_est[1].orientation),\
        #              state.header.stamp, "BDI_static_foot", "World")
        # elif 2 == static_foot_index: # both feet are static
        #     self._tf.TransformBroadcaster().sendTransform( (0, 0, 0), (0, 0, 0, 1), state.header.stamp, "BDI_static_foot", "World")
        # else: # problem
        #     self._tf.TransformBroadcaster().sendTransform( (0, 0, -10), (0, 0, 0, 1), state.header.stamp, "BDI_static_foot", "World")

    # def _TransforFromGlobalToBDI(self,step_data,state):
    #     foot_off_set = (0.06, 0.0, -0.085) # off-set from foot frame ('l_foot') to center of foot on ground (step_data refrence point)
    #     static_foot_index = (step_data.foot_index+1) % 2 # the foot that we arn't placing
    #     ## USING PELVIS AS REFRENCE:        
    #     # t = self._tf.TransformListener().getLatestCommonTime('World','BDI_pelvis')
    #     # trans2BDI,rot2BDI_quat = self._GetTf('World','BDI_pelvis',t)
    #     # pelvis_global_position , pelvis_global_rotation_q = self._GetTf('World','pelvis',t)
    #     # ## check pelvis placement error at time t, BDI Vs. Global: (the relation (position), pelvis to feet, at time t should be identical -> error=0 ) 
    #     # BDI_l_foot,rot = self._GetTf('BDI_pelvis','BDI_l_foot',t)
    #     # BDI_r_foot,rot = self._GetTf('BDI_pelvis','BDI_r_foot',t)
    #     # global_l_foot,rot = self._GetTf('pelvis','l_foot',t)
    #     # global_r_foot,rot = self._GetTf('pelvis','r_foot',t)
    #     # err_l_foot = ( (global_l_foot[0]-(BDI_l_foot[0]-foot_off_set[0]))**2 + (global_l_foot[1]-(BDI_l_foot[1]-foot_off_set[1]))**2 \
    #     #              + (global_l_foot[2]-(BDI_l_foot[2]-foot_off_set[2]))**2 )**0.5
    #     # err_r_foot = ( (global_r_foot[0]-(BDI_r_foot[0]-foot_off_set[0]))**2 + (global_r_foot[1]-(BDI_r_foot[1]-foot_off_set[1]))**2 \
    #     #              + (global_r_foot[2]-(BDI_r_foot[2]-foot_off_set[2]))**2 )**0.5
    #     # pelvis_global_rotation_euler = euler_from_quaternion(pelvis_global_rotation_q)
        
    #     ## !!!---- Transformation Calculation: -------!!!:
    #     ## 1) Orientation difference between Global and BDI coordinates is calculated when robot is static (on initialize).
    #     ##    To determine BDI orientation we add difference (_GetOrientationDelta0Values) to Global orientation.  
    #     ## 2) Translation vector from static foot to new FP is calculated in Global coordinates (glabal_trans_delta_vec).
    #     ##    To determine BDI position we rotate vector to BDI coord. system and add the BDI static foot position (using one homogeneous transformations).
    #     if 0 == static_foot_index:
    #         static_foot_global_position , static_foot_global_rotation_q = self._GetTf('World','l_foot')
    #         trans2BDI,rot2BDI_quat = self._GetTf('World','BDI_l_foot')
    #     else:
    #         static_foot_global_position , static_foot_global_rotation_q = self._GetTf('World','r_foot')
    #         trans2BDI,rot2BDI_quat = self._GetTf('World','BDI_r_foot')
    #     static_foot_global_rotation_euler = euler_from_quaternion(static_foot_global_rotation_q)
    #     rot2BDI_euler = euler_from_quaternion(rot2BDI_quat) # Problem that rot2BDI_quat (foot_pos_est) doesn't return foot orientation (returns pelvis ori.?)  
        
    #     # new_BDI_foot_pose = BDI_static_foot_pose + Global_pose_delta_between_feet
    #     global_X_delta = (step_data.pose.position.x-foot_off_set[0]) - static_foot_global_position[0] #pelvis_global_position[0] #
    #     global_Y_delta = (step_data.pose.position.y-foot_off_set[1]) - static_foot_global_position[1] #pelvis_global_position[1] #
    #     global_Z_delta = (step_data.pose.position.z-foot_off_set[2]) - static_foot_global_position[2] #pelvis_global_position[2] #
    #     glabal_trans_delta_vec = np.matrix([[global_X_delta],[global_Y_delta],[global_Z_delta],[1.0]]) # numpy matrix (vector)
    #     # rotation correction using euler angles:
    #     des_global_roll, des_global_pitch, des_global_yaw = euler_from_quaternion([step_data.pose.orientation.x, step_data.pose.orientation.y, step_data.pose.orientation.z, step_data.pose.orientation.w])
    #     global_roll_delta = des_global_roll - static_foot_global_rotation_euler[0] #pelvis_global_rotation_euler[0] #
    #     global_pitch_delta = des_global_pitch - static_foot_global_rotation_euler[1] #pelvis_global_rotation_euler[0] #
    #     global_yaw_delta = des_global_yaw - static_foot_global_rotation_euler[2] #pelvis_global_rotation_euler[0] #

    #     # homogeneous transformations:                                   
    #     Global2BDI_q = quaternion_from_euler(self._roll_delta0, self._pitch_delta0, self._yaw_delta0)
    #     transform_world2BDI = self._tf.TransformListener().fromTranslationRotation(trans2BDI, Global2BDI_q) #state.step_feedback.desired_step_saturated.pose.orientation)# rot2BDI_quat) 
    #        # homogeneous trans. of static foot to BDI coord. (Returns a Numpy 4x4 matrix for a transform)
    #     # transform_static2new_FP = tf.fromTranslationRotation(glabal_trans_delta_vec, glabal_rot_delta_vec) # homogeneous trans. of static foot to new foot placement pose,
    #     #   # the transform is according to relative coordinates (delta) calculated in global coordinate system (but not connected to global coord. which only a ref. system)  
    #     # transform_world2new_FP = transform_world2BDI*transform_static2new_FP # homogeneous trans. of new foot placemen to BDI coord.
    #     BDI_new_FP = transform_world2BDI*glabal_trans_delta_vec
    #     # rospy.loginfo("_TransforFromGlobalToBDI:: BDI static foot -pos: x=%f, y=%f , z=%f; ori: roll=%f, pitch=%f , yaw=%f; \
    #     #               foot_pos_est- pos: x=%f ori: x=%f, y=%f z=%f w=%f;" \
    #     #      % (trans2BDI[0], trans2BDI[1], trans2BDI[2], rot2BDI_euler[0], rot2BDI_euler[1], rot2BDI_euler[2],\
    #     #         state.foot_pos_est[0].position.x, state.foot_pos_est[0].orientation.x,state.foot_pos_est[0].orientation.y,\
    #     #         state.foot_pos_est[0].orientation.z,state.foot_pos_est[0].orientation.w) )

    #     step_data.pose.position.x = BDI_new_FP.item(0) #[state.foot_pos_est[static_foot_index].position.x + BDI_X_delta
    #     step_data.pose.position.y = BDI_new_FP.item(1) #state.foot_pos_est[static_foot_index].position.y + BDI_Y_delta
    #     step_data.pose.position.z = BDI_new_FP.item(2) #state.foot_pos_est[static_foot_index].position.z + BDI_Z_delta
                
    #     # BDI_static_foot_roll, BDI_static_foot_pitch, BDI_static_foot_yaw = euler_from_quaternion([state.foot_pos_est[static_foot_index].orientation.x,\
    #     #  state.foot_pos_est[static_foot_index].orientation.y, state.foot_pos_est[static_foot_index].orientation.z, state.foot_pos_est[static_foot_index].orientation.w])        
    #     #Q = quaternion_from_euler(rot2BDI_euler[0] + global_roll_delta, rot2BDI_euler[1] + global_pitch_delta, rot2BDI_euler[2] + global_yaw_delta)
    #     BDI_new_FP_q = quaternion_from_euler(self._roll_delta0 + des_global_roll, self._pitch_delta0 + des_global_pitch, self._yaw_delta0 + des_global_yaw)
    #     step_data.pose.orientation.x = BDI_new_FP_q[0]
    #     step_data.pose.orientation.y = BDI_new_FP_q[1]
    #     step_data.pose.orientation.z = BDI_new_FP_q[2]
    #     step_data.pose.orientation.w = BDI_new_FP_q[3]

    #     # rospy.loginfo("_TransforFromGlobalToBDI:: command position: x=%f, y=%f , z=%f; des Global delta: roll=%f, pitch=%f , yaw=%f; global delta : roll=%f, pitch=%f , yaw=%f" \
    #     #      % (step_data.pose.position.x, step_data.pose.position.y, step_data.pose.position.z, des_global_roll, des_global_pitch, des_global_yaw,\
    #     #         global_roll_delta, global_pitch_delta, global_yaw_delta) )
    #     #print Q
    #     rospy.loginfo("_TransforFromGlobalToBDI:: command relative to static foot: distanceXY=%f, z=%f, yaw=%f, Global FP: pitch=%f, roll=%f "\
    #                  % ( (global_X_delta**2+global_Y_delta**2)**0.5, global_Z_delta, global_yaw_delta,des_global_pitch,des_global_roll) )
    #     return step_data

    # def _GetOrientationDelta0Values(self):
    #     # learn delta0 values on initialize:
    #     global_euler = euler_from_quaternion([self._Global_Static_orientation_q.x,self._Global_Static_orientation_q.y,self._Global_Static_orientation_q.z,self._Global_Static_orientation_q.w])
    #     BDI_euler = euler_from_quaternion([self._BDI_Static_orientation_q.x,self._BDI_Static_orientation_q.y,self._BDI_Static_orientation_q.z,self._BDI_Static_orientation_q.w])
    #     self._roll_delta0 = global_euler[0] - BDI_euler[0]  # [rad] initial orientation difference between BDI odom and Global
    #     self._pitch_delta0 = global_euler[1] - BDI_euler[1] # [rad] initial orientation difference between BDI odom and Global
    #     self._yaw_delta0 = global_euler[2] - BDI_euler[2] # [rad] initial orientation difference between BDI odom and Global  

    # def _GetTf(self,base_frame,get_frames,time=rospy.Time(0)):
    #     # waiting for transform to be avilable
    #     time_out = rospy.Duration(2)
    #     polling_sleep_duration = rospy.Duration(0.01)
    #     while self._tf.TransformListener().waitForTransform (base_frame, get_frames, time, time_out, polling_sleep_duration) and not rospy.is_shutdown():
    #             rospy.loginfo("AL_WalkingMode - _GetTf:: Not ready for Global To BDI transform")
    #     try:
    #       (translation,rotation_q) = self._tf.TransformListener().lookupTransform(base_frame, get_frames, time)  #  rospy.Time(0) to use latest availble transform 
    #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
    #       print ex
    #       rospy.loginfo("AL_WalkingMode - _GetTf:: tf exception")
    #       translation = [0,0,0]
    #       rotation_q = [0,0,0,1]
    #       #continue
    #     return translation,rotation_q


# def vec2tuple(vector):
#     res = ( vector.x, vector.y, vector.z )
#     try:
#         res = res + (vector.w,)
#     except AttributeError:
#         pass # print 'oops'
#     return res
