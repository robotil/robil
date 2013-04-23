#!/usr/bin/env python

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################
import roslib; roslib.load_manifest('C42_ZMPWalk')
from robot_state import Robot_State
from hip_z_orientation_correction import *

class StepStrategyError(Exception):
    """
        For information as to why the exception was raised, see StepStrategyError.Message attribute
    """
    def __init__(self,strMessage):
        self.Message = strMessage
    
#----------------------------------------------------------------------------------

class StepStrategy(object):
    """
        The StepStrategy class is intended to be used with the StepStateMachine class,
        in order to reuse code for separate yet similar states.
    """
    def __init__(self,robotState,walkingTrajectory,transformListener,ZMP_Preview_BufferX,ZMP_Preview_BufferY):
        self._RobotState = robotState
        self._WalkingTrajectory = walkingTrajectory
        self._WalkingTrajectory.step_length = 0.0
        self._TransformListener = transformListener

        self._pelvis_des = self._RobotState.place_in_Ori( 0, 0, 0 ) # pelvis desired rotation (0,0,0)<=>stand up straight
        # self._pelvis_des = self._RobotState.place_in_Pos_Ori( 0, 0, 0, 0, 0, 0 ) # change for performing TURN STEP

        
#----------------------------------------------------------------------------------
        
class StepStrategyNone(StepStrategy):
    """
        The StepStrategyNone class is intended to be used with the StepStateMachine class,
        This strategy is used for states in which no action should be made, such as the Idle state
    """
    def __init__(self,robotState,walkingTrajectory,transformListener,ZMP_Preview_BufferX,ZMP_Preview_BufferY):
        StepStrategy.__init__(self,robotState,walkingTrajectory,transformListener,ZMP_Preview_BufferX,ZMP_Preview_BufferY)
    
    def Initialize(self,bend_knees):
        # TODO: make sure robot is static (in start position) and tf is running
        # init Robot State using tf data getting hip to com relative position:
        self._RobotState.static_init_with_tf_data(self._TransformListener,bend_knees)
        self._swing_foot_at_init = copy.copy(self._RobotState.swing_foot)
        
    def CalculateFootSwingTrajectory(self,step_time,step_length,step_width,step_height,dt,pre_step,first_step,full_step,last_step,k,k_total,k_start_swing,k_stop_swing):
        pass 
    
    def GetWalkingTrajectory(self,COMx, COMx_dot, p_pre_con_x,COMy, COMy_dot, p_pre_con_y,p_ref_x,p_ref_y,step_length,step_width,step_height,zmp_width,step_time,bend_knees,Des_Orientation,imu_orientation,k,dt,k_total,k_start_swing,k_stop_swing):

        # init output message (before starting to walk)
        [stance_hip_0, swing_y_sign, swing_hip_dy]=self._RobotState.Get_foot_coord_params(step_width) # assumes start walking in step phase 1 (set in self._RobotState init)

        self._WalkingTrajectory.step_length = 0.0
        self._WalkingTrajectory.step_width = step_width
        self._WalkingTrajectory.step_height = step_height
        self._WalkingTrajectory.zmp_width = zmp_width
        self._WalkingTrajectory.step_time = step_time
        self._WalkingTrajectory.bend_knees = bend_knees

        self._WalkingTrajectory.stance_hip = copy.copy( stance_hip_0 )
        self._WalkingTrajectory.pelvis_d = self._pelvis_des
        self._WalkingTrajectory.swing_foot = self._swing_foot_at_init
        self._WalkingTrajectory.swing_hip = self._RobotState.place_in_Pos( self._WalkingTrajectory.stance_hip.x, (self._WalkingTrajectory.stance_hip.y + swing_hip_dy) , self._WalkingTrajectory.stance_hip.z) # constraint: hips at same hight and advance together
        self._WalkingTrajectory.pelvis_m = self._RobotState.pelvis_m

        self._WalkingTrajectory.zmp_ref = self._RobotState.place_in_Pos(0,0,0)
        self._WalkingTrajectory.zmp_pc = self._RobotState.place_in_Pos(0,0,0)
        self._WalkingTrajectory.com_ref = self._RobotState.place_in_Pos(0,0,0)
        self._WalkingTrajectory.com_dot_ref = self._RobotState.place_in_Pos(0,0,0)
        self._WalkingTrajectory.com_m = self._RobotState.com_m

        self._WalkingTrajectory.step_phase = self._RobotState.Get_step_phase()
        # for debug use
        self._WalkingTrajectory.stance_hip_m = self._RobotState.stance_hip
        self._WalkingTrajectory.swing_hip_m = self._RobotState.swing_hip
        self._WalkingTrajectory.swing_foot_m = self._RobotState.swing_foot

        return self._WalkingTrajectory

#----------------------------------------------------------------------------------
        
class StepStrategyWalk(StepStrategy):
    """
        The StepStrategyNone class is intended to be used with the StepStateMachine class,
        This strategy is used for states in which no action should be made, such as the Idle state
    """
    def __init__(self,robotState,walkingTrajectory,transformListener,SwingTrajectory,orientation_correction,ZMP_Preview_BufferX,ZMP_Preview_BufferY):
        StepStrategy.__init__(self,robotState,walkingTrajectory,transformListener,ZMP_Preview_BufferX,ZMP_Preview_BufferY)
        self._SwingTrajectory = SwingTrajectory
        self._OrientationCorrection = orientation_correction
    
    def CalculateFootSwingTrajectory(self,step_time,step_length,step_width,step_height,dt,pre_step,first_step,full_step,last_step,k,k_total,k_start_swing,k_stop_swing):
        [self._stance_hip_0, swing_y_sign, self._swing_hip_dy] = self._RobotState.Get_foot_coord_params(step_width,step_length)
        self._swing_foot_y = step_width*swing_y_sign # final position of swing_foot y coordinate
        robot_foot_state = copy.copy(self._RobotState.swing_foot_at_start_of_step_phase)  #copy.copy(self._RobotState.swing_foot)
        [self._swing_k, lifting_swing_foot] = self._SwingTrajectory.Get_swing_foot_traj(k, step_time, robot_foot_state, step_length,self._swing_foot_y,step_height,dt,pre_step,first_step,full_step,last_step,k_total,k_start_swing,k_stop_swing)
        self._RobotState.Set_step_phase( foot_lift = lifting_swing_foot )
    
    def GetWalkingTrajectory(self,COMx, COMx_dot, p_pre_con_x,COMy, COMy_dot, p_pre_con_y,p_ref_x,p_ref_y,step_length,step_width,step_height,\
        zmp_width,step_time,bend_knees,Des_Orientation,imu_orientation,k,dt,k_total,k_start_swing,k_stop_swing):

        self._WalkingTrajectory.step_length = step_length
        self._WalkingTrajectory.step_width = step_width
        self._WalkingTrajectory.step_height = step_height
        self._WalkingTrajectory.zmp_width = zmp_width
        self._WalkingTrajectory.step_time = step_time
        self._WalkingTrajectory.bend_knees = bend_knees

        self._WalkingTrajectory.swing_foot = copy.copy(self._swing_k)                              #added by Israel 24.2
        self._WalkingTrajectory.swing_foot.y = copy.copy(self._swing_foot_y)
        self._WalkingTrajectory.stance_hip.x = COMx + self._stance_hip_0.x #- self._previous_step_length/2 #-D #TODO: incorporate step_length of previous step in Robot State and include it in stance_hip_0.x (foot coord. change)
                                                   # implemented in robot_state->Get_foot_coord_params: res_stance_hip_0[0] = res_stance_hip_0[0] - self._previous_step_length/2
        self._WalkingTrajectory.stance_hip.y = COMy + self._stance_hip_0.y
        self._WalkingTrajectory.stance_hip.z = self._stance_hip_0.z
        
        self._WalkingTrajectory.pelvis_d = self._pelvis_des #place_in_Orientation( 0, 0, 0 )
        
        self._WalkingTrajectory.swing_hip = self._RobotState.place_in_Pos( self._WalkingTrajectory.stance_hip.x, (self._WalkingTrajectory.stance_hip.y + self._swing_hip_dy), self._WalkingTrajectory.stance_hip.z) # constraint: hips at same hight and advance together
        self._WalkingTrajectory.pelvis_m = self._RobotState.pelvis_m
        
        self._WalkingTrajectory.zmp_ref = self._RobotState.place_in_Pos( p_ref_x[0], p_ref_y[0], 0)
        self._WalkingTrajectory.zmp_pc = self._RobotState.place_in_Pos( p_pre_con_x, p_pre_con_y, 0)
        self._WalkingTrajectory.com_ref = self._RobotState.place_in_Pos( COMx, COMy, 0)
        self._WalkingTrajectory.com_dot_ref = self._RobotState.place_in_Pos( COMx_dot, COMy_dot, 0)
        self._WalkingTrajectory.com_m = self._RobotState.com_m # last sampled value
        
        self._WalkingTrajectory.step_phase = self._RobotState.Get_step_phase()
        # for debug use
        self._WalkingTrajectory.stance_hip_m = self._RobotState.stance_hip # last sampled value
        self._WalkingTrajectory.swing_hip_m = self._RobotState.swing_hip # last sampled value
        self._WalkingTrajectory.swing_foot_m = self._RobotState.swing_foot # filtered value
        
        self._WalkingTrajectory.hip_z_orientation = self._OrientationCorrection.hip_z_orientation_correction(Des_Orientation,imu_orientation,k,dt,self._RobotState.Get_step_phase(),k_total,k_start_swing,k_stop_swing)

        self._previous_iteration_step_length = copy.copy(self._WalkingTrajectory.step_length)

        return self._WalkingTrajectory