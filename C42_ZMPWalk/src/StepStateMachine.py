#!/usr/bin/env python

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

import roslib; roslib.load_manifest('C42_ZMPWalk')
#import rospy, sys, os.path
#from pylab import *

from StateMachine import *
from ZmpStrategyStep import *
from C42_ZMPWalk.msg import walking_trajectory
import rospy
import tf
from robot_state import Robot_State
from zmp_init import init_pose
from zmp_profiles import *
import copy
from collections import deque

###################################################################################
#--------------------------- Exceptions -------------------------------------------
###################################################################################

class StepStateMachineError(StateMachineError):
    """
        For information as to why the exception was raised, see Message attribute
    """
    def __init__(self,strMessage):
        StateMachineError.__init__(self,strMessage)
    
###################################################################################
#--------------------------- States -----------------------------------------------
###################################################################################

class StepState(State):
    """
        The StepState class is intended to be used with the StepStateMachine class
    """    
    def __init__(self,strStateName,StepStrategy,dt):
        State.__init__(self,strStateName)
        self._Strategy = StepStrategy
        
        # Step parameters
        # self._fD = 0.0     #!!! moved into preview_controller !!!#        # Distance? I don't know...
        #self._DistanceRefX = 0.0
        
        self._pre_step = 0
        self._first_step = 0
        self._full_step = 1
        self._last_step = 0
        
        # Walking Parameters
        self._turn_angle  = 0.0 # [rad], default value 
        self._step_length = 0.1 #0.01  # [m]
        self._step_width  = 0.182#0.178  # 0.178  # [m]
        self._zmp_width   = 0.178 #0.120  #0.02 # #0.168 #when lift foot 0.130, feet on ground 0.110
        self._step_time   = 1.2 #1   # [sec]
        self._bend_knees  = 0.055 #0.12 #0.18 #0.04  # [m]     
        self._step_height = 0.035 #0.05 #0.03 #0.05  # [m] 
        self._trans_ratio_of_step = 1.0 #0.9 #1.0 #0.8 # units fraction: 0-1.0 ; fraction of step time to be used for transition. 1.0 = all of step time is transition
        self._trans_slope_steepens_factor = 8/self._step_time #2 # 1 transition Sigmoid slope (a)

        self._k_total = self._step_time*(1/dt)
        self._k_start_swing =  floor(self._k_total/3)
        self._k_stop_swing =   floor(2*self._k_total/3)

        # 2) preceeding steps (full_step)
        self._p_ref_x = Step_forward_x(0, self._step_length, self._trans_ratio_of_step, self._trans_slope_steepens_factor, self._step_time, dt)
        self._p_ref_y = Step_onto_right_foot(0, self._zmp_width, self._trans_ratio_of_step, self._trans_slope_steepens_factor, self._step_time, dt)
    
    def GetStepTime(self):
        return self._step_time
    
    def UpdateStepCounter(self,stepCounter):
        return stepCounter
    
    def CalculateFootSwingTrajectory(self,dt,k):
        self._Strategy.CalculateFootSwingTrajectory(self._step_time,self._step_length,self._step_width,self._step_height,dt,\
            self._pre_step,self._first_step,self._full_step,self._last_step,k,self._k_total,self._k_start_swing,self._k_stop_swing)

    def GetWalkingTrajectory(self,COMx, COMx_dot, p_pre_con_x,COMy, COMy_dot, p_pre_con_y,p_ref_x,p_ref_y,Des_Orientation,imu_orientation,k,dt):
        return self._Strategy.GetWalkingTrajectory(COMx, COMx_dot, p_pre_con_x,COMy, COMy_dot, p_pre_con_y,p_ref_x,p_ref_y, \
            self._step_length,self._step_width,self._step_height,self._zmp_width,self._step_time,self._bend_knees, \
            Des_Orientation,imu_orientation,k,dt,self._k_total,self._k_start_swing,self._k_stop_swing)

    def UpdatePreview(self):
        return self._Strategy.UpdatePreview()
    
    def GetId(self):
        raise StepStateMachineError("Forgot to implement GetId did ya?!")

    def GetZmpProfileX(self):
        return self._p_ref_x

    def GetZmpProfileY(self):
        return self._p_ref_y
        
#----------------------------------------------------------------------------------

class IdleStepState(StepState):
    """
        The IdleStepState class is intended to be used with the StepStateMachine class,
        When the ZMP is Idle
    """
    def __init__(self,StepStrategy,dt):
        StepState.__init__(self,"Idle",StepStrategy,dt)
        
    def GetId(self):
        return 0
        
#----------------------------------------------------------------------------------

class InitializeStepState(StepState):
    """
        The InitializeStepState class is intended to be used with the StepStateMachine class,
        When the ZMP is Initialized
    """
    def __init__(self,StepStrategy,dt):
        StepState.__init__(self,"Initializing",StepStrategy,dt)
        
    def OnEnter(self):
        # moving to intial pose:
        init_pose()  # !!! need to disable tf listener drc2_tools to prevent clash !!!
        self._Strategy.Initialize(self._bend_knees)
                
    def GetId(self):
        return 1

#----------------------------------------------------------------------------------

class FailureStepState(StepState):
    """
        The FailureStepState class is intended to be used with the StepStateMachine class,
        When the ZMP Fails to initialize
    """
    def __init__(self,StepStrategy,dt):
        StepState.__init__(self,"Failing",StepStrategy,dt)
        
    def GetId(self):
        return -1
#----------------------------------------------------------------------------------

class PreStepState(StepState):
    """
        The PreStepState class is intended to be used with the StepStateMachine class,
        When the ZMP starts walking
    """
    def __init__(self,StepStrategy,robotState,dt):
        StepState.__init__(self,"PreStep",StepStrategy,dt)
        
        self._robotState = robotState
 
        self._pre_step = 1
        self._first_step = 0
        self._full_step = 0
        self._last_step = 0
        
        # create ZMP_ref profiles: 
        # # 1) to start walking (pre_step + first_step)
        # self._p_ref_x = Start_sagital_x(0, self._step_length, self._trans_ratio_of_step, self._trans_slope_steepens_factor, self._step_time, dt)
        # self._p_ref_y = Start_lateral_y_weight_to_left_foot(0, self._zmp_width, self._trans_ratio_of_step, self._trans_slope_steepens_factor, self._step_time, dt)
        # 1) to start walking (pre_step)
        self._p_ref_x = Pre_Step_sagital_x(0, self._step_length, self._trans_ratio_of_step, self._trans_slope_steepens_factor, self._step_time, dt)
        self._p_ref_y = Pre_Step_lateral_y(0, self._zmp_width, self._trans_ratio_of_step, self._trans_slope_steepens_factor, self._step_time, dt)

    def OnEnter(self):
        rospy.loginfo("started walking")
        rospy.loginfo("time:")
        rospy.loginfo(rospy.get_time())

        self._robotState.Set_step_phase(value = 1)

        #self._Strategy.LoadNewStep(self._p_ref_x_start,self._p_ref_x_forward_step,self._p_ref_y_start,r_[ self._p_ref_y_step_right, self._p_ref_y_step_left ]) # new_step, following_steps_cycle
        
    def OnExit(self):
        # completed pre_step
        rospy.loginfo("done pre step")
        rospy.loginfo("time:")
        rospy.loginfo(rospy.get_time())

    def GetId(self):
        return 2
    
#----------------------------------------------------------------------------------

class FirstStepState(StepState):
    """
        The FirstStepState class is intended to be used with the StepStateMachine class,
        When the ZMP starts walking
    """
    def __init__(self,StepStrategy,dt):
        StepState.__init__(self,"FirstStep",StepStrategy,dt)
        
        self._pre_step = 0
        self._first_step = 1
        self._full_step = 0
        self._last_step = 0

        self._p_ref_x = First_Step_sagital_x(0, self._step_length, self._trans_ratio_of_step, self._trans_slope_steepens_factor, self._step_time, dt)
        self._p_ref_y = First_Step_lateral_y(0, self._zmp_width, self._trans_ratio_of_step, self._trans_slope_steepens_factor, self._step_time, dt)
 
    def OnExit(self):
        rospy.loginfo("done first step")
        rospy.loginfo("time:")
        rospy.loginfo(rospy.get_time())

    def UpdateStepCounter(self,stepCounter):
        stepCounter = stepCounter+1
        return stepCounter

    def GetId(self):
        return 3
    
#----------------------------------------------------------------------------------

class PreStoppingFirstStepState(StepState):
    """
        The StoppingFirstStepState class is intended to be used with the StepStateMachine class,
        When the ZMP starts walking but has to stop at the same time
    """
    def __init__(self,StepStrategy,walkingTrajectory,robotState,dt):
        StepState.__init__(self,"PreStoppingFirstStep",StepStrategy,dt)
        self._WalkingTrajectory = walkingTrajectory
        
        self._pre_step = 0
        self._first_step = 1
        self._full_step = 0
        self._last_step = 1

    def OnEnter(self):
        pass
        
    def GetId(self):
        return 14

#----------------------------------------------------------------------------------

class StopLeftStepState(StepState):
    """
        The StopLeftStepState class is intended to be used with the StepStateMachine class,
        When the ZMP stops walking on a left step
    """
    def __init__(self,StepStrategy,walkingTrajectory,robotState,dt):
        StepState.__init__(self,"StopLeft",StepStrategy,dt)

        self._WalkingTrajectory = walkingTrajectory
        self._robotState = robotState
        
        self._pre_step = 0
        self._first_step = 0
        self._full_step = 0
        self._last_step = 1

        # const template:
        self._p_ref_const_zero = Constant_Template(0, self._step_time, dt)
        # Stop walking profile:
        self._p_ref_x = Stop_sagital_x(0, self._step_length, self._trans_ratio_of_step, self._trans_slope_steepens_factor, self._step_time, dt)
        self._p_ref_y = Stop_lateral_y_from_left_foot(0, self._step_width, self._trans_ratio_of_step, self._trans_slope_steepens_factor, self._step_time, dt)


    def OnEnter(self):
        rospy.loginfo("Starting STOP LEFT step, time:")
        rospy.loginfo(rospy.get_time())

        self._robotState.Set_step_phase(value = 3)

        #self._Strategy.LoadNewStep(self._p_ref_x_stop, self._p_ref_const_zero, self._p_ref_y_stop, self._p_ref_const_zero)
        
    def GetId(self):
        return 13

#----------------------------------------------------------------------------------

class StopRightStepState(StepState):
    """
        The StopRightStepState class is intended to be used with the StepStateMachine class,
        When the ZMP stops walking on a right step
    """
    def __init__(self,StepStrategy,walkingTrajectory,robotState,dt):
        StepState.__init__(self,"StopRight",StepStrategy,dt)

        self._WalkingTrajectory = walkingTrajectory
        self._robotState = robotState
        
        self._pre_step = 0
        self._first_step = 0
        self._full_step = 0
        self._last_step = 1

        # const template:
        self._p_ref_const_zero = Constant_Template(0, self._step_time, dt)
        # Stop walking profile:
        self._p_ref_x = Stop_sagital_x(0, self._step_length, self._trans_ratio_of_step, self._trans_slope_steepens_factor, self._step_time, dt)
        self._p_ref_y = Stop_lateral_y_from_right_foot(0, self._step_width, self._trans_ratio_of_step, self._trans_slope_steepens_factor, self._step_time, dt)


    def OnEnter(self):
        rospy.loginfo("Starting STOP RIGHT step, time:")
        rospy.loginfo(rospy.get_time())

        self._robotState.Set_step_phase(value = 1)

        #self._Strategy.LoadNewStep(self._p_ref_x_stop, self._p_ref_const_zero, self._p_ref_y_stop, self._p_ref_const_zero)
        
    def GetId(self):
        return 12    

#----------------------------------------------------------------------------------

class PreStoppingLeftStepState(StepState):
    """
        The StoppingLeftStepState class is intended to be used with the StepStateMachine class,
        When the ZMP has to stop while taking a left step
    """
    def __init__(self,StepStrategy,walkingTrajectory,robotState,dt):
        StepState.__init__(self,"PreStoppingLeft",StepStrategy,dt)
        self._WalkingTrajectory = walkingTrajectory

    def OnEnter(self):
        pass

    def GetId(self):
        return 6
    
#----------------------------------------------------------------------------------

class PreStoppingRightStepState(StepState):
    """
        The StoppingRightStepState class is intended to be used with the StepStateMachine class,
        When the ZMP has to stop while taking a right step
    """
    def __init__(self,StepStrategy,walkingTrajectory,robotState,dt):
        StepState.__init__(self,"PreStoppingRight",StepStrategy,dt)
        self._WalkingTrajectory = walkingTrajectory
        
    def OnEnter(self):
        pass
        
    def GetId(self):
        return 7

#----------------------------------------------------------------------------------

class EmergencyStopState(StepState):
    """
        The EmergencyStopState class is intended to be used with the StepStateMachine class,
        When the ZMP has to stop immediately
    """
    def __init__(self,StepStrategy,walkingTrajectory,robotState,dt):
        StepState.__init__(self,"EmergencyStop",StepStrategy,dt)
        self._WalkingTrajectory = walkingTrajectory
        
        self._pre_step = 0
        self._first_step = 0
        self._full_step = 0
        self._last_step = 1

        #TODO
        #self._p_ref_x = 
        #self._p_ref_y = 

    def OnEnter(self):
        pass

    def GetId(self):
        return 15
    
#----------------------------------------------------------------------------------

class StoppingLeftStepState(StepState):
    """
        The StoppingLeftStepState class is intended to be used with the StepStateMachine class,
        When the ZMP has to stop while taking a left step
    """
    def __init__(self,StepStrategy,walkingTrajectory,robotState,dt):
        StepState.__init__(self,"StoppingLeft",StepStrategy,dt)

        self._WalkingTrajectory = walkingTrajectory
        self._robotState = robotState
        
        self._pre_step = 0
        self._first_step = 0
        self._full_step = 1
        self._last_step = 0

      # Stop walking profile:
        self._p_ref_x = Stop_sagital_x(0, self._step_length, self._trans_ratio_of_step, self._trans_slope_steepens_factor, self._step_time, dt)
        self._p_ref_y = Stop_lateral_y_from_left_foot(0, self._step_width, self._trans_ratio_of_step, self._trans_slope_steepens_factor, self._step_time, dt)


    def OnEnter(self):
        rospy.loginfo("Starting STOP LEFT step, time:")
        rospy.loginfo(rospy.get_time())

        self._robotState.Set_step_phase(value = 3)
        #self._Strategy.LoadNewStep(self._p_ref_x_forward_step, r_[self._p_ref_x_forward_step, self._p_ref_x_stop], self._p_ref_y_step_right,r_[self._p_ref_y_step_left, self._p_ref_y_stop])

    def GetId(self):
        return 9
    
#----------------------------------------------------------------------------------

class PreStopLeftStepState(StepState):
    """
        The PreStopLeftStepState class is intended to be used with the StepStateMachine class,
        When the ZMP has to stop
    """
    def __init__(self,StepStrategy,walkingTrajectory,robotState,dt):
        StepState.__init__(self,"PreStopLeft",StepStrategy,dt)

        self._WalkingTrajectory = walkingTrajectory
        self._robotState = robotState
        
        self._pre_step = 0
        self._first_step = 0
        self._full_step = 0
        self._last_step = 1

        # const template:
        self._p_ref_const_zero = Constant_Template(0, self._step_time, dt)
        # Stop walking profile:
        self._p_ref_x = Stop_sagital_x(0, self._step_length, self._trans_ratio_of_step, self._trans_slope_steepens_factor, self._step_time, dt)
        self._p_ref_y = Stop_lateral_y_from_right_foot(0, self._step_width, self._trans_ratio_of_step, self._trans_slope_steepens_factor, self._step_time, dt)


    def OnEnter(self):
        rospy.loginfo("continue STOP LEFT step, time:")
        rospy.loginfo(rospy.get_time())

        self._robotState.Set_step_phase(value = 3)
        #self._Strategy.LoadNewStep(self._p_ref_x_forward_step, r_[self._p_ref_x_stop, self._p_ref_const_zero], self._p_ref_y_step_right,r_[self._p_ref_y_stop, self._p_ref_const_zero])

    def GetId(self):
        return 10
    
#----------------------------------------------------------------------------------

class StoppingRightStepState(StepState):
    """
        The StoppingRightStepState class is intended to be used with the StepStateMachine class,
        When the ZMP has to stop while taking a right step
    """
    def __init__(self,StepStrategy,walkingTrajectory,robotState,dt):
        StepState.__init__(self,"StoppingRight",StepStrategy,dt)

        self._WalkingTrajectory = walkingTrajectory
        self._robotState = robotState
        
        self._pre_step = 0
        self._first_step = 0
        self._full_step = 1
        self._last_step = 0

      # Stop walking profile:
        self._p_ref_x = Stop_sagital_x(0, self._step_length, self._trans_ratio_of_step, self._trans_slope_steepens_factor, self._step_time, dt)
        self._p_ref_y = Stop_lateral_y_from_right_foot(0, self._step_width, self._trans_ratio_of_step, self._trans_slope_steepens_factor, self._step_time, dt)


    def OnEnter(self):
        rospy.loginfo("Starting STOP Right step, time:")
        rospy.loginfo(rospy.get_time())

        self._robotState.Set_step_phase(value = 1)
        #self._Strategy.LoadNewStep(self._p_ref_x_forward_step, r_[self._p_ref_x_forward_step, self._p_ref_x_stop], self._p_ref_y_step_left,r_[self._p_ref_y_step_right, self._p_ref_y_stop])

    def GetId(self):
        return 8
    
#----------------------------------------------------------------------------------

class PreStopRightStepState(StepState):
    """
        The PreStopRightStepState class is intended to be used with the StepStateMachine class,
        When the ZMP has to stop
    """
    def __init__(self,StepStrategy,walkingTrajectory,robotState,dt):
        StepState.__init__(self,"PreStopRight",StepStrategy,dt)

        self._WalkingTrajectory = walkingTrajectory
        self._robotState = robotState
        
        self._pre_step = 0
        self._first_step = 0
        self._full_step = 0
        self._last_step = 1

        # const template:
        self._p_ref_const_zero = Constant_Template(0, self._step_time, dt)
        # Stop walking profile:
        self._p_ref_x = Stop_sagital_x(0, self._step_length, self._trans_ratio_of_step, self._trans_slope_steepens_factor, self._step_time, dt)
        self._p_ref_y = Stop_lateral_y_from_left_foot(0, self._step_width, self._trans_ratio_of_step, self._trans_slope_steepens_factor, self._step_time, dt)


    def OnEnter(self):
        rospy.loginfo("continue STOP RIGHT step, time:")
        rospy.loginfo(rospy.get_time())

        self._robotState.Set_step_phase(value = 1)
        #self._Strategy.LoadNewStep(self._p_ref_x_forward_step, r_[self._p_ref_x_stop, self._p_ref_const_zero], self._p_ref_y_step_left,r_[self._p_ref_y_stop, self._p_ref_const_zero])

    def GetId(self):
        return 11
    
#----------------------------------------------------------------------------------

class RightStepState(StepState):
    """
        The RightStepState class is intended to be used with the StepStateMachine class,
        When the ZMP is taking a right step
    """
    def __init__(self,StepStrategy,walkingTrajectory,robotState,dt):
        StepState.__init__(self,"Right",StepStrategy,dt)
        self._WalkingTrajectory = walkingTrajectory
        self._robotState = robotState

        self._pre_step = 0
        self._first_step = 0
        self._full_step = 1
        self._last_step = 0

        self._p_ref_y = Step_onto_left_foot(0, self._zmp_width, self._trans_ratio_of_step, self._trans_slope_steepens_factor, self._step_time, dt)
        
    def OnEnter(self):
        rospy.loginfo("starting right step")
        self._robotState.Set_step_phase(value = 1)

        #self._Strategy.LoadNewStep(self._p_ref_x_forward_step,self._p_ref_x_forward_step,self._p_ref_y_step_left,r_[ self._p_ref_y_step_right,self._p_ref_y_step_left ])


    def UpdateStepCounter(self,stepCounter):
        stepCounter = stepCounter+1
        return stepCounter
                
    def GetId(self):
        return 5
    
#----------------------------------------------------------------------------------

class LeftStepState(StepState):
    """
        The LeftStepState class is intended to be used with the StepStateMachine class,
        When the ZMP is taking a left step
    """
    def __init__(self,StepStrategy,walkingTrajectory,robotState,dt):
        StepState.__init__(self,"Left",StepStrategy,dt)
        self._WalkingTrajectory = walkingTrajectory
        self._robotState = robotState
        
        self._pre_step = 0
        self._first_step = 0
        self._full_step = 1
        self._last_step = 0

        self._p_ref_y = Step_onto_right_foot(0, self._zmp_width, self._trans_ratio_of_step, self._trans_slope_steepens_factor, self._step_time, dt)
        
    def OnEnter(self):
        rospy.loginfo("starting left step")
        self._robotState.Set_step_phase(value = 3)
        
        #self._Strategy.LoadNewStep(self._p_ref_x_forward_step,self._p_ref_x_forward_step,self._p_ref_y_step_right,r_[ self._p_ref_y_step_left, self._p_ref_y_step_right ])

    def UpdateStepCounter(self,stepCounter):
        stepCounter = stepCounter+1
        return stepCounter
   
    def GetId(self):
        return 4

#----------------------------------------------------------------------------------

class TurnRight_LeftStepState(StepState):
    """
        The TurnRight_LeftStepState class is intended to be used with the StepStateMachine class,
        When the ZMP begins a right turn taking a left step
    """
    def __init__(self,StepStrategy,walkingTrajectory,robotState,dt):
        StepState.__init__(self,"TurnRightBeginLeft",StepStrategy,dt)
        self._WalkingTrajectory = walkingTrajectory
        self._robotState = robotState
        
        self._pre_step = 0
        self._first_step = 0
        self._full_step = 1
        self._last_step = 0

        #TODO
        #self._p_ref_x = 
        #self._p_ref_y = 
        
    def OnEnter(self):
        rospy.loginfo("starting turn right with left step")
        self._robotState.Set_step_phase(value = 3)
        
        #self._Strategy.LoadNewStep(self._p_ref_x_forward_step,self._p_ref_x_forward_step,self._p_ref_y_step_right,r_[ self._p_ref_y_step_left, self._p_ref_y_step_right ])

    def UpdateStepCounter(self,stepCounter):
        stepCounter = stepCounter+1
        return stepCounter
   
    def GetId(self):
        return 16

#----------------------------------------------------------------------------------

class TurnRight_RightStepState(StepState):
    """
        The TurnRight_RightStepState class is intended to be used with the StepStateMachine class,
        When the ZMP finishes a right turn taking a right step
    """
    def __init__(self,StepStrategy,walkingTrajectory,robotState,dt):
        StepState.__init__(self,"TurnRightEndRight",StepStrategy,dt)
        self._WalkingTrajectory = walkingTrajectory
        self._robotState = robotState
        
        self._pre_step = 0
        self._first_step = 0
        self._full_step = 1
        self._last_step = 0

        #TODO
        #self._p_ref_x = 
        #self._p_ref_y = 
        
    def OnEnter(self):
        rospy.loginfo("finishing turn right with right step")
        self._robotState.Set_step_phase(value = 1)
        
        #self._Strategy.LoadNewStep(self._p_ref_x_forward_step,self._p_ref_x_forward_step,self._p_ref_y_step_right,r_[ self._p_ref_y_step_left, self._p_ref_y_step_right ])

    def UpdateStepCounter(self,stepCounter):
        stepCounter = stepCounter+1
        return stepCounter
   
    def GetId(self):
        return 17

###################################################################################
#------------------------------ Preview -------------------------------------------
###################################################################################

class BufferData(object):
    """
        The BufferData class is intended to be used with the StepStateMachine class,
        it contains preview data of StepStateMachine of which steps are going to performed next.
    """
    def __init__(self,strNextTransition,turn_angle,step_length,step_width,zmp_width,step_time,bend_knees,step_height,trans_ratio_of_step):
        self.NextTransition = strNextTransition
        self._turn_angle  = turn_angle   # [rad]
        self._step_length = step_length  # [m]
        self._step_width  = step_width   # [m]
        self._zmp_width   = zmp_width    # [m]
        self._step_time   = step_time    # [sec]
        self._bend_knees  = bend_knees   # [m]     
        self._step_height = step_height  # [m] 
        self._trans_ratio_of_step = trans_ratio_of_step # units fraction: 0-1.0 ; fraction of step time to be used for transition. 1.0 = all of step time is transition

    def GetNextTransition(self):
        return self.NextTransition
###################################################################################
#--------------------------- State Machine ----------------------------------------
###################################################################################

class StepStateMachine(StateMachine):
    """
        The StepStateMachine class is a finite state machine that handles the stepping logic for the ZMP module
    """
    
    def __init__(self,robotState,walkingTrajectory,transformListener,ZMP_Preview_BufferX,ZMP_Preview_BufferY,SwingTrajectory,UpdateRateHz,orientation_correction):
        self._UpdateRateHz = UpdateRateHz
        # Turn Parameters:
        self._MaxStepTurn = 45*math.pi/180 # units [rad], the maximal Turn that can be performed in one step sequance 
        self._TurnThreshold = 0.030        # units [rad], below this value a turn will not be performed 
        self._ZMP_Preview_BufferX = ZMP_Preview_BufferX
        self._ZMP_Preview_BufferY = ZMP_Preview_BufferY
        # Set up strategies
        self._StepStrategyNone = StepStrategyNone(robotState,walkingTrajectory,transformListener,ZMP_Preview_BufferX,ZMP_Preview_BufferY)
        self._StepStrategyWalk = StepStrategyWalk(robotState,walkingTrajectory,transformListener,SwingTrajectory,orientation_correction,ZMP_Preview_BufferX,ZMP_Preview_BufferY)
        # Set up the actual state machine
        # Define the Idle state as the default state
        dt = 1.0/self._UpdateRateHz
        StateMachine.__init__(self,IdleStepState(self._StepStrategyNone,dt))
        # Add states
        StateMachine.AddState(self,InitializeStepState(self._StepStrategyNone,dt))
        StateMachine.AddState(self,FailureStepState(self._StepStrategyNone,dt))
        StateMachine.AddState(self,PreStepState(self._StepStrategyWalk,robotState,dt))
        StateMachine.AddState(self,FirstStepState(self._StepStrategyWalk,dt))
        StateMachine.AddState(self,PreStoppingFirstStepState(self._StepStrategyWalk,walkingTrajectory,robotState,dt))
        StateMachine.AddState(self,PreStoppingLeftStepState(self._StepStrategyWalk,walkingTrajectory,robotState,dt))
        StateMachine.AddState(self,PreStoppingRightStepState(self._StepStrategyWalk,walkingTrajectory,robotState,dt))
        StateMachine.AddState(self,StoppingLeftStepState(self._StepStrategyWalk,walkingTrajectory,robotState,dt))
        StateMachine.AddState(self,StoppingRightStepState(self._StepStrategyWalk,walkingTrajectory,robotState,dt))
        StateMachine.AddState(self,StopLeftStepState(self._StepStrategyWalk,walkingTrajectory,robotState,dt))
        StateMachine.AddState(self,StopRightStepState(self._StepStrategyWalk,walkingTrajectory,robotState,dt))
        StateMachine.AddState(self,PreStopLeftStepState(self._StepStrategyWalk,walkingTrajectory,robotState,dt))
        StateMachine.AddState(self,PreStopRightStepState(self._StepStrategyWalk,walkingTrajectory,robotState,dt))
        StateMachine.AddState(self,RightStepState(self._StepStrategyWalk,walkingTrajectory,robotState,dt))
        StateMachine.AddState(self,LeftStepState(self._StepStrategyWalk,walkingTrajectory,robotState,dt))
        StateMachine.AddState(self,EmergencyStopState(self._StepStrategyWalk,walkingTrajectory,robotState,dt))
        StateMachine.AddState(self,TurnRight_LeftStepState(self._StepStrategyWalk,walkingTrajectory,robotState,dt))
        StateMachine.AddState(self,TurnRight_RightStepState(self._StepStrategyWalk,walkingTrajectory,robotState,dt))
        # Add transitions
        StateMachine.AddTransition(self,"Initializing",         "Fail",         "Failing")
        StateMachine.AddTransition(self,"Initializing",         "Start",        "PreStep")
        #StateMachine.AddTransition(self,"Initializing",         "Stop",         "Idle")
        #StateMachine.AddTransition(self,"Failing",              "Stop",         "Idle")
        StateMachine.AddTransition(self,"PreStep",              "NextStep",     "FirstStep")
        StateMachine.AddTransition(self,"FirstStep",            "NextStep",     "Left")
        StateMachine.AddTransition(self,"FirstStep",            "Stop",         "PreStoppingFirstStep")
        StateMachine.AddTransition(self,"Left",                 "NextStep",     "Right")
        StateMachine.AddTransition(self,"Left",                 "Stop",         "PreStoppingLeft")
        StateMachine.AddTransition(self,"Left",                 "EmergencyStop","EmergencyStop")
        StateMachine.AddTransition(self,"Right",                "NextStep",     "Left")
        StateMachine.AddTransition(self,"Right",                "Stop",         "PreStoppingRight")
        StateMachine.AddTransition(self,"Right",                "EmergencyStop","EmergencyStop")
        StateMachine.AddTransition(self,"PreStoppingFirstStep", "NextStep",     "StoppingLeft")
        StateMachine.AddTransition(self,"PreStoppingRight",     "NextStep",     "StoppingLeft")
        StateMachine.AddTransition(self,"PreStoppingLeft",      "NextStep",     "StoppingRight")
        StateMachine.AddTransition(self,"StoppingLeft",         "NextStep",     "PreStopRight")
        StateMachine.AddTransition(self,"StoppingRight",        "NextStep",     "PreStopLeft")
        StateMachine.AddTransition(self,"PreStopRight",         "NextStep",     "StopLeft")
        StateMachine.AddTransition(self,"PreStopLeft",          "NextStep",     "StopRight")
        StateMachine.AddTransition(self,"StopRight",            "NextStep",     "Idle")
        StateMachine.AddTransition(self,"StopLeft",             "NextStep",     "Idle")
        StateMachine.AddTransition(self,"EmergencyStop",        "NextStep",     "Idle")
        StateMachine.AddTransition(self,"FirstStep",            "TurnRight",     "TurnRightBeginLeft")
        StateMachine.AddTransition(self,"Right",                "TurnRight",     "TurnRightBeginLeft")
        StateMachine.AddTransition(self,"TurnRightBeginLeft",   "NextStep",      "TurnRightEndRight")
        StateMachine.AddTransition(self,"TurnRightEndRight",    "TurnRight",     "TurnRightBeginLeft")
        StateMachine.AddTransition(self,"TurnRightEndRight",    "NextStep",      "Left")

        # Values at init: (to enable reset of init values without recreating object)
        # self.init_values() # commented because done externaly (at begining of task)
    
    def init_values(self):
        self._counter = 0
        self._stepCounter = 0
        # Turn Variables:
        self._TurnCmdInput = 0.0        # units [rad], Input Turn command recieved by StepStateMachine, angle signs: (+) turn left, (-) turn right
        self._ExecutedTurnCmd = 0.0     # units [rad], Turn command that is being carried out
        self._TotalTurnRemaining = 0.0   # units [rad], total turn remaining to perform
        self._DistanceToNextTurn = 0.5
        # Preview of State Machine:
        self._StatePreviewBuffer = deque([]) # contains a list of BufferData class, infomation needed from _CurrentState to _PreviewState: Transitions, step times,...
        self._PreviewState = StateMachine.GetCurrentState(self) # the state at which the preview is 

    def Initialize(self):
        self.init_values()
        self._CurrentState.OnExit()
        self._CurrentState = self._States["Initializing"]
        self._CurrentState.OnEnter()

        # init _PreviewState and _StatePreviewBuffer:
        self._PreviewState = self._States["Initializing"] # reset to "Initializing" state
        self._PromotePreviewState("Start") # "Initializing" to "PreStep"
        self._PromotePreviewState("NextStep") # "PreStep" to "FirstStep"
        self._PromotePreviewState(self._GetPreviewTransition()) # "FirstStep" to wherever transition sends us
        #rospy.loginfo("class StepStateMachine init_values: PreviewState = %s" % (self._PreviewState.Name) )
        ##DEBUG
        #self._ZMP_Preview_BufferX.Debug_PlotBuffer()
        #self._ZMP_Preview_BufferY.Debug_PlotBuffer()  

    def Fail(self):
        StateMachine.PerformTransition(self,"Fail")

    def Start(self):
        if(StateMachine.PerformTransition(self,"Start")):
            self._counter = 0

    def NextStep(self):
        StateMachine.PerformTransition(self,"NextStep")
        self._counter = 0

    def TurnLeft(self):
        StateMachine.PerformTransition(self,"TurnLeft")
        self._counter = 0

    def TurnRight(self):
        StateMachine.PerformTransition(self,"TurnRight")
        self._counter = 0

    def _PerformWalkTransition(self,transition):
        StateMachine.PerformTransition(self,transition)
        self._counter = 0

    def Stop(self):
        StateMachine.PerformTransition(self,"Stop")

    def EmergencyStop(self):
        StateMachine.PerformTransition(self,"EmergencyStop")
            
    def CalculateFootSwingTrajectory(self):
        StateMachine.GetCurrentState(self).CalculateFootSwingTrajectory(1.0/self._UpdateRateHz,self._counter)
        
    def GetWalkingTrajectory(self,COMx, COMx_dot, p_pre_con_x,COMy, COMy_dot, p_pre_con_y,p_ref_x,p_ref_y,Des_Orientation,imu_orientation):
        return StateMachine.GetCurrentState(self).GetWalkingTrajectory(COMx, COMx_dot, p_pre_con_x,COMy, COMy_dot, p_pre_con_y,p_ref_x,p_ref_y,Des_Orientation,imu_orientation,self._counter,1.0/self._UpdateRateHz)
    
    def UpdatePreview(self):
        #rospy.loginfo("UpdatePreview - StepTime(%f) Counter(%d) Timer(%f)" % (StateMachine.GetCurrentState(self).GetStepTime(),self._counter, self._counter*1.0/self._UpdateRateHz))
        if (StateMachine.GetCurrentState(self).GetStepTime() <= self._counter*1.0/self._UpdateRateHz):
            self._stepCounter = StateMachine.GetCurrentState(self).UpdateStepCounter(self._stepCounter)
            rospy.loginfo("done step number = %d" % (self._stepCounter))
            rospy.loginfo("time:")
            rospy.loginfo(rospy.get_time())

            self._StatePreviewBuffer.popleft() # remove data of last step (that was just completed)
            self._PromotePreviewState( self._GetPreviewTransition() ) # load preview of new step. Updates: turn variables, _PreviewState, preview with ZMP profile and _StatePreviewBuffer  
            self._PerformWalkTransition( self._StatePreviewBuffer[0].GetNextTransition() ) # update _CurrentState from oldest preview data
            # # TODO: Transition need to be received from StepStatePreviewBuffer
            # # End of step -> State Transitions:
            # if self._DecideToTurn():
            #     if self._ExecutedTurnCmd > 0:
            #         self.TurnLeft()
            #     else:
            #         self.TurnRight()
            # else:
            #     self.NextStep()
            #self.NextStep()
            #p_ref_x,p_ref_y,loaded_new_step_trigger_x,loaded_new_step_trigger_y = StateMachine.GetCurrentState(self).UpdatePreview()
            p_ref_x,loaded_new_step_trigger_x = self._ZMP_Preview_BufferX.update_Preview()
            p_ref_y,loaded_new_step_trigger_y = self._ZMP_Preview_BufferY.update_Preview()
            #rospy.loginfo("StepStateMachine UpdatePreview: _counter = %f, p_ref_x[0] = %f, p_ref_y[0] = %f" % (self._counter, p_ref_x[0], p_ref_y[0]) )                   
        else:
            #p_ref_x,p_ref_y,loaded_new_step_trigger_x,loaded_new_step_trigger_y = StateMachine.GetCurrentState(self).UpdatePreview()
            p_ref_x,loaded_new_step_trigger_x = self._ZMP_Preview_BufferX.update_Preview()
            p_ref_y,loaded_new_step_trigger_y = self._ZMP_Preview_BufferY.update_Preview()
            # if (StateMachine.GetCurrentState(self).GetStepTime() <= (self._counter+2)*1.0/self._UpdateRateHz):
            #     rospy.loginfo("StepStateMachine UpdatePreview: _counter = %f, p_ref_x[0] = %f, p_ref_y[0] = %f" % (self._counter, p_ref_x[0], p_ref_y[0]) )
        self._counter = self._counter+1
        return p_ref_x,p_ref_y,loaded_new_step_trigger_x,loaded_new_step_trigger_y  
    
    def GetStateId(self):
        return StateMachine.GetCurrentState(self).GetId()

    def SetTurnCmd(self, turn_cmd):
        self._TurnCmdInput = turn_cmd

    def SetDistanceToNextTurn(self, distance):
        self._DistanceToNextTurn = distance

    def GetTurnDirection(self):
        turn = "TurnLeft"
        if (0.0>self._ExecutedTurnCmd):
            turn = "TurnRight"
        return turn

    def _DecideToTurn(self):
        """
          Function returns True if _PreviewState needs to perform a TURN (received from Input turn cmd)
          else return False. 
          The needs to perform a TURN is dependent on the turn variables which are updated as follows:
          1) A NEW turn command is excuted only after completing previous turning command.
          2) A turn command may take a few turn step cycles (for larg turn commands).
          3) A turn will be performed only above a certain threshold (_TurnThreshold).
        """
        result = False
        # 1) if completed previous turn command Update from Turn Input command:
        if ( abs(self._TotalTurnRemaining) < self._TurnThreshold ):
            if ( abs(self._TurnCmdInput) >= self._TurnThreshold ):
                self._TotalTurnRemaining = copy.copy(self._TurnCmdInput)
            else:
                self._TotalTurnRemaining = 0.0
        # sign of turn:
        if self._TotalTurnRemaining > 0.0:
            turn_sign = 1 # left turn
        else:
            turn_sign = -1 # right turn
        # 2) divid "big" Turn into a few steps: 
        if abs(self._TotalTurnRemaining) >= self._MaxStepTurn:
            # turn needs more than one turn step seq.
            self._ExecutedTurnCmd = self._MaxStepTurn*turn_sign
            self._TotalTurnRemaining = self._TotalTurnRemaining - self._ExecutedTurnCmd
        else:
            self._ExecutedTurnCmd = copy.copy(self._TotalTurnRemaining)
            self._TotalTurnRemaining = 0.0
        # 3) Check if need to turn:
        if ( abs(self._ExecutedTurnCmd) >= self._TurnThreshold ):
            result = True
        return result

    def _GetPreviewTransition(self):
        """
          Function return transition string of turn direction or "NextStep" transition according to 
          the need to turn and if transition is viable from _PreviewState.
        """
        if self._DecideToTurn():
            turn_dir = self.GetTurnDirection() # returns: "TurnRight" or "TurnLeft"
            state,transition_exists = StateMachine.GetStateAtTransition(self,self._PreviewState.Name,turn_dir)
            if transition_exists: # !!! "TurnLeft" can only start after Left Step and "TurnRight" can only start after Right Step !!!
                transition = turn_dir
            else:
                transition = "NextStep" # Wrong turning foot
        else:
            transition = "NextStep" # No turn
        return transition

    def _PromotePreviewState(self,transition):
        """
          1) Performs transition of _PreviewState to transition state when we know that transition exists.
             (logic to check if transition exist and which transition to perform is done before calling this function )
          2) Loads preview with ZMP profile of new _PreviewState. 
        """
        state,transition_exists = StateMachine.GetStateAtTransition(self,self._PreviewState.Name,transition)
        if transition_exists:
            self._PreviewState = state
            StepData = BufferData(transition,state._turn_angle,state._step_length,state._step_width,state._zmp_width,state._step_time,state._bend_knees,state._step_height,state._trans_ratio_of_step)
            self._StatePreviewBuffer.append(StepData)
            rospy.loginfo("StepStateMachine _PromotePreviewState: _StatePreviewBuffer NextTransition= %s" % ( self._StatePreviewBuffer[-1].GetNextTransition() ))
            self._ZMP_Preview_BufferX.load_PreviewStep(self._PreviewState.GetZmpProfileX())
            self._ZMP_Preview_BufferY.load_PreviewStep(self._PreviewState.GetZmpProfileY())
        else:
            rospy.loginfo("StepStateMachine _PromotePreviewState: ERROR state transition doesn't exist")
        

###################################################################################
# a little testing script
###################################################################################

if __name__ == "__main__":
    Machine = StepStateMachine()
    print Machine.GetCurrentState().Name
    Machine.Initialize()
    print Machine.GetCurrentState().Name
    Machine.Fail()
    print Machine.GetCurrentState().Name
    Machine.Initialize()
    print Machine.GetCurrentState().Name
    Machine.Start()
    print Machine.GetCurrentState().Name
    Machine.NextStep()
    print Machine.GetCurrentState().Name
    Machine.NextStep()
    print Machine.GetCurrentState().Name
    Machine.NextStep()
    print Machine.GetCurrentState().Name
    Machine.NextStep()
    print Machine.GetCurrentState().Name
    Machine.Stop()
    print Machine.GetCurrentState().Name
    Machine.NextStep()
    print Machine.GetCurrentState().Name
    Machine.NextStep()
    print Machine.GetCurrentState().Name
    Machine.NextStep()
    print Machine.GetCurrentState().Name


