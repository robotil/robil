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
        self._step_length = 0.12 #0.01  # [m]
        self._step_width  = 0.182 #0.182#0.178  # 0.178  # [m]
        self._zmp_width   =  0.178 #0.178 #0.120  #0.02 # #0.168 #when lift foot 0.130, feet on ground 0.110
        self._step_time   = 1.2 #1   # [sec]
        self._bend_knees  = 0.05 #0.12 #0.18 #0.04  # [m]     
        self._step_height = 0.035 #0.035 #0.05 #0.03 #0.05  # [m] 
        self._trans_ratio_of_step = 1.0 #0.9 #1.0 #0.8 # units fraction: 0-1.0 ; fraction of step time to be used for transition. 1.0 = all of step time is transition
        self._trans_slope_steepens_factor = 8/self._step_time #2 # 1 transition Sigmoid slope (a)

        self._k_total = self._step_time*(1/dt)
        self._k_start_swing =  floor(self._k_total/3)
        self._k_stop_swing =   floor(2*self._k_total/3)

        # 2) preceeding steps (full_step)
        self._p_ref_x_forward_step = Step_forward_x(0, self._step_length, self._trans_ratio_of_step, self._trans_slope_steepens_factor, self._step_time, dt)
        self._p_ref_y_step_right = Step_onto_right_foot(0, self._zmp_width, self._trans_ratio_of_step, self._trans_slope_steepens_factor, self._step_time, dt)
        self._p_ref_y_step_left = Step_onto_left_foot(0, self._zmp_width, self._trans_ratio_of_step, self._trans_slope_steepens_factor, self._step_time, dt)
        
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
        # 1) to start walking (pre_step + first_step)
        self._p_ref_x_start = Start_sagital_x(0, self._step_length, self._trans_ratio_of_step, self._trans_slope_steepens_factor, self._step_time, dt)
        self._p_ref_y_start = Start_lateral_y_weight_to_left_foot(0, self._zmp_width, self._trans_ratio_of_step, self._trans_slope_steepens_factor, self._step_time, dt)

    def OnEnter(self):
        rospy.loginfo("started walking")
        rospy.loginfo("time:")
        rospy.loginfo(rospy.get_time())

        self._robotState.Set_step_phase(value = 1)                                                              # added by Israel to initiate step phase 24.2
          
        self._Strategy.LoadNewStep(self._p_ref_x_start,self._p_ref_x_forward_step,self._p_ref_y_start,r_[ self._p_ref_y_step_right, self._p_ref_y_step_left ])
        
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
        self._p_ref_x_stop = Stop_sagital_x(0, self._step_length, self._trans_ratio_of_step, self._trans_slope_steepens_factor, self._step_time, dt)
        self._p_ref_y_stop = Stop_lateral_y_from_left_foot(0, self._step_width, self._trans_ratio_of_step, self._trans_slope_steepens_factor, self._step_time, dt)


    def OnEnter(self):
        rospy.loginfo("Starting STOP LEFT step, time:")
        rospy.loginfo(rospy.get_time())

        self._robotState.Set_step_phase(value = 3)

        self._Strategy.LoadNewStep(self._p_ref_x_stop, self._p_ref_const_zero, self._p_ref_y_stop, self._p_ref_const_zero)
        
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
        self._p_ref_x_stop = Stop_sagital_x(0, self._step_length, self._trans_ratio_of_step, self._trans_slope_steepens_factor, self._step_time, dt)
        self._p_ref_y_stop = Stop_lateral_y_from_right_foot(0, self._step_width, self._trans_ratio_of_step, self._trans_slope_steepens_factor, self._step_time, dt)


    def OnEnter(self):
        rospy.loginfo("Starting STOP RIGHT step, time:")
        rospy.loginfo(rospy.get_time())

        self._robotState.Set_step_phase(value = 1)

        self._Strategy.LoadNewStep(self._p_ref_x_stop, self._p_ref_const_zero, self._p_ref_y_stop, self._p_ref_const_zero)
        
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
        self._p_ref_x_stop = Stop_sagital_x(0, self._step_length, self._trans_ratio_of_step, self._trans_slope_steepens_factor, self._step_time, dt)
        self._p_ref_y_stop = Stop_lateral_y_from_left_foot(0, self._step_width, self._trans_ratio_of_step, self._trans_slope_steepens_factor, self._step_time, dt)


    def OnEnter(self):
        rospy.loginfo("Starting STOP LEFT step, time:")
        rospy.loginfo(rospy.get_time())

        self._robotState.Set_step_phase(value = 3)
        self._Strategy.LoadNewStep(self._p_ref_x_forward_step, r_[self._p_ref_x_forward_step, self._p_ref_x_stop], self._p_ref_y_step_right,r_[self._p_ref_y_step_left, self._p_ref_y_stop])

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
        self._p_ref_x_stop = Stop_sagital_x(0, self._step_length, self._trans_ratio_of_step, self._trans_slope_steepens_factor, self._step_time, dt)
        self._p_ref_y_stop = Stop_lateral_y_from_right_foot(0, self._step_width, self._trans_ratio_of_step, self._trans_slope_steepens_factor, self._step_time, dt)


    def OnEnter(self):
        rospy.loginfo("continue STOP LEFT step, time:")
        rospy.loginfo(rospy.get_time())

        self._robotState.Set_step_phase(value = 3)
        self._Strategy.LoadNewStep(self._p_ref_x_forward_step, r_[self._p_ref_x_stop, self._p_ref_const_zero], self._p_ref_y_step_right,r_[self._p_ref_y_stop, self._p_ref_const_zero])

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
        self._p_ref_x_stop = Stop_sagital_x(0, self._step_length, self._trans_ratio_of_step, self._trans_slope_steepens_factor, self._step_time, dt)
        self._p_ref_y_stop = Stop_lateral_y_from_right_foot(0, self._step_width, self._trans_ratio_of_step, self._trans_slope_steepens_factor, self._step_time, dt)


    def OnEnter(self):
        rospy.loginfo("Starting STOP Right step, time:")
        rospy.loginfo(rospy.get_time())

        self._robotState.Set_step_phase(value = 1)
        self._Strategy.LoadNewStep(self._p_ref_x_forward_step, r_[self._p_ref_x_forward_step, self._p_ref_x_stop], self._p_ref_y_step_left,r_[self._p_ref_y_step_right, self._p_ref_y_stop])

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
        self._p_ref_x_stop = Stop_sagital_x(0, self._step_length, self._trans_ratio_of_step, self._trans_slope_steepens_factor, self._step_time, dt)
        self._p_ref_y_stop = Stop_lateral_y_from_left_foot(0, self._step_width, self._trans_ratio_of_step, self._trans_slope_steepens_factor, self._step_time, dt)


    def OnEnter(self):
        rospy.loginfo("continue STOP RIGHT step, time:")
        rospy.loginfo(rospy.get_time())

        self._robotState.Set_step_phase(value = 1)
        self._Strategy.LoadNewStep(self._p_ref_x_forward_step, r_[self._p_ref_x_stop, self._p_ref_const_zero], self._p_ref_y_step_left,r_[self._p_ref_y_stop, self._p_ref_const_zero])

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
        
    def OnEnter(self):
        rospy.loginfo("starting right step")
        self._robotState.Set_step_phase(value = 1)

        self._Strategy.LoadNewStep(self._p_ref_x_forward_step,self._p_ref_x_forward_step,self._p_ref_y_step_left,r_[ self._p_ref_y_step_right,self._p_ref_y_step_left ])


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
        
    def OnEnter(self):
        rospy.loginfo("starting left step")
        self._robotState.Set_step_phase(value = 3)
        
        self._Strategy.LoadNewStep(self._p_ref_x_forward_step,self._p_ref_x_forward_step,self._p_ref_y_step_right,r_[ self._p_ref_y_step_left, self._p_ref_y_step_right ])

    def UpdateStepCounter(self,stepCounter):
        stepCounter = stepCounter+1
        return stepCounter
   
    def GetId(self):
        return 4

###################################################################################
#--------------------------- State Machine ----------------------------------------
###################################################################################

class StepStateMachine(StateMachine):
    """
        The StepStateMachine class is a finite state machine that handles the stepping logic for the ZMP module
    """
    
    def __init__(self,robotState,walkingTrajectory,transformListener,ZMP_Preview_BufferX,ZMP_Preview_BufferY,SwingTrajectory,UpdateRateHz,orientation_correction):
        self._counter = 0
        self._stepCounter = 0
        self._UpdateRateHz = UpdateRateHz
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
        # Add transitions
        StateMachine.AddTransition(self,"Idle",                 "Initialize",   "Initializing")
        StateMachine.AddTransition(self,"Initializing",         "Fail",         "Failing")
        StateMachine.AddTransition(self,"Initializing",         "Start",        "PreStep")
        #StateMachine.AddTransition(self,"Initializing",         "Stop",         "Idle")
        StateMachine.AddTransition(self,"Failing",              "Initialize",   "Initializing")
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

    def Initialize(self):
        StateMachine.PerformTransition(self,"Initialize")
        self._stepCounter = 0
        self._counter = 0

    def Fail(self):
        StateMachine.PerformTransition(self,"Fail")

    def Start(self):
        if(StateMachine.PerformTransition(self,"Start")):
            self._counter = 0

    def NextStep(self):
        StateMachine.PerformTransition(self,"NextStep")
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
        if (StateMachine.GetCurrentState(self).GetStepTime() < self._counter*1.0/self._UpdateRateHz):
            self.NextStep()
            p_ref_x,p_ref_y,loaded_new_step_trigger_x,loaded_new_step_trigger_y = StateMachine.GetCurrentState(self).UpdatePreview()
            self._stepCounter = StateMachine.GetCurrentState(self).UpdateStepCounter(self._stepCounter)
            rospy.loginfo("done step number = %d" % (self._stepCounter))
            rospy.loginfo("time:")
            rospy.loginfo(rospy.get_time())
        else:
            p_ref_x,p_ref_y,loaded_new_step_trigger_x,loaded_new_step_trigger_y = StateMachine.GetCurrentState(self).UpdatePreview()

        self._counter = self._counter+1
        return p_ref_x,p_ref_y,loaded_new_step_trigger_x,loaded_new_step_trigger_y  
    def GetStateId(self):
        return StateMachine.GetCurrentState(self).GetId()

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


