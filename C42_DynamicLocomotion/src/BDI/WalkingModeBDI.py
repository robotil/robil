#!/usr/bin/env python
import roslib
roslib.load_manifest('C42_DynamicLocomotion')
from Abstractions.WalkingMode import *
import time
from atlas_msgs.msg import AtlasCommand, AtlasSimInterfaceCommand, AtlasSimInterfaceState, AtlasState, AtlasBehaviorStepData
from sensor_msgs.msg import Imu
import PyKDL
from tf_conversions import posemath
from atlas_msgs.msg import AtlasSimInterfaceCommand, AtlasSimInterfaceState, AtlasState
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from sensor_msgs.msg import Imu

from BDI_Odometer import *
from BDI_StateMachine import *

import math
import rospy
import sys
import copy

from nav_msgs.msg import Odometry

from geometry_msgs.msg import Pose
from BDI_Strategies import *
from Abstractions.StepQueue import *
from LocalPathPlanner import FootPlacement


###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

class WalkingModeBDI(WalkingMode):
    def __init__(self,localPathPlanner):
        WalkingMode.__init__(self,localPathPlanner)
        self.step_index_for_reset = 0
        # Initialize atlas mode and atlas_sim_interface_command publishers        
        self.asi_command = rospy.Publisher('/atlas/atlas_sim_interface_command', AtlasSimInterfaceCommand, None, False, True, None)
        self._Odometer = BDI_Odometer()
        ##############################
        #self._StrategyForward = BDI_StrategyForward(self._Odometer)
        self._stepDataInit = BDI_Strategy(self._Odometer)
        self._StepQueue = StepQueue()
        self._SteppingStonesQueue = PathQueue()
        self._SteppingStonesPath = []
        self._setStep = True 
        self._passedSteppingStones = False
        ##############################
        self._BDI_StateMachine = BDI_StateMachine(self._Odometer)
        #self._odom_position = Pose()
        self._odom_sub = rospy.Subscriber('/ground_truth_odom',Odometry,self._odom_cb)
        self._bDone = False
        self.asi_state = rospy.Subscriber('/atlas/atlas_sim_interface_state', AtlasSimInterfaceState, self.asi_state_cb)
        self._atlas_imu_sub = rospy.Subscriber('/atlas/imu', Imu, self._get_imu)
        rospy.sleep(0.3)

    def Initialize(self):
        WalkingMode.Initialize(self)
        # Subscriber
        self._path_sub = rospy.Subscriber('/path',C31_Waypoints,self._path_cb)
    
        self._bDone = False
        # # Puts robot into freeze behavior, all joints controlled
        # # Put the robot into a known state
        k_effort = [0] * 28
        # freeze = AtlasSimInterfaceCommand(None,AtlasSimInterfaceCommand.FREEZE, None, None, None, None, k_effort )
        # self.asi_command.publish(freeze)
        
        # # Puts robot into stand_prep behavior, a joint configuration suitable
        # # to go into stand mode
        # stand_prep = AtlasSimInterfaceCommand(None,AtlasSimInterfaceCommand.STAND_PREP, None, None, None, None, k_effort)
        # self.asi_command.publish(stand_prep)

        # rospy.sleep(2.0)
        # self.mode.publish("nominal")
        
        # Put robot into stand position
        stand = AtlasSimInterfaceCommand(None,AtlasSimInterfaceCommand.STAND, None, None, None, None, k_effort)
                
        rospy.sleep(0.3)
        
        self.asi_command.publish(stand)
        
        # Initialize some variables before starting.

        self.is_swaying = False
        self._BDI_StateMachine.Initialize(self.step_index_for_reset)
    
    def StartWalking(self):
        self._bDone = False
        return 0.3
    
    def Walk(self):
        WalkingMode.Walk(self)
        self._yaw = 0
        self._Odometer.SetYaw(self._yaw)
    
    def Stop(self):
        if(WalkingMode.Stop(self)):
            self._BDI_StateMachine.Stop()
        
    
    def EmergencyStop(self):
        # Puts robot into freeze behavior, all joints controlled
        # Put the robot into a known state
        k_effort = [0] * 28
        # freeze = AtlasSimInterfaceCommand(None,AtlasSimInterfaceCommand.FREEZE, None, None, None, None, k_effort )
        # self.asi_command.publish(freeze)
        # rospy.sleep(0.3)
        
        # Put robot into stand position
        stand = AtlasSimInterfaceCommand(None,AtlasSimInterfaceCommand.STAND, None, None, None, None, k_effort)

    def IsDone(self):
        return self._bDone

###################################################################################
#--------------------------- CallBacks --------------------------------------------
###################################################################################

    def _path_cb(self,path):
        rospy.loginfo('got path %s',path)
        p = []
        for wp in path.points:
            p.append(Waypoint(wp.x,wp.y))
        self.SetPath(p)


    # /atlas/atlas_sim_interface_state callback. Before publishing a walk command, we need
    # the current robot position   
    def asi_state_cb(self, state):
        if self._LPP.GetDoingQual() and (16.2 < self._LPP.GetPos().GetX()) and (self._LPP.GetPos().GetX() < 19.7) and not self._passedSteppingStones:
            if (True == self._setStep): # Initialize before stepping stones
                self.step_index = state.walk_feedback.next_step_index_needed -1
                step1,step2,step3,step4 = self.initSteppingStoneStepData(self.step_index, state)
                self._StepQueue.Initialize(step1,step2,step3,step4)
                self.GetSteppingStoneStepPlan(state)
                print("asi_state_cb:: SteppingStonesQueue length: ",self._SteppingStonesQueue.Length() )
                self._setStep = False
            command = self.SteppingStoneCommand(state);
            #print(command)
        else:
            if self._LPP.GetDoingQual() and (False == self._setStep): # Initialize after stepping stones
                self._passedSteppingStones = True
                self._Odometer.SetPosition(state.pos_est.position.x,state.pos_est.position.y)
                print("asi_state_cb:: Initialize after stepping stones. Odometer X,Y: ",self._Odometer.GetGlobalPosition() )       
                #self.Initialize()
                print("asi_state_cb:: Initialize after stepping stones. step index: ",self.step_index )
                WalkingMode.Initialize(self)
                self._BDI_StateMachine.Initialize(self.step_index)
                # self.step_index = state.behavior_feedback.walk_feedback.next_step_index_needed -1
                # step1,step2,step3,step4 = self.initSteppingStoneStepData(self.step_index, state)

            command = self.HandleStateMsg(state)
            self._bDone = self._WalkingModeStateMachine.IsDone()
            ######################
            self._setStep = True

        # command = self._StateMachine.HandleStateMsg(state)
        # self._bDone = self._StateMachine.IsDone()
        if (0 !=command):
            self.asi_command.publish(command)

    def _odom_cb(self,odom):
        # SHOULD USE:
        self._LPP.UpdatePosition(odom.pose.pose.position.x,odom.pose.pose.position.y)
        #self._odom_position = odom.pose.pose
 
    def _get_imu(self,msg):  #listen to /atlas/imu/pose/pose/orientation
        roll, pitch, self._yaw = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])


###############################################################################################


    def SteppingStoneCommand(self,state):
        command = 0 
        nextindex = state.walk_feedback.next_step_index_needed
        if (nextindex>self.step_index):
            self.step_index += 1
            command = self.GetCommand()
            # Instead of generating new steps, just pop a predefined queue
            stepData = self._StepQueue.Push(self._SteppingStonesQueue.Pop())
            #print("SteppingStoneCommand:: SteppingStonesQueue length: ",self._SteppingStonesQueue.Length() )
            #print("SteppingStoneCommand:: index: ",self.step_index)
        return command

    # def SteppingStoneCommandStatic(self, state):
        
    #     # When the robot status_flags are 1 (SWAYING), you can publish the next step command.
    #     if (state.behavior_feedback.step_feedback.status_flags == 1 and self._setStep):
    #         self.step_index += 1
    #         self._setStep = False
    #         print("Step " + str(self.step_index))
    #     elif (state.behavior_feedback.step_feedback.status_flags == 2):
    #         self._setStep = True
    #         print("No Step")
        
    #     is_right_foot = self.step_index % 2
        
    #     command = AtlasSimInterfaceCommand()
    #     command.behavior = AtlasSimInterfaceCommand.STEP

    #     # k_effort is all 0s for full bdi control of all joints
    #     command.k_effort = [0] * 28
        
    #     # step_index should always be one for a step command
    #     command.step_params.desired_step.step_index = 1
    #     command.step_params.desired_step.foot_index = is_right_foot
        
    #     # duration has far as I can tell is not observed
    #     command.step_params.desired_step.duration = 0.63
        
    #     # swing_height is not observed
    #     command.step_params.desired_step.swing_height = 0.1

    #     #if self.step_index > 30:
    #         #print(str(self.calculate_pose(self.step_index)))
    #     # Determine pose of the next step based on the number of steps we have taken
    #     command.step_params.desired_step.pose = self.calculate_pose(self.step_index,state)
        
    #     return command

    # # This method is used to calculate a pose of step based on the step_index
    # # The step poses just cause the robot to walk in a circle
    # def calculate_pose(self, step_index,state):
    #     # Right foot occurs on even steps, left on odd
    #     is_right_foot = step_index % 2
    #     is_left_foot = 1 - is_right_foot
                      
    #     # Calculate orientation quaternion
    #     Q = quaternion_from_euler(0, 0, 0)
    #     pose = Pose()
    #     pose.position.x = state.pos_est.position.x - 3
    #     pose.position.y = state.pos_est.position.y + 0.15*is_left_foot
        
    #     # The z position is observed for static walking, but the foot
    #     # will be placed onto the ground if the ground is lower than z
    #     pose.position.z = 0
        
    #     pose.orientation.x = Q[0]
    #     pose.orientation.y = Q[1]
    #     pose.orientation.z = Q[2]
    #     pose.orientation.w = Q[3]

    #     return pose

    def GetCommand(self):
        command = AtlasSimInterfaceCommand()
        command.behavior = AtlasSimInterfaceCommand.WALK
        for i in range(4):
            command.walk_params.step_queue[i] = self._StepQueue.Peek(i)
        return command

    def initSteppingStoneStepData(self,step_index,state):
        print("initSteppingStoneStepData - index need to match:: cmd index = ",step_index,"state first index = ",state.walk_feedback.step_queue_saturated[0].step_index)
        stepData1 = copy.deepcopy(state.walk_feedback.step_queue_saturated[0])
        stepData2 = copy.deepcopy(state.walk_feedback.step_queue_saturated[1])
        stepData3 = copy.deepcopy(stepData2)
        stepData4 = copy.deepcopy(stepData2)

        # corrections to step 3 and 4: y position foot width corection +  ?alinement with stones?
        step_width = 0.25 # [meters] 
        stepData3.step_index = step_index + 2
        stepData3.foot_index = stepData3.step_index%2
        stepData4.step_index = step_index + 3
        if 0 == stepData3.foot_index: # Left foot
            stepData3.pose.position.y = stepData2.pose.position.y + step_width 
        else: # Right foot
            stepData3.pose.position.y = stepData2.pose.position.y - step_width
        # stepData3.pose.orientation.x = 0.0
        # stepData3.pose.orientation.y = 0.0
        # stepData3.pose.orientation.z = 0.0
        # stepData3.pose.orientation.w = 1.0

        return stepData1,stepData2,stepData3,stepData4

    def GetSteppingStoneStepPlan(self,state):
        # foot placement path:       
        path_start_foot_index = 1 # start stepping of SteppingStonesPath with Right foot 
        #Stepping stones centers in world cord. [FootPlacement(16.7,8.0,0.0),FootPlacement(17.4,8.0,0.0),FootPlacement(17.9,8.5,0.0),FootPlacement(18.6,8.5,0.0),FootPlacement(19.1,8.0,0.0),FootPlacement(20.0,8.0,0.0)]
        self._SteppingStonesPath = [FootPlacement(16.9,7.87,0.0),FootPlacement(16.9,8.13,0.0),FootPlacement(17.3,7.87,0.0),FootPlacement(17.3,8.13,0.0),\
            FootPlacement(17.5,7.87,deg2r(30.0)),FootPlacement(17.5,8.2,deg2r(45.0)),FootPlacement(17.6,8.05,deg2r(45.0)),\
            FootPlacement(17.8,8.55,0.0),FootPlacement(17.8,8.35,0.0),FootPlacement(17.85,8.65,0.0),FootPlacement(18.0,8.35,0.0),FootPlacement(18.1,8.6,0.0),\
            FootPlacement(18.5,8.35,0.0),FootPlacement(18.5,8.6,0.0),FootPlacement(18.7,8.35,deg2r(-30.0)),FootPlacement(18.7,8.5,deg2r(-30.0)),\
            FootPlacement(19.0,8.0,deg2r(-30.0)),FootPlacement(19.0,8.17,0.0),FootPlacement(19.25,7.87,0.0),FootPlacement(19.25,8.13,0.0),\
            FootPlacement(19.65,7.87,0.0),FootPlacement(19.65,8.13,0.0),FootPlacement(19.9,7.87,0.0),FootPlacement(19.9,8.13,0.0),FootPlacement(20.0,7.87,0.0),FootPlacement(20.0,8.13,0.0),\
            FootPlacement(20.0,7.87,0.0),FootPlacement(20.0,8.13,0.0),FootPlacement(20.0,7.87,0.0),FootPlacement(20.0,8.13,0.0),FootPlacement(20.0,7.87,0.0),FootPlacement(20.0,8.13,0.0),\
            FootPlacement(20.0,7.87,0.0),FootPlacement(20.0,8.13,0.0),FootPlacement(20.0,7.87,0.0),FootPlacement(20.0,8.13,0.0),FootPlacement(20.0,7.87,0.0),FootPlacement(20.0,8.13,0.0)]
        ## building SteppingStonesQueue:
        # difference between BDI est position and LPP position (world cord.)
        deltaX = state.pos_est.position.x - self._LPP.GetPos().GetX()
        deltaY = state.pos_est.position.y - self._LPP.GetPos().GetY()
        deltaYaw = 0.0
        deltaFootPlacement = FootPlacement(deltaX,deltaY,deltaYaw)
        # feet positions for step data = self._SteppingStonesPath + deltaFootPlacement

        index = self._StepQueue.Peek(3).step_index  # + 1
        # step in place:
        # stepData1 = copy.deepcopy(self._StepQueue.Peek(2)) # using stepData3 to have the correct foot y position
        # stepData1.step_index = index
        # stepData1.foot_index = stepData1.step_index%2
        # index += 1
        # stepData2 = copy.deepcopy(self._StepQueue.Peek(3)) # using stepData3 to have the correct foot y position
        # stepData2.step_index = index 
        # stepData2.foot_index = stepData2.step_index%2
        # self._SteppingStonesQueue.Append([stepData1, stepData2])
        if index%2 == path_start_foot_index:
            index += 1
            stepData3 = copy.deepcopy(self._StepQueue.Peek(2)) # using stepData3 to have the correct foot y position
            stepData3.step_index = index
            stepData3.foot_index = stepData3.step_index%2
            self._SteppingStonesQueue.Append([stepData3])

        index += 1
        for i in range(len(self._SteppingStonesPath)):
            # need to calc. step_index and foot_index for step data
            stepData = self._stepDataInit.GetStepData(index + i) #self._StrategyForward
            stepData.pose.position.x = self._SteppingStonesPath[i].GetX() + deltaX
            stepData.pose.position.y = self._SteppingStonesPath[i].GetY() + deltaY
            step_yaw = self._SteppingStonesPath[i].GetYaw() + deltaYaw
            # Calculate orientation quaternion
            Q = quaternion_from_euler(0, 0, step_yaw)
            stepData.pose.orientation.x = Q[0]
            stepData.pose.orientation.y = Q[1]
            stepData.pose.orientation.z = Q[2]
            stepData.pose.orientation.w = Q[3]
            #print("GetSteppingStoneStepPlan:: step Data: ",stepData)
            self._SteppingStonesQueue.Append([stepData])

    def HandleStateMsg(self,state):
        command = 0
        if ("Idle" == self._WalkingModeStateMachine.GetCurrentState().Name):
            pass
        elif ("Wait" == self._WalkingModeStateMachine.GetCurrentState().Name):
            self.step_index_for_reset = state.walk_feedback.next_step_index_needed - 1
            self._Odometer.SetPosition(state.pos_est.position.x,state.pos_est.position.y)
            self._BDI_StateMachine.Initialize(self.step_index_for_reset)
            self._BDI_StateMachine.GoForward()
            self._WalkingModeStateMachine.PerformTransition("Go")
            #yaw?
        elif ("Walking" == self._WalkingModeStateMachine.GetCurrentState().Name):
            if (self._LPP.IsActive()):
                # x,y = self._Odometer.GetGlobalPosition()
                # self._LPP.UpdatePosition(x,y)
                self._BDI_StateMachine.SetPathError(self._LPP.GetPathError())
              
                targetYaw = self._LPP.GetTargetYaw()
                delatYaw = targetYaw - self._Odometer.GetYaw()
                  
                debug_transition_cmd = "NoCommand"
                if (math.sin(delatYaw) > 0.6):
                    #print("Sin(Delta)",math.sin(delatYaw), "Left")
                    debug_transition_cmd = "TurnLeft"
                    self._BDI_StateMachine.TurnLeft(targetYaw)
                elif (math.sin(delatYaw) < -0.6):
                    #print("Sin(Delta)",math.sin(delatYaw), "Right")
                    debug_transition_cmd = "TurnRight"
                    self._BDI_StateMachine.TurnRight(targetYaw)
            else:
                debug_transition_cmd = "Stop"
                self._BDI_StateMachine.Stop()
                if (self._BDI_StateMachine.IsDone()):
                    self._WalkingModeStateMachine.PerformTransition("Finished")
                    
            command = self._BDI_StateMachine.Step(state.walk_feedback.next_step_index_needed)
            
            if (0 !=command):
                rospy.loginfo("WalkingModeBDI, asi_state_cb: State Machine Transition Cmd = %s" % (debug_transition_cmd) )
        elif ("Done" == self._WalkingModeStateMachine.GetCurrentState().Name):
                pass
        else:
                raise Exception("BDI_WalkingModeStateMachine::Bad State Name")
    
        return command


def deg2r(deg):
    return (deg*math.pi/180.0)

