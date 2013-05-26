#!/usr/bin/env python

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

import rospy
import roslib
roslib.load_manifest('C42_DynamicLocomotion')

from Abstractions.WalkingMode import *
from Abstractions.Odometer import *
from DD_PathPlanner import *

from tf.transformations import quaternion_from_euler, euler_from_quaternion

from atlas_msgs.msg import AtlasCommand, AtlasSimInterfaceCommand, AtlasSimInterfaceState, AtlasState, AtlasBehaviorStepData
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from std_msgs.msg import String

from C42_DynamicLocomotion.srv import *
from C42_DynamicLocomotion.msg import Foot_Placement_data

class DD_WalkingMode(WalkingMode):
    def __init__(self):
        self._LPP = DD_PathPlanner()
        WalkingMode.__init__(self,self._LPP)
        self.step_index_for_reset = 0
        # Initialize atlas atlas_sim_interface_command publisher       
        self.asi_command = rospy.Publisher('/atlas/atlas_sim_interface_command', AtlasSimInterfaceCommand, None, False, True, None)
        self._Odometer = Odometer()
        self._bDone = False
        self._StepIndex = 1
        self._command = 0
        
    def Initialize(self):
        WalkingMode.Initialize(self)
        # Subscribers:
        self._odom_sub = rospy.Subscriber('/ground_truth_odom',Odometry,self._odom_cb)
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
            resp = self._foot_placement_client(0)
            if 1 == resp.done:
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
        command = 0
        if(self._StepIndex < state.walk_feedback.next_step_index_needed):
            command = self.HandleStateMsg(state)
        if (0 != command):
            #print("Step",self._StepIndex,command)
            self.asi_command.publish(command)
            self._StepIndex = self._StepIndex+1

    def _odom_cb(self,odom):
        self._LPP.UpdatePosition(odom.pose.pose.position.x,odom.pose.pose.position.y)
 
    def _get_imu(self,msg):  #listen to /atlas/imu/pose/pose/orientation
        roll, pitch, yaw = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self._Odometer.SetYaw(yaw)


###############################################################################################


