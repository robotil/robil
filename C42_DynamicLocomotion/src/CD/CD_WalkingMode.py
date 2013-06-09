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
import roslib
import PyKDL

roslib.load_manifest('C42_DynamicLocomotion')

from Abstractions.WalkingMode import *
from Abstractions.Odometer import *

from CD_PathPlanner import *

from atlas_msgs.msg import AtlasCommand, AtlasSimInterfaceCommand, AtlasSimInterfaceState, AtlasState, AtlasBehaviorStepData
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from C31_PathPlanner.msg import C31_Waypoints
from C25_GlobalPosition.msg import C25C0_ROP
from DW.JointController import JointCommands_msg_handler
from tf_conversions import posemath
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class CD_WalkingMode(WalkingMode):
    def __init__(self):
        WalkingMode.__init__(self,CD_PathPlanner())
        self.step_index_for_reset = 0
        # Initialize atlas mode and atlas_sim_interface_command publishers        
        self.asi_command = rospy.Publisher('/atlas/atlas_sim_interface_command', AtlasSimInterfaceCommand, None, False, True, None)
        self._realRobot = CD_RealRobot(self._LPP)
        self._phantomRobot = CD_PhantomRobot(CD_PathPlanner(),Odometer())
        
        self._bDone = False

        ############################
        #joint controller
        self._cur_jnt = [0]*28
        robot_name = "atlas"
        jnt_names = ['back_lbz', 'back_mby', 'back_ubx', 'neck_ay', #3
                           'l_leg_uhz', 'l_leg_mhx', 'l_leg_lhy', 'l_leg_kny', 'l_leg_uay', 'l_leg_lax', #9
                           'r_leg_uhz', 'r_leg_mhx', 'r_leg_lhy', 'r_leg_kny', 'r_leg_uay', 'r_leg_lax', #15
                           'l_arm_usy', 'l_arm_shx', 'l_arm_ely', 'l_arm_elx', 'l_arm_uwy', 'l_arm_mwx', #21
                           'r_arm_usy', 'r_arm_shx', 'r_arm_ely', 'r_arm_elx', 'r_arm_uwy', 'r_arm_mwx'] #27
        self._JC = JointCommands_msg_handler(robot_name,jnt_names)
        
    def Initialize(self,parameters):
        WalkingMode.Initialize(self,parameters)
        self._realRobot.Initialize()
        self._phantomRobot.Initialize()
        
        # Subscriber
        self._Subscribers["Path"] = rospy.Subscriber('/path',C31_Waypoints,self._path_cb)
        self._Subscribers["Odometry"] = rospy.Subscriber('/C25/publish',C25C0_ROP,self._odom_cb)
        # self._Subscribers["Odometry"] = rospy.Subscriber('/ground_truth_odom',Odometry,self._odom_cb) 
        self._Subscribers["ASI_State"]  = rospy.Subscriber('/atlas/atlas_sim_interface_state', AtlasSimInterfaceState, self.asi_state_cb)
        self._Subscribers["IMU"]  = rospy.Subscriber('/atlas/imu', Imu, self._get_imu)
        self._Subscribers["JointStates"] = rospy.Subscriber('/atlas/joint_states', JointState, self._get_joints)
        rospy.sleep(0.3)
    
        self._bDone = False
        
        k_effort = [0] * 28
        k_effort[0:4] = 4*[255]
        k_effort[16:28] = 12*[255]
        self._JC.set_k_eff(k_effort)
        self._JC.set_all_pos(self._cur_jnt)
        self._JC.send_command()

        # Put robot into stand position
        stand = AtlasSimInterfaceCommand(None,AtlasSimInterfaceCommand.STAND, None, None, None, None, k_effort)
                
        rospy.sleep(0.3)
        
        self.asi_command.publish(stand)
    
    def StartWalking(self):
        self._bDone = False
        return 0.3
    
    def Walk(self):
        WalkingMode.Walk(self)
    
    def Stop(self):
        WalkingMode.Stop(self)        
    
    def EmergencyStop(self):
        k_effort = [0] * 28
        k_effort[3] = 255
        stand = AtlasSimInterfaceCommand(None,AtlasSimInterfaceCommand.STAND, None, None, None, None, k_effort)
        self.asi_command.publish(stand)

    def IsDone(self):
        return self._bDone
    
    def HandleStateMsg(self,state):
        command = 0
        if ("Idle" == self._WalkingModeStateMachine.GetCurrentState().Name):
            self._phantomRobot.SetPosition(state.pos_est.position.x,state.pos_est.position.y)
        elif ("Wait" == self._WalkingModeStateMachine.GetCurrentState().Name):
            self._phantomRobot.SetPosition(state.pos_est.position.x,state.pos_est.position.y)
            self._WalkingModeStateMachine.PerformTransition("Go")
        elif ("Walking" == self._WalkingModeStateMachine.GetCurrentState().Name):
            self._PhantomRobot.Step()
            command = self._realRobot.GetCommand()
            if (self._realRobot.IsDone()):
                self._WalkingModeStateMachine.PerformTransition("Finished")
        elif ("Done" == self._WalkingModeStateMachine.GetCurrentState().Name):
            self._bDone = True
        else:
            raise Exception("QS_WalkingModeStateMachine::Bad State Name")
    
        return command

###################################################################################
#--------------------------- CallBacks --------------------------------------------
###################################################################################

    def _path_cb(self,path):
        rospy.loginfo('got path %s',path)
        p = []
        for wp in path.points:
            p.append(Waypoint(wp.x,wp.y))
        self.SetPath(p)
        self._phantomRobot.SetPath(p)

    def _get_joints(self,msg):
        self._cur_jnt = msg.position

    # /atlas/atlas_sim_interface_state callback. Before publishing a walk command, we need
    # the current robot position   
    def asi_state_cb(self, state):
        if(self._phantomRobot.GetIndex() < state.walk_feedback.next_step_index_needed):
            command = self.HandleStateMsg(state)
        self._bDone = self._WalkingModeStateMachine.IsDone()
        if (0 !=command):
            self.asi_command.publish(command)

    def _odom_cb(self,odom):
        # SHOULD USE:
        self._realRobot.UpdatePosition(odom.pose.pose.pose.position.x,odom.pose.pose.pose.position.y) # from C25_GlobalPosition
        #self._LPP.UpdatePosition(odom.pose.pose.position.x,odom.pose.pose.position.y) # from /ground_truth_odom
 
    def _get_imu(self,msg):  #listen to /atlas/imu/pose/pose/orientation
        roll, pitch, yaw = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self._realRobot.SetYaw(yaw)

