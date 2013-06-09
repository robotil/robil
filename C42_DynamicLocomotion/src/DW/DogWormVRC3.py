#! /usr/bin/env python
import roslib
roslib.load_manifest('C42_DynamicLocomotion')
#import roslib; roslib.load_manifest('DogWorm')
import math, rospy, os, rosparam
import tf
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from numpy import zeros, array, linspace, arange
import numpy as np
from JointController import JointCommands_msg_handler
from JointController import hand_joint_controller
from robot_state import robot_state
from atlas_msgs.msg import AtlasState
from math import ceil
import yaml
from copy import copy
from std_srvs.srv import Empty
##nedded for tests
from C25_GlobalPosition.msg import C25C0_ROP
from Abstractions.Interface_tf import *
class DW_Controller(object):
    """DW_Controller"""
    def __init__(self,iTf):
        super(DW_Controller, self).__init__()
        self._iTf = iTf

        ##################################################################
        ######################## GAIT PARAMETERS #########################
        ##################################################################
        self.CurSeqStep = 0
        self.CurSeqStep2 = 0
        self.Throtle = 1
        self.RotFlag = 0
        self.FollowPath = 1
        self.DesOri = 0
        
        ##################################################################
        ###################### Basic Standing Pose #######################
        ##################################################################

        self.BasStndPose = zeros(28)
        self.BasStndPose[5] = 0.1
        self.BasStndPose[5+6] = -0.1
        self.BasStndPose[6] = self.BasStndPose[6+6] = -0.2
        self.BasStndPose[7] = self.BasStndPose[7+6] = 0.4
        self.BasStndPose[8] = self.BasStndPose[8+6] = -0.2
        self.BasStndPose[9] = -0.1
        self.BasStndPose[9+6] = 0.1
        self.BasStndPose[17] = -1.3
        self.BasStndPose[17+6] = 1.3 
        self.BasStndPose[18] = self.BasStndPose[18+6] = 1.5
        #         # Pose[18] -= 1.5
        #         # Pose[20] += 3.1
        # # self.BasStndPose[18] = self.BasStndPose[18+6] = 0
        # self.BasStndPose[20] = self.BasStndPose[20+6] = 1.5
        self.BasStndPose[21] = 0.4
        self.BasStndPose[27] = -0.4

        self.BaseHandPose = zeros(12)
        self.BaseHandPose[1] = self.BaseHandPose[1+3] = self.BaseHandPose[1+6] = 1.5


        ##################################################################
        ####################### Sit Down Sequence ########################
        ##################################################################

        self.SitDwnSeq1 = copy(self.BasStndPose)
        self.SitDwnSeq1[1] = 0.5
        self.SitDwnSeq1[6] = self.SitDwnSeq1[6+6] = -1.5
        self.SitDwnSeq1[7] = self.SitDwnSeq1[7+6] = 2.4
        self.SitDwnSeq1[8] = self.SitDwnSeq1[8+6] = -1.2
        self.SitDwnSeq1[16] = self.SitDwnSeq1[16+6] = 1.05
        self.SitDwnSeq1[17] = -1.25
        self.SitDwnSeq1[17+6] = 1.25
        self.SitDwnSeq1[18] = self.SitDwnSeq1[18+6] = 2.2
        self.SitDwnSeq1[21] = 0.2
        self.SitDwnSeq1[21+6] = -0.2

        ##################################################################
        ################# Crab Forward Walking Sequence ##################
        ##################################################################

        T = 1.5
        self.RobotCnfg = []
        self.StepDur = []

        # Sequence Step 1: Touch ground with pelvis, lift legs
        ThisRobotCnfg = copy(self.SitDwnSeq1)
        ThisRobotCnfg[1] = 0.9
        ThisRobotCnfg[8] = ThisRobotCnfg[8+6] = -0.1
        ThisRobotCnfg[6] = ThisRobotCnfg[6+6] = -1.9
        self.RobotCnfg.append(ThisRobotCnfg)
        self.StepDur.append(0.1*T)
        
        # Sequence Step 2: Extend legs
        ThisRobotCnfg = copy(self.RobotCnfg[0][:])
        ThisRobotCnfg[7] = ThisRobotCnfg[7+6] = 1.1
        ThisRobotCnfg[8] = ThisRobotCnfg[8+6] = 0.8
        self.RobotCnfg.append(ThisRobotCnfg)
        self.StepDur.append(0.2*T)

        # Sequence Step 3: Put legs down, bringing torso forward and raising arms
        ThisRobotCnfg = copy(self.RobotCnfg[1][:])
        ThisRobotCnfg[6] = ThisRobotCnfg[6+6] = -1.4
        ThisRobotCnfg[16] = ThisRobotCnfg[16+6] = 1.1
        ThisRobotCnfg[17] = -1.
        ThisRobotCnfg[17+6] = 1.
        ThisRobotCnfg[18] = ThisRobotCnfg[18+6] = 2.3
        ThisRobotCnfg[19] = 2
        ThisRobotCnfg[19+6] = -2
        self.RobotCnfg.append(ThisRobotCnfg)
        self.StepDur.append(0.6*T)

        # Sequence Step 4: Touch ground with arms closer to pelvis and lift pelvis
        ThisRobotCnfg = copy(self.RobotCnfg[2][:])
        ThisRobotCnfg[16] = ThisRobotCnfg[16+6] = 0.6
        ThisRobotCnfg[17] = -1.35
        ThisRobotCnfg[17+6] = 1.35
        ThisRobotCnfg[18] = ThisRobotCnfg[18+6] = 2
        ThisRobotCnfg[19] = 0.1
        ThisRobotCnfg[19+6] = -0.1
        self.RobotCnfg.append(ThisRobotCnfg)
        self.StepDur.append(0.4*T)

        # Sequence Step 5: Bring pelvis forward, closer to legs
        ThisRobotCnfg = copy(self.RobotCnfg[3][:])
        ThisRobotCnfg[7] = ThisRobotCnfg[7+6] = 2.4
        ThisRobotCnfg[8] = ThisRobotCnfg[8+6] = -0.2
        ThisRobotCnfg[16] = ThisRobotCnfg[16+6] = 1.0
        ThisRobotCnfg[17] = -1.1
        ThisRobotCnfg[17+6] = 1.1
        ThisRobotCnfg[18] = ThisRobotCnfg[18+6] = 2.5
        self.RobotCnfg.append(ThisRobotCnfg)
        self.StepDur.append(0.6*T)

        ##################################################################
        ################# Crab Backward Walking Sequence #################
        ##################################################################

        T = 1
        self.RobotCnfg2 = []
        self.StepDur2 = []

        # Sequence Step 1: Bring pelvis down to the ground and lift arms
        ThisRobotCnfg = copy(self.SitDwnSeq1)
        ThisRobotCnfg[1] = 0.5
        ThisRobotCnfg[4] = 0.1
        ThisRobotCnfg[4+6] = -0.1
        ThisRobotCnfg[6] = ThisRobotCnfg[6+6] = -1.7
        ThisRobotCnfg[7] = ThisRobotCnfg[7+6] = 1.0
        ThisRobotCnfg[8] = ThisRobotCnfg[8+6] = 0.8
        ThisRobotCnfg[16] = ThisRobotCnfg[16+6] = 1.5
        ThisRobotCnfg[17] = -0.6
        ThisRobotCnfg[17+6] = 0.6
        ThisRobotCnfg[18] = ThisRobotCnfg[18+6] = 2.5
        ThisRobotCnfg[19] = 1.8
        ThisRobotCnfg[19+6] = -1.8
        self.RobotCnfg2.append(ThisRobotCnfg)
        self.StepDur2.append(0.3*T)

        #[-0.066, 0.082 Sequence Step 2: Extend arms
        ThisRobotCnfg = copy(self.RobotCnfg2[0][:])
        ThisRobotCnfg[16] = ThisRobotCnfg[16+6] = 1.4
        ThisRobotCnfg[17] = -0.4
        ThisRobotCnfg[17+6] = 0.4
        ThisRobotCnfg[18] = ThisRobotCnfg[18+6] = 2.9
        ThisRobotCnfg[19] = 0.2
        ThisRobotCnfg[19+6] = -0.2
        self.RobotCnfg2.append(ThisRobotCnfg)
        self.StepDur2.append(0.4*T)

        # Sequence Step 3: Extend torso, fall back on arms, lift and fold legs
        ThisRobotCnfg = copy(self.RobotCnfg2[1][:])
        ThisRobotCnfg[1] = 0.4
        ThisRobotCnfg[6] = ThisRobotCnfg[6+6] = -1.6
        ThisRobotCnfg[7] = ThisRobotCnfg[7+6] = 2.6
        ThisRobotCnfg[8] = ThisRobotCnfg[8+6] = 0.2
        self.RobotCnfg2.append(ThisRobotCnfg)
        self.StepDur2.append(0.4*T)

        # Sequence Step 4: Place legs on ground and lift pelvis
        ThisRobotCnfg = copy(self.RobotCnfg2[2][:])
        ThisRobotCnfg[6] = ThisRobotCnfg[6+6] = -1.
        ThisRobotCnfg[7] = ThisRobotCnfg[7+6] = 2.2
        ThisRobotCnfg[8] = ThisRobotCnfg[8+6] = 0
        ThisRobotCnfg[18] = ThisRobotCnfg[18+6] = 2.8
        ThisRobotCnfg[19] = 0.4
        ThisRobotCnfg[19+6] = -0.4
        self.RobotCnfg2.append(ThisRobotCnfg)
        self.StepDur2.append(0.4*T)

        # Sequence Step 5: Move pelvis back, between arms
        ThisRobotCnfg = copy(self.RobotCnfg2[3][:])
        ThisRobotCnfg[1] = 0.5
        ThisRobotCnfg[6] = ThisRobotCnfg[6+6] = -1.7
        ThisRobotCnfg[7] = ThisRobotCnfg[7+6] = 1.0
        ThisRobotCnfg[8] = ThisRobotCnfg[8+6] = 0.8
        ThisRobotCnfg[16] = ThisRobotCnfg[16+6] = 0.5
        ThisRobotCnfg[17] = -1.35
        ThisRobotCnfg[17+6] = 1.35
        ThisRobotCnfg[18] = ThisRobotCnfg[18+6] = 2.3
        ThisRobotCnfg[19] = 0.5
        ThisRobotCnfg[19+6] = -0.5
        self.RobotCnfg2.append(ThisRobotCnfg)
        self.StepDur2.append(0.7*T)

        ##################################################################
        ########################## INITIALIZE ############################
        ##################################################################

    def Initialize(self,Terrain):
        self._robot_name = "atlas"
        self._jnt_names = ['back_lbz', 'back_mby', 'back_ubx', 'neck_ay', #3
                           'l_leg_uhz', 'l_leg_mhx', 'l_leg_lhy', 'l_leg_kny', 'l_leg_uay', 'l_leg_lax', #9
                           'r_leg_uhz', 'r_leg_mhx', 'r_leg_lhy', 'r_leg_kny', 'r_leg_uay', 'r_leg_lax', #15
                           'l_arm_usy', 'l_arm_shx', 'l_arm_ely', 'l_arm_elx', 'l_arm_uwy', 'l_arm_mwx', #21
                           'r_arm_usy', 'r_arm_shx', 'r_arm_ely', 'r_arm_elx', 'r_arm_uwy', 'r_arm_mwx'] #27

        # Initialize joint commands handler
        self.JC = JointCommands_msg_handler(self._robot_name,self._jnt_names)
        self.LHC = hand_joint_controller("left")
        self.RHC = hand_joint_controller("right")
        # Initialize robot state listener
        self.RS = robot_state(self._jnt_names)
        print("DW::Initialize")
        self.GlobalPos = 0
        self.GlobalOri = 0
        self._counter = 0
        self._terrain = Terrain
        self._fall_count = 0
        self.FALL_LIMIT = 3
        # self.reset_srv = rospy.ServiceProxy('/gazebo/reset_models', Empty)

        ##################################################################
        ######################## Controller Gains ########################
        ##################################################################

        self.JC.set_gains('l_leg_uhz',1000,0,10)
        self.JC.set_gains('r_leg_uhz',1000,0,10)
        self.JC.set_gains('l_leg_mhx',1000,0,10)
        self.JC.set_gains('r_leg_mhx',1000,0,10)
        self.JC.set_gains('back_lbz',5000,0,10)
        self.JC.set_gains('back_ubx',1000,0,10)
        self.JC.set_gains('l_arm_usy',1000,0,10)
        self.JC.set_gains('r_arm_usy',1000,0,10)
        self.JC.set_gains('l_arm_shx',1000,0,10)
        self.JC.set_gains('r_arm_shx',1000,0,10)
        self.JC.set_gains('l_arm_ely',2000,0,10)
        self.JC.set_gains('r_arm_ely',2000,0,10)
        self.JC.set_gains("l_arm_elx",1200,0,5)
        self.JC.set_gains("r_arm_elx",1200,0,5)
        self.JC.set_gains("l_arm_mwx",1200,0,5)
        self.JC.set_gains("r_arm_mwx",1200,0,5)

    ##################################################################
    ########################### FUNCTIONS ############################
    ##################################################################

    def RS_cb(self,msg):
        self.RS.UpdateState(msg)

    def Odom_cb(self,msg):
        if 1000 <= self._counter: 
            print ("Odom_cb::", self.GlobalPos)
            self._counter = 0
        self._counter += 1
        self.GlobalPos = msg.pose.pose.pose.position # from C25_GlobalPosition
        self.GlobalOri = msg.pose.pose.pose.orientation # from C25_GlobalPosition
        # self.GlobalPos = msg.pose.pose.position    # from /ground_truth_odom
        # self.GlobalOri = msg.pose.pose.orientation # from /ground_truth_odom


    def ResetPose(self):
        self.JC.set_all_pos([0]*28)
        self.JC.send_command()

    def reset(self):
        self.reset_srv()
        rospy.sleep(1)

        while self.GlobalPos.z<0.9 or self.GlobalPos.z>1: #or abs(self.GlobalPos.x)>0.5:
        # while self.GlobalPos.z<0.25 or self.GlobalPos.z>0.4: #or abs(self.GlobalPos.x)>0.5:
            self.reset_srv()
            rospy.sleep(1)

    def current_ypr(self):

        quat = copy(self.GlobalOri)
        (r, p, y) = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return (y,p,r)
        # RPY = self.RS.GetIMU()
        # return (RPY[2], RPY[1], RPY[0])

    def DeltaAngle(self,DesAngle,CurAngle):
        Delta = DesAngle - CurAngle
        if Delta > math.pi:
            Delta-=2*math.pi
        elif Delta < -math.pi:
            Delta+=2*math.pi
        return Delta

    def SeqWithBalance(self,pos1,pos2,T,dt, COMref = 0):
        if len(pos1) == len(pos2) == len(self._jnt_names):
            N = ceil(T/dt)
            pos1 = array(pos1)
            pos2 = array(pos2)

            init_com, rot = self._iTf.TransformListener().lookupTransform('/com','/l_foot',rospy.Time(0))
            if COMref != 0:
                COMref0 = [init_com[0],init_com[1]]
                init_com = [0]*2

            for ratio in linspace(0, 1, N):
                cur_com, rot = self._iTf.TransformListener().lookupTransform('/com','/l_foot',rospy.Time(0))
                interpCommand = (1-ratio)*pos1 + ratio * pos2
  
                if COMref != 0:
                    init_com[0] = (1-ratio)*COMref0[0] + ratio*COMref[0]
                    init_com[1] = (1-ratio)*COMref0[1] + ratio*COMref[1]

                # Balance torso
                back_mby_pos = interpCommand[1]+3*(cur_com[0]-init_com[0])
                back_ubx_pos = interpCommand[2]+2*(cur_com[1]-init_com[1])
  
                self.JC.set_all_pos([ float(x) for x in interpCommand ])
                self.JC.set_pos("back_mby",back_mby_pos)
                self.JC.set_pos("back_ubx",back_ubx_pos)

                self.JC.send_command()
                rospy.sleep(dt)

        else:
            print 'position command legth doest fit'

    def Sit(self,T):
        self.JC.set_gains("l_arm_elx",5,0,20)
        self.JC.set_gains("r_arm_elx",5,0,20)
        self.SeqWithBalance(self.RS.GetJointPos(),self.SitDwnSeq1,T*0.3,0.005)
        self.JC.set_pos("l_leg_uay",-0.1)
        self.JC.set_pos("r_leg_uay",-0.1)
        self.JC.set_gains("l_leg_uay",10,0,5,set_default = False)
        self.JC.set_gains("r_leg_uay",10,0,5,set_default = False)
        self.JC.set_gains("l_leg_lax",10,0,5,set_default = False)
        self.JC.set_gains("r_leg_lax",10,0,5,set_default = False)
        self.JC.send_command()
        rospy.sleep(T*0.2)
        self.JC.set_gains("l_arm_elx",1200,0,5)
        self.JC.set_gains("r_arm_elx",1200,0,5)
        self.JC.set_gains("l_leg_uay",50,0,5,set_default = False)
        self.JC.set_gains("r_leg_uay",50,0,5,set_default = False)
        self.JC.set_gains("l_leg_lax",50,0,5,set_default = False)
        self.JC.set_gains("r_leg_lax",50,0,5,set_default = False)
        self.JC.send_command()
        rospy.sleep(T*0.4)

    def DoPath(self,Path):
        for Point in Path:
            if Point[2] == "fwd":
                self.Throtle=1
            if Point[2] == "bwd":
                self.Throtle=0.6

            self.GoToPoint(Point)

    def GoToPoint(self,Point):
        # Calculate distance and orientation to target
        while (0 == self.GlobalPos):
            rospy.sleep(1)
            print("Waiting for GlobalPos")
        DeltaPos = [Point[0]-self.GlobalPos.x,Point[1]-self.GlobalPos.y]
        Distance = math.sqrt(DeltaPos[0]**2+DeltaPos[1]**2)

        # Get current orientation
        y,p,r = self.current_ypr()
        T_ori = math.atan2(DeltaPos[1],DeltaPos[0])
        self.DesOri = T_ori
        if Point[2] == "bwd":
            T_ori += math.pi

        # Rotate in place towards target
        if abs(self.DeltaAngle(T_ori,y))>0.1:
            self.RotateToOri(T_ori)

        # Crawl towards target
        if Point[2] == "fwd":
            self.Crawl()
        if Point[2] == "bwd":
            self.BackCrawl()

        while True:
            # Calculate distance and orientation to target
            DeltaPos = [Point[0]-self.GlobalPos.x,Point[1]-self.GlobalPos.y]
            Distance = math.sqrt(DeltaPos[0]**2+DeltaPos[1]**2)
            T_ori = math.atan2(DeltaPos[1],DeltaPos[0])
            self.DesOri = T_ori
            if Point[2] == "bwd":
                T_ori += math.pi

            if Distance<0.4:
                print "Reached Waypoint"
                break

            y,p,r = self.current_ypr()
            # Check tipping
            self.CheckTipping()

            # Rotate in place towards target
            if self._fall_count < self.FALL_LIMIT: 
                Drift = abs(self.DeltaAngle(T_ori,y))
                if 0.5<Drift<1.4 and Distance>1:
                    self.RotateToOri(T_ori)
                if Drift>1.4:
                    self.RotateToOri(T_ori)

                # Crawl towards target
                if Point[2] == "fwd":
                    self.Crawl()
                if Point[2] == "bwd":
                    self.BackCrawl()
            else:
                self.FollowPath = 0

    def Crawl(self):
        if self.FollowPath == 1:
            # Update sequence to correct orientation
            y,p,r = self.current_ypr()
            Correction = self.DeltaAngle(self.DesOri,y)

            Delta = Correction/0.15
            if abs(Delta)>1:
                Delta /= abs(Delta)

            self.AddRotation(Delta)

        self.GoToSeqStep(5)
        self.RotFlag = 0

    def BackCrawl(self):
        if self.FollowPath == 1:
            # Update sequence to correct orientation
            y,p,r = self.current_ypr()
            Correction = self.DeltaAngle(self.DesOri+math.pi,y)

            Delta = Correction/0.22
            if abs(Delta)>1:
                Delta /= abs(Delta)

            self.AddBackRotation(Delta)

        #     if self.RotFlg == 1:
        #         self.AddBackRotation(Delta)

        if self.RotFlag == 1:
            self.RotFlag = 2
            self.GoToSeqStep(1)

        self.GoToBackSeqStep(5)

    def GoToSeqStep(self,Step):
        if Step == self.CurSeqStep:
            pass
        else:
            if Step > 4:
                Step = 0
                self.DoSeqStep()
            while self.CurSeqStep != Step:
                self.DoSeqStep()

    def GoToBackSeqStep(self,Step):
        if Step == self.CurSeqStep2:
            pass
        else:
            if Step > 4:
                Step = 0
                self.DoInvSeqStep()
            while self.CurSeqStep2 != Step:
                self.DoInvSeqStep()

    def DoSeqStep(self):
        self.JC.send_pos_traj(self.RS.GetJointPos(),self.RobotCnfg[self.CurSeqStep],self.StepDur[self.CurSeqStep]/self.Throtle,0.005) 
        self.CurSeqStep += 1
        if self.CurSeqStep > 4:
            self.CurSeqStep = 0

    def DoInvSeqStep(self):
        self.JC.send_pos_traj(self.RS.GetJointPos(),self.RobotCnfg2[self.CurSeqStep2],self.StepDur2[self.CurSeqStep2]/self.Throtle,0.005) 
        self.CurSeqStep2 += 1
        if self.CurSeqStep2 > 4:
            self.CurSeqStep2 = 0

    def AddRotation(self,Delta): # EXPERIMENTAL
        # Delta of 1 gives approx. 0.15 radians turn left
        # Add gait changes to appropriate step
        self.RobotCnfg[1][4] = self.RobotCnfg[1][4+6] = Delta*0.2
        self.RobotCnfg[1][9] = self.RobotCnfg[1][9+6] = Delta*0.12
        # self.RobotCnfg[2][0] = -Delta*0.12
        # self.RobotCnfg[2][2] = -Delta*0.10
        # self.RobotCnfg[3][2] = 0
        # self.RobotCnfg[4][0] = 0
        self.RobotCnfg[4][4] = self.RobotCnfg[4][4+6] = 0
        self.RobotCnfg[4][9] = self.RobotCnfg[4][9+6] = 0

        # Insert those changes in following steps as well
        self.RobotCnfg[2][4] = self.RobotCnfg[2][4+6] = Delta*0.2
        self.RobotCnfg[2][9] = self.RobotCnfg[2][9+6] = Delta*0.12
        self.RobotCnfg[3][4] = self.RobotCnfg[3][4+6] = Delta*0.2
        self.RobotCnfg[3][9] = self.RobotCnfg[3][9+6] = Delta*0.12
        # self.RobotCnfg[3][0] = -Delta*0.12
        self.RobotCnfg[0][4] = self.RobotCnfg[0][4+6] = 0
        self.RobotCnfg[0][9] = self.RobotCnfg[0][9+6] = 0

    def AddBackRotation(self,Delta): # EXPERIMENTAL
        # Delta of 1 gives approx. 0.22 radians turn left
        if Delta>0:
            dID = 0
        else:
            dID = 6

        # Add gait changes to appropriate step
        self.RobotCnfg2[1][0] = 0.3*Delta
        self.RobotCnfg2[1][2] = 0.15*Delta
        self.RobotCnfg2[3][0] = 0
        self.RobotCnfg2[3][2] = 0

        # Insert those changes in following steps as well
        self.RobotCnfg2[2][0] = 0.3*Delta
        self.RobotCnfg2[2][2] = 0.15*Delta
        self.RobotCnfg2[3][0] = 0.3*Delta
        self.RobotCnfg2[3][2] = 0.15*Delta
        self.RobotCnfg2[4][0] = 0
        self.RobotCnfg2[4][2] = 0
        self.RobotCnfg2[0][0] = 0
        self.RobotCnfg2[0][2] = 0

    def RotateToOri(self,Bearing):
        if self._terrain == "MUD":
            self.RotateToOriInMud(Bearing)
        else:
            if self.RotFlag == 2:
                self.GoToBackSeqStep(1)
            self.RotFlag = 1

            # Make sure Bearing is from -pi to +pi
            Bearing = Bearing % (2*math.pi)
            if Bearing > math.pi:
                Bearing -= 2*math.pi
            if Bearing < math.pi:
                Bearing += 2*math.pi

            # Get into "break-dance" configuration
            #self.GoToSeqStep(3)
            pos=copy(self.RobotCnfg[2][:])
            pos[1] = 0.9
            pos[6] = pos[6+6] = -1.7
            pos[8] = pos[8+6] = 0.7
            pos[16] = pos[16+6] = 0.9
            pos[17] = -0.9
            pos[17+6] = 0.9
            pos[18] = pos[18+6] = 2
            pos[19] = 1.0
            pos[19+6] = -1.0
            self.JC.send_pos_traj(self.RS.GetJointPos(),pos,1,0.01) 

            # Get current orientation
            y0,p,r = self.current_ypr()
            Angle=self.DeltaAngle(Bearing,y0)

            while abs(Angle)>0.15: # Error of 9 degrees
                Delta = Angle/0.75
                if abs(Delta)>1:
                    Delta/=abs(Delta)
                if 0<Delta<0.35:
                    Delta+=0.25
                if -0.35<Delta<0:
                    Delta-=0.25

                self.RotSpotSeq(Delta)

                # Check tipping
                self.CheckTipping()
                
                # Get current orientation
                y,p,r = self.current_ypr()
                if abs(self.DeltaAngle(y,y0))<0.1:
                    # Robot isn't turning, try RotateInMud
                    return self.RotateToOriInMud(Bearing)

                y0 = y
                Angle=self.DeltaAngle(Bearing,y0)

            # Return to original configuration
            self.JC.send_pos_traj(self.RS.GetJointPos(),self.RobotCnfg[0][:],0.5,0.01) 
            self.CurSeqStep = 0
            return 1

    def RotateToOriInMud(self,Bearing):
        self.RotFlag = 1

        # Make sure Bearing is from -pi to +pi
        Bearing = Bearing % (2*math.pi)
        if Bearing > math.pi:
            Bearing -= 2*math.pi
        if Bearing < math.pi:
            Bearing += 2*math.pi

        self.JC.send_pos_traj(self.RS.GetJointPos(),self.RobotCnfg2[4][:],1.5,0.01)

        # Get current orientation
        y0,p,r = self.current_ypr()
        Angle=self.DeltaAngle(Bearing,y0)

        while abs(Angle)>0.1: # Error of 3 degrees
            Delta = Angle/0.45
            if abs(Delta)>1:
                Delta/=abs(Delta)
            # if 0<Delta<0.35:
            #     Delta+=0.25
            # if -0.35<Delta<0:
            #     Delta-=0.25

            self.RotOnMudSeq(Delta)
            # Check tipping
            self.CheckTipping()
            # Get current orientation
            y,p,r = self.current_ypr()
            if abs(self.DeltaAngle(y,y0))<0.1:
                # Robot isn't turning, Give up
                return 0

            y0 = y
            # Angle=self.DeltaAngle(Bearing,y0)
            Angle=self.DeltaAngle(Bearing,y0)

        # Return to original configuration
        self.JC.send_pos_traj(self.RS.GetJointPos(),self.RobotCnfg[0][:],0.5,0.01) 
        self.CurSeqStep = 0
        return 1

    def RotSpotSeq(self,Delta):
        # Delta of 1 gives a left rotation of approx. 0.75 radians

        # Init configuration
        pos=copy(self.RobotCnfg[2][:])
        pos[1] = 0.8
        pos[6] = pos[6+6] = -1.7
        pos[8] = pos[8+6] = 0.8
        pos[16] = pos[16+6] = 0.9
        pos[17] = -0.9
        pos[17+6] = 0.9
        pos[18] = pos[18+6] = 2
        pos[19] = 1.0
        pos[19+6] = -1.0

        T=1.
        if Delta>0:
            sign = 1
            dID = 0
        else:
            sign = -1
            dID = 6
        Delta = abs(Delta)

        # Lift first leg
        pos[7+dID] = 1.3-0.5*Delta
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.05*T,0.005) 
        # Rotate first leg outwards until it touches ground
        pos[4+dID] = Delta*sign*0.35
        pos[4+6-dID] = -Delta*sign*0.35
        pos[5+6-dID] = -Delta*sign*0.2
        pos[5+dID] = Delta*sign*0.2
        pos[9+dID] = Delta*sign*0.2
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.3*T,0.005) 
        # Lift other leg, now all the weight is on first leg
        pos[7+dID] = 1.3-0.1*Delta
        pos[7+6-dID] = 1.3-0.5*Delta
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.05*T,0.005) 
        # Return first leg and torso to original configuration
        pos[4+dID] = 0
        pos[4+6-dID] = 0
        pos[5+6-dID] = 0
        pos[5+dID] = 0
        pos[7+dID] = 1.3
        pos[9+dID] = 0
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.5*T,0.005) 
        # Return other leg to original configuration
        pos[7+6-dID] = 1.3
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.1*T,0.005) 
        rospy.sleep(0.1*T)

    def RotOnMudSeq(self,Delta):
        # Delta of 1 gives a left rotation of approx. 0.45 radians
        if Delta>0:
            dID = 0
        else:
            dID = 6

        T=1

        # Get into starting position
        pos = copy(self.RobotCnfg2[4][:])
        pos[1] = 0.8
        pos[6] = pos[6+6] = -1.3
        pos[16] = pos[16+6] = 0.2
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.5*T,0.01)
        # pos[6] -= 0.2

        # Lift first leg
        pos[6+dID] = -1.7
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.2*T,0.01)

        # Rotate it to the side
        pos[4+dID] = 0.8*Delta
        pos[0] = -0.4*Delta
        pos[4+6-dID] = -0.4*Delta
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.4*T,0.01)

        # Lower first leg / Lift second leg
        pos[6+dID] = -1.3
        pos[6+6-dID] = -1.8
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.2*T,0.01)

        # Rotate pelvis / Close first leg
        pos[0] = -0.8*Delta
        pos[4+dID] = 0
        pos[4+6-dID] = 0
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.4*T,0.01)

        # Return to init
        pos[1] = 0.9
        pos[6] = pos[6+6] = -1.3
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.2*T,0.01)

        # Lift arms
        pos = copy(self.RobotCnfg2[0][:])
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,1*T,0.01)

    def CheckTipping(self):
        result = 0
        while result == 0:
            # Get current orientation
            y,p,r = self.current_ypr()
            self._fall_count += 1
            if abs(p)>0.4*math.pi or abs(r)>0.8*math.pi:
                # Robot tipped backwards
                result = self.BackTipRecovery()
            elif r>math.pi/4:
                # Robot tipped to the right
                result = self.TipRecovery("right")
            elif r<-math.pi/4:
                # Robot tipped to the left
                result = self.TipRecovery("left")
            else:
                result = 1
                self.FollowPath = 1
                self._fall_count = 0


    def TipRecovery(self,side):
        if side == "right":
            dID = 0
        sign = 1
        if side == "left":
            dID = 6
            sign = -1

        # Extend legs and flex arm
        pos = copy(self.RobotCnfg[4][:])
        pos[7] = pos[7+6] = 0.8
        pos[17+6-dID] = 0
#        pos[18+6-dID] = 2.8
        pos[19+6-dID] = -sign*1.8
        pos[21+6-dID] = -sign*1.4
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.4,0.01)

        # Widen leg stance, turn with hip
        pos[2] = -sign*0.5
        pos[4] = 0.5
        pos[4+6] = -0.5
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.4,0.01)

        # Push with arm to rotate
        pos[16+6-dID] = 0.2
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,1,0.01)
        rospy.sleep(0.5)

        # Return to final sequence pos (forward)
        pos = copy(self.RobotCnfg[4][:])
        pos[4] = 0.3
        pos[4+6] = -0.3
        pos[5] = 0.3
        pos[5+6] = -0.3
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.5,0.01)

        rospy.sleep(1.5)

        # Get current orientation
        y,p,r = self.current_ypr()
        if abs(r)<math.pi/4:
            # Success!
            return 1
        else:
            return 0  

    def BackTipRecovery(self):
        # "Flatten" body on ground, bend elbows
        pos = copy(self.RobotCnfg[4][:])
        pos[1] = 0
        pos[6] = pos[6+6] = -0.4
        pos[7] = pos[7+6] = 0.8
        pos[8] = pos[8+6] = 0.3
        pos[16] = pos[16+6] = 0
        pos[17] = 0.4
        pos[17+6] = -0.4
        pos[18] = pos[18+6] = 3.14
        pos[19] = 1.1
        pos[19+6] = -1.1
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.5,0.01)

        # Raise torso on elbows
        pos[1] = 0.8
        pos[16] = pos[16+6] = 1.4
        pos[17] = 0
        pos[17+6] = 0
        pos[18] = pos[18+6] = 2.6
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.5,0.01)

        # Extend arms
        pos[1] = 0.8
        pos[6] = pos[6+6] = -1.4
        pos[7] = pos[7+6] = 2.2
        pos[16] = pos[16+6] = 1.4
        pos[17] = -1.3
        pos[17+6] = 1.3
        pos[19] = pos[19+6] = 0
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,2,0.01)

        rospy.sleep(1.5)

        # Get current orientation
        y,p,r = self.current_ypr()
        if abs(p)<0.4*math.pi:
            # Success!
            return 1
        else:
            return 0

    def FrontTipRecovery(self):

        pos = [0, 0, 0, 0,
            0, 0, -0.02, 0.04, -0.02, 0,
            0, 0, -0.02, 0.04, -0.02, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0]
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.5,0.01)

        pos = [ 0, 0, 0, 0,
            0, 0, 0, 0, -2.1, 0,
            0, 0, 0, 0, -2.1, 0,
            0, 0, 0, 1.5, 0, 0,
            0, 0, 0, -1.5, 0, 0]
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.5,0.01)

        pos = [0, 0.8, 0, 0,
            0, 0, -2, 2, -2.1, 0,
            0, 0, -2, 2, -2.1, 0,
            -1.7, -0.3, 0, 1.5, 0, 0,
            -1.7, 0.3, 0, -1.5, 0, 0]
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.2,0.01)

        pos = [0, 1.1, 0, 0,
            0, 0, -2.2, 2.8, -2.6, 0,
            0, 0, -2.2, 2.8, -2.6, 0,
            -1.4, -0.8, 0, 0, 0, 1.5,
            -1.4, 0.8, 0, 0, 0, -1.5] 
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.2,0.01)

        pos = [0, 0.8, 0, 0,
            0, 0, -2, 2.8, -2.6, 0,
            0, 0, -2, 2.8, -2.6, 0,
            -1.5, -0.8, 0, 0, 0, 0,
            -1.5, 0.8, 0, 0, 0, 0]
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.2,0.01)
        rospy.sleep(1)
        pos = copy(self.RobotCnfg[4][:])
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,1,0.01)
        # Get current orientation
        y,p,r = self.current_ypr()
        if abs(p)<0.4*math.pi:
            # Success!
            return 1
        else:
            return 0

    def DynStandUp(self):
        # Works only a small percentage of the time
        # SeqWithBalance needs to be updated to include a COM horizontal motion
        # (since the hip begins the motion from behind the feet)
        self.JC.set_gains("l_arm_mwx",1200,0,5)
        self.JC.set_gains("r_arm_mwx",1200,0,5)

        T = 1

        # Go to init pos, supported on hands and feet, with pelvis back between hands
        pos = copy(self.RobotCnfg2[4][:])
        pos[7] = pos[7+6] = 1.2
        pos[8] = pos[8+6] = 0.7
        pos[18] = pos[18+6] = 1.2
        pos[20] = pos[20+6] = 1
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,1.2*T,0.005)

        # [Insert description here]
        pos[16] = pos[16+6] = 0.3
        pos[17] = -1.4
        pos[17+6] = 1.4
        pos[19] = pos[19+6] = 0
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.6*T,0.005)

        rospy.sleep(0.5)

        # Tuck legs in and start rotation
        pos[1] = 0.8
        pos[5] = 0.1
        pos[5+6] = -0.1
        pos[6] =  -1.7
        pos[6+6] = -1.7
        pos[7] =  2.4
        pos[7+6] = 2.4
        pos[9] = -0.1
        pos[9+6] = 0.1
        pos[16] = pos[16+6] = -0.5
        pos[17] = -1.2
        pos[17+6] = 1.2
        pos[8] = pos[8+6] = -0.7
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.5*T,0.005)

        pos[1] = 0.2
        pos[7] =  2.1
        pos[7+6] = 2.1
        pos[16] = pos[16+6] = -1.4
        pos[17] = -0.7
        pos[17+6] = 0.7
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.6*T,0.005)

        rospy.sleep(1)

        # Use hands to push off and place COM on feet
        self.JC.set_gains('l_leg_uay',900,0,10)
        self.JC.set_gains('r_leg_uay',900,0,10)
        self.JC.set_gains('l_leg_lax',900,0,10)
        self.JC.set_gains('r_leg_lax',900,0,10)
        self.JC.set_gains('l_arm_usy',1000,0,10)
        self.JC.set_gains('r_arm_usy',1000,0,10)
        self.JC.set_gains('l_arm_shx',1000,0,10)
        self.JC.set_gains('r_arm_shx',1000,0,10)
        pos[1] = 0.7
        pos[7] =  2.4
        pos[7+6] = 2.4
        pos[16] = pos[16+6] = -1.4
        pos[17] = -1.1
        pos[17+6] = 1.1
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.6*T,0.005)

        rospy.sleep(0.5)
        pos[17] = -0.9
        pos[17+6] = 0.9
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.6*T,0.005)
        rospy.sleep(0.5)

        rospy.sleep(2)
        self.SeqWithBalance(self.RS.GetJointPos(),self.BasStndPose,3,0.005,[-0.02, 0.175])
        rospy.sleep(0.8)
        self.JC.send_pos_traj(self.RS.GetJointPos(),self.BasStndPose,0.5*T,0.005)

    def StandUp(self):
        RPY = self.RS.GetIMU()
        D, R = self._iTf.TransformListener().lookupTransform('/l_foot','/pelvis',rospy.Time(0))
        while not (abs(RPY[0])<= 0.1 and abs(RPY[1])<=0.1 and D[2] >= 0.8):
            self.CheckTipping()
            self.DynStandUp()
            rospy.sleep(1)
            RPY = self.RS.GetIMU()
            D, R = self._iTf.TransformListener().lookupTransform('/l_foot','/pelvis',rospy.Time(0))
            print 'roll: ',RPY[0],'pitch: ',RPY[1],'D: ',D[2]

##################################################################
######################### USAGE EXAMPLE ##########################
##################################################################

if __name__=='__main__':
    rospy.init_node("DW_test")
    # rospy.sleep(0.5)
    iTF = Interface_tf()
    DW = DW_Controller(iTF)
    DW.Initialize(Terrain = "MUD") 
    rospy.Subscriber('/C25/publish',C25C0_ROP,DW.Odom_cb)
    #self._Subscribers["Odometry"] = rospy.Subscriber('/ground_truth_odom',Odometry,self._Controller.Odom_cb)
    rospy.Subscriber('/atlas/atlas_state',AtlasState,DW.RS_cb)
    rospy.sleep(0.5)
    DW.LHC.set_all_pos(DW.BaseHandPose)
    DW.RHC.set_all_pos(DW.BaseHandPose)
    DW.LHC.send_command()
    DW.RHC.send_command()
    # rospy.sleep(2)
    
    DW.Sit(1.5)       
    rospy.sleep(0.5)

    Point1 = [1.15,-10.27,"fwd"] # Point close to right side of gate
    Point2 = [3.19,-8.5,"fwd"] # Point on the right of first mount
    Point3 = [1.78,-5.08,"fwd"] # Point before crossing "ukaf"
    Point4 = [5.46,-4.32,"fwd"] # Point after crossing "ukaf"
    Point5 = [4.74,-1.55,"fwd"] # Fork point
    Point6 = [4.29,1.28,"fwd"] # Daring option, point before cross
    Point7 = [6.76,4.98,"fwd"] # Daring option, point after cross
    Point8 = [6.08,6.77,"fwd"] # Final gate
    Path = [Point1,Point2,Point3,Point4,Point5,Point6,Point7,Point8]
    # rospy.sleep(2)
    # DW.DoPath(Path)
    # DW.FrontTipRecovery()

    