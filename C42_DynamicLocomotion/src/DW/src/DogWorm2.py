#! /usr/bin/env python
import roslib; roslib.load_manifest('DogWorm')
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

class DW_Controller(object):
    """DW_Controller"""
    def __init__(self, arg):
        super(DW_Controller, self).__init__()

        ##################################################################
        ######################## GAIT PARAMETERS #########################
        ##################################################################

        self.CurSeqStep = 0
        self.FollowPath = 1
        self.DesOri = 0
        
        ##################################################################
        ###################### Basic Standing Pose #######################
        ##################################################################

        self.BasStndPose = zeros(28)
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
        ##################### Crab Walking Sequence ######################
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
        ThisRobotCnfg[7] = ThisRobotCnfg[7+6] = 1.4
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
        ########################## INITIALIZE ############################
        ##################################################################

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
        self.MsgSub = rospy.Subscriber('/'+self._robot_name+'/atlas_state',AtlasState,self.RS_cb)
        self.OdomSub = rospy.Subscriber('/ground_truth_odom',Odometry,self.Odom_cb)
        self.GlobalPos = 0
        self.GlobalOri = 0

        self.reset_srv = rospy.ServiceProxy('/gazebo/reset_models', Empty)

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
        self.JC.set_gains('l_arm_ely',1000,0,10)
        self.JC.set_gains('r_arm_ely',1000,0,10)
        self.JC.set_gains("l_arm_elx",1200,0,5)
        self.JC.set_gains("r_arm_elx",1200,0,5)
        self.JC.set_gains("l_arm_mwx",1200,0,5)
        self.JC.set_gains("r_arm_mwx",1200,0,5)
        self.JC.send_command()

    ##################################################################
    ########################### FUNCTIONS ############################
    ##################################################################

    def SeqWithBalance(self,pos1,pos2,T,dt):
        if len(pos1) == len(pos2) == len(self._jnt_names):
            N = ceil(T/dt)
            pos1 = array(pos1)
            pos2 = array(pos2)
            COM_pos0x = self.GlobalPos.x
            COM_pos0y = self.GlobalPos.y
            back_mby_pos = 0

            for ratio in linspace(0, 1, N):
              interpCommand = (1-ratio)*pos1 + ratio * pos2

              # Balance torso
              back_mby_pos = interpCommand[1]-3*(self.GlobalPos.x - COM_pos0x)
              back_ubx_pos = interpCommand[2]+2*(self.GlobalPos.y - COM_pos0y)

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
        self.JC.set_gains("l_leg_uay",10,0,5)
        self.JC.set_gains("r_leg_uay",10,0,5)
        self.JC.set_gains("l_leg_lax",10,0,5)
        self.JC.set_gains("r_leg_lax",10,0,5)
        self.JC.send_command()
        rospy.sleep(T*0.2)
        self.JC.set_gains("l_arm_elx",1200,0,5)
        self.JC.set_gains("r_arm_elx",1200,0,5)
        self.JC.set_gains("l_leg_uay",50,0,5)
        self.JC.set_gains("r_leg_uay",50,0,5)
        self.JC.set_gains("l_leg_lax",50,0,5)
        self.JC.set_gains("r_leg_lax",50,0,5)
        self.JC.send_command()
        rospy.sleep(T*0.4)

    def ResetPose(self):
        self.JC.set_all_pos([0]*28)
        self.JC.send_command()

    def RS_cb(self,msg):
        self.RS.UpdateState(msg)

    def Odom_cb(self,msg):
        self.GlobalPos = msg.pose.pose.position
        self.GlobalOri = msg.pose.pose.orientation

    def current_ypr(self):
        quat = copy(self.GlobalOri)
        (r, p, y) = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return (y, p, r)

    def reset(self):
        self.reset_srv()
        rospy.sleep(1)

        while self.GlobalPos.z<0.9 or self.GlobalPos.z>0.94 or abs(self.GlobalPos.x)>0.5:
            self.reset_srv()
            rospy.sleep(1)

    def AddRotation(self,Delta): # EXPERIMENTAL
        # Delta of 1 gives approx. 0.15 radians turn left
        # Add gait changes to appropriate step
        self.RobotCnfg[1][4] = self.RobotCnfg[1][4+6] = Delta*0.2
        self.RobotCnfg[1][9] = self.RobotCnfg[1][9+6] = Delta*0.12
        self.RobotCnfg[2][0] = -Delta*0.12
        self.RobotCnfg[2][2] = -Delta*0.10
        self.RobotCnfg[3][2] = 0
        self.RobotCnfg[4][0] = 0
        self.RobotCnfg[4][4] = self.RobotCnfg[4][4+6] = 0
        self.RobotCnfg[4][9] = self.RobotCnfg[4][9+6] = 0

        # Insert those changes in following steps as well
        self.RobotCnfg[2][4] = self.RobotCnfg[2][4+6] = Delta*0.2
        self.RobotCnfg[2][9] = self.RobotCnfg[2][9+6] = Delta*0.12
        self.RobotCnfg[3][4] = self.RobotCnfg[3][4+6] = Delta*0.2
        self.RobotCnfg[3][9] = self.RobotCnfg[3][9+6] = Delta*0.12
        self.RobotCnfg[3][0] = -Delta*0.12
        self.RobotCnfg[0][4] = self.RobotCnfg[0][4+6] = 0
        self.RobotCnfg[0][9] = self.RobotCnfg[0][9+6] = 0

    def GoToSeqStep(self,Step):
        if Step == self.CurSeqStep:
            pass
        else:
            if Step > 4:
                Step = 0
                self.DoSeqStep()
            while self.CurSeqStep != Step:
                self.DoSeqStep()

    def DoSeqStep(self):
        self.JC.send_pos_traj(self.RS.GetJointPos(),self.RobotCnfg[self.CurSeqStep],self.StepDur[self.CurSeqStep],0.005) 
        self.CurSeqStep += 1
        if self.CurSeqStep > 4:
            self.CurSeqStep = 0
        
    def RotateOnSpot(self,Angle):
        # Get into "break-dance" configuration
        self.GoToSeqStep(3)
        pos=copy(self.RobotCnfg[2][:])
        pos[1] = 0.7
        pos[16] = pos[16+6] = 0.5
        pos[17] = -1.4
        pos[17+6] = 1.4
        pos[18] = pos[18+6] = 2
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.5,0.01) 

        if abs(Angle) < 0.55:
            Delta = Angle/0.55
            self.RotSpotSeq(Delta)
        else:
            # Get current orientation
            y0,p,r = self.current_ypr()

            # Calculate number of turns required
            N = int(math.floor(abs(Angle)/0.55))

            # Perform turns
            if Angle>0:
                for x in range(N):
                    self.RotSpotSeq(1)
            else:
                for x in range(N):
                    self.RotSpotSeq(-1)

            # Get current orientation
            y1,p,r = self.current_ypr()

            Delta = (Angle - (y1-y0))/0.55
            self.RotSpotSeq(Delta)

    def RotSpotSeq(self,Delta):
        # Delta of 1 gives a left rotation of approx. 0.55 radians

        # Init configuration
        pos=copy(self.RobotCnfg[2][:])
        pos[1] = 0.7
        pos[16] = pos[16+6] = 0.5
        pos[17] = -1.4
        pos[17+6] = 1.4
        pos[18] = pos[18+6] = 2

        T=2
        if Delta>0:
            sign = 1
            dID = 0
        else:
            sign = -1
            dID = 6
        Delta = abs(Delta)

        # Lift first leg
        pos[7+dID] = 1.4-0.5*Delta
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.05*T,0.005) 
        # Rotate first leg outwards until it touches ground
        pos[0] = Delta*sign*0.1 # Rotate torso to shift COM over to first foot
        pos[4+dID] = Delta*sign*0.5
        pos[5+dID] = Delta*sign*0.25
        pos[9+dID] = Delta*sign*0.25
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.3*T,0.005) 
        # Lift other leg, now all the weight is on first leg
        pos[7+6-dID] = 1.4-0.6*Delta
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.05*T,0.005) 
        # Return first leg and torso to original configuration
        pos[0] = 0
        pos[4+dID] = 0
        pos[5+dID] = 0
        pos[7+dID] = 1.4
        pos[9+dID] = 0
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.3*T,0.005) 
        # Return other leg to original configuration
        pos[7+6-dID] = 1.4
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.3*T,0.005) 

    def Run(self,TimeOut = 0):
        rospy.sleep(0.1)

        self.JC.reset_command()
        self.JC.reset_gains()
        self.JC.send_pos_traj(self.RS.GetJointPos(),self.BasStndPose,0.5,0.01) 
        self.LHC.set_all_pos(self.BaseHandPose)
        self.RHC.set_all_pos(self.BaseHandPose)
        self.LHC.send_command()
        self.RHC.send_command()

        self.reset()
        rospy.sleep(1)

        self.Sit(2)
        rospy.sleep(0.5)

        # for x in range(100):
        #     if self.FollowPath == 1:
        #         # Update sequence to correct orientation
        #         y,p,r = self.current_ypr()
        #         Correction = self.DesOri - y
        #         if Correction > 2*math.pi:
        #             Correction -= 2*math.pi
        #         if Correction < -2*math.pi:
        #             Correction += 2*math.pi
        #         print Correction

        #         Delta = Correction/0.15
        #         if abs(Delta)>1:
        #             Delta /= abs(Delta)

        #         self.AddRotation(Delta)

        #     self.GoToSeqStep(5)
        
        # self.GoToSeqStep(1)

        # pos=copy(self.RobotCnfg[2][:])
        # self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.5,0.01) 
        self.RotateOnSpot(-math.pi/2)
        self.RotateOnSpot(math.pi)



##################################################################
######################### USAGE EXAMPLE ##########################
##################################################################

if __name__=='__main__':
    DW = DW_Controller([])
    DW.Run(60)
