#! /usr/bin/env python
import roslib; roslib.load_manifest('C46_MountVehicle')
import math, rospy, os, rosparam
import tf
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from C25_GlobalPosition.msg import C25C0_ROP
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

class STC_Controller(object):
    """STC_Controller"""
    def __init__(self, arg):
        super(STC_Controller, self).__init__()        
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
        self.BasStndPose[21] = 0.4
        self.BasStndPose[21+6] = -0.4

        self.BaseHandPose = zeros(12)
        self.BaseHandPose[1] = self.BaseHandPose[1+3] = self.BaseHandPose[1+6] = 1.5
        self.OpenHandPose = zeros(12)
        self.ClosedHandPose = zeros(12)
        self.ClosedHandPose[1] = self.ClosedHandPose[1+3] = self.ClosedHandPose[1+6] = self.ClosedHandPose[1+9] = 1.5
        self.ClosedHandPose[2] = self.ClosedHandPose[2+3] = self.ClosedHandPose[2+6] = self.ClosedHandPose[2+9] = 1.5

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
        self.OdomSub = rospy.Subscriber('/C25/publish',C25C0_ROP,self.Odom_cb)
        self.GlobalPos = 0
        self.GlobalOri = 0

        self.reset_srv = rospy.ServiceProxy('/gazebo/reset_models', Empty)
        
        self.TF = tf.TransformListener()

        ##################################################################
        ######################## Controller Gains ########################
        ##################################################################

        self.JC.set_gains('l_leg_uhz',1000,0,50)
        self.JC.set_gains('r_leg_uhz',1000,0,50)
        self.JC.set_gains('l_leg_mhx',1000,0,50)
        self.JC.set_gains('r_leg_mhx',1000,0,50)
        self.JC.set_gains('back_lbz',5000,0,100)
        self.JC.set_gains('back_ubx',5000,0,100)
        self.JC.set_gains('back_mby',5000,0,100)
        self.JC.set_gains('l_arm_usy',1000,0,10)
        self.JC.set_gains('r_arm_usy',1000,0,10)
        self.JC.set_gains('l_arm_shx',1000,0,10)
        self.JC.set_gains('r_arm_shx',1000,0,10)
        self.JC.set_gains('l_arm_ely',1000,0,10)
        self.JC.set_gains('r_arm_ely',1000,0,10)
        self.JC.set_gains("l_arm_elx",900,0,5)
        self.JC.set_gains("r_arm_elx",900,0,5)
        self.JC.set_gains("l_arm_mwx",900,0,5)
        self.JC.set_gains("r_arm_mwx",900,0,5)
        self.JC.send_command()

    ##################################################################
    ########################### FUNCTIONS ############################
    ##################################################################

    def RS_cb(self,msg):
        self.RS.UpdateState(msg)

    def Odom_cb(self,msg):
        self.GlobalPos = msg.pose.pose.pose.position
        self.GlobalOri = msg.pose.pose.pose.orientation

    def ResetPose(self):
        self.JC.set_all_pos([0]*28)
        self.JC.send_command()

    def reset(self):
        self.reset_srv()
        rospy.sleep(1)

        # while self.GlobalPos.z<0.9 or self.GlobalPos.z>1: #or abs(self.GlobalPos.x)>0.5:
        # # while self.GlobalPos.z<0.25 or self.GlobalPos.z>0.4: #or abs(self.GlobalPos.x)>0.5:
        #     self.reset_srv()
        #     rospy.sleep(1)

    def current_ypr(self):
	try:
		quat = copy(self.GlobalOri)
		(r, p, y) = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
	except:
		(r,p,y)=[0,0,0]        
	return (y, p, r)

    def DeltaAngle(self,DesAngle,CurAngle):
        Delta = DesAngle - CurAngle
        if Delta > math.pi:
            Delta-=2*math.pi
        elif Delta < -math.pi:
            Delta+=2*math.pi
        return Delta

    def SeqWithBalance(self,pos1,pos2,T,dt,ref_link = '/l_foot',COMref = 0):
        if len(pos1) == len(pos2) == len(self._jnt_names):
            N = ceil(T/dt)
            pos1 = array(pos1)
            pos2 = array(pos2)

            init_com, rot = self.TF.lookupTransform('/com',ref_link,rospy.Time(0))
            if COMref != 0:
                COMref0 = [init_com[0],init_com[1]]
                init_com = [0]*2

            for ratio in linspace(0, 1, N):
                cur_com, rot = self.TF.lookupTransform('/com',ref_link,rospy.Time(0))
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


    def Run(self,TimeOut = 0):
        rospy.sleep(0.1)
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,1,0.01)

    def SwingIn(self):
        # Raise arms above head
        pos = copy(self.BasStndPose)
        pos[3] = 1
        pos[17] = 1.8
        pos[17+6] = -1.8
        pos[6] = pos[6+6] = 0
        pos[7] = pos[7+6] = 0.1
        pos[8] = pos[8+6] = -0.05
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.6,0.005)

        pos[8] = pos[8+6] = -0.2
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.5,0.005)

        rospy.sleep(0.5)

        # "Grasp" top pole with wrists
        # Lift own weight by flexing elbows
        pos[8] = pos[8+6] = 0.6
        pos[19] = 0.8
        pos[19+6] = -0.8
        pos[21] = 1.5
        pos[21+6] = -1.5
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.5,0.005)

        # Lift legs of ground and flex pelvis+hips dynamically
        pos[1] = 1.5
        pos[5] = pos[5+6] = 0
        pos[6] = pos[6+6] = -1.6
        pos[7] = pos[7+6] = 2.4
        pos[8] = pos[8+6] = -0.2
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.5,0.005) 
        pos[8] = pos[8+6] = -0.6 
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.2,0.005)  

        # Check current time
        T0 = rospy.Time.now().to_sec()
        while True:
            # Get current orientation
            r,p,y = self.current_ypr()

            if p<-1.3:
                print "Wheeeee..."
                pos[7] = pos[7+6] = 0
                self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.15,0.005) 
                break 
            elif p>-0.5 and rospy.Time.now().to_sec()-T0>4:
                # This is taking too long, extend and flex again
                pos[6] = pos[6+6] = -0.7
                pos[7] = pos[7+6] = 1.3
                self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.8,0.005) 
                pos[6] = pos[6+6] = -1.6
                pos[7] = pos[7+6] = 2.4
                pos[8] = pos[8+6] = -0.2
                self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.2,0.005) 
                T0 = rospy.Time.now().to_sec()
            else:
                rospy.sleep(0.005)

        print "...eeeee!"
        pos[1] = 0
        pos[6] = pos[6+6] = 0
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.15,0.005) 

        rospy.sleep(0.5)
        # "Catch" seat with heels and pull
        pos[7] = pos[7+6] = 1.2
        pos[8] = pos[8+6] = 0.7
        pos[6+6] = -1.5
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.6,0.005) 
        rospy.sleep(0.5)
        # Release right hand to "hang on the side"
        pos[21+6] = 0.5
        pos[5] = -0.3
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.5,0.005) 
        # Extend left arm to lower robot smoothly
        pos[16] = -1.5
        pos[17] = 0.0
        pos[18] = 1.9
        pos[19] = 0
        pos[20] = 1.5
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.5,0.005) 
        # Release left hand
        pos[21] = -0.5
        pos[8+6] = -0.1
        self.JC.set_gains('r_leg_uay',30,0,1,set_default=False)
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.2,0.005) 
        rospy.sleep(0.3)

    def GetHandsIn(self):
        pos = copy(self.BasStndPose)
        pos[1] = 0
        pos[3] = 1
        pos[5] = pos[5+6] = 0
        pos[6] = 0
        pos[6+6] = -1.6
        pos[7] = pos[7+6] = 1.2
        pos[8] = pos[8+6] = 0.7
        pos[8+6] = -0.1
        pos[16] = -1.5
        pos[17] = 0.0
        pos[18] = 1.9
        pos[19] = 0
        pos[20] = 1.5
        pos[21] = 0
        pos[17+6] = -1.8
        pos[19+6] = -0.8
        pos[21+6] = 0.5 

        # Bring left arm into the car
        pos[16] = -1.3
        pos[17] = -1.2
        pos[18] = 0.6
        pos[19] = 1.8
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.5,0.005)

        # Rotate right arm, preparing to enter the car
        pos[16+6] = -0.3
        pos[17+6] = -0.4
        pos[18+6] = 0
        pos[19+6] = -2
        pos[21+6] = -1.5
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,1,0.005)
        # Rotate arm at shoulder to help robot lay flat on the seat
        # (with help from right leg)
        pos[2] = 0.2
        pos[4+6] = -0.5
        pos[16+6] = 1.0
        pos[17+6] = -0.1
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.5,0.005)
        rospy.sleep(0.3)
        # Rotate right arm into car
        pos[16+6] = -1.5
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.7,0.005)
        pos[17+6] = 1.3
        pos[18+6] = 0.4
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.8,0.005)
        # Straighten hands
        pos[16] = -1.35
        pos[16+6] = -1.35
        pos[17] = -1.8
        pos[17+6] = 1.3
        pos[18] = pos[18+6] = 0.3
        pos[19] = 0.2
        pos[19+6] = -0.2
        pos[20] = pos[20+6] = 0
        pos[21] = pos[21+6] = 0
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.5,0.005)
        self.JC.reset_gains(8+6)

    def SitUp(self):
        pos = copy(self.BasStndPose)
        pos[0] = 0
        pos[2] = 0.5
        pos[1] = 0
        pos[3] = 1
        pos[5] = pos[5+6] = 0
        pos[5+6] = -0.5
        pos[6] = pos[6+6] = 0
        pos[7] = pos[7+6] = 1.6
        pos[8] = pos[8+6] = 0
        pos[16] = -1.35
        pos[16+6] = -1.45
        pos[17] = -1.8
        pos[17+6] = 1.8
        pos[18] = 0.3
        pos[18+6] = 0.4
        pos[19] = 0.2
        pos[19+6] = -0.2
        pos[20] = pos[20+6] = 0
        pos[21] = pos[21+6] = 0

        # Extend knees
        pos[7] = 0
        pos[7+6] = 0.7
        pos[8+6] = -0.7
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.3,0.005)    

        # Flex torso and hips
        # Move right arm up so it won't interfere with the st. wheel
        pos[1] = 1.5
        pos[5] = 0.5
        pos[6] = pos[6+6] = -0.7
        # pos[7+6] = 1.4
        pos[17] = -0.7
        pos[16+6] = -1.1
        pos[17+6] = -0.4
        pos[19+6] = -1
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.5,0.005)
        rospy.sleep(0.3)

        # Use left arm to push on frame to help sit up
        # Push on dashboard with right hand
        pos[6] = -1.2
        pos[7] = 1.3 #1.5
        pos[8] = -0.7
        pos[4+6] = -0.5
        pos[5+6] = 0.1
        pos[7+6] = 1.2
        # pos[7] = 1.7
        pos[17] = 1.1 # 0.9
        pos[19] = 0.6 # 0.8 # 1.3
        pos[16+6] = 0.5
        pos[17+6] = 0.2
        pos[18+6] = 1.0
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.5,0.005)

        # "Numb" legs
        # self.JC.set_gains('l_leg_lhy',10,0,10,set_default=False)
        self.JC.set_gains('l_leg_kny',10,0,10,set_default=False)
        self.JC.set_gains('l_leg_lax',30,0,1,set_default=False)
        self.JC.set_gains('r_leg_kny',10,0,10,set_default=False)
        self.JC.set_gains('r_leg_lax',30,0,1,set_default=False)

        # Straighten most body joints into sitting position
        pos[0] = 0.5
        pos[2] = 0
        pos[1] = 0.2
        pos[4] = -0.2
        pos[5] = 0
        pos[5+6] = 0
        pos[6] = -1.9 #-1.7 # -1.6
        pos[6+6] = -1.9
        pos[7] = 1.5
        pos[7+6] = 2
        pos[17] = 0.6
        pos[19] = 0.8 # 1.3
        pos[16+6] = -0.4
        pos[17+6] =  0.5
        pos[19+6] = -0.6
        self.JC.set_gains('r_leg_lax',30,0,1,set_default=False)
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,1.5,0.005)
        rospy.sleep(0.5)
        self.JC.set_gains('r_leg_kny',30,0,1,set_default=False)

        # Overcompensate with back_y to make sure the robot sits
        # pos[1] = -0.3
        # pos[2] = 0.3
        pos[17] = -0.9
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.5,0.005)
        rospy.sleep(0.5)

        # # Return back_y to 0
        # # pos[0] = 0
        # pos[1] = 0
        # pos[2] = 0
        # pos[6+6] = -1.5
        # pos[7+6] = 1.8
        # self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.5,0.005)


        # self.JC.reset_gains(6)
        self.JC.reset_gains(7)
        self.JC.reset_gains(9)
        self.JC.reset_gains(7+6)
        self.JC.reset_gains(9+6)

        ######################################
        pos[0] = 0
        pos[1] = -0.1
        pos[4] = 0.2
        pos[4+6] = -0.2
        pos[5] = pos[5+6] = 0
        pos[6] = pos[6+6] = -1.4
        pos[7] = pos[7+6] = 1.4
        pos[19] = 1.5
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.5,0.005)
        pos[7] = 1.2
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.5,0.005)
        pos[18] = 0.7
        pos[19] = 1.1
        pos[19+6] = -0.9
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.5,0.005)

        ######################################

        # Bend right arm to grab gear stick
        # pos[0] = -0.5
        # pos[16+6] = 0
        # pos[17+6] = 0.2
        # pos[18+6] = 1.0
        # pos[19+6] = -1.7
        # self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.5,0.005)

    def RotateToPedals(self,DesOri):
        pos = copy(self.BasStndPose)
        pos[0] = 0 #-0.5 #0.2
        pos[1] = 0
        pos[2] = 0
        pos[3] = 1
        pos[4] = -0.2
        pos[5] = 0
        pos[6] = -1.6
        pos[7] = 1.8
        pos[8] = 0.7
        pos[4+6] = -0.5
        pos[5+6] = 0
        pos[6+6] = -1.5
        pos[7+6] = 1.8
        pos[8+6] = -0.7
        pos[16] = -1.35
        pos[17] = -0.9
        pos[18] = 0.3
        pos[19] = 1.3
        pos[16+6] = -0.6
        pos[17+6] =  0.2 
        pos[18+6] = 1.0
        pos[19+6] = -0.4
        # pos[16+6] = 0
        # pos[17+6] = 0.2
        # pos[18+6] = 1.0
        # pos[19+6] = -1.7
        pos[20] = pos[20+6] = 0
        pos[21] = pos[21+6] = 0

        # Lower right arm and rotate it inward
        # Rotate left arm outward
        # Straighten legs
        # pos[4] = pos[4+6] = 0
        pos[5] = pos[5+6] = 0
        # pos[6] = pos[6+6] = -1.8
        pos[8] = pos[8+6] = -0.7
        pos[16] = -1.1
        pos[16+6] = -1.1
        pos[17+6] = 0.6
        pos[19+6] = -0.8
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.5,0.005)

        # Push back with legs
        pos[16+6] = -0.9
        pos[7] = 1.3
        pos[7+6] = 1.4
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,1,0.005)
        pos[0] = 0.2
        pos[4] = 0.2
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.4,0.005)
        pos[0] = 0
        pos[4] = 0
        pos[7] = pos[7+6] = 1.7
        pos[6] = -1.4
        # pos[8] = 0
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.8,0.005)
        # pos[6] = -1.4
        # self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.4,0.005)

        # Insert left arm
        pos[17] = -1.5
        pos[19] = 2
        pos[21] = 1.0
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.8,0.005)
        pos[0] = -0.4
        pos[16+6] = -1.3
        pos[17+6] = 0.8
        pos[19+6] = -0.6
        pos[16] = -1.6
        pos[18] = 0.8
        pos[21] = 0
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.8,0.005)

        # Extend and rotate left arm outward
        pos[16] = -1.5
        pos[17] = -1.1
        pos[18] = 1.0
        pos[19] = 1.0
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.8,0.005)
        pos[16] = -1.3
        pos[18] = 0.3
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.8,0.005)

        # Rotate in place towards target
        self.JC.set_gains('r_leg_lax',30,0,1,set_default=False)
        self.JC.set_gains('l_leg_lax',30,0,1,set_default=False)
     #    # Check current time
     #    T0 = rospy.Time.now().to_sec()
     #    while True:
     #        # Get current orientation
     #        r,p,y = STC.current_ypr()
     #        print DesOri, y

     #        # Check if more than 15 seconds elapsed
     #        if rospy.Time.now().to_sec()-T0>10:
     #            if abs(self.DeltaAngle(DesOri,y))>0.1:
     #                self.RotateWithLegs(-self.DeltaAngle(DesOri,y),pos)
     #            else:
     #                break
		    
     #            if rospy.Time.now().to_sec()-T0>20:
     #                print "Rotate to pedals: FAIL!"
     #                print "Giving up... Please try again"
     #                #break
     #        else: 
     #            if abs(self.DeltaAngle(DesOri,y))>0.1:
     #                pass
	    # self.RotateWithLegs(self.DeltaAngle(DesOri,y),pos)

     #        rospy.sleep(0.01)
        pos[0] = 0.2
        pos[2] = -0.2
        pos[4] = 0.3
        pos[4+6] = -0.3
        pos[5+6] = -0.2
        pos[6+6] = -1.7
        pos[7] = 1.5
        pos[8] = 0.7
        pos[8+6] = -0.7
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.5,0.005)
        self.JC.reset_gains(9)
        self.JC.reset_gains(9+6)

        # Full gas ahead
        self.JC.set_gains('r_leg_uay',30,0,1,set_default=False)
        pos[7+6] = 1.0
        self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.5,0.005)

    def RotateWithLegs(self,Delta,pos):
        # # Push back with legs
        # pos[7] = pos[7+6] = 0.4
        # self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.7,0.005)
        # pos[7] = pos[7+6] = 1.7
        # self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.5,0.005)

        if Delta>0:
            print "Turning right..."
            # Rotate torso with help from the legs
            pos[6] = pos[6+6] = -1.9
            pos[7] = pos[7+6] = 1.8
            self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.2,0.005)
            pos[0] = 0.5
            pos[4] = 0 # NEW
            pos[4+6] = 0 # NEW
            # pos[16+6] = -1.3
            pos[17+6] = 0.8
            pos[18+6] = 1.2
            self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.5,0.005)
            pos[6] = -1.6
            pos[7] = 1.7
            self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.2,0.005)
            pos[0] = 0
            pos[4] = 0.3
            pos[4+6] = -0.3
            self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.5,0.005)
            pos[6+6] = -1.6
            pos[7+6] = 1.7
            self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.2,0.005)
        else:
            print "Turning left..."
            # Rotate torso with help from the legs
            pos[6] = pos[6+6] = -1.8
            pos[7] = pos[7+6] = 1.8
            self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.2,0.005)
            pos[0] = -0.5
            pos[4] = 0 # NEW
            pos[4+6] = 0 # NEW
            # pos[16+6] = -1.3
            pos[17+6] = 0.8
            pos[18+6] = 1.2
            self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.5,0.005)
            pos[6+6] = -1.6
            pos[7+6] = 1.7
            self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.2,0.005)
            pos[0] = 0
            pos[4] = 0.3
            pos[4+6] = -0.3
            self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.5,0.005)
            pos[6] = -1.6
            pos[7] = 1.7
            self.JC.send_pos_traj(self.RS.GetJointPos(),pos,0.2,0.005)

##################################################################
######################### USAGE EXAMPLE ##########################
##################################################################

if __name__=='__main__':
    STC = STC_Controller([])

    rospy.sleep(0.5)
    STC.LHC.set_all_pos(STC.ClosedHandPose)
    STC.LHC.send_command()
    STC.RHC.set_all_pos(STC.ClosedHandPose)
    STC.RHC.send_command()

    STC.JC.send_pos_traj(STC.RS.GetJointPos(),STC.BasStndPose,0.5,0.005)
    rospy.sleep(0.5)
    #STC.reset()
    rospy.sleep(0.5)

    STC.SwingIn()
    rospy.sleep(0.3)

    STC.GetHandsIn()

    rospy.sleep(0.3)
    STC.SitUp()

    # rospy.sleep(0.5)

    # Continue = 1
    # # Check current time
    # T0 = rospy.Time.now().to_sec()
    # while True:
    #     # Get current orientation
    #     r,p,y = STC.current_ypr()
    #     print r,p

    #     # Wait to see if the robot was able to properly sit up
    #     if 0.65<r<1.2 and abs(p)<0.3: ######## GOTTA CHECK THIS PARAMS! ########
    #         print "Sit up: SUCCESS!"
    #         Continue = 1
    #         break

    #     # Check if more than 3 seconds elapsed
    #     if rospy.Time.now().to_sec()-T0>0.5:
    #         print "Sit up: FAIL!"
    #         print "Giving up... Please try again"
    #         Continue = 1
    #         break

    #     rospy.sleep(0.01)

    # if Continue == 1:
    #     STC.RotateToPedals(0)

    # pos = copy(STC.BasStndPose)
    # pos[0] = 0
    # pos[1] = 0
    # pos[2] = 0
    # pos[3] = 1
    # pos[4] = 0.3
    # pos[4+6] = -0.3
    # pos[5] = pos[5+6] = 0
    # pos[6] = -1.8
    # pos[6+6] = -1.6
    # pos[7] = pos[7+6] = 1.7
    # pos[8] = pos[8+6] = 0
    # pos[16] = -1.4
    # pos[17] = -1.5
    # pos[18] = 0.8
    # pos[19] = 2
    # pos[16+6] = -1.1
    # pos[17+6] = 0.8
    # pos[18+6] = 1.2
    # pos[19+6] = -0.8
    # pos[20] = pos[20+6] = 0
    # pos[21] = pos[21+6] = 0
