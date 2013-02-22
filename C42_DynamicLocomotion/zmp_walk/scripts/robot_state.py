#!/usr/bin/env python

###############################################################################
####                                                                         ##
####  robot_state.py                                                         ##
####  Created - Yuval 22/2/2013                                              ##
####  last updated - version 1.0, Yuval 22/2/2013                            ##
####                                                                         ##
####    Robot_State:   ##
####                       .                   ##
####                                                                         ##
####    Used in zmp_main.py to provide filtered position data to the IK      ##
####    module and stores position data at the begining and end of each step ##
####    phase for use of creating profiles (swing).                          ##
####                                                                         ##
####                                                                         ##
###############################################################################

import roslib; roslib.load_manifest('zmp_walk')
import rospy
from zmp_walk.msg import Position, Orientation, Pos_and_Ori
import copy


class Robot_State:
    'Keeps track of robots lower limbs positions using tf - foward kinematics, filters sampled data '
    # Class parameters:

    def __init__(self, name):
        self.name = name
        self.last_tran = [ [0,0,0], [0,0,0], [0,0,0], [0,0,0], [0,0,0] ]  # previous translation values, insert according to default starting position values 
        self.last_rot = [ [0,0,0], [0,0,0], [0,0,0], [0,0,0], [0,0,0] ]  # previous rotation values, insert according to default starting position values 
        self.stance_hip = Position()
        self.swing_foot = Pos_and_Ori()
        self.swing_hip = Position()
        self.pelvis_m = Orientation()
        self.com_m = Position()
        # for i in range(len(self.Gd)) :
        #     rospy.loginfo("Gd coefficient in place %i) %.8f" % (i, self.Gd[i]) )
        # # Wait 1 second
        # rospy.sleep(1)

    # Auxiliary functions:
    def place_in_Pos(self,x,y,z):
        res = Position()
        res.x = x; res.y = y; res.z = z;
        return (res)

    def place_in_Ori(self,r,p,w):
        res = Orientation()
        res.r = r; res.p = p; res.w = w;
        return (res)

    def place_in_Pos_Ori(self,x,y,z,r,p,w):
        res = Pos_and_Ori()
        res.x = x; res.y = y; res.z = z; res.r = r; res.p = p; res.w = w;
        return (res)

    def getRobot_State(self,step_phase,listener):
        # tf frames:
        if ( step_phase == 1 ) or ( step_phase == 2 ): 
            # stance = left, swing/support = right
            base_frame = 'l_foot'
            # frames to transform: stance hip, com_m, pelvis_m, swing_hip, swing_foot
            get_frames = [ 'l_uleg', 'com', 'pelvis', 'r_uleg', 'r_foot' ] # DRC (from tf) names 
        elif ( step_phase == 3 ) or ( step_phase == 4 ): 
            # stance = right, swing/support = left
            base_frame = 'r_foot'
            # frames to transform: stance hip, com_m, pelvis_m, swing_hip, swing_foot
            get_frames = [ 'r_uleg', 'com', 'pelvis', 'l_uleg', 'l_foot' ] # DRC (from tf) names    
        # get transforms:
        tran = self.last_tran
        rot = self.last_rot
        for i in range(0,5):
            successful_tf = True
            try:
              (translation,rotation) = listener.lookupTransform(base_frame, get_frames[i], rospy.Time(0))  #  rospy.Time(0) to use latest availble transform 
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
              successful_tf = False
              # print ex
              rospy.loginfo("tf exception: %s" % (ex.args))
              # Wait 1 second
              # rospy.sleep(1)
              continue
            if successful_tf:
                tran[i] = translation
                rot[i] = rotation

        self.stance_hip = self.place_in_Pos( tran[0][0], tran[0][1], tran[0][2] )
        self.swing_foot = self.place_in_Pos_Ori( tran[4][0], tran[4][1], tran[4][2], rot[4][0], rot[4][1], rot[4][2] )
        self.swing_hip = self.place_in_Pos( tran[3][0], tran[3][1], tran[3][2] ) 
        self.pelvis_m = self.place_in_Ori( rot[2][0], rot[2][1], rot[2][2] )
        self.com_m = self.place_in_Pos( tran[1][0], tran[1][1], tran[1][2] )

        self.last_tran = tran
        self.last_rot = rot

        return ()


    # need to run after tf is running and robot is static, to get information on robots starting position     
    def static_init_with_tf_data(self,listener,bend_knees): 
        # init state:
        # waiting for transform to be avilable
        time_out = rospy.Duration(2)
        polling_sleep_duration = rospy.Duration(0.01)
        # stance = left, swing = right
        base_frame = 'l_foot'
        # frames to transform: stance hip, com_m, pelvis_m, swing_hip, swing_foot
        get_frames = [ 'l_uleg', 'com', 'pelvis', 'r_uleg', 'r_foot' ] # DRC (from tf) names 
        for i in range(0,5):
            while listener.waitForTransform (base_frame, get_frames[i], rospy.Time(0), time_out, polling_sleep_duration) and not rospy.is_shutdown():
                rospy.loginfo("Not ready for Forward Kinematics transform")

        self.getRobot_State(3,listener)

        com_0_from_r_foot = copy.deepcopy( self.com_m )
        self.r_stance_hip_0 = copy.deepcopy( self.stance_hip ) 

        self.getRobot_State(1,listener)

        com_0_from_l_foot = copy.deepcopy( self.com_m )
        self.l_stance_hip_0 = copy.deepcopy( self.stance_hip )
        rs_from_l_foot_swing_foot = copy.deepcopy( self.swing_foot )
        rs_from_l_foot_swing_hip = copy.deepcopy( self.swing_hip )
        rs_from_l_foot_pelvis_m = copy.deepcopy( self.pelvis_m )

        # TODO: in order to get better readings one should repeat the lines above a few times while init position function and average the readings  

        # constraint: we want hips height (z) and advance (x) to be the same
        hip_height = 0.7999 - bend_knees #(self.l_stance_hip_0.z + self.r_stance_hip_0.z)/2
        hip_sagital = -0.02 # (self.l_stance_hip_0.x + self.r_stance_hip_0.x)/2 # x position relative to foot
        self.l_stance_hip_0.z = hip_height; self.r_stance_hip_0.z = hip_height;
        self.l_stance_hip_0.x = hip_sagital; self.r_stance_hip_0.x = hip_sagital;

        # self.com2l_stance_hip = Position(); self.com2r_stance_hip = Position(); 
        # self.com2l_stance_hip.x = com_0_from_l_foot.x - self.l_stance_hip_0.x # to be used when using measured com. we get the relative postion of the left stance hip
        # self.com2l_stance_hip.y = com_0_from_l_foot.y - self.l_stance_hip_0.y
        # self.com2l_stance_hip.z = com_0_from_l_foot.z - self.l_stance_hip_0.z
        # self.com2r_stance_hip.x = com_0_from_r_foot.x - self.r_stance_hip_0.x # to be used when using measured com. we get the relative postion of the right stance hip
        # self.com2r_stance_hip.y = com_0_from_r_foot.y - self.r_stance_hip_0.y
        # self.com2r_stance_hip.z = com_0_from_r_foot.z - self.r_stance_hip_0.z
        # use with measured com  e.g: com_m - com2_stance_hip + com_ref = stance hip in the foot coordinate system to be used by IK
        # use with out e.g: stance_hip_0 + com_ref = stance hip in the foot coordinate system to be used by IK

        self.swing_foot = rs_from_l_foot_swing_foot
        self.swing_hip = rs_from_l_foot_swing_hip
        self.pelvis_m = rs_from_l_foot_pelvis_m
        self.com_m = com_0_from_l_foot

        return ()
