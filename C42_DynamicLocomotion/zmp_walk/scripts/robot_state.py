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
import pylab as pl
import copy


class Robot_State:
    'Keeps track of robots lower limbs positions using tf - foward kinematics, filters sampled data '
    # Class parameters:
    sample_buffer_size = 10 # number of samples to keep in history of robot state (tf location readings) 

    def __init__(self, name):
        self.name = name
        self.last_tran = [ [0,0,0], [0,0,0], [0,0,0], [0,0,0], [0,0,0] ]  # previous translation values, insert according to default starting position values 
        self.last_rot = [ [0,0,0], [0,0,0], [0,0,0], [0,0,0], [0,0,0] ]  # previous rotation values, insert according to default starting position values 
        
        self.stance_hip = Position()
        self.swing_foot = Pos_and_Ori()
        self.swing_hip = Position()
        self.pelvis_m = Orientation()
        self.com_m = Position()

        # # init buffer of robot state
        # self.stance_hip_arr = zeros((Robot_State.sample_buffer_size,3)) #Position()
        # self.swing_foot_arr = zeros((Robot_State.sample_buffer_size,6)) #Pos_and_Ori()
        # self.swing_hip_arr = zeros((Robot_State.sample_buffer_size,3)) #Position()
        # self.pelvis_m_arr = zeros((Robot_State.sample_buffer_size,3)) #Orientation()
        # self.com_m_arr = zeros((Robot_State.sample_buffer_size,3)) #Position()

        self.stance_hip_filtered = pl.zeros((3)) #Position()
        self.swing_foot_filtered = pl.zeros((6)) #Pos_and_Ori()
        self.swing_hip_filtered = pl.zeros((3)) #Position()
        self.pelvis_m_filtered = pl.zeros((3)) #Orientation()
        self.com_m_filtered = pl.zeros((3)) #Position()

        self.__step_phase = 1 # Double-Support left leg in front
        self.init_Change_step_phase()

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

    def array2Pos(self,arr):
        res = Position()
        res.x = float(arr[0]); res.y = float(arr[1]); res.z = float(arr[2]);
        return (res)

    def array2Ori(self,arr):
        res = Orientation()
        res.r = float(arr[0]); res.p = float(arr[1]); res.w = float(arr[2]);
        return (res)

    def array2Pos_Ori(self,arr):
        res = Pos_and_Ori()
        res.x = float(arr[0]); res.y = float(arr[1]); res.z = float(arr[2]); res.r = float(arr[3]); res.p = float(arr[4]); res.w = float(arr[5]);
        return (res)

    # def sum_List_of_Lists(self,data):
    #     sum_list = []

    #     for col in enumerate(data[0]):
    #         sum_list.append(math.fsum(row[col] for row in data))

    #     return(sum_list)

    # External auxiliary method:
    def Get_step_phase(self):
        step_phase = copy.copy(self.__step_phase)
        return(step_phase)

    # External auxiliary method:
    def Set_step_phase(self,value):
        self.previous_step_phase = copy.deepcopy(self.__step_phase)
        self.step_phase_changed = True
        self.__step_phase = copy.deepcopy(value)
        return()

    # External auxiliary method:
    def Get_l_foot_coord_params(self):
        return(self.array2Pos( self.l_stance_hip_0 ))

    # External auxiliary method:
    def Get_r_foot_coord_params(self):
        return(self.array2Pos( self.r_stance_hip_0 ))

    # Class internal auxiliary method:
    def init_Change_step_phase(self): 
        ## Save previous step phase data:
        self.previous_stance_hip_filtered = self.stance_hip_filtered.copy()
        self.previous_swing_foot_filtered = self.swing_foot_filtered.copy()
        self.previous_swing_hip_filtered = self.swing_hip_filtered.copy()
        self.previous_pelvis_m_filtered = self.pelvis_m_filtered.copy()
        self.previous_com_m_filtered = self.com_m_filtered.copy()
        ## init buffer of robot state if changing feet (changing refrence coordinate system)
        # TODO: may want to update buffer from previous step phase after doing the necessary transform
        if (self.__step_phase == 1) or (self.__step_phase == 3):
            self.stance_hip_arr = pl.zeros((Robot_State.sample_buffer_size,3)) #Position()
            self.swing_foot_arr = pl.zeros((Robot_State.sample_buffer_size,6)) #Pos_and_Ori()
            self.swing_hip_arr = pl.zeros((Robot_State.sample_buffer_size,3)) #Position()
            self.pelvis_m_arr = pl.zeros((Robot_State.sample_buffer_size,3)) #Orientation()
            self.com_m_arr = pl.zeros((Robot_State.sample_buffer_size,3)) #Position()

            self.num_of_samples_in_buffer = 0
            #self.buffer_full = False
        
        ## Reset step phase average parameters
        self.num_of_avg_samples = 0
        self.reset_avg_flag = True

        self.step_phase_changed = False
        return()

    # Class internal auxiliary method:
    def Moving_Avg_Filter(self):
        self.stance_hip_filtered = pl.sum(self.stance_hip_arr,axis=0)/self.num_of_samples_in_buffer
        self.swing_foot_filtered = pl.sum(self.swing_foot_arr,axis=0)/self.num_of_samples_in_buffer
        self.swing_hip_filtered = pl.sum(self.swing_hip_arr,axis=0)/self.num_of_samples_in_buffer
        self.pelvis_m_filtered = pl.sum(self.pelvis_m_arr,axis=0)/self.num_of_samples_in_buffer
        self.com_m_filtered = pl.sum(self.com_m_arr,axis=0)/self.num_of_samples_in_buffer
        return()

    # Class internal auxiliary method:
    def Update_State(self, tran, rot):
        stance_hip_new = pl.array([[ tran[0][0], tran[0][1], tran[0][2] ]])
        swing_foot_new = pl.array([[ tran[4][0], tran[4][1], tran[4][2], rot[4][0], rot[4][1], rot[4][2] ]])
        swing_hip_new = pl.array([[ tran[3][0], tran[3][1], tran[3][2] ]])
        pelvis_m_new = pl.array([[ rot[2][0], rot[2][1], rot[2][2] ]])
        com_m_new = pl.array([[ tran[1][0], tran[1][1], tran[1][2] ]])
        
        ## Check if step phase changed and update robot state accordingly:
        if self.step_phase_changed:
            self.init_Change_step_phase()

        ## update samples in buffer:
        if self.num_of_samples_in_buffer < Robot_State.sample_buffer_size: # when buffer isn't full count number of samples in buffer 
            self.num_of_samples_in_buffer += 1
        #     self.buffer_full = False
        # else:
        #     self.buffer_full = True

        
        # rospy.loginfo("Robot State Update_State: swing_foot_arr size - col = %f, row = %f" \
        #        % ( self.swing_foot_arr[1:].shape[0], self.swing_foot_arr[1:].shape[1] ) )
        # # Wait 1 second
        # rospy.sleep(1)

        #buffer_end = self.sample_buffer_size + 1
        self.stance_hip_arr = pl.vstack([ self.stance_hip_arr[1:], stance_hip_new ])
        self.swing_foot_arr = pl.vstack([ self.swing_foot_arr[1:], swing_foot_new ])
        self.swing_hip_arr = pl.vstack([ self.swing_hip_arr[1:], swing_hip_new ])
        self.pelvis_m_arr = pl.vstack([ self.pelvis_m_arr[1:], pelvis_m_new ])
        self.com_m_arr = pl.vstack([ self.com_m_arr[1:], com_m_new ])

        ## filter: calc. moving average
        self.Moving_Avg_Filter()

        ## Calc. step phase average
        self.num_of_avg_samples += 1
        # self.last_Fint = force
        # self.Fint_sum += force

        # #self.last_update_stamp = time_stamp
        # if self.reset_avg_flag:
        #     self.avg_start_time = rospy.get_rostime() #time_stamp
        #     self.reset_avg_flag = False

        ## Selecet desired State value:
        self.stance_hip = self.array2Pos( self.stance_hip_filtered )
        self.swing_foot = self.array2Pos_Ori( self.swing_foot_filtered )
        self.swing_hip = self.array2Pos( self.swing_hip_filtered ) #_arr[-1,:] )
        self.pelvis_m = self.array2Ori( self.pelvis_m_filtered )
        self.com_m = self.array2Pos( self.com_m_filtered )  


        return()

    # External method:
    def getRobot_State(self, listener, step_phase = None ):
        if step_phase is None:
            step_phase = self.__step_phase
        # tf frames:
        if ( step_phase == 1 ) or ( step_phase == 2 ): 
            # stance = left, swing/support = right
            base_frame = 'l_foot'
            # frames to transform: stance hip, com_m_arr, pelvis_m_arr, swing_hip_arr, swing_foot_arr
            get_frames = [ 'l_uleg', 'com', 'pelvis', 'r_uleg', 'r_foot' ] # DRC (from tf) names 
        elif ( step_phase == 3 ) or ( step_phase == 4 ): 
            # stance = right, swing/support = left
            base_frame = 'r_foot'
            # frames to transform: stance hip, com_m_arr, pelvis_m_arr, swing_hip_arr, swing_foot_arr
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

        self.Update_State(tran, rot)

        self.last_tran = tran
        self.last_rot = rot

        return ()

    # External method:
    # need to run after tf is running and robot is static, to get information on robots starting position     
    def static_init_with_tf_data(self,listener,bend_knees): 
        # init state:
        # waiting for transform to be avilable
        time_out = rospy.Duration(2)
        polling_sleep_duration = rospy.Duration(0.01)
        # stance = left, swing = right
        base_frame = 'l_foot'
        # frames to transform: stance hip, com_m_arr, pelvis_m_arr, swing_hip_arr, swing_foot_arr
        get_frames = [ 'l_uleg', 'com', 'pelvis', 'r_uleg', 'r_foot' ] # DRC (from tf) names 
        for i in range(0,5):
            while listener.waitForTransform (base_frame, get_frames[i], rospy.Time(0), time_out, polling_sleep_duration) and not rospy.is_shutdown():
                rospy.loginfo("Not ready for Forward Kinematics transform")

        # Wait 1 second used for printing
        rospy.sleep(1)

        for i in range(0,Robot_State.sample_buffer_size):
            self.getRobot_State( listener = listener, step_phase = 3)
            rospy.loginfo("Robot State static_init-getRS R sample num.%i: stance_hip_filtered - x = %f, y = %f, z = %f;  stance_hip_last - x = %f, y = %f, z = %f" \
               % ( self.num_of_samples_in_buffer, self.stance_hip.x, self.stance_hip.y, self.stance_hip.z, float(self.stance_hip_arr[-1,0]), float(self.stance_hip_arr[-1,1]), float(self.stance_hip_arr[-1,2])) )
            # Wait 0.02 second
            rospy.sleep(0.03)

        com_0_from_r_foot = self.com_m_filtered.copy() #copy.deepcopy( self.com_m_arr )
        self.r_stance_hip_0 = self.stance_hip_filtered.copy() #copy.deepcopy( self.stance_hip_arr ) 
        delta_between_hips = self.stance_hip_filtered - self.swing_hip_filtered
        self.rf_swing_hip_dy = pl.sqrt( pl.dot( delta_between_hips,delta_between_hips ) ) # distance between hips in right leg frame

        # Wait 1 second used for printing
        rospy.sleep(1)

        self.init_Change_step_phase() # reset robot state buffers 

        for i in range(0,Robot_State.sample_buffer_size):
            self.getRobot_State( listener = listener, step_phase = 1)
            rospy.loginfo("Robot State static_init-getRS L sample num.%i: stance_hip_filtered - x = %f, y = %f, z = %f;  stance_hip_last - x = %f, y = %f, z = %f" \
               % ( self.num_of_samples_in_buffer, self.stance_hip.x, self.stance_hip.y, self.stance_hip.z, float(self.stance_hip_arr[-1,0]), float(self.stance_hip_arr[-1,1]), float(self.stance_hip_arr[-1,2])) )
            # Wait 0.02 second
            rospy.sleep(0.03)
        
        # Wait 1 second used for printing
        rospy.sleep(1)
        
        self.l_stance_hip_0 = self.stance_hip_filtered.copy() #copy.deepcopy( self.stance_hip_arr )
        # com_0_from_l_foot = self.com_m_arr.copy() #copy.deepcopy( self.com_m_arr )
        # rs_from_l_foot_swing_foot = self.swing_foot_arr.copy() #copy.deepcopy( self.swing_foot_arr )
        # rs_from_l_foot_swing_hip = self.swing_hip_arr.copy() #copy.deepcopy( self.swing_hip_arr )
        # rs_from_l_foot_pelvis_m = self.pelvis_m_arr.copy() #copy.deepcopy( self.pelvis_m_arr )

        delta_between_hips = self.stance_hip_filtered - self.swing_hip_filtered
        self.lf_swing_hip_dy = pl.sqrt( pl.dot( delta_between_hips,delta_between_hips ) )  # distance between hips in left leg frame

        # TODO: in order to get better readings one should repeat the lines above a few times while init position function and average the readings  

        # constraint: we want hips height (z) and advance (x) to be the same
        hip_height = 0.7999 - bend_knees #(self.l_stance_hip_0.z + self.r_stance_hip_0.z)/2
        hip_sagital = -0.02 # (self.l_stance_hip_0.x + self.r_stance_hip_0.x)/2 # x position relative to foot
        self.l_stance_hip_0[2] = hip_height; self.r_stance_hip_0[2] = hip_height; # update z coord.
        self.l_stance_hip_0[0] = hip_sagital; self.r_stance_hip_0[0] = hip_sagital; # update x coord.

        # self.com2l_stance_hip = Position(); self.com2r_stance_hip = Position(); 
        # self.com2l_stance_hip.x = com_0_from_l_foot.x - self.l_stance_hip_0.x # to be used when using measured com. we get the relative postion of the left stance hip
        # self.com2l_stance_hip.y = com_0_from_l_foot.y - self.l_stance_hip_0.y
        # self.com2l_stance_hip.z = com_0_from_l_foot.z - self.l_stance_hip_0.z
        # self.com2r_stance_hip.x = com_0_from_r_foot.x - self.r_stance_hip_0.x # to be used when using measured com. we get the relative postion of the right stance hip
        # self.com2r_stance_hip.y = com_0_from_r_foot.y - self.r_stance_hip_0.y
        # self.com2r_stance_hip.z = com_0_from_r_foot.z - self.r_stance_hip_0.z
        # use with measured com  e.g: com_m_arr - com2_stance_hip + com_ref = stance hip in the foot coordinate system to be used by IK
        # use with out e.g: stance_hip_0 + com_ref = stance hip in the foot coordinate system to be used by IK

        rospy.loginfo("Robot State static_init: lf_swing_hip_dy = %f l_stance_hip_0 - x = %f, y = %f, z = %f; rf_swing_hip_dy = %f r_stance_hip_0 - x = %f, y = %f, z = %f" \
               % (self.lf_swing_hip_dy, float(self.l_stance_hip_0[0]), float(self.l_stance_hip_0[1]), float(self.l_stance_hip_0[2]), self.rf_swing_hip_dy, float(self.r_stance_hip_0[0]), float(self.r_stance_hip_0[1]), float(self.r_stance_hip_0[2])) )


        # self.swing_foot_arr = rs_from_l_foot_swing_foot
        # self.swing_hip_arr = rs_from_l_foot_swing_hip
        # self.pelvis_m_arr = rs_from_l_foot_pelvis_m
        # self.com_m_arr = com_0_from_l_foot

        return ()
