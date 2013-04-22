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

import roslib; roslib.load_manifest('C42_ZMPWalk')
import rospy
from C42_ZMPWalk.msg import Position, Orientation, Pos_and_Ori
import pylab as pl
import math as ma
import copy


class Robot_State:
    'Keeps track of robots lower limbs positions using tf - foward kinematics, filters sampled data '
    # Class parameters:
    sample_buffer_size = 10 # number of samples to keep in history of robot state (tf location readings) 
    samples_in_start_of_step_phase_avg = 10 # number of samples to average pose at begining of step phase

    def __init__(self, name):
        self.name = name
        self.last_tran = [ [0,0,0], [0,0,0], [0,0,0], [0,0,0], [0,0,0] ]  # previous translation values, insert according to default starting position values 
        self.last_rot = [ [0,0,0], [0,0,0], [0,0,0], [0,0,0], [0,0,0] ]  # previous rotation values, insert according to default starting position values 
        
        self.stance_hip = Position()
        self.swing_foot = Pos_and_Ori()
        self.swing_hip = Position()
        self.pelvis_m = Pos_and_Ori() #Orientation()
        self.com_m = Position()
        self.swing_foot_at_start_of_step_phase = Pos_and_Ori()

        # # init buffer of robot state
        # self.stance_hip_arr = zeros((Robot_State.sample_buffer_size,3)) #Position()
        # self.swing_foot_arr = zeros((Robot_State.sample_buffer_size,6)) #Pos_and_Ori()
        # self.swing_hip_arr = zeros((Robot_State.sample_buffer_size,3)) #Position()
        # self.pelvis_m_arr = zeros((Robot_State.sample_buffer_size,3)) #Orientation()
        # self.com_m_arr = zeros((Robot_State.sample_buffer_size,3)) #Position()

        self.stance_hip_filtered = pl.zeros((3)) #Position()
        self.swing_foot_filtered = pl.zeros((6)) #Pos_and_Ori()
        self.swing_hip_filtered = pl.zeros((3)) #Position()
        self.pelvis_m_filtered = pl.zeros((6)) #Orientation()
        self.com_m_filtered = pl.zeros((3)) #Position()

        self.com_0_from_l_foot = pl.zeros((3)) #Position()

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
    def Set_step_phase(self, value = None, foot_lift = False):       
        if value is None:
            # value = self.__step_phase
            if foot_lift and ( (self.__step_phase == 1) or (self.__step_phase == 3) ): 
            # on the start of foot lift we move the step phase one step forward
                self.previous_step_phase = copy.copy(self.__step_phase)
                self.step_phase_changed = True
                self.__step_phase = self.__step_phase + 1 

        else: # set step phase to given value
            self.previous_step_phase = copy.copy(self.__step_phase)
            self.step_phase_changed = True
            self.__step_phase = copy.copy(value)
        return()

    # External auxiliary method:
    def Get_foot_coord_params(self,step_width):
    # Retrieves parameters that are depended on the coordinate system that we are using (right or left foot).
    # Which coordinate system we are using is determined by the current step phase:
        if ( self.__step_phase == 1 ) or ( self.__step_phase == 2 ):
            # stance = left, swing = right
            res_stance_hip_0 = self.l_stance_hip_0.copy() # left stance hip initial position (before starting to walk) in left foot coord. system
            res_stance_hip_0[1] = (self.swing_hip_dy - step_width)/2 # update y coord
            res_pelvis_pos_0 = self.l_pelvis_pos_0.copy() 
            res_pelvis_pos_0[1] = -step_width/2
            res_swing_y_sign = -1.0 # relative direction of swing leg from current coord system 
        elif ( self.__step_phase == 3 ) or ( self.__step_phase == 4 ): 
            # stance = right, swing = left
            res_stance_hip_0 = self.r_stance_hip_0.copy() # right stance hip initial position (before starting to walk) in right foot coord. system
            res_stance_hip_0[1] = (step_width - self.swing_hip_dy)/2 # update y coord
            res_pelvis_pos_0 = self.r_pelvis_pos_0.copy() 
            res_pelvis_pos_0[1] = step_width/2
            res_swing_y_sign = 1.0 # relative direction of swing leg from current coord system
        
        res_stance_hip_0[0] = res_stance_hip_0[0] - self.change_in_foot_coordinates[0]/2
        res_swing_hip_dy = self.swing_hip_dy*res_swing_y_sign # swing hip relative position to stance hip
        return(self.array2Pos( res_stance_hip_0 ), res_swing_y_sign, res_swing_hip_dy )

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
            self.pelvis_m_arr = pl.zeros((Robot_State.sample_buffer_size,6)) #Orientation()
            self.com_m_arr = pl.zeros((Robot_State.sample_buffer_size,3)) #Position()

            self.num_of_samples_in_buffer = 0
            #self.buffer_full = False
            self.change_in_foot_coordinates = self.previous_swing_foot_filtered[0:3].copy()

        if self.__step_phase == 1:
            self.com_0_from_foot = self.com_0_from_l_foot
        if self.__step_phase == 3:
            self.com_0_from_foot = self.com_0_from_r_foot

        ## Reset step phase average parameters
        self.count_avg_samples = 0
        #self.reset_avg_flag = True
        self.stance_hip_sum = pl.zeros((3)) #Position()
        self.swing_foot_sum = pl.zeros((6)) #Pos_and_Ori()
        self.swing_hip_sum = pl.zeros((3)) #Position()
        self.pelvis_m_sum = pl.zeros((6)) #Orientation()
        self.com_m_sum = pl.zeros((3)) #Position()

        self.count_start_samples = 0

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
        stance_hip_new = pl.array([ tran[0][0], tran[0][1], tran[0][2] ])
        swing_foot_new = pl.array([ tran[4][0], tran[4][1], tran[4][2], rot[4][0], rot[4][1], rot[4][2] ])
        swing_hip_new = pl.array([ tran[3][0], tran[3][1], tran[3][2] ])
        pelvis_m_new = pl.array([ tran[2][0], tran[2][1], tran[2][2], rot[2][0], rot[2][1], rot[2][2] ])
        com_m_new = pl.array([ tran[1][0], tran[1][1], tran[1][2] ])
        
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
        self.count_avg_samples += 1

        self.stance_hip_sum = self.stance_hip_sum + stance_hip_new
        self.swing_foot_sum = self.swing_foot_sum + swing_foot_new
        self.swing_hip_sum = self.swing_hip_sum + swing_hip_new
        self.pelvis_m_sum = self.pelvis_m_sum + pelvis_m_new
        self.com_m_sum = self.com_m_sum + com_m_new

        self.stance_hip_avg = self.stance_hip_sum/self.count_avg_samples
        self.swing_foot_avg = self.swing_foot_sum/self.count_avg_samples
        self.swing_hip_avg = self.swing_hip_sum/self.count_avg_samples
        self.pelvis_m_avg = self.pelvis_m_sum/self.count_avg_samples
        self.com_m_avg = self.com_m_sum/self.count_avg_samples

        # #self.last_update_stamp = time_stamp
        # if self.reset_avg_flag:
        #     self.avg_start_time = rospy.get_rostime() #time_stamp
        #     self.reset_avg_flag = False

        ## Pose at begining of step phase:
        if self.count_start_samples < Robot_State.samples_in_start_of_step_phase_avg:
            self.count_start_samples += 1
            self.stance_hip_start = self.stance_hip_avg.copy()
            self.swing_foot_start = self.swing_foot_avg.copy()
            self.swing_hip_start = self.swing_hip_avg.copy()
            self.pelvis_m_start = self.pelvis_m_avg.copy()
            self.com_m_start = self.com_m_avg.copy()

        # rospy.loginfo("Robot State Update_State: count_start_samples = %f, swing_foot_start x=%f, y=%f,  z=%f, p=%f, r=%f, w=%f" \
        #        % ( self.count_start_samples, self.swing_foot_start[0], self.swing_foot_start[1], self.swing_foot_start[2], self.swing_foot_start[3], self.swing_foot_start[4], self.swing_foot_start[5]) ) #,self.swing_foot_start.shape[1] ) )
        # # Wait 1 second
        # rospy.sleep(1)

        ## Selecet desired State value to output:
        self.stance_hip = self.array2Pos( self.stance_hip_arr[-1,:] ) # last sampled value
        self.swing_foot = self.array2Pos_Ori( self.swing_foot_filtered )
        self.swing_hip = self.array2Pos( self.swing_hip_arr[-1,:] ) #_arr[-1,:] ) # last sampled value
        self.pelvis_m = self.array2Ori( self.pelvis_m_filtered[3:6] ) # future change to self.array2Pos_Ori( self.pelvis_m_filtered ) 
        self.com_m = self.array2Pos( self.com_m_arr[-1,:] ) # last sampled value 
        self.com_m.y =  self.com_m_arr[-1,1] - self.com_0_from_foot[1] #+ self.change_in_foot_coordinates[1]
        self.swing_foot_at_start_of_step_phase = self.array2Pos_Ori( self.swing_foot_start )

        return()

    # External method:
    def getRobot_State(self, listener, step_phase = None ):
        if step_phase is None:
            step_phase = self.__step_phase
        # tf frames:
        if ( step_phase == 1 ) or ( step_phase == 2 ): 
            # stance = left, swing/support = right
            base_frame =  'l_foot' #'l_talus'
            # frames to transform: stance hip, com_m_arr, pelvis_m_arr, swing_hip_arr, swing_foot_arr
            get_frames = [ 'l_uleg', 'com', 'pelvis', 'r_uleg', 'r_foot', 'l_lleg' ] # DRC (from tf) names # 'l_lleg' add for debug links length
        elif ( step_phase == 3 ) or ( step_phase == 4 ): 
            # stance = right, swing/support = left
            base_frame = 'r_foot'
            # frames to transform: stance hip, com_m_arr, pelvis_m_arr, swing_hip_arr, swing_foot_arr
            get_frames = [ 'r_uleg', 'com', 'pelvis', 'l_uleg', 'l_foot', 'r_lleg' ] # DRC (from tf) names # 'r_lleg' add for debug links length   
        # get transforms:
        tran = self.last_tran
        rot = self.last_rot
        for i in range(0,5):
            successful_tf = True
            try:
              (translation,rotation) = listener.lookupTransform(base_frame, get_frames[i], rospy.Time(0))  #  rospy.Time(0) to use latest availble transform 
            except: # (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
              successful_tf = False
              # print ex
              rospy.loginfo("tf exception: ")  # %s" % (ex.args))
              # Wait 1 second
              # rospy.sleep(1)
              continue
            if successful_tf:
                tran[i] = translation
                rot[i] = rotation

        # ## debug calc. links length:
        # (translation,rotation) = listener.lookupTransform(base_frame, get_frames[5], rospy.Time(0))  #  rospy.Time(0) to use latest availble transform 
        # lower_leg_length = ma.sqrt(translation[0]**2 + translation[1]**2 + translation[2]**2)
        # upper_leg_length = ma.sqrt((tran[0][0]-translation[0])**2 + (tran[0][1]-translation[1])**2 + (tran[0][2]-translation[2])**2)
        # rospy.loginfo("Knee position: x=%f, y=%f , z=%f, Hip position: x=%f, y=%f , z=%f; Lower leg length = %f, Upper leg length = %f" \
        #     % (translation[0], translation[1], translation[2], tran[0][0], tran[0][1], tran[0][2], lower_leg_length, upper_leg_length) )

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
        get_frames = [ 'l_uleg', 'com', 'pelvis', 'r_uleg', 'r_foot', 'r_lleg' ] # DRC (from tf) names 
        for i in range(0,6):
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

        self.com_0_from_r_foot = self.com_m_filtered.copy() #copy.deepcopy( self.com_m_arr )
        self.r_stance_hip_0 = self.stance_hip_filtered.copy() #copy.deepcopy( self.stance_hip_arr ) 
        self.r_pelvis_pos_0 = self.pelvis_m_filtered[0:3].copy()
        delta_between_hips = self.stance_hip_filtered - self.swing_hip_filtered
        self.rf_swing_hip_dy = pl.sqrt( pl.dot( delta_between_hips,delta_between_hips ) ) # distance between hips in right leg frame
        delta_between_feet = self.swing_foot_filtered[0:3].copy()
        rf_step_width = pl.sqrt( pl.dot( delta_between_feet,delta_between_feet ) )  # distance between feet in right leg coord. frame

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
        self.l_pelvis_pos_0 = self.pelvis_m_filtered[0:3].copy()
        self.com_0_from_l_foot = self.com_m_filtered.copy()
        # com_0_from_l_foot = self.com_m_arr.copy() #copy.deepcopy( self.com_m_arr )
        # rs_from_l_foot_swing_foot = self.swing_foot_arr.copy() #copy.deepcopy( self.swing_foot_arr )
        # rs_from_l_foot_swing_hip = self.swing_hip_arr.copy() #copy.deepcopy( self.swing_hip_arr )
        # rs_from_l_foot_pelvis_m = self.pelvis_m_arr.copy() #copy.deepcopy( self.pelvis_m_arr )

        delta_between_hips = self.stance_hip_filtered - self.swing_hip_filtered
        self.lf_swing_hip_dy = pl.sqrt( pl.dot( delta_between_hips,delta_between_hips ) )  # distance between hips in left leg coord. frame
        delta_between_feet = self.swing_foot_filtered[0:3].copy()
        lf_step_width = pl.sqrt( pl.dot( delta_between_feet,delta_between_feet ) )  # distance between feet in left leg coord. frame

        # TODO: in order to get better readings one should repeat the lines above a few times while init position function and average the readings  

        self.swing_hip_dy = (self.lf_swing_hip_dy + self.rf_swing_hip_dy)/2
        step_width = (lf_step_width + rf_step_width)/2

        # constraint: we want hips height (z) and advance (x) to be the same
        hip_height = 0.7999 - bend_knees #(self.l_stance_hip_0.z + self.r_stance_hip_0.z)/2
        hip_sagital = 0.05 #0.05 #-0.02 # (self.l_stance_hip_0.x + self.r_stance_hip_0.x)/2 # x position relative to foot
        self.l_stance_hip_0[2] = hip_height; self.r_stance_hip_0[2] = hip_height; # update z coord.
        self.l_stance_hip_0[0] = hip_sagital; self.r_stance_hip_0[0] = hip_sagital; # update x coord.
        self.l_stance_hip_0[1] = (self.swing_hip_dy - step_width)/2 # update y coord
        self.r_stance_hip_0[1] = (step_width - self.swing_hip_dy)/2 # update y coord

        pelvis_height = 0.85 - bend_knees 
        pelvis_sagital = 0.05
        self.l_pelvis_pos_0[2] = hip_height; self.r_pelvis_pos_0[2] = hip_height; # update z coord.
        self.l_pelvis_pos_0[0] = hip_sagital; self.r_pelvis_pos_0[0] = hip_sagital; # update x coord.
        self.l_pelvis_pos_0[1] = -step_width/2 # update y coord
        self.r_pelvis_pos_0[1] = step_width/2 # update y coord
        
        # self.com2l_stance_hip = Position(); self.com2r_stance_hip = Position(); 
        # self.com2l_stance_hip.x = com_0_from_l_foot.x - self.l_stance_hip_0.x # to be used when using measured com. we get the relative postion of the left stance hip
        # self.com2l_stance_hip.y = com_0_from_l_foot.y - self.l_stance_hip_0.y
        # self.com2l_stance_hip.z = com_0_from_l_foot.z - self.l_stance_hip_0.z
        # self.com2r_stance_hip.x = com_0_from_r_foot.x - self.r_stance_hip_0.x # to be used when using measured com. we get the relative postion of the right stance hip
        # self.com2r_stance_hip.y = com_0_from_r_foot.y - self.r_stance_hip_0.y
        # self.com2r_stance_hip.z = com_0_from_r_foot.z - self.r_stance_hip_0.z
        # use with measured com  e.g: com_m_arr - com2_stance_hip + com_ref = stance hip in the foot coordinate system to be used by IK
        # use with out e.g: stance_hip_0 + com_ref = stance hip in the foot coordinate system to be used by IK

        rospy.loginfo("Robot State static_init: step_width = %f, lf_swing_hip_dy = %f, l_stance_hip_0 - x = %f, y = %f, z = %f; rf_swing_hip_dy = %f r_stance_hip_0 - x = %f, y = %f, z = %f" \
               % (step_width, self.lf_swing_hip_dy, float(self.l_stance_hip_0[0]), float(self.l_stance_hip_0[1]), float(self.l_stance_hip_0[2]), self.rf_swing_hip_dy, float(self.r_stance_hip_0[0]), float(self.r_stance_hip_0[1]), float(self.r_stance_hip_0[2])) )


        # self.swing_foot_arr = rs_from_l_foot_swing_foot
        # self.swing_hip_arr = rs_from_l_foot_swing_hip
        # self.pelvis_m_arr = rs_from_l_foot_pelvis_m
        # self.com_m_arr = com_0_from_l_foot

        return ()
