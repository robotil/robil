#!/usr/bin/env python

###############################################################################
####                                                                         ##
####  swing_trajectory.py (based on zmp_main(old).py)                        ##
####  Created - Israel 21/2/2013 	                                     ##
####  last updated - version 1.0, Israel 21/2/2013                            ##
####                                                                         ##
####                                                                         ##
####                                                                         ##
###############################################################################

import roslib; roslib.load_manifest('C42_ZMPWalk')
import rospy, sys, os.path
from pylab import *
from zmp_profiles import *
import copy

class vector():
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.r = 0.0
        self.p = 0.0
        self.w = 0.0

class Swing_Trajectory:
    'Creates trajectory for swing foot depending on step phase and walking phase'
    # Class parameters:

    def __init__(self, name):
        self.name = name
        #self.step_start_state = 0



    def Get_swing_foot_traj(self, k, step_time, robot_foot_state, step_length,step_width, step_height,dt,pre_step,first_step,full_step,last_step):
    
        k_total = step_time*(1/dt)
        k_start_swing =  floor(k_total/3)
        k_stop_swing =   floor(2*k_total/3)
        
        res_swing_k = vector()

        # step_phase = rs.Get_step_phase()
    
        # robot_foot_state = copy.copy(rs.swing_foot)            ########### need to change this !!!!!!!!!!!!!!!!!!!!!!
        #rospy.loginfo( step_phase )
        # if step_phase == 1 or step_phase == 2: # makes sure to correct step width for right and left leg
        #    step_width = -1*abs(step_width)
        # elif step_phase == 3 or step_phase == 4:
        #    step_width = abs(step_width)
    
        if pre_step:
    
           res_swing_k = copy.copy( robot_foot_state )
           lifting_swing_foot = False
    
    
        if first_step or full_step:
             
            if k<k_start_swing:
         
                res_swing_k =  copy.copy( robot_foot_state )
                lifting_swing_foot = False
    
            ### the following can be eareased -  though it is used now due to strong vibrations in the curennt state:######
            #    if first_step:
            #          res_swing_k.x = 0
            #    elif full_step:
             #         res_swing_k.x = -step_length
         
            #    res_swing_k.y = step_width
            #   res_swing_k.z = 0
            #    res_swing_k.r = 0
            #    res_swing_k.p = 0
           #     res_swing_k.w = 0
           ######################################### 
            elif(k>=k_start_swing  and k<k_stop_swing ):
    
                if k==k_start_swing:
                   self.step_start_state = copy.copy( robot_foot_state )
                # if step_phase == 1:  
                #     step_phase = 2
                # elif step_phase == 3:
                #     step_phase = 4
    
                res_swing_k = self.full_step_traj(self.step_start_state,step_length,step_width, step_height,k_start_swing, k_stop_swing,k_total,k,dt)
                lifting_swing_foot = True

            else:
                
                #res_swing_k = robot_foot_state
                res_swing_k.x = step_length
                res_swing_k.y = step_width
                res_swing_k.z = 0
                res_swing_k.r = 0
                res_swing_k.p = 0
                res_swing_k.w = 0
                lifting_swing_foot = False
    
        if last_step: 
    
            if k<k_start_swing:
                    
               res_swing_k = copy.copy( robot_foot_state )
               lifting_swing_foot = False
     
                
            if(k>k_start_swing  and k<k_stop_swing ):
                if k==k_start_swing:
                   self.step_start_state = copy.copy( robot_foot_state )
                # if step_phase == 1:  
                #     rs.Set_step_phase(2)
                # elif step_phase == 3:
                #     rs.Set_step_phase(4)
                res_swing_k = self.last_step_traj(self.step_start_state,step_length,step_width, step_height,k_start_swing, k_stop_swing,k_total,k,dt)
                lifting_swing_foot = True

            else:
                 
                #res_swing_k = robot_foot_state
                res_swing_k.x = step_length
                res_swing_k.y = step_width
                res_swing_k.z = 0
                res_swing_k.r = 0
                res_swing_k.p = 0
                res_swing_k.w = 0
                lifting_swing_foot = False
        return(res_swing_k, lifting_swing_foot)


    def full_step_traj(self, start_state,step_length,step_width,step_height,k_start_swing,k_stop_swing,k_total,k,dt):
    
        res_swing_k = vector()
        number_of_samples = k_stop_swing - k_start_swing
        sample_time = dt
    
        a = transition_Min_jerk(start_state.x,step_length, number_of_samples) #, sample_time)
        res_swing_k.x = a[k-k_start_swing]
        b = transition_Min_jerk(start_state.y,step_width, number_of_samples) #,, sample_time)
        res_swing_k.y = b[k-k_start_swing]
        c = transition_Min_jerk_via_point(start_state.z,start_state.z+step_height, 0,number_of_samples, sample_time)
        res_swing_k.z = c[k-k_start_swing]
        
        
        res_swing_k.r = 0
        res_swing_k.p = 0
        res_swing_k.w = 0
    
        return(res_swing_k)



    def last_step_traj(self, start_state,step_length,step_width,step_height,k_start_swing,k_stop_swing,k_total,k,dt):
    
        res_swing_k = vector()
        number_of_samples = k_stop_swing - k_start_swing
        sample_time = dt
    
        a = transition_Min_jerk(start_state.x,0, number_of_samples) #,, sample_time)
        res_swing_k.x = a[k-k_start_swing]
        b = transition_Min_jerk(start_state.y,step_width, number_of_samples) #,, sample_time)
        res_swing_k.y = b[k-k_start_swing]
        c = transition_Min_jerk_via_point(start_state.z,start_state.z+step_height, 0,number_of_samples, sample_time)
        res_swing_k.z = c[k-k_start_swing]
        
        res_swing_k.r = 0
        res_swing_k.p = 0
        res_swing_k.w = 0
    
    
    
        return(res_swing_k)