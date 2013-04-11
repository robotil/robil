#!/usr/bin/env python

###################################################################################
####	                                                                         ##
####	hip_z_orientation_correction.py                                          ##
####	Created - Israel 14/03/2013 	                                         ##
####                                                                             ##
####                                                                             ##
####	                                                                  	 ##
###################################################################################			

import roslib; roslib.load_manifest('C42_ZMPWalk') 
import rospy, sys #,os.path
from pylab import *
from C42_ZMPWalk.msg import LeftRight  #traj
from zmp_profiles import *
import tf
import copy
from robot_state import Robot_State


class Orientation_Control(): 
      

     def __init__(self):
         self.hip_z_correction = 0
         self.Z_corr_sum = 0
         self.Z_corr_avg = 0

     def hip_z_orientation_correction( self,Des_Orientation,imu_orientation,k,dt,step_phase,k_total,k_start_swing,k_stop_swing):
         
         hip_z = LeftRight()
         (r, p, w) = tf.transformations.euler_from_quaternion([imu_orientation.x, imu_orientation.y, imu_orientation.z, imu_orientation.w])
         
         if k<k_start_swing:

            Z_corr_k =  Des_Orientation - w

            if Z_corr_k > math.pi:
               Z_corr_k = Z_corr_k - 2*math.pi
            if Z_corr_k < -math.pi:
               Z_corr_k = Z_corr_k + 2*math.pi
           
            self.Z_corr_sum = self.Z_corr_sum + Z_corr_k
            
         if k==k_start_swing:
            self.Z_corr_avg = self.Z_corr_sum/k_start_swing
            self.Z_corr_sum = 0
         const = 0.5
         if abs(self.Z_corr_avg) >= const*0.06:
            del_Z_corr_max = const*0.06
         elif abs(self.Z_corr_avg) >= const*0.03:
            del_Z_corr_max = const*0.03
         else: 
            del_Z_corr_max = const*0.01

         number_of_samples = k_stop_swing - k_start_swing

         if (step_phase==1) or (step_phase==3):
 
                if (step_phase==1): #right is swing leg
                    hip_z.left = 0.0 
                    hip_z.right = self.hip_z_correction
                if (step_phase==3): #left is swing leg
                    hip_z.right =  0.0 
                    hip_z.left =  self.hip_z_correction

        
         if (step_phase==2): #right is swing leg
             
             s = k - k_start_swing
             
             if self.Z_corr_avg > del_Z_corr_max:

                if k < k_stop_swing:
                   hip_z.right =  Min_jerk_by_k(self.hip_z_correction,0.0, number_of_samples,s)
                   hip_z.left =  Min_jerk_by_k(0.0,-del_Z_corr_max, number_of_samples,s)
                else:
                   hip_z.right =  0.0 
                   hip_z.left = -del_Z_corr_max
                   self.hip_z_correction = hip_z.left

             elif self.Z_corr_avg < -del_Z_corr_max :

                if k < k_stop_swing:
                   hip_z.right =  Min_jerk_by_k(self.hip_z_correction,0.0, number_of_samples,s)
                   hip_z.left =  Min_jerk_by_k(0.0,del_Z_corr_max, number_of_samples,s)
                else:
                   hip_z.right =  0.0
                   hip_z.left = del_Z_corr_max
                   self.hip_z_correction = hip_z.left
             else:
               
                if k < k_stop_swing:
                   hip_z.right =  Min_jerk_by_k(self.hip_z_correction,0.0, number_of_samples,s)
                   hip_z.left =  0.0
                else:
                   hip_z.right =  0.0
                   hip_z.left = 0.0
                   self.hip_z_correction = 0.0

         if (step_phase==4): #left is swing leg

            s = k - k_start_swing
            
            if self.Z_corr_avg > del_Z_corr_max :
                 
                if k < k_stop_swing:
                   hip_z.left =  Min_jerk_by_k(self.hip_z_correction,0.0, number_of_samples,s)
                   hip_z.right = Min_jerk_by_k(0.0,-del_Z_corr_max, number_of_samples,s)
                else:
                   hip_z.left = 0.0
                   hip_z.right = -del_Z_corr_max
                   self.hip_z_correction = hip_z.right

            elif self.Z_corr_avg < -del_Z_corr_max :
                 
                if k < k_stop_swing:
                   hip_z.left =  Min_jerk_by_k(self.hip_z_correction,0.0, number_of_samples,s)
                   hip_z.right =  Min_jerk_by_k(0.0,del_Z_corr_max, number_of_samples,s)
                else:
                   hip_z.left = 0.0
                   hip_z.right = del_Z_corr_max
                   self.hip_z_correction = hip_z.right
            else:
           
                if k < k_stop_swing:
                   hip_z.left =  Min_jerk_by_k(self.hip_z_correction,0.0, number_of_samples,s)
                   hip_z.right =  0.0
                else:
                   hip_z.right =  0.0
                   hip_z.left = 0.0
                   self.hip_z_correction = 0.0

         return hip_z
