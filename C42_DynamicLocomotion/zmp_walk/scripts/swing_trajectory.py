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

import roslib; roslib.load_manifest('zmp_walk')
import rospy, sys, os.path
from pylab import *
from zmp_profiles import *

class vector():
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.r = 0.0
        self.p = 0.0
        self.w = 0.0


def full_step_traj(step_phase,start_state,step_length,step_width,step_height,k_start_swing,k_stop_swing,k_total,k,dt):

    swing_k = vector()
    number_of_samples = k_stop_swing - k_start_swing
    sample_time = dt

    a = transition_Min_jerk(start_state.x,step_length, number_of_samples, sample_time)
    swing_k.x = a[k-k_start_swing]
    b = transition_Min_jerk(start_state.y,step_width, number_of_samples, sample_time)
    swing_k.y = b[k-k_start_swing]
    c = transition_Min_jerk_via_point(start_state.z,start_state.z+step_height, 0,number_of_samples, sample_time)
    swing_k.z = c[k-k_start_swing]
    



    return(swing_k)



def last_step_traj(step_phase,start_state,step_length,step_width,step_height,k_start_swing,k_stop_swing,k_total,k,dt):

    swing_k = vector()
    number_of_samples = k_stop_swing - k_start_swing
    sample_time = dt

    a = transition_Min_jerk(start_state.x,0, number_of_samples, sample_time)
    swing_k.x = a[k-k_start_swing]
    b = transition_Min_jerk(start_state.y,step_width, number_of_samples, sample_time)
    swing_k.y = b[k-k_start_swing]
    c = transition_Min_jerk_via_point(start_state.z,start_state.z+step_height, 0,number_of_samples, sample_time)
    swing_k.z = c[k-k_start_swing]
    



    return(swing_k)


