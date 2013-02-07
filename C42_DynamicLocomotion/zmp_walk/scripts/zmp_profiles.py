#!/usr/bin/env python

###############################################################################
####                                                                         ##
####  zmp_profiles.py (based on zmp_main(old).py)                            ##
####  Created - Yuval 05/2/2013 	                                         ##
####  last updated - version 1.0, Yuval 07/2/2013                            ##
####                                                                         ##
####    Used for creating ZMP reference profiles in the sagital (x) and      ## 
####    lateral (y) planes. Each function describes a desired template of    ##
####    ZMP refrence motion. The templates are replicated by the Preview     ##
####    Buffer to achieve the desired walking pattern.                       ##
####    Directions: x is the forward direction of the robot, y is the left   ##
####    of the robot (and z is up).                                          ##
####    The reference profiles are in world coordinate system, where the     ##
####    origin (x,y)=(0,0) is the starting point of the COM (when the robot  ##
####    is static). This is under the assumption that the robot is           ##
####    symmetrical and starts with both feet alined. Then the ZMP ref point ##
####    for the left foot is at position (0, step_width/2) and for the right ##
####    foot at position (0, -step_width/2). If we want to add an off-set    ##
####    from the origin of the ZMP profile to the actual point of the ZMP we ##
####    can do so by adding it to ZMP_start_pos (can be used when making     ##
####    intial calibration if COM isn't at the desired place. Or to update   ##
####    the initial position of the ZMP at the begining of each step).       ##
####                                                                         ##
####                                                                         ##
###############################################################################

import roslib; roslib.load_manifest('zmp_walk')
import rospy, sys, os.path
from pylab import *


# Sigmoid Function: an auxiliary function to smoothen step trasitions

def  transitionSigmoid(step_size, trans_slope_steepens_factor, number_of_samples, sample_time):
    # trans_slope_steepens_factor = a   1 # change slop of ZMP -> com
    samples_array = linspace(0,number_of_samples-1, number_of_samples) - floor(float(number_of_samples)/2)
    s = samples_array * sample_time
    sigmoid = step_size/(1+exp(-trans_slope_steepens_factor*s))

    return (sigmoid)

def  transitionSigmoid_firstHalf(step_size, trans_slope_steepens_factor, number_of_samples, sample_time):
    # trans_slope_steepens_factor = a   1 # change slop of ZMP -> com
    samples_array = linspace(-(number_of_samples-1), 0, number_of_samples)
    s = samples_array * sample_time
    sigmoid = 2*step_size/(1+exp(-trans_slope_steepens_factor*s))

    return (sigmoid)

def  transitionSigmoid_secondHalf(step_size, trans_slope_steepens_factor, number_of_samples, sample_time):
    # trans_slope_steepens_factor = a   1 # change slop of ZMP -> com
    samples_array = linspace(1,number_of_samples, number_of_samples)
    s = samples_array * sample_time
    sigmoid = 2*step_size/(1+exp(-trans_slope_steepens_factor*s)) - step_size

    return (sigmoid)

# def  transitionSigmoid(step_size, trans_slope_steepens_factor, transition_time, sample_time):
#     # trans_slope_steepens_factor = a   1 # change slop of ZMP -> com
#     s =  arange(-transition_time/2 , transition_time/2 + sample_time , sample_time)
#     sigmoid = step_size/(1+exp(-trans_slope_steepens_factor*s))

#     return (sigmoid)

# def  transitionSigmoid_firstHalf(step_size, trans_slope_steepens_factor, transition_time, sample_time):
#     # trans_slope_steepens_factor = a   1 # change slop of ZMP -> com
#     s =  arange(-transition_time/2, 0 , sample_time)
#     sigmoid = step_size/(1+exp(-trans_slope_steepens_factor*s))

#     return (sigmoid)


#################################################################
#                                                               #
#              ZMP Reference Profile Templates                  #
#                                                               #
#################################################################

def  Constant_Template(const_value, step_time, sample_time):
    samples_in_step = ceil (step_time / sample_time)

    # step 1 : Constant value
    p_ref = const_value*ones( samples_in_step )

    return (p_ref)




#######################################################
#                                                     #
#              sagital profiles  (x)                  #
#                                                     #
#######################################################

def  Start_sagital_x(ZMP_start_pos, step_length, trans_ratio_of_step, trans_slope_steepens_factor, step_time, sample_time):
    samples_in_step = ceil (step_time / sample_time)
    samples_in_transition = floor (trans_ratio_of_step * samples_in_step)
    samples_in_start_step_trans = floor (samples_in_transition/2)
    samples_in_end_step_trans = samples_in_transition - samples_in_start_step_trans
    samples_in_step_without_trans = samples_in_step - samples_in_transition
    # One step sequence is divided into the following sampling order:
    # 1) samples_in_start_step_trans
    # 2) samples_in_step_without_trans
    # 3) samples_in_end_step_trans
    # total number of samples = samples_in_step

    # step 1 until last transition of step2 : stand in place, see Start_lateral_y (step 1)
    #p_ref0_0x = zeros( samples_in_start_step_trans + samples_in_step_without_trans )
    p_ref0_0x = ZMP_start_pos*ones( samples_in_step + samples_in_start_step_trans + samples_in_step_without_trans )
    
    # step 2 end: transition
    half_sigmoid_x = transitionSigmoid_firstHalf(step_length/2 - ZMP_start_pos, trans_slope_steepens_factor, samples_in_end_step_trans, sample_time)

    p_ref_x = r_[ p_ref0_0x , ZMP_start_pos + half_sigmoid_x ]

    return (p_ref_x)

def  Step_forward_x(ZMP_start_pos, step_length, trans_ratio_of_step, trans_slope_steepens_factor, step_time, sample_time):
    samples_in_step = ceil (step_time / sample_time)
    samples_in_transition = floor (trans_ratio_of_step * samples_in_step)
    samples_in_start_step_trans = floor (samples_in_transition/2)
    samples_in_end_step_trans = samples_in_transition - samples_in_start_step_trans
    samples_in_step_without_trans = samples_in_step - samples_in_transition
    # One step sequence is divided into the following sampling order:
    # 1) samples_in_start_step_trans
    # 2) samples_in_step_without_trans
    # 3) samples_in_end_step_trans
    # total number of samples = samples_in_step

    half_step_length = step_length/2

    # step begin : transition
    half_sigmoid_x_begin = transitionSigmoid_secondHalf( half_step_length - ZMP_start_pos, trans_slope_steepens_factor, samples_in_start_step_trans, sample_time)

    # step middle: with out transitions
    p_ref1x = half_step_length*ones( samples_in_step_without_trans )
    
    # step 2 end: transition
    half_sigmoid_x_end = transitionSigmoid_firstHalf(half_step_length, trans_slope_steepens_factor, samples_in_end_step_trans, sample_time)

    p_ref_x = r_[ ZMP_start_pos + half_sigmoid_x_begin, p_ref1x , half_sigmoid_x_end + half_step_length ]

    return (p_ref_x)

def  Stop_sagital_x(ZMP_start_pos, step_length, trans_ratio_of_step, trans_slope_steepens_factor, step_time, sample_time):
    samples_in_step = ceil (step_time / sample_time)
    samples_in_transition = floor (trans_ratio_of_step * samples_in_step)
    samples_in_start_step_trans = floor (samples_in_transition/2)
    samples_in_end_step_trans = samples_in_transition - samples_in_start_step_trans
    samples_in_step_without_trans = samples_in_step - samples_in_transition
    # One step sequence is divided into the following sampling order:
    # 1) samples_in_start_step_trans
    # 2) samples_in_step_without_trans
    # 3) samples_in_end_step_trans
    # total number of samples = samples_in_step

    half_step_length = step_length/2

    # step 1 begin : transition
    half_sigmoid_x_begin = transitionSigmoid_secondHalf( half_step_length - ZMP_start_pos, trans_slope_steepens_factor, samples_in_start_step_trans, sample_time)
    
    # step 1 middle to step 2 end: no movement
    p_ref1x = half_step_length*ones( samples_in_step_without_trans + samples_in_end_step_trans + samples_in_step )

    p_ref_x = r_[ ZMP_start_pos + half_sigmoid_x_begin , ZMP_start_pos + p_ref1x ]

    return (p_ref_x)




#######################################################
#                                                     #
#              lateral profiles  (y)                  #
#                                                     #
#######################################################

def  Start_lateral_y_weight_to_left_foot(ZMP_start_pos, step_width, trans_ratio_of_step, trans_slope_steepens_factor, step_time, sample_time):
    # tuning parameters:
    #trans_slope_steepens_factor = 1 # slop factor of transition, bigger values give a steeper slop

    samples_in_step = ceil (step_time / sample_time)
    samples_in_transition = floor (trans_ratio_of_step * samples_in_step)
    samples_in_start_step_trans = floor (samples_in_transition/2)
    samples_in_end_step_trans = samples_in_transition - samples_in_start_step_trans
    samples_in_step_without_trans = samples_in_step - samples_in_transition
    # One step sequence is divided into the following sampling order:
    # 1) samples_in_start_step_trans 
    # 2) samples_in_step_without_trans
    # 3) samples_in_end_step_trans
    # total number of samples = samples_in_step

    # step 1 : stand in place (we want to minimize the transient effect of the preview)
    # can be skiped in the future by inserting the appropriate initial conditions to the preview controller 
    #p_ref0_0y = (step_width/2)*ones( samples_in_start_step_trans + samples_in_step_without_trans )
    p_ref0_0y = ZMP_start_pos*ones( samples_in_start_step_trans + samples_in_step_without_trans )         ## uncomment to use ZMP_start_pos

    # step 1 to step 2 transition:
    samples_in_trans = samples_in_end_step_trans + samples_in_start_step_trans
    sigmoid_y_small_step = transitionSigmoid( step_width/2 - ZMP_start_pos, trans_slope_steepens_factor, samples_in_trans, sample_time )
    
    # step 2 middle: with out transitions
    p_ref1y = step_width/2*ones( samples_in_step_without_trans )

    # step 2 end: transition
    half_sigmoid_y = transitionSigmoid_firstHalf(step_width/2, trans_slope_steepens_factor, samples_in_end_step_trans, sample_time)



    p_ref_y = r_[ p_ref0_0y , ZMP_start_pos + sigmoid_y_small_step, p_ref1y, step_width/2 - half_sigmoid_y]

    return (p_ref_y)

def  Step_onto_right_foot(ZMP_start_pos, step_width, trans_ratio_of_step, trans_slope_steepens_factor, step_time, sample_time):
    samples_in_step = ceil (step_time / sample_time)
    samples_in_transition = floor (trans_ratio_of_step * samples_in_step)
    samples_in_start_step_trans = floor (samples_in_transition/2)
    samples_in_end_step_trans = samples_in_transition - samples_in_start_step_trans
    samples_in_step_without_trans = samples_in_step - samples_in_transition
    # One step sequence is divided into the following sampling order:
    # 1) samples_in_start_step_trans 
    # 2) samples_in_step_without_trans
    # 3) samples_in_end_step_trans
    # total number of samples = samples_in_step

    # step begin : transition
    half_sigmoid_y_begin = transitionSigmoid_secondHalf(-(step_width/2 + ZMP_start_pos), trans_slope_steepens_factor, samples_in_start_step_trans, sample_time)
    
    # step middle: with out transitions
    p_ref1y = -step_width/2*ones( samples_in_step_without_trans )

    # step 2 end: transition
    half_sigmoid_y = transitionSigmoid_firstHalf(step_width/2, trans_slope_steepens_factor, samples_in_end_step_trans, sample_time)



    p_ref_y = r_[ ZMP_start_pos + half_sigmoid_y_begin , p_ref1y, half_sigmoid_y - step_width/2 ]

    return (p_ref_y)


def  Step_onto_left_foot(ZMP_start_pos, step_width, trans_ratio_of_step, trans_slope_steepens_factor, step_time, sample_time):
    samples_in_step = ceil (step_time / sample_time)
    samples_in_transition = floor (trans_ratio_of_step * samples_in_step)
    samples_in_start_step_trans = floor (samples_in_transition/2)
    samples_in_end_step_trans = samples_in_transition - samples_in_start_step_trans
    samples_in_step_without_trans = samples_in_step - samples_in_transition
    # One step sequence is divided into the following sampling order:
    # 1) samples_in_start_step_trans 
    # 2) samples_in_step_without_trans
    # 3) samples_in_end_step_trans
    # total number of samples = samples_in_step

    # step begin : transition
    half_sigmoid_y_begin = transitionSigmoid_secondHalf( (step_width/2 - ZMP_start_pos), trans_slope_steepens_factor, samples_in_start_step_trans, sample_time)
    
    # step middle: with out transitions
    p_ref1y = step_width/2*ones( samples_in_step_without_trans )

    # step 2 end: transition
    half_sigmoid_y = transitionSigmoid_firstHalf(step_width/2, trans_slope_steepens_factor, samples_in_end_step_trans, sample_time)



    p_ref_y = r_[ ZMP_start_pos + half_sigmoid_y_begin , p_ref1y, step_width/2 - half_sigmoid_y ]

    return (p_ref_y)


def  Stop_lateral_y_from_left_foot(ZMP_start_pos, step_width, trans_ratio_of_step, trans_slope_steepens_factor, step_time, sample_time):
    samples_in_step = ceil (step_time / sample_time)
    samples_in_transition = floor (trans_ratio_of_step * samples_in_step)
    samples_in_start_step_trans = floor (samples_in_transition/2)
    samples_in_end_step_trans = samples_in_transition - samples_in_start_step_trans
    samples_in_step_without_trans = samples_in_step - samples_in_transition
    # One step sequence is divided into the following sampling order:
    # 1) samples_in_start_step_trans 
    # 2) samples_in_step_without_trans
    # 3) samples_in_end_step_trans
    # total number of samples = samples_in_step

    ## Step onto right foot:
    # step 1 begin : transition
    half_sigmoid_y_begin = transitionSigmoid_secondHalf(-(step_width/2 + ZMP_start_pos), trans_slope_steepens_factor, samples_in_start_step_trans, sample_time)
    
    # step 1 middle: with out transitions
    p_ref1y = -step_width/2*ones( samples_in_step_without_trans )

    ## move and stay at double support:
    # step 1 to step 2 transition:
    samples_in_trans = samples_in_end_step_trans + samples_in_start_step_trans
    sigmoid_y_small_step = transitionSigmoid( step_width/2 - ZMP_start_pos, trans_slope_steepens_factor, samples_in_trans, sample_time )

    # step 2 middle and end: 
    p_ref2y = zeros( samples_in_step_without_trans + samples_in_end_step_trans )



    p_ref_y = r_[ ZMP_start_pos + half_sigmoid_y_begin , p_ref1y, sigmoid_y_small_step - step_width/2, p_ref2y ]

    return (p_ref_y)

def  Stop_lateral_y_from_right_foot(ZMP_start_pos, step_width, trans_ratio_of_step, trans_slope_steepens_factor, step_time, sample_time):
    samples_in_step = ceil (step_time / sample_time)
    samples_in_transition = floor (trans_ratio_of_step * samples_in_step)
    samples_in_start_step_trans = floor (samples_in_transition/2)
    samples_in_end_step_trans = samples_in_transition - samples_in_start_step_trans
    samples_in_step_without_trans = samples_in_step - samples_in_transition
    # One step sequence is divided into the following sampling order:
    # 1) samples_in_start_step_trans 
    # 2) samples_in_step_without_trans
    # 3) samples_in_end_step_trans
    # total number of samples = samples_in_step

    ## Step onto right foot:
    # step 1 begin : transition
    half_sigmoid_y_begin = transitionSigmoid_secondHalf( (step_width/2 - ZMP_start_pos), trans_slope_steepens_factor, samples_in_start_step_trans, sample_time)
    
    # step 1 middle: with out transitions
    p_ref1y = step_width/2*ones( samples_in_step_without_trans )

    ## move and stay at double support:
    # step 1 to step 2 transition:
    samples_in_trans = samples_in_end_step_trans + samples_in_start_step_trans
    sigmoid_y_small_step = transitionSigmoid( -(step_width/2 + ZMP_start_pos), trans_slope_steepens_factor, samples_in_trans, sample_time )

    # step 2 middle and end: 
    p_ref2y = zeros( samples_in_step_without_trans + samples_in_end_step_trans )



    p_ref_y = r_[ ZMP_start_pos + half_sigmoid_y_begin , p_ref1y, step_width/2 + sigmoid_y_small_step, p_ref2y ]

    return (p_ref_y)


