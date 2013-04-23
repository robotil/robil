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

import roslib; roslib.load_manifest('C42_ZMPWalk')
import rospy, sys, os.path
from pylab import *

# min jerk functions - added by Israel 24.2

def  Min_jerk_by_k(x0,xf, number_of_samples,s): #, sample_time):
    # x0,xf - are the starting and ending point position of the transition, accordingly
   # rospy.loginfo("min_jerk_value=%d" % s)
    # s is an index between 0 and number_of_samples-1 
    s1 = s/(number_of_samples-1) # s/max(s)
    min_jerk_value = x0 + (x0 -xf)*( 15*(s1)**4 - 6*(s1)**5 -10*(s1)**3 )
                                                                    #  rospy.loginfo("min_jerk_value=%f" % min_jerk_value)
    return (min_jerk_value)

def Min_jerk_via_point_by_k(x0,xm,xf, number_of_samples, sample_time, s):
    # x0,xm,xf - are the starting middle and ending point position of the transition, accordingly
    # s is an index between 0 and number_of_samples-1 
  #  rospy.loginfo("min_jerk_value=%d" % s)
    N = number_of_samples
    dt = sample_time
    tf = (N-1)*dt
    tau1 = 0.5
    
    tau = s*dt/tf

    A1 = 1.0/(tf**5 * tau1**2 * (1.0 - tau1)**5 )
    A2 = 1.0/(tf**5 * tau1**5 * (1.0 - tau1)**5 )

    C1 = A1*( (xf-x0)*(300.0*tau1**5 - 1200.0*tau1**4 + 1600.0*tau1**3) + tau1**2*(-720.0*xf + 120.0*xm + 600.0*x0) + (x0-xm)*(300.0*tau1-200.0) )
    pi1 = A2*( (xf-x0)*(120.0*tau1**5 - 300.0*tau1**4 + 200.0*tau1**3)  -20.0*(xm-x0) )
    
    if (tau <= tau1 ):
        T1 = tau
        min_jerk_value = (tf**5)/720.0*(pi1*(tau1**4*(15.0*T1**4-30.0*T1**3)+tau1**3*(80.0*T1**3-30.0*T1**4)-60.0*T1**3*tau1**2+30.0*T1**4*tau1-6*T1**5)+C1*(15.0*T1**4-10.0*T1**3-6.0*T1**5)) + x0

    else:
        T2 = tau
        min_jerk_value = (tf**5)/720.0*(pi1*(tau1**4*(15.0*T2**4-30.0*T2**3+30.0*T2-15.0)+tau1**3*(-30.0*T2**4+ 80.0*T2**3-60.0*T2**2+10.0))+C1*(-6.0*T2**5+15.0*T2**4-10.0*T2**3 + 1.0)) + xf

    return (min_jerk_value)
    
# min jerk functions - added by Israel 24.2
def  transition_Min_jerk(x0,xf, number_of_samples): #, sample_time):
    # x0,xf - are the starting and ending point position of the transition, accordingly
    samples_array = linspace(0,number_of_samples-1, number_of_samples) #- floor(float(number_of_samples)/2)
    #s = samples_array * sample_time
    s1 = samples_array/(number_of_samples-1) # s/max(s)
    min_jerk_profile = x0 + (x0 -xf)*( 15*(s1)**4 - 6*(s1)**5 -10*(s1)**3 )

    return (min_jerk_profile)

#!!! transition_Min_jerk(x0,xf,N) = r_[ transition_Min_jerk_firstHalf(x0,xm, floor(N/2), N) , transition_Min_jerk_secondHalf(xm,xf, N-floor(N/2), N) ] !!!

def  transition_Min_jerk_firstHalf(x0,xm, samples_in_half_trans, samples_in_whole_trans):
    # x0,xm - values of the resulting array start from x0 and end at xm  
    xf = 2.0*xm - x0
    samples_array = linspace(0,samples_in_half_trans-1, samples_in_half_trans) 
    s1 = samples_array/(samples_in_whole_trans-1)
    half_min_jerk_profile = x0 + (x0 - xf)*( 15*(s1)**4 - 6*(s1)**5 -10*(s1)**3 )

    return (half_min_jerk_profile)

def  transition_Min_jerk_secondHalf(xm,xf, samples_in_half_trans, samples_in_whole_trans):
    # xm,xf - values of the resulting array start from xm and end at xf
    x0 = 2.0*xm - xf
    samples_in_first_half_trans = samples_in_whole_trans - samples_in_half_trans
    samples_array = linspace( samples_in_first_half_trans, samples_in_whole_trans-1, samples_in_half_trans ) 
    s1 = samples_array/(samples_in_whole_trans-1)
    half_min_jerk_profile = x0 + (x0 - xf)*( 15*(s1)**4 - 6*(s1)**5 -10*(s1)**3 ) 

    return (half_min_jerk_profile)

def transition_Min_jerk_via_point(x0,xm,xf, number_of_samples, sample_time):

    N = number_of_samples
    dt = sample_time
    tf = (N-1)*dt
    tau1 = 0.5

    T = linspace(0,N-1, N)*dt/tf
    
    T1 = T[0:floor(N/2)]
    T2 = T[floor(N/2):N]

    A1 = 1.0/(tf**5 * tau1**2 * (1.0 - tau1)**5 )
    A2 = 1.0/(tf**5 * tau1**5 * (1.0 - tau1)**5 )

    C1 = A1*( (xf-x0)*(300.0*tau1**5 - 1200.0*tau1**4 + 1600.0*tau1**3) + tau1**2*(-720.0*xf + 120.0*xm + 600.0*x0) + (x0-xm)*(300.0*tau1-200.0) )
    pi1 = A2*( (xf-x0)*(120.0*tau1**5 - 300.0*tau1**4 + 200.0*tau1**3)  -20.0*(xm-x0) )

    x1 = (tf**5)/720.0*(pi1*(tau1**4*(15.0*T1**4-30.0*T1**3)+tau1**3*(80.0*T1**3-30.0*T1**4)-60.0*T1**3*tau1**2+30.0*T1**4*tau1-6*T1**5)+C1*(15.0*T1**4-10.0*T1**3-6.0*T1**5)) + x0


    x2 = (tf**5)/720.0*(pi1*(tau1**4*(15.0*T2**4-30.0*T2**3+30.0*T2-15.0)+tau1**3*(-30.0*T2**4+ 80.0*T2**3-60.0*T2**2+10.0))+C1*(-6.0*T2**5+15.0*T2**4-10.0*T2**3 + 1.0)) + xf


    min_jerk_profile = r_[ x1,x2 ]


    return (min_jerk_profile)
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
    #half_trans_x = ZMP_start_pos + transitionSigmoid_firstHalf(step_length/2 - ZMP_start_pos, trans_slope_steepens_factor, samples_in_end_step_trans, sample_time)
    half_trans_x = transition_Min_jerk_firstHalf( ZMP_start_pos, step_length/2, samples_in_end_step_trans, samples_in_transition)

    p_ref_x = r_[ p_ref0_0x , half_trans_x ]

    return (p_ref_x)

## Start_sagital_x was divided into two single steps: Pre_Step_sagital_x and First_Step_sagital_x
def  Pre_Step_sagital_x(ZMP_start_pos, step_length, trans_ratio_of_step, trans_slope_steepens_factor, step_time, sample_time):
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

    # step 1 : stand in place, see Start_lateral_y (step 1)
    #p_ref0_0x = zeros( samples_in_start_step_trans + samples_in_step_without_trans )
    p_ref0_0x = ZMP_start_pos*ones( samples_in_step )
    
    p_ref_x = p_ref0_0x 

    return (p_ref_x)

def  First_Step_sagital_x(ZMP_start_pos, step_length, trans_ratio_of_step, trans_slope_steepens_factor, step_time, sample_time):
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

    # step 2 until last transition of step2 : stand in place, see Start_lateral_y (step 1)
    #p_ref0_0x = zeros( samples_in_start_step_trans + samples_in_step_without_trans )
    p_ref0_0x = ZMP_start_pos*ones( samples_in_start_step_trans + samples_in_step_without_trans )
    
    # step 2 end: transition
    #half_trans_x = ZMP_start_pos + transitionSigmoid_firstHalf(step_length/2 - ZMP_start_pos, trans_slope_steepens_factor, samples_in_end_step_trans, sample_time)
    half_trans_x = transition_Min_jerk_firstHalf( ZMP_start_pos, step_length/2, samples_in_end_step_trans, samples_in_transition)

    p_ref_x = r_[ p_ref0_0x , half_trans_x ]

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
    # half_trans_x_begin = ZMP_start_pos + transitionSigmoid_secondHalf( half_step_length - ZMP_start_pos, trans_slope_steepens_factor, samples_in_start_step_trans, sample_time)
    half_trans_x_begin = transition_Min_jerk_secondHalf( ZMP_start_pos, half_step_length, samples_in_start_step_trans, samples_in_transition)

    # step middle: with out transitions
    p_ref1x = half_step_length*ones( samples_in_step_without_trans )
    
    # step 2 end: transition
    #half_trans_x_end = transitionSigmoid_firstHalf(half_step_length, trans_slope_steepens_factor, samples_in_end_step_trans, sample_time)
    half_trans_x_end = transition_Min_jerk_firstHalf(0, half_step_length, samples_in_end_step_trans, samples_in_transition)

    p_ref_x = r_[ half_trans_x_begin, p_ref1x , half_trans_x_end + half_step_length ]

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
    # half_trans_x_begin = ZMP_start_pos + transitionSigmoid_secondHalf( half_step_length - ZMP_start_pos, trans_slope_steepens_factor, samples_in_start_step_trans, sample_time)
    half_trans_x_begin = transition_Min_jerk_secondHalf( ZMP_start_pos, half_step_length, samples_in_start_step_trans, samples_in_transition)
    
    # step 1 middle to step 2 end: no movement
    p_ref1x = half_step_length*ones( samples_in_step_without_trans + samples_in_end_step_trans + samples_in_step )

    p_ref_x = r_[ half_trans_x_begin , ZMP_start_pos + p_ref1x ]

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
    #trans_y_small_step = ZMP_start_pos + transitionSigmoid( step_width/2 - ZMP_start_pos, trans_slope_steepens_factor, samples_in_trans, sample_time )
    trans_y_small_step = transition_Min_jerk( ZMP_start_pos, step_width/2, samples_in_trans)
    
    # step 2 middle: with out transitions
    p_ref1y = step_width/2*ones( samples_in_step_without_trans )

    # step 2 end: transition
    # half_trans_y = transitionSigmoid_firstHalf(step_width/2, trans_slope_steepens_factor, samples_in_end_step_trans, sample_time)
    half_trans_y = transition_Min_jerk_firstHalf(0,step_width/2, samples_in_end_step_trans, samples_in_transition)

    p_ref_y = r_[ p_ref0_0y , trans_y_small_step, p_ref1y, step_width/2 - half_trans_y]

    return (p_ref_y)

## Start_lateral_y_weight_to_left_foot was divided into two single steps: Pre_Step_lateral_y and First_Step_lateral_y
def  Pre_Step_lateral_y(ZMP_start_pos, step_width, trans_ratio_of_step, trans_slope_steepens_factor, step_time, sample_time):
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
    p_ref0_0y = ZMP_start_pos*ones( samples_in_start_step_trans + samples_in_step_without_trans )
    #p_ref0_0y = ZMP_start_pos*ones( samples_in_step )      

    # step 1 end: transition
    half_trans_y = transition_Min_jerk_firstHalf( ZMP_start_pos, step_width/4.0, samples_in_start_step_trans, samples_in_transition)
    
    p_ref_y = r_[ p_ref0_0y , half_trans_y ]

    return (p_ref_y)

def  First_Step_lateral_y(ZMP_start_pos, step_width, trans_ratio_of_step, trans_slope_steepens_factor, step_time, sample_time):
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

    # step 2 begin : transition
    half_trans_y_begin = transition_Min_jerk_secondHalf( step_width/4.0, step_width/2.0, samples_in_start_step_trans, samples_in_transition)
    #half_trans_y_begin = transition_Min_jerk_secondHalf( ZMP_start_pos, step_width/2.0, samples_in_start_step_trans, samples_in_transition)
    
    # step 2 middle: with out transitions
    p_ref1y = step_width/2.0*ones( samples_in_step_without_trans )

    # step 2 end: transition
    # half_trans_y = transitionSigmoid_firstHalf(step_width/2, trans_slope_steepens_factor, samples_in_end_step_trans, sample_time)
    half_trans_y = transition_Min_jerk_firstHalf(0,step_width/2, samples_in_end_step_trans, samples_in_transition)

    p_ref_y = r_[ half_trans_y_begin, p_ref1y, step_width/2 - half_trans_y]

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
    #half_trans_y_begin = ZMP_start_pos + transitionSigmoid_secondHalf(-(step_width/2 + ZMP_start_pos), trans_slope_steepens_factor, samples_in_start_step_trans, sample_time)
    half_trans_y_begin = transition_Min_jerk_secondHalf( ZMP_start_pos,-(step_width/2), samples_in_start_step_trans, samples_in_transition)
    
    # step middle: with out transitions
    p_ref1y = -step_width/2*ones( samples_in_step_without_trans )

    # step 2 end: transition
    # half_trans_y = transitionSigmoid_firstHalf(step_width/2, trans_slope_steepens_factor, samples_in_end_step_trans, sample_time)
    half_trans_y = transition_Min_jerk_firstHalf(0,step_width/2, samples_in_end_step_trans, samples_in_transition)

    p_ref_y = r_[ half_trans_y_begin , p_ref1y, half_trans_y - step_width/2 ]

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
    # half_trans_y_begin = ZMP_start_pos + transitionSigmoid_secondHalf( (step_width/2 - ZMP_start_pos), trans_slope_steepens_factor, samples_in_start_step_trans, sample_time)
    half_trans_y_begin = transition_Min_jerk_secondHalf( ZMP_start_pos, step_width/2, samples_in_start_step_trans, samples_in_transition)
    
    # step middle: with out transitions
    p_ref1y = step_width/2*ones( samples_in_step_without_trans )

    # step 2 end: transition
    # half_trans_y = transitionSigmoid_firstHalf(step_width/2, trans_slope_steepens_factor, samples_in_end_step_trans, sample_time)
    half_trans_y = transition_Min_jerk_firstHalf( 0, step_width/2, samples_in_end_step_trans, samples_in_transition)

    p_ref_y = r_[ half_trans_y_begin , p_ref1y, step_width/2 - half_trans_y ]

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
    # half_trans_y_begin = ZMP_start_pos + transitionSigmoid_secondHalf(-(step_width/2 + ZMP_start_pos), trans_slope_steepens_factor, samples_in_start_step_trans, sample_time)
    half_trans_y_begin = transition_Min_jerk_secondHalf( ZMP_start_pos,-(step_width/2), samples_in_start_step_trans, samples_in_transition)
    
    # step 1 middle: with out transitions
    p_ref1y = -step_width/2*ones( samples_in_step_without_trans )

    ## move and stay at double support:
    # step 1 to step 2 transition:
    samples_in_trans = samples_in_end_step_trans + samples_in_start_step_trans
    # trans_y_small_step = transitionSigmoid( step_width/2 ?(- ZMP_start_pos)?, trans_slope_steepens_factor, samples_in_trans, sample_time )
    trans_y_small_step = transition_Min_jerk( 0, step_width/2 + ZMP_start_pos, samples_in_trans)

    # step 2 middle and end: 
    p_ref2y = zeros( samples_in_step_without_trans + samples_in_end_step_trans )



    p_ref_y = r_[ half_trans_y_begin, p_ref1y, trans_y_small_step - step_width/2, p_ref2y ]

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
    # half_trans_y_begin = ZMP_start_pos + transitionSigmoid_secondHalf( (step_width/2 - ZMP_start_pos), trans_slope_steepens_factor, samples_in_start_step_trans, sample_time)
    half_trans_y_begin = transition_Min_jerk_secondHalf( ZMP_start_pos, step_width/2, samples_in_start_step_trans, samples_in_transition)
    
    # step 1 middle: with out transitions
    p_ref1y = step_width/2*ones( samples_in_step_without_trans )

    ## move and stay at double support:
    # step 1 to step 2 transition:
    samples_in_trans = samples_in_end_step_trans + samples_in_start_step_trans
    trans_y_small_step = transition_Min_jerk( 0,-(step_width/2 - ZMP_start_pos), samples_in_trans)

    # step 2 middle and end: 
    p_ref2y = zeros( samples_in_step_without_trans + samples_in_end_step_trans )



    p_ref_y = r_[ half_trans_y_begin , p_ref1y, step_width/2 + trans_y_small_step, p_ref2y ]

    return (p_ref_y)

#######################################################
#                                                     #
#              Turn profiles  (x,y)                   #
#                                                     #
#######################################################


# angle thetta is with respect to the x axes
#if turn to left thetta is positive
#if turn to right thetta is negative
def  Turn_second_half_of_step(ZMP_start_pos, step_length, trans_ratio_of_step, trans_slope_steepens_factor, step_time, sample_time,thetta):
    ## First Step of a two step turn sequence, rotates the second half of the ZMP step profile by angle thetta. 

    p_ref_x_forward_step = Step_forward_x(ZMP_start_pos, step_length, trans_ratio_of_step, trans_slope_steepens_factor, step_time, sample_time)
    if 0.0 >= thetta: # TURNING RIGHT - right foot is stance (coord. system is relative to right foot)
        p_ref_y_side_step = Step_onto_right_foot(ZMP_start_pos, step_length, trans_ratio_of_step, trans_slope_steepens_factor, step_time, sample_time)
    else: # TURNING LEFT - left foot is stance (coord. system is relative to letf foot)
        p_ref_y_side_step = Step_onto_left_foot(ZMP_start_pos, step_length, trans_ratio_of_step, trans_slope_steepens_factor, step_time, sample_time)

    Rz_thetta = array([[cos(thetta), -sin(thetta)],
                       [sin(thetta),  cos(thetta)]])  
                             
    
    samples = floor(p_ref_x_forward_step.size/2)
    turn_start_pos_x = p_ref_x_forward_step[samples]
    turn_start_pos_y = p_ref_y_side_step[samples]

    p_ref1x = p_ref_x_forward_step[0:samples-1]
    p_ref1y = p_ref_y_side_step[0:samples-1]

    p_ref_turnx = p_ref_x_forward_step[samples:]-turn_start_pos_x*ones( samples )
    p_ref_turny = p_ref_y_side_step[samples:]-turn_start_pos_y*ones( samples )

    p_turn_all= array([p_ref_turnx,p_ref_turny])

    turn_poz =   array([turn_start_pos_x*ones( samples ),
                        turn_start_pos_y*ones( samples )])

    p_turn = dot(Rz_thetta,p_turn_all) + turn_poz
    # print(samples)
    # print(turn_start_pos_x)
    # print(turn_start_pos_y)
    p_turn_x = r_[ p_ref1x , p_turn[0] ]
    p_turn_y = r_[ p_ref1y , p_turn[1] ]
    
    return p_turn_x,p_turn_y
   
# # DON'T NEED TO ROTATE SECOND STEP (because stance foot is rotating)
# def  Turn_first_half_of_step(ZMP_start_pos, step_length, trans_ratio_of_step, trans_slope_steepens_factor, step_time, sample_time,thetta):
# """
#     Second Step of a two step turn sequence, rotates the first half of the ZMP step profile by angle thetta. 
# """
#     p_ref_x_forward_step = Step_forward_x(ZMP_start_pos, step_length, trans_ratio_of_step, trans_slope_steepens_factor, step_time, sample_time)
#     if 0.0 >= thetta: # TURNING RIGHT - left foot is stance (coord. system is relative to letf foot)
#         p_ref_y_side_step = Step_onto_left_foot(ZMP_start_pos, step_length, trans_ratio_of_step, trans_slope_steepens_factor, step_time, sample_time)
#     else: #TURNING LEFT - right foot is stance (coord. system is relative to right foot)
#         p_ref_y_side_step = Step_onto_right_foot(ZMP_start_pos, step_length, trans_ratio_of_step, trans_slope_steepens_factor, step_time, sample_time)

#     Rz_thetta = array([[cos(thetta), -sin(thetta)],
#                        [sin(thetta),  cos(thetta)]])  
                             
    
#     samples = floor(p_ref_x_forward_step.size/2)
#     # samples = p_ref_x.size/2
#     turn_start_pos_x = p_ref_x_forward_step[samples]
#     turn_start_pos_y = p_ref_y_side_step[samples]

    
#     p_ref1x = p_ref_x_forward_step[samples:]
#     p_ref1y = p_ref_y_side_step[samples:]

#     p_ref_turnx = p_ref_x_forward_step[0:samples]-turn_start_pos_x*ones(samples)
#     p_ref_turny = p_ref_y_side_step[0:samples]-turn_start_pos_y*ones(samples)

#     p_turn_all= array([p_ref_turnx,p_ref_turny])

#     turn_poz =   array([turn_start_pos_x*ones( samples ),
#                         turn_start_pos_y*ones( samples )])

#     p_turn = dot(Rz_thetta,p_turn_all) + turn_poz
#     print(samples)
#     print(turn_start_pos_x)
#     print(turn_start_pos_y)
#     p_turn_x = r_[ p_turn[0],p_ref1x]
#     p_turn_y = r_[ p_turn[1],p_ref1y]
#     res = [p_turn_x,p_turn_y]
    

#     return (res)