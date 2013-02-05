#!/usr/bin/env python

###############################################################################
####                                                                         ##
####  zmp_profiles.py (based on zmp_main(old).py)                            ##
####  Created - Yuval 05/2/2013 	                                         ##
####  last updated - version 1.0, Yuval 05/2/2013                            ##
####                                                                         ##
####    Used for creating ZMP reference profiles in the sagital (x) and      ## 
####    lateral (y) planes                                                   ##
####                                                                         ##
####                                                                         ##
###############################################################################

import roslib; roslib.load_manifest('zmp_walk')
import rospy, sys, os.path
from pylab import *


# Sigmoid Function: an auxiliary function to smoothen step trasitions

def  transitionSigmoid(step_size, slope_factor, transition_time, sample_time):
	# slope_factor = a   1 # change slop of ZMP -> com
	s =  arange(-transition_time/2 , transition_time/2 + sample_time , sample_time)
	sigmoid = step_size/(1+exp(-a*s))

    return (sigmoid)

def  transitionSigmoid_Half(step_size, slope_factor, transition_time, sample_time):
	# slope_factor = a   1 # change slop of ZMP -> com
	s =  arange(-transition_time/2, 0 , sample_time)
	sigmoid = step_size/(1+exp(-a*s))

    return (sigmoid)


#######################################################
#                                                     #
#              sagital profiles  (x)                  #
#                                                     #
#######################################################

def  Start_sagital_x(ZMP_start_pos, step_time, sample_time):
	p_ref0_0x = ZMP_start_pos*ones(( len(t0x)-1 ))

    return (p_ref_x)




#######################################################
#                                                     #
#              lateral profiles  (y)                  #
#                                                     #
#######################################################

def  Start_lateral_y_weight_to_left_foot(ZMP_start_pos, step_width, transition_ratio, step_time, sample_time):
	# tuning parameters:
    slope_f = 1 # slop factor of transition, bigger values give a steeper slop

	samples_in_step = step_time * sample_time
	samples_in_transition = transition_ratio * step_time * sample_time

	# step 1 : stand in place (we want to minimize the transient effect of the preview)
	# can be skiped in the future by inserting the appropriate initial conditions to the preview controller 
	p_ref0_0y = (step_width/2)*ones( samples_in_step - samples_in_transition/2 )
	#p_ref0_0y = ZMP_start_pos*ones( samples_in_step - samples_in_transition/2 )

	# step 1 to step 2 transition:
	transition_time = transition_ratio * step_time
	sigmoid_y_half_step = transitionSigmoid(step_width/2, slope_f, transition_time, sample_time)

	# step 2 with out transitions:
	p_ref1y = step_width*ones( samples_in_step - samples_in_transition + 1 )

	# step 2 end transition:
	half_sigmoid_y = transitionSigmoid(step_width, slope_f, transition_time, sample_time)



	p_ref_y = r_[ p_ref0_0y , sigmoid_y_half_step+step_width/2 , p_ref1y, step_width - half_sigmoid_y]

    return (p_ref_y)
