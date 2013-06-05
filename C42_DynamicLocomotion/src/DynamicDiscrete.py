#!/usr/bin/env python

from DynamicLocomotion import *

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

if __name__ == '__main__':
    rospy.init_node('DynamicQS')
    node = DynamicLocomotion("DynamicQS",'DD')
    rospy.spin()
    print "C42_DynamicLocomotion node Closed"