#!/usr/bin/env python

from DynamicLocomotion import *
from WalkingModeChooser import *

import sys

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

if __name__ == '__main__':
#    walkingMode = 'BDI'
#    try:
#        opts, args = getopt.getopt(sys.argv[1:],"m:",["mode="])
#    except getopt.GetoptError:
#        print 'WalkerNode.py -mode <QS or DD or BDI>'
#        sys.exit(2)
#    for opt, arg in opts:
#        if opt in ("-m", "--mode"):
#            if arg in ('BDI','QS','DD'):
#                walkingMode = arg
#            else:
#               print (arg, " not recognized as a walking mode")

    try:
        walkingMode = sys.argv[1]
    except Exception:
        print "Usage: WalkingMode.py <Option>"
        print "Options are:"
        print " BDI for Dynamic Continuous mode"
        print " QS for Discrete Quasi-Static mode"
        print " DD for Discrete Dynamic mode"
        sys.exit(2)
    if walkingMode in ('BDI','QS','DD'):
        rospy.init_node('WalkerNode')       
        walkingModeChooser = WalkingModeChooser(walkingMode)
        node = DynamicLocomotion("WalkerNode",walkingModeChooser)
        rospy.spin()
    else:
        print "Usage: WalkingMode.py <Option>"
        print "Options are:"
        print " BDI for Dynamic Continuous mode"
        print " QS for Discrete Quasi-Static mode"
        print " DD for Discrete Dynamic mode"
    print "WalkerNode Closed"

               