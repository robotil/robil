#!/usr/bin/env python

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

from DynamicLocomotion import *
from WalkingModeChooser import *

import sys

class WalkerNode:
    def __init__(self):
        pass
    
    def PrintHelp(self):
        print "Usage: WalkingMode.py <Option> <Parameter1> <Parameter2> ..."
        print "Options are:"
        print " CD for Dynamic Continuous mode"
        print " QS for Discrete Quasi-Static mode"
        print " DD for Discrete Dynamic mode"
        print " DW for DW mode"
        print " AP for Align Pose mode"
        print " Rot for Rotation"
        print " Trans for Translation"
        print "Parameters:"
        print " For Rot an angle of rotation in Radians is expected as Parameter 1"
        print " For Trans a Translation in local coordinates (relative to the Robot's center) is expected:"
        print "    <Parameter1> - X coordinate for translation"
        print "    <Parameter2> - Y coordinate for translation"

if __name__ == '__main__':
    wn = WalkerNode()
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
        wn.PrintHelp()
        sys.exit(2)
    if walkingMode in ('CD'):
        rospy.init_node('WalkerNode_Continuous')       
        walkingModeChooser = WalkingModeChooser(walkingMode)
        node = DynamicLocomotion("WalkerNode_Continuous",walkingModeChooser)
        print "WalkerNode_Continuous TASK created"
        rospy.spin()
    elif walkingMode in ('QS','DD'):
        rospy.init_node('WalkerNode_Discrete')       
        walkingModeChooser = WalkingModeChooser(walkingMode)
        node = DynamicLocomotion("WalkerNode_Discrete",walkingModeChooser)
        print "WalkerNode_Discrete TASK created"
        rospy.spin()
    elif walkingMode in ('DW'):
        rospy.init_node('WalkerNode_DW')       
        walkingModeChooser = WalkingModeChooser(walkingMode)
        node = DynamicLocomotion("WalkerNode_DW",walkingModeChooser)
        print "WalkerNode walk on four TASK created"
        rospy.spin()
    elif walkingMode in ('AP'):
        rospy.init_node('WalkerNode_AP')       
        walkingModeChooser = WalkingModeChooser(walkingMode)
        node = DynamicLocomotion("WalkerNode_AP",walkingModeChooser)
        print "WalkerNode Discrete Aline Pose mode TASK created"
        rospy.spin()
    else:
        wn.PrintHelp()
    print "WalkerNode Closed"
    
