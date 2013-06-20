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
        print "Usage: WalkingMode.py <Option>"
        print "Options are:"
        print " CD for Dynamic Continuous mode"
        print " QS for Discrete Quasi-Static mode"
        print " DD for Discrete Dynamic mode"
        print " DW for DW mode"
        print " AP for Align Pose mode"
        print " Rot for Rotation"
        print " Trans for Translation"
        print " Trans for Translation 10cm"

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
        print "WalkerNode_Continuous created"
        rospy.spin()
    elif walkingMode in ('QS','DD'):
        rospy.init_node('WalkerNode_Discrete')       
        walkingModeChooser = WalkingModeChooser(walkingMode)
        node = DynamicLocomotion("WalkerNode_Discrete",walkingModeChooser)
        print "WalkerNode_Discrete created"
        rospy.spin()
    elif walkingMode in ('DW'):
        rospy.init_node('WalkerNode_DW')       
        walkingModeChooser = WalkingModeChooser(walkingMode)
        node = DynamicLocomotion("WalkerNode_DW",walkingModeChooser)
        print "WalkerNode_DW created"
        rospy.spin()
    elif walkingMode in ('AP'):
        rospy.init_node('WalkerNode_AP')       
        walkingModeChooser = WalkingModeChooser(walkingMode)
        node = DynamicLocomotion("WalkerNode_AP",walkingModeChooser)
        print "WalkerNode_AP created"
        rospy.spin()
    elif walkingMode in ('Rot'):
        rospy.init_node('WalkerNode_Rotation')
        walkingModeChooser = WalkingModeChooser(walkingMode)
        node = DynamicLocomotion("WalkerNode_Rotation",walkingModeChooser)
        print "WalkerNode_Rotation created"
        rospy.spin()
    elif walkingMode in ('Trans'):
        rospy.init_node('WalkerNode_Translation')
        walkingModeChooser = WalkingModeChooser(walkingMode)
        node = DynamicLocomotion("WalkerNode_Translation",walkingModeChooser)
        print "WalkerNode_Translation created"
        rospy.spin()
    elif walkingMode in ('Trans10'):
        rospy.init_node('WalkerNode_Translation10')
        walkingModeChooser = WalkingModeChooser(walkingMode)
        node = DynamicLocomotion("WalkerNode_Translation10",walkingModeChooser)
        print "WalkerNode_Translation10 created"
        rospy.spin()
    else:
        wn.PrintHelp()
    print "WalkerNode Closed"
    
