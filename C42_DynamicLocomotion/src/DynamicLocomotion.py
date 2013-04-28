#!/usr/bin/env python

from Abstractions.DynamicWalker import *
from WalkingModeChooser import *
import rospy
import roslib
roslib.load_manifest('C42_DynamicLocomotion')

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

class DynamicLocomotion(object):
    
    def __init__(self):
        self._ModeChooser = WalkingModeChooser()
        self._DynamicWalker = DynamicWalker(self._ModeChooser)
        
    def DynamicLocomotionTask(self):
        
        self._DynamicWalker.Initialize()
        self._DynamicWalker.Start()
        self._DynamicWalker.WaitForPath()
        self._DynamicWalker.Walk()
        # if self._DynamicWalker.Walk():
        #     self._DynamicWalker.Stop()
        #     return RTResult_SUCCESSED("Finished in Success")
        # else:
        #     return RTResult_PREEPTED()

if __name__ == '__main__':
    rospy.init_node('DynamicLocomotion')
    node = DynamicLocomotion()
    # Harness robot, with gravity off
    mode = rospy.Publisher('/atlas/mode', String, None, False, True, None)
    mode.publish("harnessed")
    node.DynamicLocomotionTask()
