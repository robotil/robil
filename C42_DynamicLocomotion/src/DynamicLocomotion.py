#!/usr/bin/env python

from Abstractions.DynamicWalker import *
from WalkingModeChooser import *
import rospy
import roslib
roslib.load_manifest('C42_DynamicLocomotion')
from C31_PathPlanner.msg import C31_Waypoints
from RobilTaskPy import *

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

class DynamicLocomotion(RobilTask):
    
    def __init__(self,name):
        RobilTask.__init__(self, name)
        self._DynamicWalker = DynamicWalker(WalkingModeChooser())
        self._interval = rospy.Rate(0.3)
        
    def DynamicLocomotionTask(self):
        
        self._DynamicWalker.Initialize()
        self._DynamicWalker.Start()
        self.WaitForPath()
        self._DynamicWalker.Walk()
        # if self._DynamicWalker.Walk():
        #     self._DynamicWalker.Stop()
        #     return RTResult_SUCCESSED("Finished in Success")
        # else:
        #     return RTResult_PREEPTED()

    def WaitForPath(self):
        self._DynamicWalker._LPP.Stop()
        self._path_sub = rospy.Subscriber('/C31PathPlanner',C31_Waypoints,self._path_cb)

        while not self._DynamicWalker._LPP.IsActive():
            # if self.isPreepted():
            #     self._DynamicWalker.Stop()
            #     print "Preempt ZmpWalk: Preempted before path was received"
            #     return RTResult_PREEPTED()
            self._interval.sleep()

###################################################################################
#--------------------------- CallBacks -----------------------------------------------
###################################################################################

    def _path_cb(self,path):
        
        rospy.loginfo('got path %s',path)
        
        p = []
        for wp in path.points:
            p.append(Waypoint(wp.x,wp.y))
        self._DynamicWalker.SetPath(p)


###################################################################################
#---------------------------  a little testing script -----------------------------
###################################################################################
if __name__ == '__main__':
    rospy.init_node('DynamicLocomotion')
    node = DynamicLocomotion("DynamicLocomotion")
    # Harness robot, with gravity off
    mode = rospy.Publisher('/atlas/mode', String, None, False, True, None)
    mode.publish("harnessed")
    node.DynamicLocomotionTask()
