#!/usr/bin/env python

import roslib;roslib.load_manifest('C42_DynamicLocomotion')
import rospy
from Abstractions.DynamicWalker import *
from WalkingModeChooser import *
from C31_PathPlanner.msg import C31_Waypoints
from RobilTaskPy import *
from LocalPathPlanner import *
import actionlib

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

class DynamicLocomotion(RobilTask):
    
    def __init__(self,name):
        RobilTask.__init__(self, name)
        lpp = LocalPathPlanner()
        self._DynamicWalker = DynamicWalker(WalkingModeChooser(lpp),lpp)
        self._interval = rospy.Rate(2)
        
    def task(self, name, uid, parameters):
        
        self._DynamicWalker.Initialize()

        self._DynamicWalker.Start()

        if (False == self.WaitForPath()):
            return RTResult_PREEPTED()

        self._DynamicWalker.Walk()

        while not self._DynamicWalker.IsDone():
            # if self.isPreepted():
            #     self._DynamicWalker.EmergencyStop()
            #     return RTResult_PREEPTED()
            self._interval.sleep()

        self._DynamicWalker.Stop()

        print("SUCCESS!!")

        return RTResult_SUCCESSED("Finished in Success")

    def WaitForPath(self):
        self._DynamicWalker._LPP.Stop()
        self._path_sub = rospy.Subscriber('/path',C31_Waypoints,self._path_cb)

        while not self._DynamicWalker.IsReady():
            # if self.isPreepted():
            #     self._DynamicWalker.Stop()
            #     print "Preempt Walker: Preempted before path was received"
            #     return False
            self._interval.sleep()
        return True


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
    #node.task()
    rospy.spin()
    print "C42_DynamicLocomotion node Closed"