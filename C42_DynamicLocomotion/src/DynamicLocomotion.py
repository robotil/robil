#!/usr/bin/env python

import roslib;roslib.load_manifest('C42_DynamicLocomotion')
import rospy
from Abstractions.Walker import *
from WalkingModeChooser import *
from C31_PathPlanner.msg import C31_Waypoints
from RobilTaskPy import *
from LocalPathPlanner import *
import actionlib
from std_msgs.msg import Int32

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

class DynamicLocomotion(RobilTask):
    
    def __init__(self,name):
        RobilTask.__init__(self, name)
        lpp = LocalPathPlanner()
        lpp.SetDoingQual(False)
        self._Walker = Walker(WalkingModeChooser(lpp))
        self._interval = rospy.Rate(2)

        ## TOPIC setup:
        self._path_sub = rospy.Subscriber('/path',C31_Waypoints,self._path_cb)
        self._debug_cmd_sub = rospy.Subscriber('walker_command',Int32,self._debug_command)
    
    def _init_values(self):
        self._debug_cmd = 1 # default value, has no effect

    def task(self, name, uid, parameters):

        #initialize values:
        self._init_values()
        
        if (False == self.WaitForPath()):
            return RTResult_PREEPTED()

        self._Walker.Initialize()

        self._Walker.Start()

        self._Walker.Walk()

        rospy.loginfo("DynamicLocomotion, task: %s" % ("Starting to walk") )
        while not self._Walker.IsDone():
            if self.isPreepted() or (3 == self._debug_cmd):
                self._Walker.Stop()
                return RTResult_PREEPTED()
            self._interval.sleep()

        self._Walker.Stop()

        print("SUCCESS!!")

        return RTResult_SUCCESSED("Finished in Success")

    def WaitForPath(self):
        self._Walker._LPP.Stop()

        rospy.loginfo("DynamicLocomotion, WaitForPath: %s" % ("Waiting to receive /path ...") )
        while not self._Walker.IsReady():
            if self.isPreepted():
                self._Walker.Stop()
                print "Preempt Walker: Preempted before path was received"
                return RTResult_PREEPTED()
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
        self._Walker.SetPath(p)

    def _debug_command(self,debug_command): # cmd = 0 -> STOP; cmd = 3 -> Emergency STOP
        self._debug_cmd = debug_command.data
        rospy.loginfo("DEBUG - recieved debug_command = %i" % (self._debug_cmd) )
        rospy.loginfo("time:")
        rospy.loginfo(rospy.get_time())
        if 3 == self._debug_cmd:
             #self._ZmpStateMachine.EmergencyStop()
             rospy.loginfo('DEBUG - _Walker "Emergency STOP" command')
        elif 0 == self._debug_cmd:
             self._Walker.Stop()
             rospy.loginfo('DEBUG - _Walker STOP command')

###################################################################################
#---------------------------  a little testing script -----------------------------
###################################################################################
if __name__ == '__main__':
    rospy.init_node('DynamicLocomotion')
    node = DynamicLocomotion("DynamicLocomotion")
    # Harness robot, with gravity off
    #mode = rospy.Publisher('/atlas/mode', String, None, False, True, None)
    #mode.publish("harnessed")
    #node.task()
    rospy.spin()
    print "C42_DynamicLocomotion node Closed"