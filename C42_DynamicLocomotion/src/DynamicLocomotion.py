#!/usr/bin/env python

import roslib;roslib.load_manifest('C42_DynamicLocomotion')
import rospy
from Abstractions.Walker import *
from WalkingModeChooser import *
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
    
    def __init__(self,name,walkingModeChooser):
        RobilTask.__init__(self, name)
        self._Walker = Walker(walkingModeChooser)
        self._interval = rospy.Rate(0.6)
    
    def _init_values(self):
        self._debug_cmd = 1 # default value, has no effect

    def task(self, name, uid, parameters):
        #initialize values:
        self._init_values()

        if({} == parameters):
            parameters=None
        
        ## TOPIC setup:
        self._debug_cmd_sub = rospy.Subscriber('walker_command',Int32,self._debug_command)

        self._Walker.Initialize(parameters)

        if (True != self.WaitForPath()):
            return RTResult_PREEPTED()

        self._Walker.Start()

        self._Walker.Walk()

        rospy.loginfo("DynamicLocomotion, task: %s" % ("Starting to walk") )
        while not self._Walker.IsDone():
            if self.isPreepted() or (3 == self._debug_cmd):
                self._Walker.Stop()
                return RTResult_PREEPTED()
            self._interval.sleep()

        self._Walker.Stop()
        
        self._debug_cmd_sub.unregister
        
        if(WalkerResultEnum.Success == self._Walker.GetResult()):
            print("SUCCESS!!")
            return RTResult_SUCCESSED("Finished in Success")
        else:
            print("FAIL")
            return RTResult(RobilTask_FAULT, "", "Did not end in success", False) 

    def WaitForPath(self):
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

    def _debug_command(self,debug_command): # cmd = 0 -> STOP; cmd = 3 -> Emergency STOP
        self._debug_cmd = debug_command.data
        rospy.loginfo("DEBUG - received debug_command = %i" % (self._debug_cmd) )
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
    walkingModeChooser = WalkingModeChooser('BDI',False)
    node = DynamicLocomotion("DynamicLocomotion",walkingModeChooser)
    rospy.spin()
    print "C42_DynamicLocomotion node Closed"