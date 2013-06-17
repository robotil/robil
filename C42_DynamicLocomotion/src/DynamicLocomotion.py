#!/usr/bin/env python

import roslib;roslib.load_manifest('C42_DynamicLocomotion')
import rospy
from Abstractions.Walker import *
from WalkingModeChooser import *
from RobilTaskPy import *
from LocalPathPlanner import *
import actionlib
from std_msgs.msg import Int32

from C42_State.msg import *

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

        self._publisher_motion_type = rospy.Publisher("/motion_state/motion_type", MotionType)
        self._publisher_standing_position = rospy.Publisher("/motion_state/standing_position", StandingPosition)

        self._Walker.Initialize(parameters)

        if (True != self.WaitForPath()):
            self.Unsubscribe()
            return RTResult_PREEPTED()

        self._Walker.Start()

        self._Walker.Walk()

        rospy.loginfo("DynamicLocomotion, task: %s" % ("Starting to walk") )
        while not self._Walker.IsDone():
            if self.isPreepted() or (3 == self._debug_cmd):
                self._Walker.Stop()
                self.Unsubscribe()
                return RTResult_PREEPTED()
            self.PublishMotionState(parameters)
            self._Walker.Sleep()

        self._Walker.Stop()
                
        self.Unsubscribe()

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

    def Unsubscribe(self):
        self._debug_cmd_sub.unregister()

    def PublishMotionState(self,parameters):
        eWalkingMode = self._Walker.GetWalkingModeEnum()
        bPublishMotion = True
        bPublisPosition = True
        standingPosition = StandingPosition.state_standing

        if (WalkingModeChooserEnum.CD == eWalkingMode):
            motionType = MotionType.motion_continuous_walk
        elif (WalkingModeChooserEnum.QS == eWalkingMode):
            motionType = MotionType.motion_discrete_walk_quasi_static
        elif (WalkingModeChooserEnum.DD == eWalkingMode):
            motionType = MotionType.motion_discrete_walk_dynamic
        elif (WalkingModeChooserEnum.DW == eWalkingMode):
            standingPosition = StandingPosition.state_sitting

            if ((None != parameters) and ('Terrain' in parameters)):
                terrain = parameters['Terrain']
            else:
                terrain="MUD"

            if ("MUD" == terrain):
                motionType = MotionType.motion_quad_mud
            elif ("HILLS" == terrain):
                motionType = MotionType.motion_quad_hills
            elif ("DEBRIS" == terrain):
                motionType = MotionType.motion_quad_hills
            else:
                print("DynamicLocomotion::PublishMotionState WalkingModeChooserEnum = ",eWalkingMode)
                print("Unknown Terrain type: ",terrain)
                bPublishMotion = False
                bPublisPosition = False

        elif (WalkingModeChooserEnum.AP == eWalkingMode):
            motionType = MotionType.motion_align_pose
        elif(WalkingModeChooserEnum.Rot == eWalkingMode):
            motionType = MotionType.motion_direct_rotation
            bPublisPosition = False
        elif(WalkingModeChooserEnum.Trans == eWalkingMode):
            motionType = MotionType.motion_direct_translation
            bPublisPosition = False
        else:
            print("DynamicLocomotion::PublishMotionState WalkingModeChooserEnum = ",eWalkingMode)
            bPublishMotion = False
            bPublisPosition = False

        if bPublishMotion:
            self._publisher_motion_type.publish(motionType)

        if bPublisPosition:
            self._publisher_standing_position.publish(standingPosition)


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