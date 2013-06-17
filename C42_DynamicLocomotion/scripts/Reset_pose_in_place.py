#! /usr/bin/env python

###################################################################################
####                                                                             ##
####  Reset_start_pose.py (based on zmp_init.py)                                 ##
####  Created - Yuval 7/04/2013                                                  ##
####                                                                             ##
####    run this script to reset robots pose to "zero" and Reset Model Poses.    ##
####                                                                             ##
####    To run script: rosrun C42_ZMPWalk scripts/Reset_start_pose.py            ##
####   ( Make sure no other joint commands are being sent while running script ) ##
####                                                                             ##
###################################################################################     

import roslib; roslib.load_manifest('C42_DynamicLocomotion')
import rospy
from std_srvs.srv import Empty

from atlas_msgs.msg import AtlasSimInterfaceCommand
from std_msgs.msg import String

def BDI_init_pose():
    # Initialize atlas mode and atlas_sim_interface_command publishers        
    _mode = rospy.Publisher('/atlas/mode', String, None, False, \
      True, None)
    _asi_command = rospy.Publisher('/atlas/atlas_sim_interface_command', AtlasSimInterfaceCommand, None, False, True, None)

    rospy.sleep(2.0)

    k_effort = [0] * 28
    # Puts robot into freeze behavior, all joints controlled
    freeze = AtlasSimInterfaceCommand(None,AtlasSimInterfaceCommand.FREEZE, None, None, None, None, k_effort )
    _asi_command.publish(freeze)
    
    rospy.sleep(1.0)
    # Puts robot into stand_prep behavior, a joint configuration suitable
    # to go into stand mode
    stand_prep = AtlasSimInterfaceCommand(None,AtlasSimInterfaceCommand.STAND_PREP, None, None, None, None, k_effort)
    _asi_command.publish(stand_prep)

    rospy.sleep(2.0)
    _mode.publish("nominal")
    
    # Put robot into stand position
    stand = AtlasSimInterfaceCommand(None,AtlasSimInterfaceCommand.STAND, None, None, None, None, k_effort)
            
    rospy.sleep(0.3)
    
    _asi_command.publish(stand)

    # rospy.wait_for_service('gazebo/reset_models')
    # reset_gazebo_model = rospy.ServiceProxy('gazebo/reset_models', Empty)
    # reset_gazebo_model()

    # Harness robot, with gravity off
    _mode.publish("harnessed")
    k_effort = [0] * 28
    # Puts robot into freeze behavior, all joints controlled
    freeze = AtlasSimInterfaceCommand(None,AtlasSimInterfaceCommand.FREEZE, None, None, None, None, k_effort )
    _asi_command.publish(freeze)
    
    rospy.sleep(1.0)

    # Puts robot into stand_prep behavior, a joint configuration suitable
    # to go into stand mode
    stand_prep = AtlasSimInterfaceCommand(None,AtlasSimInterfaceCommand.STAND_PREP, None, None, None, None, k_effort)
    _asi_command.publish(stand_prep)

    rospy.sleep(2.0)
    _mode.publish("nominal")
    
    # Put robot into stand position
    stand = AtlasSimInterfaceCommand(None,AtlasSimInterfaceCommand.STAND, None, None, None, None, k_effort)
            
    rospy.sleep(0.3)
    
    _asi_command.publish(stand)
    rospy.loginfo("Reset_start_pose: %s" % ("Completed Reset") )

if __name__ == '__main__':
    rospy.init_node('Reset_Pose_BDI')  
    BDI_init_pose()

