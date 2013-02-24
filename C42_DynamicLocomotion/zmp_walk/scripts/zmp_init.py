#! /usr/bin/env python
import roslib
roslib.load_manifest('DRCSim2_tools')
from drc2_tools import *
import roslib
import os
import rospy
rospy.init_node('test_reset_position')
JC = JointCommands_msg_handler()
RL = robot_listner()
rospy.sleep(1)

des_pos = [ 0, 0, 0, 0, 0, 0, -0.2372, 0.4491, -0.2119, 0, 0, 0, -0.2372, 0.4491,
          -0.2119, 0, 0, -1.3, 0, 0, 0, 0, 0, 1.3, 0, 0, 0, 0]

yaml_pth = os.path.join(roslib.packages.get_pkg_dir('DRCSim2_tools'),'calibrated_controller_drc2.yaml')
JC.set_default_gains_from_yaml(yaml_pth)
JC.reset_gains()
init_pos = RL.current_pos()
JC.send_pos_traj(init_pos,des_pos,5,0.1)

