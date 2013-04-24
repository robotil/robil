#! /usr/bin/env python
import roslib
roslib.load_manifest('C42_ZMPWalk')
from drc2_tools import *
import roslib
import os
import rospy

def init_pose():
    JC = AtlasCommand_msg_handler()
    RL = robot_listner()

    # # Off-set values for IK
    # rospy.set_param("/zmp_walking/IK_zero_pose", {'l_leg_uhz':0, 'l_leg_mhx':0, 'l_leg_lhy':-0.13, 
    #                                'l_leg_kny':0.13, 'l_leg_uay':0.0, 'l_leg_lax':0, 
    #                                'r_leg_uhz':0, 'r_leg_mhx':0, 'r_leg_lhy':-0.13, 
    #                                'r_leg_kny':0.13, 'r_leg_uay':0.0, 'r_leg_lax':0, 
    #                                'l_arm_usy':0, 'l_arm_shx':-1.3, 'l_arm_ely':0, 
    #                                'l_arm_elx':0, 'l_arm_uwy':0, 'l_arm_mwx':0, 
    #                                'r_arm_usy':0, 'r_arm_shx':1.3, 'r_arm_ely':0, 
    #                                'r_arm_elx':0, 'r_arm_uwy':0, 'r_arm_mwx':0, 
    #                                'neck_ay':0, 'back_lbz':0, 'back_mby':0, 'back_ubx':0})

    rospy.sleep(1)
    
    #back_lbz = 0  
    #back_mby = 0
    #back_ubx = 0
 #   neck_ay = 0
 #  l_leg_uhz = 0 
  #  l_leg_mhx = -0.00096
 #  l_leg_lhy = -0.3564
 #   l_leg_kny = 0.6244
  #  l_leg_uay = -0.2680
  #  l_leg_lax = 0.0009
  #  r_leg_uhz = 0
   # r_leg_mhx = -0.00096
  #  r_leg_lhy = -0.3564
  #  r_leg_kny = 0.6244
 #   r_leg_uay = -0.268
 #   r_leg_lax = 0.0009
  #  l_arm_usy = 0
 #   l_arm_shx = -1.3
 #   l_arm_ely = 0
  #  l_arm_elx = 0
  #  l_arm_uwy = 0
   # l_arm_mwx = 0
  #  r_arm_usy = 0
  #  r_arm_shx = 1.3
   # r_arm_ely = 0
 #   r_arm_elx = 0
  #  r_arm_uwy = 0
#    r_arm_mwx = 0
    back_lbz = 0.0#
    back_mby = 0.03 #0.03#0.08# 0.06
    back_ubx =  0
    neck_ay = 0.0#0
    l_leg_uhz =0.0# 0 
    l_leg_mhx = -0.00096
    l_leg_lhy = -0.3564
    l_leg_kny = 0.6244
    l_leg_uay = -0.2680
    l_leg_lax = 0.0009
    r_leg_uhz = 0
    r_leg_mhx = -0.00096
    r_leg_lhy =0 -0.3564
    r_leg_kny = 0.6244
    r_leg_uay =0 -0.268
    r_leg_lax = 0.0009
    l_arm_usy = 0.9
    l_arm_shx = -1.35
    l_arm_ely = 2.3
    l_arm_elx = 1.6
    l_arm_uwy = 0.0#0
    l_arm_mwx = 0.0#0.0
    r_arm_usy = 0.9
    r_arm_shx = 1.35
    r_arm_ely = 2.3
    r_arm_elx = -1.6
    r_arm_uwy = 0
    r_arm_mwx = 0.0


    des_pos = [ back_lbz, back_mby, back_ubx, neck_ay,
      l_leg_uhz, l_leg_mhx, l_leg_lhy, l_leg_kny, l_leg_uay, l_leg_lax,
      r_leg_uhz, r_leg_mhx, r_leg_lhy, r_leg_kny, r_leg_uay, r_leg_lax,
      l_arm_usy, l_arm_shx, l_arm_ely, l_arm_elx, l_arm_uwy, l_arm_mwx,
      r_arm_usy, r_arm_shx, r_arm_ely, r_arm_elx, r_arm_uwy, r_arm_mwx]

    yaml_pth = os.path.join(roslib.packages.get_pkg_dir('C42_DRCSim2_tools'),'calibrated_controller_drc2_yuval.yaml')
    JC.set_default_gains_from_yaml(yaml_pth)
    JC.reset_gains()
    init_pos = RL.current_pos()
    JC.send_pos_traj(init_pos,des_pos,5,0.1)

if __name__ == '__main__':
    init_pose()
