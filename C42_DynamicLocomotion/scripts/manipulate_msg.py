#! /usr/bin/env python

import roslib; roslib.load_manifest('C42_DynamicLocomotion')
import rospy
from copy import copy
from numpy import linspace, uint8, array
from DW.robot_state import robot_state
from DW.JointController import JointCommands_msg_handler
from atlas_msgs.msg import *
class manipulate_test(object):
    def __init__(self):
        self._robot_name = "atlas"
        self._jnt_names = ['back_lbz', 'back_mby', 'back_ubx', 'neck_ay', #3
                           'l_leg_uhz', 'l_leg_mhx', 'l_leg_lhy', 'l_leg_kny', 'l_leg_uay', 'l_leg_lax', #9
                           'r_leg_uhz', 'r_leg_mhx', 'r_leg_lhy', 'r_leg_kny', 'r_leg_uay', 'r_leg_lax', #15
                           'l_arm_usy', 'l_arm_shx', 'l_arm_ely', 'l_arm_elx', 'l_arm_uwy', 'l_arm_mwx', #21
                           'r_arm_usy', 'r_arm_shx', 'r_arm_ely', 'r_arm_elx', 'r_arm_uwy', 'r_arm_mwx'] #27
        self.JC = JointCommands_msg_handler(self._robot_name,self._jnt_names)
        self.RS = robot_state(self._jnt_names)
        self._state_sub = rospy.Subscriber('/'+self._robot_name+'/atlas_state',AtlasState,self.RS_cb)
        self._state_sub = rospy.Subscriber('/atlas/atlas_sim_interface_state',AtlasSimInterfaceState,self._asi_state_cb)
        self._keff = []
    def RS_cb(self,msg):
        self.RS.UpdateState(msg)
    def _asi_state_cb(self,msg):
        k_eff = [ord(k) for k in msg.k_effort]
        self.JC.set_k_eff(k_eff)


    def move_neck(self):
        init_pos = self.RS.GetJointPos()
        self.JC.set_all_pos(init_pos)
        pos1 = self.RS.GetJointPos()[3]
        pos2 = 0.7 # 0.4
        dt = 0.05;
        N = 50
        for ratio in linspace(0, 1, N):
            interpCommand = (1-ratio)*pos1 + ratio * pos2
            self.JC.set_pos(3,interpCommand)
            self.JC.send_command()
            rospy.sleep(dt)
        # for k in xrange(5):
        #     for ratio in linspace(0, 1, N):
        #         interpCommand = (1-ratio)*pos1 + ratio * pos2
        #         self.JC.set_pos(3,interpCommand)
        #         self.JC.send_command()
        #         rospy.sleep(dt)
        #     pos2 = -1
        #     for ratio in linspace(0, 1, N):
        #         interpCommand = (1-ratio)*pos1 + ratio * pos2
        #         self.JC.set_pos(3,interpCommand)
        #         self.JC.send_command()
        #         rospy.sleep(dt)
        #     pos2 = 1

if __name__ == '__main__':
    rospy.init_node('bdi_test')
    CNT = manipulate_test()
    rospy.sleep(0.5)
    CNT.move_neck()




