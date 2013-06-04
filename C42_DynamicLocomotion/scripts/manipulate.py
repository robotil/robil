#! /usr/bin/env python

import roslib; roslib.load_manifest('C42_DynamicLocomotion')
import rospy
from numpy import linspace
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
        self.RS = robot_state(self._jnt_names)
        self._state_sub = rospy.Subscriber('/'+self._robot_name+'/atlas_state',AtlasState,self.RS_cb)
        self.JC = JointCommands_msg_handler(self._robot_name,self._jnt_names)
        self._asi_com_pub = rospy.Publisher('/atlas/atlas_sim_interface_command',AtlasSimInterfaceCommand)
        self._asi_command = AtlasSimInterfaceCommand()
    def stand(self):
        self._asi_command = AtlasSimInterfaceCommand()
        self._asi_command.behavior = AtlasSimInterfaceCommand.STAND   
        self._asi_com_pub.publish(self._asi_command)  
    def manipulate(self,joints):
        self._asi_command = AtlasSimInterfaceCommand()
        self._asi_command.behavior = AtlasSimInterfaceCommand.MANIPULATE
        self._asi_command.k_effort = [0 for k in self._jnt_names]
        for k in joints:
            self._asi_command.k_effort[k] = 255
        self._asi_command.manipulate_params.use_desired = True
        self._asi_command.manipulate_params.desired.pelvis_height = 1.2
        self._asi_command.manipulate_params.desired.pelvis_yaw = 0.0
        self._asi_command.manipulate_params.desired.pelvis_lat = 0.0
        self._asi_com_pub.publish(self._asi_command)
    def RS_cb(self,msg):
        self.RS.UpdateState(msg)

    def control_joints(self):
        pos1 = self.RS.GetJointPos()[3]
        pos2 = -1.0 #-1 # desired position
        dt = 0.05;
        N = 50
        self.JC.bdi_control()
        for ratio in linspace(0, 1, N):
            interpCommand = (1-ratio)*pos1 + ratio * pos2
            self.JC.set_joint_command(3,1000,10,10,255,interpCommand,0.0)
            #self.JC.set_joint_command(3+6,1000,10,10,255,interpCommand,0.0)
            self.JC.send_command()
            rospy.sleep(dt)

if __name__ == '__main__':
    rospy.init_node('bdi_test')
    CNT = manipulate_test()
    rospy.sleep(0.5)
    jnt = [k for k in xrange(16,28)]
    jnt = [k for k in xrange(0,4)]
    CNT.manipulate(jnt)
    CNT.control_joints()




