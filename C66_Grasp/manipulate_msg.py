#! /usr/bin/env python

import roslib; roslib.load_manifest('atlas_msgs')
import rospy
from atlas_msgs.msg import *
class manipulate_test(object):
    def __init__(self):        
        self._jnt_names = ['back_lbz', 'back_mby', 'back_ubx', 'neck_ay', #3
                               'l_leg_uhz', 'l_leg_mhx', 'l_leg_lhy', 'l_leg_kny', 'l_leg_uay', 'l_leg_lax', #9
                               'r_leg_uhz', 'r_leg_mhx', 'r_leg_lhy', 'r_leg_kny', 'r_leg_uay', 'r_leg_lax', #15
                               'l_arm_usy', 'l_arm_shx', 'l_arm_ely', 'l_arm_elx', 'l_arm_uwy', 'l_arm_mwx', #21
                               'r_arm_usy', 'r_arm_shx', 'r_arm_ely', 'r_arm_elx', 'r_arm_uwy', 'r_arm_mwx'] #27

        self._asi_com_pub = rospy.Publisher('/atlas/atlas_sim_interface_command',AtlasSimInterfaceCommand)
        self._asi_command = AtlasSimInterfaceCommand()

   # def stand(self):
   #     self._asi_command = AtlasSimInterfaceCommand()
   #     self._asi_command.behavior = AtlasSimInterfaceCommand.STAND
   #     self._asi_com_pub.publish(self._asi_command)

    def stand(self,joints):
        #self._asi_command.behavior = AtlasSimInterfaceCommand.MANIPULATE
	self._asi_command.behavior = AtlasSimInterfaceCommand.STAND        
	self._asi_command.k_effort = [0 for k in self._jnt_names]
        for k in joints:
            self._asi_command.k_effort[k] = 255
        self._asi_command.manipulate_params.use_desired = True
        self._asi_command.manipulate_params.desired.pelvis_height = 1.2
        self._asi_command.manipulate_params.desired.pelvis_yaw = 0.0
        self._asi_command.manipulate_params.desired.pelvis_lat = 0.0
        self._asi_com_pub.publish(self._asi_command)

if __name__ == '__main__':
    rospy.init_node('bdi_test')
    CNT = manipulate_test()
    rospy.sleep(0.5)
    #CNT.stand()
    rospy.sleep(2)
    jnt = [k for k in xrange(15,28)]
    #jnt.append(0)
    #jnt.append(1)
    #jnt.append(2)
    jnt.append(3)
    CNT.stand(jnt)




