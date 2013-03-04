#!/usr/bin/env python
import roslib; roslib.load_manifest('leg_ik')
from leg_ik.srv import *
from leg_ik.msg import *
from std_msgs.msg import Float64
import rospy, math, sys




def nothing():

    rospy.init_node('nothing')
    
    l_arm_shx = rospy.Publisher('/l_arm_shx_position_controller/command', Float64)
    r_arm_shx = rospy.Publisher('/r_arm_shx_position_controller/command', Float64)
    
    l_arm_elx = rospy.Publisher('/l_arm_elx_position_controller/command', Float64)
    r_arm_elx = rospy.Publisher('/r_arm_elx_position_controller/command', Float64)
   
    l_arm_ely = rospy.Publisher('/l_arm_ely_position_controller/command', Float64)
    r_arm_ely = rospy.Publisher('/r_arm_ely_position_controller/command', Float64)




    rospy.loginfo( "nothing node is ready" )

    rospy.loginfo( "waiting 2 seconds for robot to initiate" )
    rospy.sleep(2)
    rospy.loginfo( "going to put down hands" )

    l_arm_shx.publish(-1.3)
    r_arm_shx.publish(1.3)
    l_arm_elx.publish(0.0)
    r_arm_elx.publish(0.0) 
    l_arm_ely.publish(0.0)
    r_arm_ely.publish(0.0)

    rospy.spin()

if __name__ == '__main__':

    nothing()

    

