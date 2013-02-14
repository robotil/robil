#!/usr/bin/env python

#############################################################################
####                                                                       ##
####    creates the LEG_IK node which computes the IK of the foot in 3D    ##
####    the service handles LegIk.srv sevices                              ##
####    request is foot coordinates (x,y,z) with reference to the hip      ##
####    resposes are hip (mhx,lhy,uhz) and knee (kny) angles               ##
####    use by typing rostopic pub posi leg_ik/mod "pos: {x: 0.01, y: 0.2, ##
####    z: -0.2, r: 0.0, p: 0.0, w: 0.0}                                   ##
####                                                                       ##
####    usage:                                                             ##
####    1) rosrun ZMP zmp_main.py rosrun                                   ##
####    2) rosrun leg_ik leg_ik_server3D.py                                ##
####    3) leg_ik IK_Caller.py                                             ##
####    4) start walking  rostopic pub /zmp_walk_command std_msgs/Int32 1  ##
####    5) stop walking   rostopic pub /zmp_walk_command std_msgs/Int32 0  ##
####                                                                       ##
#############################################################################

import roslib; roslib.load_manifest('leg_ik')
# from leg_ik.srv import *
# from leg_ik.msg import *
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from zmp_walk.msg import walking_trajectory
import rospy, math, sys
from Impedance_Control import Joint_Stiffness_Controller
from pylab import *
from leg_ik_func import swing_leg_ik,stance_leg_ik
from IKException import IKReachException

class Nasmpace: pass
ns = Nasmpace()
#ns.LegAng = LegIkResponse()

JSC_l_leg_lax = Joint_Stiffness_Controller('l_leg_lax', 4000, 1) # joint name, stiffness, update_period [sec]
JSC_l_leg_mhx = Joint_Stiffness_Controller('l_leg_mhx', 8000, 1) # joint name, stiffness, update_period [sec]
JSC_r_leg_lax = Joint_Stiffness_Controller('r_leg_lax', 4000, 1) # joint name, stiffness, update_period [sec]
JSC_r_leg_mhx = Joint_Stiffness_Controller('r_leg_mhx', 8000, 1) # joint name, stiffness, update_period [sec]


# swing_leg_ik = rospy.ServiceProxy('swing_leg_ik', LegIk)
# stance_leg_ik = rospy.ServiceProxy('stance_leg_ik', LegIk)

#################################################################################
#                     request from IK and publish angles                        #
#################################################################################

def get_from_zmp(msg):
    try:
        if ( msg.step_phase == 1 ) or ( msg.step_phase == 2 ): # left leg is stance
            # [mhx,lhy,uhz,kny,lax,uay]
            right_leg_angles = swing_leg_ik(msg.swing_foot,msg.swing_hip,msg.pelvis_m)
            left_leg_angles = stance_leg_ik(msg.stance_hip,msg.pelvis_d)
        elif ( msg.step_phase == 3 ) or ( msg.step_phase == 4 ): # right leg is stance
            # [mhx,lhy,uhz,kny,lax,uay]
            left_leg_angles = swing_leg_ik(msg.swing_foot,msg.swing_hip,msg.pelvis_m)
            right_leg_angles = stance_leg_ik(msg.stance_hip,msg.pelvis_d)

    except IKReachException as exc:
        rospy.loginfo('IKException: %s leg is out of reach, req pos: %f ,%f, %f',exc.foot,exc.requested_pos[0],exc.requested_pos[1],exc.requested_pos[2])
        return



        
    ns.l_leg_lax.publish( left_leg_angles[4] ) #JSC_l_leg_lax.getCMD(ns.LegAng.ang.lax) )
    ns.l_leg_uay.publish( left_leg_angles[5] )
    ns.l_leg_kny.publish( left_leg_angles[3] )
    ns.l_leg_uhz.publish( left_leg_angles[2] )
    ns.l_leg_lhy.publish( left_leg_angles[1] )
    ns.l_leg_mhx.publish( left_leg_angles[0] ) #JSC_l_leg_mhx.getCMD(ns.LegAng.ang.mhx) )

    ns.r_leg_lax.publish( right_leg_angles[4] ) #JSC_r_leg_lax.getCMD(ns.LegAng.ang.lax + lax_stance) )
    ns.r_leg_uay.publish( right_leg_angles[5] )
    ns.r_leg_kny.publish( right_leg_angles[3] )
    ns.r_leg_uhz.publish( right_leg_angles[2] )
    ns.r_leg_lhy.publish( right_leg_angles[1] )
    ns.r_leg_mhx.publish( right_leg_angles[0] ) #JJSC_r_leg_mhx.getCMD(ns.LegAng.ang.mhx + mhx_stance) )
        
    ns.back_mby.publish( 0.0 )
    ns.back_ubx.publish( 0.0 )
    ns.back_lbz.publish( 0.0 )

    #     rospy.loginfo("JC L_leg: lax = %f effort = %f, uay = %f, kny = %f, uhz = %f, lhy = %f, mhx = %f effort = %f, mby = %f, ubx= %f" %  \
    #                       (ns.LegAng.ang.lax, JSC_l_leg_lax.latest_effort, ns.LegAng.ang.uay, ns.LegAng.ang.kny, ns.LegAng.ang.uhz, \
    #                        ns.LegAng.ang.lhy, ns.LegAng.ang.mhx, JSC_l_leg_mhx.latest_effort, ns.LegAng.ang.mby, ns.LegAng.ang.ubx))

    


##########################################################################################
# request from joint_states publisher joints state to update Stiffness Controllers state #
##########################################################################################

def get_joint_states(msg):
    
    if JSC_l_leg_lax.JS_i == -1: # Update joint state indexs if not updated
        try:
            JSC_l_leg_lax.JS_i = msg.name.index(JSC_l_leg_lax.name)       
            JSC_l_leg_mhx.JS_i = msg.name.index(JSC_l_leg_mhx.name)        
            JSC_r_leg_lax.JS_i = msg.name.index(JSC_r_leg_lax.name)        
            JSC_r_leg_mhx.JS_i = msg.name.index(JSC_r_leg_mhx.name)
        
        except rospy.ROSInterruptException: pass
        else:
            rospy.loginfo(" update Stiffness Controllers JS_i - successful")

    JSC_l_leg_lax.UpdateState(msg.position[JSC_l_leg_lax.JS_i], msg.velocity[JSC_l_leg_lax.JS_i], msg.effort[JSC_l_leg_lax.JS_i], msg.header.stamp)
    JSC_l_leg_mhx.UpdateState(msg.position[JSC_l_leg_mhx.JS_i], msg.velocity[JSC_l_leg_mhx.JS_i], msg.effort[JSC_l_leg_mhx.JS_i], msg.header.stamp)
    JSC_r_leg_lax.UpdateState(msg.position[JSC_r_leg_lax.JS_i], msg.velocity[JSC_r_leg_lax.JS_i], msg.effort[JSC_r_leg_lax.JS_i], msg.header.stamp)
    JSC_r_leg_mhx.UpdateState(msg.position[JSC_r_leg_mhx.JS_i], msg.velocity[JSC_r_leg_mhx.JS_i], msg.effort[JSC_r_leg_mhx.JS_i], msg.header.stamp)
    #rospy.loginfo("Stiffness Controllers joint state updated ")   

#######################################################################################
#                                 init publishers                                     #
#######################################################################################

def LEG_IK():

    rospy.init_node('LEG_IK')
    # rospy.wait_for_service('swing_leg_ik')
    # rospy.wait_for_service('stance_leg_ik')
    

    ns.r_leg_lax = rospy.Publisher('/r_leg_lax_position_controller/command', Float64)
    ns.r_leg_uay = rospy.Publisher('/r_leg_uay_position_controller/command', Float64)
    ns.r_leg_kny = rospy.Publisher('/r_leg_kny_position_controller/command', Float64)
    ns.r_leg_uhz = rospy.Publisher('/r_leg_uhz_position_controller/command', Float64)
    ns.r_leg_lhy = rospy.Publisher('/r_leg_lhy_position_controller/command', Float64)
    ns.r_leg_mhx = rospy.Publisher('/r_leg_mhx_position_controller/command', Float64)

    ns.l_leg_lax = rospy.Publisher('/l_leg_lax_position_controller/command', Float64)
    ns.l_leg_uay = rospy.Publisher('/l_leg_uay_position_controller/command', Float64)
    ns.l_leg_kny = rospy.Publisher('/l_leg_kny_position_controller/command', Float64)
    ns.l_leg_uhz = rospy.Publisher('/l_leg_uhz_position_controller/command', Float64)
    ns.l_leg_lhy = rospy.Publisher('/l_leg_lhy_position_controller/command', Float64)
    ns.l_leg_mhx = rospy.Publisher('/l_leg_mhx_position_controller/command', Float64)

    ns.back_mby = rospy.Publisher('/back_mby_position_controller/command', Float64)
    ns.back_ubx = rospy.Publisher('/back_ubx_position_controller/command', Float64)
    ns.back_lbz = rospy.Publisher('/back_lbz_position_controller/command', Float64)

    rospy.loginfo( "LEG_IK node is ready" )

    rospy.loginfo( "waiting 1 seconds for robot to initiate" )
    rospy.sleep(1)
  


    sub1=rospy.Subscriber("zmp_out", walking_trajectory, get_from_zmp) # traj, get_from_zmp)
    sub2=rospy.Subscriber("joint_states", JointState, get_joint_states)

    rospy.spin()

if __name__ == '__main__':

    LEG_IK()
