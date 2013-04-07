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

import roslib; roslib.load_manifest('C42_Leg_IK')
from atlas_msgs.msg import ForceTorqueSensors
from drc2_tools import *
from foot_contact_filter import contact_filter
# from leg_ik.srv import *
# from leg_ik.msg import *
from std_msgs.msg import Float64
#from sensor_msgs.msg import *
from C42_ZMPWalk.msg import walking_trajectory
import rospy, math, sys, os
from Impedance_Control import Joint_Stiffness_Controller , Joint_Stiffness_Controller_2 , Position_Stiffness_Controller, Position_Stiffness_Controller_2
from pylab import *
from leg_ik_func import swing_leg_ik,stance_leg_ik
from IKException import IKReachException
from geometry_msgs.msg import *
import yaml
import copy

class Nasmpace: pass
ns = Nasmpace()
#ns.LegAng = LegIkResponse()

JSC_r_leg_mhx = Joint_Stiffness_Controller('r_leg_mhx', 8000, 0.05) # joint name, stiffness, update_period [sec]
JSC_l_leg_mhx = Joint_Stiffness_Controller('l_leg_mhx', 8000, 0.05) # joint name, stiffness, update_period [sec]
JSC_r_leg_lax = Joint_Stiffness_Controller('r_leg_lax', 5500, 0.05) # joint name, stiffness, update_period [sec]
JSC_l_leg_lax = Joint_Stiffness_Controller('l_leg_lax', 5500, 0.05) # joint name, stiffness, update_period [sec]
JSC_r_leg_uay = Joint_Stiffness_Controller('r_leg_uay', 8000, 0.05) # joint name, stiffness, update_period [sec]
JSC_l_leg_uay = Joint_Stiffness_Controller('l_leg_uay', 8000, 0.05) # joint name, stiffness, update_period [sec]

PSC_right_swing_leg = Position_Stiffness_Controller('R_Swing Leg', 270000, False, False) # 210000 # 101000, True, False) # name, stiffness, triggered_controller, bypass_input2output [True/False]
PSC_left_swing_leg = Position_Stiffness_Controller('L_Swing Leg', 270000, False, False) # 210000 # 101000, True, False) # name, stiffness, triggered_controller, bypass_input2output [True/False]

PSC2_right_swing_leg = Position_Stiffness_Controller_2('R_Swing Leg', 301000, False, False) # 101000, True, False) # name, stiffness, triggered_controller, bypass_input2output [True/False]
PSC2_left_swing_leg = Position_Stiffness_Controller_2('L_Swing Leg', 301000, False, False) # 101000, True, False) # name, stiffness, triggered_controller, bypass_input2output [True/False]

JSC_2_l_leg_lax = Joint_Stiffness_Controller_2('l_leg_lax', 650, 1000, 0.04) # joint name, stance_stiffness, swing_stiffness, activation_ZMP_point [m]
JSC_2_r_leg_lax = Joint_Stiffness_Controller_2('r_leg_lax', 650, 1000, 0.04) # joint name, stance_stiffness, swing_stiffness, activation_ZMP_point [m]
JSC_2_l_leg_lax.ChangeStiffness(Joint_Stiffness_Controller_2.stance)
JSC_2_r_leg_lax.ChangeStiffness(Joint_Stiffness_Controller_2.swing)

JSC_2_l_leg_uay = Joint_Stiffness_Controller_2('l_leg_uay', 650, 1000, 0.04) # joint name, stance_stiffness, swing_stiffness, activation_ZMP_point [m]
JSC_2_r_leg_uay = Joint_Stiffness_Controller_2('r_leg_uay', 650, 1000, 0.04) # joint name, stance_stiffness, swing_stiffness, activation_ZMP_point [m]
JSC_2_l_leg_uay.ChangeStiffness(Joint_Stiffness_Controller_2.stance)
JSC_2_r_leg_uay.ChangeStiffness(Joint_Stiffness_Controller_2.swing)

# swing_leg_ik = rospy.ServiceProxy('swing_leg_ik', LegIk)
# stance_leg_ik = rospy.ServiceProxy('stance_leg_ik', LegIk)


##########################################################################################
# request from foot contact publisher to update Position Stiffness Controllers avg force #
##########################################################################################

# def get_r_foot_contact(msg):
    
#     PSC2_right_swing_leg.UpdateForce(msg.force.z)
#     JSC_2_r_leg_lax.UpdateFeedBack(msg.force.z, msg.torque.x)
#     JSC_2_r_leg_uay.UpdateFeedBack(msg.force.z, msg.torque.x)

# def get_l_foot_contact(msg):
    
#     PSC2_left_swing_leg.UpdateForce(msg.force.z) 
#     JSC_2_l_leg_lax.UpdateFeedBack(msg.force.z, msg.torque.x) 
#     JSC_2_l_leg_uay.UpdateFeedBack(msg.force.z, msg.torque.x)

def get_foot_contact(msg):

    ns.filter.update(msg)
    buf = ns.filter.get_buffer()

    PSC_right_swing_leg.UpdateForce( -msg.r_foot.force.z ) # -buf[0].r_foot.force.z ) #-msg.r_foot.force.z ) # 
    PSC2_right_swing_leg.UpdateFeedBack(-msg.r_foot.force.z, -buf[0].r_foot.force.z )
    JSC_2_r_leg_lax.UpdateFeedBack(-msg.r_foot.force.z, -msg.r_foot.torque.x)
    JSC_2_r_leg_uay.UpdateFeedBack(-msg.r_foot.force.z, -msg.r_foot.torque.y)
   
    PSC_left_swing_leg.UpdateForce( -msg.l_foot.force.z ) # -buf[0].l_foot.force.z ) #-msg.l_foot.force.z ) # 
    PSC2_left_swing_leg.UpdateFeedBack(-msg.l_foot.force.z, -buf[0].l_foot.force.z) 
    JSC_2_l_leg_lax.UpdateFeedBack(-msg.l_foot.force.z, -msg.l_foot.torque.x) 
    JSC_2_l_leg_uay.UpdateFeedBack(-msg.l_foot.force.z, -msg.l_foot.torque.y)

#################################################################################
#                     request from IK and publish angles                        #
#################################################################################

def get_from_zmp(msg):
    try:
        ## Desired Force Profile on swing leg
        half_robot_weight = 864.75/2 # units [N], half robots weight without feet
        com_y_max = msg.zmp_width/2       #0.095 # maximal movement of COM in y direction
        min_support_force = 200 # minimal weight that we want to keep on swing leg while shifting weight to stance leg

       # desired_normal_force = half_robot_weight - abs(msg.com_ref.y/com_y_max) * (half_robot_weight - min_support_force)

        # PSC_left_swing_leg.ByPassON()# bypass controller
        # PSC_right_swing_leg.ByPassON()  # bypass controller

        if ( msg.step_phase == 1 ) or ( msg.step_phase == 2 ): # left leg is stance
            # [mhx,lhy,uhz,kny,lax,uay]
            PSC_left_swing_leg.ByPassON()# bypass controller
            PSC_right_swing_leg.ByPassOFF()
            swing_fixed = copy.deepcopy(msg.swing_foot)
            ( swing_fixed.z, desired_force_L , desired_force_R ) = PSC_right_swing_leg.getCMD(msg.swing_foot.z,msg.zmp_ref.y,msg.step_phase ,msg.step_width,msg.zmp_width,msg.step_time,ns.des_l_force_pub,ns.des_r_force_pub)

            right_leg_angles = swing_leg_ik(swing_fixed,msg.swing_hip,msg.pelvis_m)
        #    right_leg_angles = swing_leg_ik(msg.swing_foot,msg.swing_hip,msg.pelvis_m)
            left_leg_angles = stance_leg_ik(msg.stance_hip,msg.pelvis_d)
        elif ( msg.step_phase == 3 ) or ( msg.step_phase == 4 ): # right leg is stance
            # [mhx,lhy,uhz,kny,lax,uay]
             PSC_right_swing_leg.ByPassON()  # bypass controller
             PSC_left_swing_leg.ByPassOFF()
             swing_fixed = copy.deepcopy(msg.swing_foot)
             ( swing_fixed.z, desired_force_L , desired_force_R ) = PSC_left_swing_leg.getCMD(msg.swing_foot.z,msg.zmp_ref.y,msg.step_phase ,msg.step_width,msg.zmp_width,msg.step_time,ns.des_l_force_pub,ns.des_r_force_pub)

             left_leg_angles = swing_leg_ik(swing_fixed,msg.swing_hip,msg.pelvis_m)
          #   left_leg_angles = swing_leg_ik(msg.swing_foot,msg.swing_hip,msg.pelvis_m)
             right_leg_angles = stance_leg_ik(msg.stance_hip,msg.pelvis_d)

        ns.actual_r_force_pub.publish( PSC2_right_swing_leg.getFilteredForce() ) # getAvgForce() ) # JSC_2_r_leg_lax.getAvgForce() ) # PSC2_right_swing_leg.getAvgForce() ) #
        ns.actual_l_force_pub.publish( PSC2_left_swing_leg.getFilteredForce() ) # getAvgForce() ) #JSC_2_l_leg_lax.getAvgForce() ) # PSC2_left_swing_leg.getAvgForce() ) #
    except IKReachException as exc:
        rospy.loginfo('IKException: %s leg is out of reach, req pos: %f ,%f, %f',exc.foot,exc.requested_pos[0],exc.requested_pos[1],exc.requested_pos[2])
        return

    ## Joint Feed Forward effort:
    l_leg_kny_eff = -desired_force_L/10.0
    r_leg_kny_eff = -desired_force_R/10.0

    ## Joint position command (to PID):

    raise_foot_tip = 0.2

    l_leg_lax =  JSC_l_leg_lax.getCMD( left_leg_angles[4] ) 
    if ( msg.step_phase == 4 ): # left leg is swing
        l_leg_uay = JSC_l_leg_uay.getCMD( left_leg_angles[5] - raise_foot_tip*msg.swing_foot.z) 
    else:
        l_leg_uay = JSC_l_leg_uay.getCMD( left_leg_angles[5] ) 
    l_leg_kny = left_leg_angles[3] + ns.joints_offset['l_leg_kny'] 
    l_leg_lhy = left_leg_angles[1] + ns.joints_offset['l_leg_lhy']
    l_leg_mhx = left_leg_angles[0]

    r_leg_lax = JSC_r_leg_lax.getCMD( right_leg_angles[4] ) 
    if ( msg.step_phase == 2 ): # right leg is swing
        r_leg_uay = JSC_r_leg_uay.getCMD( right_leg_angles[5] - raise_foot_tip*msg.swing_foot.z) 
    else:
        r_leg_uay =  JSC_r_leg_uay.getCMD( right_leg_angles[5] ) 
    r_leg_kny = right_leg_angles[3] + ns.joints_offset['r_leg_kny'] 
    r_leg_lhy = right_leg_angles[1] + ns.joints_offset['r_leg_lhy']
    r_leg_mhx = right_leg_angles[0]  
        
    back_mby =  0.0 
    back_ubx =  0.0 
    back_lbz =  0.0 


    l_leg_uhz =  msg.hip_z_orientation.left 
    r_leg_uhz =  msg.hip_z_orientation.right 



    ns.JC.set_pos('l_leg_lax', l_leg_lax )
    ns.JC.set_pos('l_leg_uay', l_leg_uay )
    ns.JC.set_mixed('l_leg_kny', l_leg_kny_eff, l_leg_kny ) # set_pos('l_leg_kny', l_leg_kny )
    ns.JC.set_pos('l_leg_uhz', l_leg_uhz )
    ns.JC.set_pos('l_leg_lhy', l_leg_lhy )
    ns.JC.set_pos('l_leg_mhx', l_leg_mhx ) 

    ns.JC.set_pos('r_leg_lax', r_leg_lax ) 
    ns.JC.set_pos('r_leg_uay', r_leg_uay )
    ns.JC.set_mixed('r_leg_kny', r_leg_kny_eff, r_leg_kny ) # set_pos('r_leg_kny', r_leg_kny )
    ns.JC.set_pos('r_leg_uhz', r_leg_uhz )
    ns.JC.set_pos('r_leg_lhy', r_leg_lhy )
    ns.JC.set_pos('r_leg_mhx', r_leg_mhx ) 

    ns.JC.set_pos('back_mby', back_mby )
    ns.JC.set_pos('back_ubx', back_ubx )
    ns.JC.set_pos('back_lbz', back_lbz )

    ns.JC.send_command()



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
            JSC_r_leg_uay.JS_i = msg.name.index(JSC_r_leg_uay.name)        
            JSC_l_leg_uay.JS_i = msg.name.index(JSC_l_leg_uay.name)

        except rospy.ROSInterruptException: pass
        else:
            rospy.loginfo(" update Stiffness Controllers JS_i - successful")

    JSC_l_leg_lax.UpdateState(msg.position[JSC_l_leg_lax.JS_i], msg.velocity[JSC_l_leg_lax.JS_i], msg.effort[JSC_l_leg_lax.JS_i], msg.header.stamp)
    JSC_l_leg_mhx.UpdateState(msg.position[JSC_l_leg_mhx.JS_i], msg.velocity[JSC_l_leg_mhx.JS_i], msg.effort[JSC_l_leg_mhx.JS_i], msg.header.stamp)
    JSC_r_leg_lax.UpdateState(msg.position[JSC_r_leg_lax.JS_i], msg.velocity[JSC_r_leg_lax.JS_i], msg.effort[JSC_r_leg_lax.JS_i], msg.header.stamp)
    JSC_r_leg_mhx.UpdateState(msg.position[JSC_r_leg_mhx.JS_i], msg.velocity[JSC_r_leg_mhx.JS_i], msg.effort[JSC_r_leg_mhx.JS_i], msg.header.stamp) 
    JSC_r_leg_uay.UpdateState(msg.position[JSC_r_leg_uay.JS_i], msg.velocity[JSC_r_leg_uay.JS_i], msg.effort[JSC_r_leg_uay.JS_i], msg.header.stamp)
    JSC_l_leg_uay.UpdateState(msg.position[JSC_l_leg_uay.JS_i], msg.velocity[JSC_l_leg_uay.JS_i], msg.effort[JSC_l_leg_uay.JS_i], msg.header.stamp) 


#######################################################################################
#                                 init publishers                                     #
#######################################################################################

def LEG_IK():

    rospy.init_node('LEG_IK')
    ns.JC = JointCommands_msg_handler()
    ns.RL = robot_listner()

    # Loading IK off-set values (These values represent the "zero pose" of the IK. They need to be added to the position calc. from the IK )
    zmp_walking_path = os.path.join(roslib.packages.get_pkg_dir('C42_ZMPWalk'), r"src/parameters/",'ZMP_Walking_Params.yaml')
    zmp_walking_params_file = file(zmp_walking_path)
    ns.joints_offset = yaml.load(zmp_walking_params_file)
    ns.joints_offset = ns.joints_offset['zmp_walking']['IK_zero_pose']
    # ns.joints_offset = rospy.get_param("/zmp_walking/IK_zero_pose")
    
    rospy.loginfo( "LEG_IK node is ready" )
    rospy.loginfo( "waiting 1 seconds for robot to initiate" )
    yaml_pth = os.path.join(roslib.packages.get_pkg_dir('C42_DRCSim2_tools'),'calibrated_controller_drc2_yuval.yaml')
    ns.JC.set_default_gains_from_yaml(yaml_pth)
    ns.JC.reset_gains()
    # r_cont_sub = rospy.Subscriber('/atlas/debug/r_foot_contact', Wrench, get_r_foot_contact)
    # l_cont_sub = rospy.Subscriber('/atlas/debug/l_foot_contact', Wrench, get_l_foot_contact)
    contact_sub = rospy.Subscriber('/atlas/force_torque_sensors', ForceTorqueSensors, get_foot_contact)
    # contact sensor filter:
    a = [1,-3.180638548874721,3.861194348994217,-2.112155355110971,0.438265142261981]
    b = [0.0004165992044065786,0.001666396817626,0.002499595226439,0.001666396817626,0.0004165992044065786]
    ns.filter = contact_filter(b = b, a = a, use_internal_subscriber = False)

    ns.des_l_force_pub = rospy.Publisher('/des_l_force', Float64)
    ns.des_r_force_pub = rospy.Publisher('/des_r_force', Float64)

    ns.actual_l_force_pub = rospy.Publisher('/actual_l_force', Float64)
    ns.actual_r_force_pub = rospy.Publisher('/actual_r_force', Float64)

    sub1=rospy.Subscriber("zmp_out", walking_trajectory, get_from_zmp) # traj, get_from_zmp)
    sub2=rospy.Subscriber("joint_states", JointState, get_joint_states)

    # TODO: get the following values from initialization using a TOPIC with a flag to pause until Robot State static init is completed
    back_lbz = 0.0
    back_mby = 0.0#0.03#0.08# 0.06
    back_ubx = 0
    neck_ay = 0
    l_leg_uhz = 0 
    l_leg_mhx = -0.00096
    l_leg_lhy = -0.3564
    l_leg_kny = 0.6244
    l_leg_uay = -0.2680
    l_leg_lax = 0.0009
    r_leg_uhz = 0
    r_leg_mhx = -0.00096
    r_leg_lhy = -0.3564
    r_leg_kny = 0.6244
    r_leg_uay = -0.268
    r_leg_lax = 0.0009
    l_arm_usy = 0
    l_arm_shx = -1.3
    l_arm_ely = 0
    l_arm_elx = 0
    l_arm_uwy = 0
    l_arm_mwx = 0
    r_arm_usy = 0
    r_arm_shx = 1.3
    r_arm_ely = 0
    r_arm_elx = 0
    r_arm_uwy = 0
    r_arm_mwx = 0

    cur_pos = [ back_lbz, back_mby, back_ubx, neck_ay,
      l_leg_uhz, l_leg_mhx, l_leg_lhy, l_leg_kny, l_leg_uay, l_leg_lax,
      r_leg_uhz, r_leg_mhx, r_leg_lhy, r_leg_kny, r_leg_uay, r_leg_lax,
      l_arm_usy, l_arm_shx, l_arm_ely, l_arm_elx, l_arm_uwy, l_arm_mwx,
      r_arm_usy, r_arm_shx, r_arm_ely, r_arm_elx, r_arm_uwy, r_arm_mwx]
    # cur_pos = ns.RL.current_pos()
    ns.JC.set_all_pos(cur_pos)
    rospy.sleep(1)

    rospy.spin()

if __name__ == '__main__':

    LEG_IK()
