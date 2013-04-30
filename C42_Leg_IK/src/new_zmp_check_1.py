#!/usr/bin/env python
########################3
# cd Projects/Robil/C42_Leg_IK/src
# python zmp_check.py
#############
####

import roslib; roslib.load_manifest('C42_Leg_IK')
#import roslib; roslib.load_manifest('C42_ZMPWalk')
from C42_ZMPWalk.msg import Position, Orientation, Pos_and_Ori
from std_msgs.msg import Float64
from sensor_msgs.msg import *
from C42_ZMPWalk.msg import walking_trajectory#,Pos_and_Ori
from C42_Leg_IK.msg import zmp_real
import rospy, math, sys
from Impedance_Control import Joint_Stiffness_Controller
from pylab import *
from leg_ik_func import swing_leg_ik,stance_leg_ik
from IKException import IKReachException
from geometry_msgs.msg import *
import copy
from atlas_msgs.msg import ForceTorqueSensors
from foot_contact_filter_1 import contact_filter
from contact_reflex import contact_reflex


#from Impedance_Control import Position_Stiffness_Controller
class Nasmpace_zmp: pass
ns = Nasmpace_zmp()
class Nasmpace_r_leg: pass
nr = Nasmpace_r_leg()
class Nasmpace_l_leg: pass
nl = Nasmpace_l_leg()
class namespace_walk: pass
nsw = namespace_walk()
#msg.step_phase  = 0
#msg.swing_foot  =  Pos_and_Ori()
#nsw.out = zmp_real()

##########################################################################################
# request from foot contact publisher
##########################################################################################

ns.zmpx_l = 0
ns.zmpy_l = 0
ns.zmpx_ld = 0
ns.zmpy_ld = 0
ns.zmpx_r = 0
ns.zmpy_r = 0
ns.zmpx_rd = 0
ns.zmpy_rd = 0
ns.zmp_double = [0.0,0.0,0.0,0.0]
ns.num_of_samples_l = 0
ns.num_of_samples_r = 0
ns.ii=1


nr.force_x = 0
nr.force_y = 0
nr.force_z = 0
nr.t_x = 0
nr.t_y = 0
nr.t_z = 0


nl.force_x = 0
nl.force_y = 0
nl.force_z = 0
nl.t_x = 0
nl.t_y = 0
nl.t_z = 0
ns.response = [nl.force_x, nl.force_y, nl.force_z, nl.t_x, nl.t_y, nl.t_z, nr.force_x, nr.force_y, nr.force_z, nr.t_x, nr.t_y, nr.t_z] 
ns.reflex=contact_reflex()

def get_l_foot_contact(msg):
    

    #rospy.loginfo("wewewewewewewe")
    nl.force_x = msg.wrench.force.x
    nl.force_y = msg.wrench.force.y
    nl.force_z = msg.wrench.force.z
    nl.t_x =  msg.wrench.torque.x
    nl.t_y =  msg.wrench.torque.y
    nl.t_z =  msg.wrench.torque.z
    ns.num_of_samples_l  = ns.num_of_samples_l + 1
    #rospy.loginfo(nl.t_x)

def get_r_foot_contact(msg):
    

    nr.force_x = msg.wrench.force.x
    nr.force_y = msg.wrench.force.y
    nr.force_z = msg.wrench.force.z 
    nr.t_x = msg.wrench.torque.x
    nr.t_y = msg.wrench.torque.y
    nr.t_z = msg.wrench.torque.z
    ns.num_of_samples_r =ns.num_of_samples_r = +1
    rospy.loginfo(ns.num_of_samples_r)


  
   
def ResetSum():

    ns.num_of_samples_r = 0
    ns.num_of_samples_l = 0
    nl.force_x = 0
    nl.force_y = 0
    nl.force_z = 0
    nl.t_x = 0
    nl.t_y = 0
    nl.t_z = 0

    nr.force_x = 0
    nr.force_y = 0
    nr.force_z = 0
    nr.t_x = 0
    nr.t_y = 0
    nr.t_z = 0     



    


def getAvgForce_Torque():

    if ns.num_of_samples_l != 0:
       rospy.get_time()
       nl.force_x = nl.force_x/ns.num_of_samples_l
       nl.force_y = nl.force_y/ns.num_of_samples_l
       nl.force_z = nl.force_z/ns.num_of_samples_l
       nl.t_x = nl.t_x/ns.num_of_samples_l
       nl.t_y = nl.t_y/ns.num_of_samples_l
       nl.t_z = nl.t_z/ns.num_of_samples_l
    if ns.num_of_samples_r != 0:
     

       nr.force_x = nr.force_x/ns.num_of_samples_r
       nr.force_y = nr.force_y/ns.num_of_samples_r
       nr.force_z = nr.force_z/ns.num_of_samples_r
       nr.t_x = nr.t_x/ns.num_of_samples_r
       nr.t_y = nr.t_y/ns.num_of_samples_r
       nr.t_z = nr.t_z/ns.num_of_samples_r
      
       ns.response = [nl.force_x,nl.force_y,nl.force_z,nl.t_x,nl.t_y,nl.t_z,nl.t_x,nl.t_y,nl.t_z ,nr.force_x,nr.force_y,nr.force_z,nr.t_x,nr.t_y,nr.t_z] 

       ResetSum()

   # # else:

       
    #  ns.response = [nl.force_x, nl.force_y, nl.force_z, nl.t_x, nl.t_y, nl.t_z, nr.force_x, nr.force_y, nr.force_z, nr.t_x, nr.t_y, nr.t_z] 
      #ns.ii =ns.ii + 1
 
      #rospy.loginfo(ns.ii)

     

      #left_contact_F_T.write(' %s %s %s %s %s %s\n'  %(str(rospy.get_time()),ns.response[2],ns.response[3],ns.response[4],msg.step_phase,ns.num_of_samples))  
    

      #right_contact_F_T.write(' %s %s %s %s %s %s\n'  %(str(rospy.get_time()),ns.response[8],ns.response[9],ns.response[10],msg.step_phase,ns.num_of_samples))
      #return (ns.response) 
    else:
     ns.response = [nl.force_x,nl.force_y,nl.force_z,nl.t_x,nl.t_y,nl.t_z,nl.t_x,nl.t_y,nl.t_z ,nr.force_x,nr.force_y,nr.force_z,nr.t_x,nr.t_y,nr.t_z]

 





def zmp_calculation(msg):

  getAvgForce_Torque()
  #rospy.loginfo(msg.swing_foot.x)
  #rospy.loginfo(msg.step_phase)
  #ns.response = [nl.force_x, nl.force_y, nl.force_z, nl.t_x, nl.t_y, nl.t_z,
  #               nr.force_x, nr.force_y, nr.force_z, nr.t_x, nr.t_y, nr.t_z] 
  #ns.response = [nl.force_x, nl.force_y, nl.force_z, nl.t_x, nl.t_y, nl.t_z, nr.force_x, nr.force_y, nr.force_z, nr.t_x, nr.t_y, nr.t_z]
  if ( msg.step_phase == 1 ) or ( msg.step_phase == 2 ): # left leg is stance
    M_tot_y = ns.response[4] + ns.response[10] + ns.response[8]*msg.swing_foot.x
    F_tot_z = ns.response[2] + ns.response[8]
    ns.zmpx_l = M_tot_y/(F_tot_z + 0.00001)
    M_tot_x = ns.response[3] + ns.response[9] + ns.response[8]*msg.swing_foot.y
    ns.zmpy_l = M_tot_x/(F_tot_z + 0.0000001)+0.182/2
    zmp_x = ns.zmpx_l 
    zmp_y = ns.zmpy_l
    zmp_left.write('%s %s %s %s %s %s\n'  %(str(rospy.get_time()),ns.zmpx_l,ns.zmpy_l,msg.step_phase,msg.zmp_ref.x,msg.zmp_ref.y))
    #zmp_right.write('%s %s %s %s %s %s\n'  %(str(rospy.get_time()),0,0,0,0,0))

  if ( msg.step_phase == 3 ) or ( msg.step_phase == 4 ): # right leg is stance 
    M_tot_y = ns.response[4] + ns.response[10] + ns.response[2]*msg.swing_foot.x
    F_tot_z = ns.response[2] + ns.response[8]
    ns.zmpx_r = M_tot_y/(F_tot_z + 0.00001)
    M_tot_x = ns.response[3] + ns.response[9] + ns.response[2]*msg.swing_foot.y
    ns.zmpy_r = M_tot_x/(F_tot_z + 0.0000001)-0.182/2
    zmp_x = ns.zmpx_r
    zmp_y = ns.zmpy_r
    
    zmp_right.write('%s %s %s %s %s %s\n'  %(str(rospy.get_time()),ns.zmpx_r,ns.zmpy_r,msg.step_phase,msg.zmp_ref.x,msg.zmp_ref.y))
    #zmp_left.write('%s %s %s %s %s %s\n'  %(str(rospy.get_time()),0,0,0,0,0))
  zmp.write('%s %s %s %s %s %s %s %s\n'  %(str(rospy.get_time()),zmp_x,zmp_y,msg.step_phase,msg.zmp_ref.x,msg.zmp_ref.y,msg.com_ref.x,msg.com_ref.y))
  
  nsw.out.zmpx_r = ns.zmpx_r
  nsw.out.zmpy_r = ns.zmpy_r
  nsw.out.zmpx_l = ns.zmpx_l
  nsw.out.zmpy_l = ns.zmpy_l
  nsw.out.zmpyBound1 =  0.062
  nsw.out.zmpyBound2 = -0.062
  nsw.out.zmpxBound1 = 0.1754
  nsw.out.zmpxBound2 = -0.083
  nsw.pub_zmp.publish(nsw.out)

def zmp_check():

    rospy.init_node('zmp_check')
        
    rospy.loginfo( "zmp_check node is ready" )
    nsw.pub_zmp = rospy.Publisher('zmpreal_out', zmp_real )

    rospy.sleep(0.5)

    #contact_sub = rospy.Subscriber('/atlas/force_torque_sensors', ForceTorqueSensors, get_foot_contact)
    left_sub = rospy.Subscriber('/atlas/debug/l_foot_contact',WrenchStamped,get_l_foot_contact)
    right_sub = rospy.Subscriber('/atlas/debug/r_foot_contact',WrenchStamped,get_r_foot_contact)
    rospy.loginfo( "zzzzzzzzzzzz is ready" )
    #contact_sub = rospy.Subscriber('/atlas/force_torque_sensors', ForceTorqueSensors, get_foot_contact)
    # contact sensor filter:
    a = [1,-3.180638548874721,3.861194348994217,-2.112155355110971,0.438265142261981]
    b = [0.0004165992044065786,0.001666396817626,0.002499595226439,0.001666396817626,0.0004165992044065786]
    #ns.filter = contact_filter(b = b, a = a, use_internal_subscriber = False)

    walking = rospy.Subscriber("zmp_out", walking_trajectory, zmp_calculation)
    nsw.out = zmp_real()
 

if __name__ == '__main__':

    right_contact_F_T = open('right_foot_walk.txt','w')
    left_contact_F_T =  open('left_foot_walk.txt','w')
    zmp_right = open('zmp_right_foot.txt','w')
    zmp_left =  open('zmp_left_foot.txt','w')
    zmp =  open('zmp.txt','w')

    zmp_check()
    rospy.spin()









