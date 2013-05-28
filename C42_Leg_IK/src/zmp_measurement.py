#!/usr/bin/env python
########################3
# cd Projects/Robil/C42_Leg_IK/src
# python zmp_real_bdi.py
#############

import roslib; roslib.load_manifest('C42_Leg_IK')
from std_msgs.msg import Float64,Bool
from sensor_msgs.msg import *
from C42_Leg_IK.msg import zmp_real
import rospy, math, sys,numpy
from IKException import IKReachException
from geometry_msgs.msg import *
import copy
from atlas_msgs.msg import ForceTorqueSensors
from foot_contact_filter import contact_filter
from contact_reflex import contact_reflex
import tf
import pylab as pl

class Nasmpace_zmp: pass
ns = Nasmpace_zmp()
class Nasmpace_r_leg: pass
nr = Nasmpace_r_leg()
class Nasmpace_l_leg: pass
nl = Nasmpace_l_leg()


ns.out = zmp_real()
ns.listener = tf.TransformListener()
ns.contact = contact_reflex()
ns.translation =0
ns.rotation =0

ns.zmpx_l = 0
ns.zmpy_l = 0
ns.zmpx_r = 0
ns.zmpy_r = 0
ns.Limit_R = 0
ns.Limit_L = 0
ns.Limit_Front = 0
ns.Limit_Back = 0

ns.errx = 0
ns.erry = 0

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

ns.response = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

def get_foot_contact(msg):
      
    ns.filter.update(msg)
    buf = ns.filter.get_buffer()
    nr.force_x = buf[0].r_foot.force.x
    nr.force_y = buf[0].r_foot.force.y
    nr.force_z = buf[0].r_foot.force.z
    nl.force_x = buf[0].l_foot.force.x
    nl.force_y = buf[0].l_foot.force.y
    nl.force_z = buf[0].l_foot.force.z

    nr.t_x = buf[0].r_foot.torque.x
    nr.t_y = -buf[0].r_foot.torque.y
    nr.t_z = buf[0].r_foot.torque.z
    nl.t_x =  buf[0].l_foot.torque.x
    nl.t_y =  -buf[0].l_foot.torque.y
    nl.t_z =  buf[0].l_foot.torque.z

    nr.contact,nl.contact = ns.contact.update(nr.force_z,nl.force_z)
    ns.response = [nl.force_x, nl.force_y, nl.force_z, nl.t_x, nl.t_y, nl.t_z, nr.force_x, nr.force_y, nr.force_z, nr.t_x, nr.t_y, nr.t_z] 

    zmp_calculation(ns.listener)
     
def zmp_calculation(listener):
  #ns.response = [nl.force_x, nl.force_y, nl.force_z, nl.t_x, nl.t_y, nl.t_z,
  #               nr.force_x, nr.force_y, nr.force_z, nr.t_x, nr.t_y, nr.t_z] 

  if ( nl.contact == 1 ) and ( nr.contact == 0 ): 
            # stance = left, swing = right
            base_frame =  'l_foot' 
            get_frame = 'r_foot' 
        
  elif ( nr.contact == 1 ) and ( nl.contact == 1 ):
            # stance = left, support = right
            base_frame = 'l_foot'
            get_frame = 'r_foot' 

 # elif ( nr.contact == 1 ) and ( nl.contact == 0 ):
  else:
            # stance = right, swing = left
            base_frame = 'r_foot'
            get_frame = 'l_foot' 

  (ns.translation,ns.rotation) = listener.lookupTransform(base_frame, get_frame, rospy.Time(0))  

  if ( nr.contact == 1 ) and ( nl.contact == 0 ):
    M_tot_y = ns.response[4] + ns.response[10] + ns.response[2]*ns.translation[0]
    F_tot_z = ns.response[2] + ns.response[8]
    ns.zmpx_r = M_tot_y/(F_tot_z + 0.00001)#
    M_tot_x = ns.response[3] + ns.response[9] + ns.response[2]*ns.translation[1]
    ns.zmpy_r = M_tot_x/(F_tot_z + 0.0000001) - abs(ns.translation[1]/2)
    zmp_x = ns.zmpx_r
    zmp_y = ns.zmpy_r
  elif ( nl.contact == 1 ) and ( nr.contact == 0 ): 
    M_tot_y = ns.response[4] + ns.response[10] + ns.response[8]*ns.translation[0]
    F_tot_z = ns.response[2] + ns.response[8]
    ns.zmpx_l = M_tot_y/(F_tot_z + 0.00001)
    M_tot_x = ns.response[3] + ns.response[9] + ns.response[8]*ns.translation[1]
    ns.zmpy_l = M_tot_x/(F_tot_z + 0.0000001) + abs(ns.translation[1]/2)
    zmp_x = ns.zmpx_l 
    zmp_y = ns.zmpy_l 

  #elif ( nr.contact == 1 ) and ( nl.contact == 1 ):
  else:
    M_tot_y = ns.response[4] + ns.response[10] + ns.response[8]*ns.translation[0]
    F_tot_z = ns.response[2] + ns.response[8]
    ns.zmpx_l = M_tot_y/(F_tot_z + 0.00001)
    M_tot_x = ns.response[3] + ns.response[9] + ns.response[8]*ns.translation[1]
    ns.zmpy_l = M_tot_x/(F_tot_z + 0.0000001) + abs(ns.translation[1]/2)
    zmp_x = ns.zmpx_l 
    zmp_y = ns.zmpy_l

  ns.out.zmpx_r = ns.zmpx_r
  ns.out.zmpy_r = ns.zmpy_r
  ns.out.zmpx_l = ns.zmpx_l
  ns.out.zmpy_l = ns.zmpy_l
  ns.out.zmpx = zmp_x
  ns.out.zmpy = zmp_y

  ns.Limit_R = -abs(ns.translation[1]/2)-0.062
  ns.Limit_L =  abs(ns.translation[1]/2)+0.062
  ns.Limit_Front = 0.1754
  ns.Limit_Back = -0.083

  ns.out.zmpyBound1 = ns.Limit_L
  ns.out.zmpyBound2 = ns.Limit_R
  ns.out.zmpxBound1 = ns.Limit_Front
  ns.out.zmpxBound2 = ns.Limit_Back
  ns.pub_zmp.publish(ns.out)

  #delta Y:
  if abs(zmp_y-ns.Limit_R) < abs(zmp_y-ns.Limit_L):
    ns.deta_y  = zmp_y - ns.Limit_R # delata positive inside the support polygon
  else:
    ns.deta_y  = ns.Limit_L - zmp_y # delata positive inside the support polygon
  ns.out.erry = ns.deta_y
  
  #delta X:
  if abs(zmp_x-ns.Limit_Back) < abs(zmp_x-ns.Limit_Front):
     ns.deta_x  = zmp_x - ns.Limit_Back # delata positive inside the support polygon

  else:
     ns.deta_x  = ns.Limit_Front - zmp_x # delata positive inside the support polygon


  ns.out.errx = ns.deta_x
  zmp.write('%s %s %s %s %s %s %s\n'  %(str(rospy.get_time()),zmp_x,zmp_y,ns.deta_x,ns.deta_y,nr.contact,nl.contact))

def fall(msg):
    if msg.data:
      zmp.write('----------------------ROBOT HAS FALLEN!---------------------------\n')  
  
def zmp_check():
    rospy.init_node('zmp_check')
    rospy.loginfo( "zmp_check node is ready" )
    ns.pub_zmp = rospy.Publisher('zmpreal_out', zmp_real )
    rospy.sleep(0.5)
    contact_sub = rospy.Subscriber('/atlas/force_torque_sensors', ForceTorqueSensors, get_foot_contact)
    fall_sub = rospy.Subscriber('/C35/falling', Bool, fall)
    
    # contact sensor filter:
    a = [1,-3.180638548874721,3.861194348994217,-2.112155355110971,0.438265142261981]
    b = [0.0004165992044065786,0.001666396817626,0.002499595226439,0.001666396817626,0.0004165992044065786]
    ns.filter = contact_filter(b = b, a = a, use_internal_subscriber = False)
    ns.out = zmp_real()
 

if __name__ == '__main__':
  #write to file
    zmp =  open('zmp.txt','w')

    zmp_check()
    rospy.spin()
