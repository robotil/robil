#!/usr/bin/env python
########################3
# cd ROSWorkspace/Robil/C42_Leg_IK/src
# python zmp_real_bdi.py
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
from foot_contact_filter import contact_filter
from contact_reflex import contact_reflex
import tf
from numpy import linalg as LA
#from Impedance_Control import Position_Stiffness_Controller
class Nasmpace_zmp: pass
ns = Nasmpace_zmp()
class Nasmpace_r_leg: pass
nr = Nasmpace_r_leg()
class Nasmpace_l_leg: pass
nl = Nasmpace_l_leg()
class namespace_walk: pass
nsw = namespace_walk()
nsw.phase  = 0
nsw.swing =  Pos_and_Ori()
nsw.out = zmp_real()

##########################################################################################
# request from foot contact publisher
##########################################################################################

ns.listener = tf.TransformListener()
ns.contact = contact_reflex()
ns.zmpx_l = 0
ns.zmpy_l = 0

ns.zmpx_r = 0
ns.zmpy_r = 0
ns.errx = 0
ns.erry = 0
ns.translation =0
ns.rotation =0



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



def get_walking_data(msg): 
  nsw.phase = copy.copy(msg.step_phase)
  nsw.swing = copy.copy(msg.swing_foot)    
  zmp_calculation(msg,ns.listener)


def get_foot_contact(msg):
    # y_filt = ForceTorqueSensors()   
    ns.filter.update(msg)
    buf = ns.filter.get_buffer()

    nr.force_x = buf[0].r_foot.force.x
    nr.force_y = buf[0].r_foot.force.y
    nr.force_z = buf[0].r_foot.force.z 
    nr.t_x = buf[0].r_foot.torque.x
    nr.t_y = -buf[0].r_foot.torque.y
    nr.t_z = buf[0].r_foot.torque.z


    nl.force_x = buf[0].l_foot.force.x
    nl.force_y = buf[0].l_foot.force.y
    nl.force_z = buf[0].l_foot.force.z
    nl.t_x =  buf[0].l_foot.torque.x
    nl.t_y = -buf[0].l_foot.torque.y
    nl.t_z =  buf[0].l_foot.torque.z
    nr.contact,nl.contact = ns.contact.update(nr.force_z,nl.force_z)
    zmp_calculation(ns.listener)


    # rospy.loginfo(nl.t_y)
    # rospy.loginfo(nr.t_y)
    # rospy.loginfo(nl.force_z)
    #rospy.loginfo(nr.t_x)
    #rospy.loginfo(nr.t_y)

 
def getAvgForce_Torque():

         
      ns.response = [nl.force_x, nl.force_y, nl.force_z, nl.t_x, nl.t_y, nl.t_z, nr.force_x, nr.force_y, nr.force_z, nr.t_x, nr.t_y, nr.t_z] 
      #rospy.loginfo(ns.response)

      # left_contact_F_T.write(' %s %s %s %s %s %s\n'  %(str(rospy.get_time()),ns.response[2],ns.response[3],ns.response[4],nsw.phase,ns.num_of_samples))  

      # right_contact_F_T.write(' %s %s %s %s %s %s\n'  %(str(rospy.get_time()),ns.response[8],ns.response[9],ns.response[10],nsw.phase,ns.num_of_samples))
      return (ns.response) 


def zmp_calculation(listener):

  getAvgForce_Torque()
  #ns.response = [nl.force_x, nl.force_y, nl.force_z, nl.t_x, nl.t_y, nl.t_z,
  #               nr.force_x, nr.force_y, nr.force_z, nr.t_x, nr.t_y, nr.t_z] 



  if ( nl.contact == 1 ) and ( nr.contact == 0 ): 
            # stance = left, swing = right
            base_frame =  'l_foot' 
            # frames to transform: 
            get_frame = 'r_foot' # DRC (from tf) names # 'l_lleg' add for debug links length
        
  elif ( nr.contact == 1 ) and ( nl.contact == 1 ):
            # stance = left, support = right
            base_frame = 'l_foot'
            # frames to transform: 
            get_frame = 'r_foot' # DRC (from tf) names # 'r_lleg' add for debug links length   

  elif ( nr.contact == 1 ) and ( nl.contact == 0 ):
            # stance = right, swing = left
            base_frame = 'r_foot'
            # frames to transform: 
            get_frame = 'l_foot' # DRC (from tf) names # 'r_lleg' add for debug links length

  rospy.loginfo(base_frame)     
  (ns.translation,ns.rotation) = listener.lookupTransform(base_frame, get_frame, rospy.Time(0))  #  rospy.Time(0) to use latest availble transform 
        
  

  if ( nr.contact == 1 ) and ( nl.contact == 0 ):

    M_tot_y = ns.response[4] + ns.response[10] + ns.response[2]*ns.translation[0]
    F_tot_z = ns.response[2] + ns.response[8]
    ns.zmpx_r = M_tot_y/(F_tot_z + 0.00001)#
    M_tot_x = ns.response[3] + ns.response[9] + ns.response[2]*ns.translation[1]
    ns.zmpy_r = M_tot_x/(F_tot_z + 0.0000001)-0.182/2
    zmp_x = ns.zmpx_r
    zmp_y = ns.zmpy_r
  elif ( nl.contact == 1 ) and ( nr.contact == 0 ): 
    M_tot_y = ns.response[4] + ns.response[10] + ns.response[8]*ns.translation[0]
    F_tot_z = ns.response[2] + ns.response[8]
    ns.zmpx_l = M_tot_y/(F_tot_z + 0.00001)
    M_tot_x = ns.response[3] + ns.response[9] + ns.response[8]*ns.translation[1]
    ns.zmpy_l = M_tot_x/(F_tot_z + 0.0000001)+0.182/2
    zmp_x =ns.zmpx_l 
    zmp_y = ns.zmpy_l 

  elif ( nr.contact == 1 ) and ( nl.contact == 1 ):
    M_tot_y = ns.response[4] + ns.response[10] + ns.response[8]*ns.translation[0]
    F_tot_z = ns.response[2] + ns.response[8]
    ns.zmpx_l = M_tot_y/(F_tot_z + 0.00001)
    M_tot_x = ns.response[3] + ns.response[9] + ns.response[8]*ns.translation[1]
    ns.zmpy_l = M_tot_x/(F_tot_z + 0.0000001)+0.182/2
    zmp_x = ns.zmpx_l 
    zmp_y = ns.zmpy_l 

  # erry =  zmp_y - msg.zmp_ref.y
  # nsw.out.erry = LA.norm(erry)
  # errx =  zmp_x - msg.zmp_ref.x


  rospy.loginfo(ns.zmpy_l)
  rospy.loginfo(zmp_y)
  rospy.loginfo(ns.zmpy_r)
  nsw.out.zmpx_r = ns.zmpx_r
  nsw.out.zmpy_r = ns.zmpy_r
  nsw.out.zmpx_l = ns.zmpx_l
  nsw.out.zmpy_l = ns.zmpy_l
  nsw.out.zmpx = zmp_x
  nsw.out.zmpy = zmp_y
  nsw.out.erry = ns.erry
  nsw.out.errx = ns.errx

  nsw.out.zmpyBound1 =  0.182/2+0.062
  nsw.out.zmpyBound2 = -0.182/2-0.062
  nsw.out.zmpxBound1 = 0.1754
  nsw.out.zmpxBound2 = -0.083
  nsw.pub_zmp.publish(nsw.out)

def zmp_check():

    rospy.init_node('zmp_check')
        
    rospy.loginfo( "zmp_check node is ready" )
    nsw.pub_zmp = rospy.Publisher('zmpreal_out', zmp_real )

    rospy.sleep(0.5)

    contact_sub = rospy.Subscriber('/atlas/force_torque_sensors', ForceTorqueSensors, get_foot_contact)
    
    
    # contact sensor filter:
    a = [1,-3.180638548874721,3.861194348994217,-2.112155355110971,0.438265142261981]
    b = [0.0004165992044065786,0.001666396817626,0.002499595226439,0.001666396817626,0.0004165992044065786]
    ns.filter = contact_filter(b = b, a = a, use_internal_subscriber = False)

    #walking = rospy.Subscriber("zmp_out", walking_trajectory, get_walking_data)
    nsw.out = zmp_real()
 

if __name__ == '__main__':

    # right_contact_F_T = open('right_foot_walk.txt','w')
    # left_contact_F_T =  open('left_foot_walk.txt','w')
    # zmp_right = open('zmp_right_foot.txt','w')
    # zmp_left =  open('zmp_left_foot.txt','w')
    # zmp =  open('zmp.txt','w')

    zmp_check()
    rospy.spin()
