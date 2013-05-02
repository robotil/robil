#!/usr/bin/env python
########################3
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

ns.zmpx_l = 0
ns.zmpy_l = 0
ns.zmpx_ld = 0
ns.zmpy_ld = 0
ns.zmpx_r = 0
ns.zmpy_r = 0
ns.zmpx_rd = 0
ns.zmpy_rd = 0
ns.zmp_double = [0.0,0.0,0.0,0.0]
ns.num_of_samples = 0


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
  zmp_calculation(msg)


def get_foot_contact(msg):

    ns.num_of_samples += 1

    nr.force_x = nr.force_x + msg.r_foot.force.x
    nr.force_y = nr.force_y + msg.r_foot.force.y
    nr.force_z = nr.force_z + msg.r_foot.force.z
    nr.t_x = nr.t_x + msg.r_foot.torque.x
    nr.t_y = nr.t_y + msg.r_foot.torque.y
    nr.t_z = nr.t_z + msg.r_foot.torque.z

    nl.force_x = nl.force_x +  msg.l_foot.force.x
    nl.force_y = nl.force_y +  msg.l_foot.force.y
    nl.force_z = nl.force_z +  msg.l_foot.force.z
    nl.t_x = nl.t_x + msg.l_foot.torque.x
    nl.t_y = nl.t_y + msg.l_foot.torque.y
    nl.t_z = nl.t_z + msg.l_foot.torque.z

    

def ResetSum():

    ns.num_of_samples = 0

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

   # if ns.num_of_samples != 0:
      rospy.get_time()
      nl.force_x = nl.force_x/ns.num_of_samples
      nl.force_y = nl.force_y/ns.num_of_samples
      nl.force_z = nl.force_z/ns.num_of_samples
      nl.t_x = nl.t_x/ns.num_of_samples
      nl.t_y = nl.t_y/ns.num_of_samples
      nl.t_z = nl.t_z/ns.num_of_samples
     

      nr.force_x = nr.force_x/ns.num_of_samples
      nr.force_y = nr.force_y/ns.num_of_samples
      nr.force_z = nr.force_z/ns.num_of_samples
      nr.t_x = nr.t_x/ns.num_of_samples
      nr.t_y = nr.t_y/ns.num_of_samples
      nr.t_z = nr.t_z/ns.num_of_samples
      
      ns.response = [nl.force_x,nl.force_y,nl.force_z,nl.t_x,nl.t_y,nl.t_z,nl.t_x,nl.t_y,nl.t_z ,nr.force_x,nr.force_y,nr.force_z,nr.t_x,nr.t_y,nr.t_z] 

      ResetSum()

   # else:

      
     # ns.response = [nl.force_x,nl.force_y,nl.force_z,nl.t_x,nl.t_y,nl.t_z,nl.t_x,nl.t_y,nl.t_z ,nr.force_x,nr.force_y,nr.force_z,nr.t_x,nr.t_y,nr.t_z] 

     

      left_contact_F_T.write('%s %s %s %s %s %s %s %s %s\n'  %(str(rospy.get_time()),ns.response[0],ns.response[1],ns.response[2],ns.response[3],ns.response[4],ns.response[5],nsw.phase,ns.num_of_samples))  
    

      right_contact_F_T.write('%s %s %s %s %s %s %s %s %s\n'  %(str(rospy.get_time()),ns.response[6],ns.response[7],ns.response[8],ns.response[9],ns.response[10],ns.response[11],nsw.phase,ns.num_of_samples))
      return (ns.response) 







def zmp_calculation(msg):

  getAvgForce_Torque()

  if ( nsw.phase == 1 ) or ( nsw.phase == 2 ): # left leg is stance
    M_tot_y = ns.response[4] + ns.response[10] + ns.response[8]*nsw.swing.x
    F_tot_z = ns.response[2] + ns.response[8]
    ns.zmpx_l = M_tot_y/(F_tot_z + 0.00001)
    M_tot_x = ns.response[3] + ns.response[9] + ns.response[8]*nsw.swing.y
    ns.zmpy_l = M_tot_x/(F_tot_z + 0.0000001)
    zmp_x = ns.zmpx_l 
    zmp_y = ns.zmpy_l
    zmp_left.write('%s %s %s %s %s %s\n'  %(str(rospy.get_time()),ns.zmpx_l,ns.zmpy_l,nsw.phase,msg.zmp_ref.x,msg.zmp_ref.y))

  if ( nsw.phase == 3 ) or ( nsw.phase == 4 ): # right leg is stance 
    M_tot_y = ns.response[4] + ns.response[10] + ns.response[2]*nsw.swing.x
    F_tot_z = ns.response[2] + ns.response[8]
    ns.zmpx_r = M_tot_y/(F_tot_z + 0.00001)
    M_tot_x = ns.response[3] + ns.response[9] + ns.response[2]*nsw.swing.y
    ns.zmpy_r = M_tot_x/(F_tot_z + 0.0000001)
    zmp_x = ns.zmpx_r
    zmp_y = ns.zmpy_r
    zmp_right.write('%s %s %s %s %s %s\n'  %(str(rospy.get_time()),ns.zmpx_r,ns.zmpy_r,nsw.phase,msg.zmp_ref.x,msg.zmp_ref.y))

  zmp.write('%s %s %s %s %s %s %s %s\n'  %(str(rospy.get_time()),zmp_x,zmp_y,nsw.phase,msg.zmp_ref.x,msg.zmp_ref.y,msg.com_ref.x,msg.com_ref.y))
  
  nsw.out.zmpx_r = ns.zmpx_r
  nsw.out.zmpy_r = ns.zmpy_r
  nsw.out.zmpx_l = ns.zmpx_l
  nsw.out.zmpy_l = ns.zmpy_l
  nsw.pub_zmp.publish(nsw.out)

def zmp_check():

    rospy.init_node('zmp_check')
        
    rospy.loginfo( "zmp_check node is ready" )
    nsw.pub_zmp = rospy.Publisher('zmpreal_out', zmp_real )

    rospy.sleep(0.5)

    contact_sub = rospy.Subscriber('/atlas/force_torque_sensors', ForceTorqueSensors, get_foot_contact)
    walking = rospy.Subscriber("zmp_out", walking_trajectory, get_walking_data)
    nsw.out = zmp_real()
 

    

if __name__ == '__main__':

    right_contact_F_T = open('right_foot_walk.txt','w')
    left_contact_F_T =  open('left_foot_walk.txt','w')
    zmp_right = open('zmp_right_foot.txt','w')
    zmp_left =  open('zmp_left_foot.txt','w')
    zmp =  open('zmp.txt','w')

    zmp_check()
    rospy.spin()
