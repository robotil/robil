#!/usr/bin/env python

#####################################################################################
####    creates the leg_ik_server node which computes the IK of the foot in 3D     ##
####    the service handles LegIk.srv sevices                                      ##
####    request is foot coordinates (x,y,z) with reference to the hip              ##
####    resposes are hip (mhx,lhy,uhz) and knee (kny) angles                       ##
#####################################################################################

import roslib; roslib.load_manifest('leg_ik')
#roslib.load_manifest('using_tf_with_com')
roslib.load_manifest('hrl_kinematics')
from leg_ik.srv import *
from leg_ik.msg import *
#from using_tf_with_com.msg import *
from hrl_kinematics.msg import *
#from ZMP.msg import *
#from imu.srv import *
#from imu.msg import *
import rospy, math , sys 
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
import tf


class Nasmpace5: pass
ns = Nasmpace5()
ns.EIx = 0.0
ns.EIy = 0.0
ns.EIz = 0.0
ns.COMx_d = 0.0
ns.COMy_d = 0.0
ns.COMz_d = 0.0

###################################################################
##                                                                #
##                COM CONTROL                                     #
##                                                                #
###################################################################

def get_com(msg):

  right_leg_is_swing=1
  
  if right_leg_is_swing==1:

   ns.com_x=msg.x[2] # with CoM array. CoM for different frames: 0- "pelvis"; 1-"r_foot"; 2- "l_foot" 
   ns.com_y=msg.y[2]
   ns.com_z=msg.z[2]
  else:
   ns.com_x=msg.x[1] # with CoM array. CoM for different frames: 0- "pelvis"; 1-"r_foot"; 2- "l_foot" 
   ns.com_y=msg.y[1]
   ns.com_z=msg.z[1]
  

  ns.EPx = ns.com_x - ns.COMx_d 
  ns.EPy = ns.com_y - ns.COMy_d 
  ns.EPz = ns.com_z - ns.COMz_d 
  
  ns.dt  = 0.01
  
  ns.EIx = (ns.EIx + ns.EPx)*ns.dt
  ns.EIy = (ns.EIy + ns.EPy)*ns.dt
  ns.EIz = (ns.EIz + ns.EPz)*ns.dt

###################################################################
#                       Swing leg IK                              #                                   
###################################################################

def swing_leg_ik(req):

    #print "Returning leg joint angles"

    l1=round(0.37700,6) #u_leg
    l2=round(0.42200,6) #l_leg
   # eps = 0.00000001

    swing_x = req.pos.Swing_x
    swing_y = req.pos.Swing_y
    swing_z = req.pos.Swing_z  -l1 -l2
    res = LegIkResponse()

    if round((swing_x**2+swing_y**2+swing_z**2),5)<=round((l1+l2)**2,5):

        L1=swing_x**2+swing_y**2+swing_z**2
        L=round(L1,5)
        res.ang.kny = math.acos(round((L-l1**2-l2**2),3)/round((2*l1*l2),3))
        #alph = math.atan2(x,-z)
        #beth = math.acos((l1**2 + L - l2**2)/(2*l1*math.sqrt(L)))
        ztilda = swing_z*math.cos(res.ang.mhx) - swing_y*math.sin(res.ang.mhx)
        cosinus = round(- (l1*ztilda+l2*math.cos(res.ang.kny)*ztilda+swing_x*l2*math.sin(res.ang.kny))/(swing_x**2+ztilda**2),3)
        sinus = round((-swing_x*l1-swing_x*l2*math.cos(res.ang.kny)+ztilda*l2*math.sin(res.ang.kny))/(swing_x**2+ztilda**2),3)
        res.ang.lhy = math.atan2(sinus,cosinus) 
        res.ang.uhz = 0.0
        res.ang.mhx = math.atan2(swing_y,-swing_z)  # math.atan2(-swing_y,swing_z) - math.pi # gave angle of 6.28 insted of 0
        res.ang.lax = -res.ang.mhx
        res.ang.uay = -(res.ang.lhy+res.ang.kny)

       # rospy.loginfo([res.ang.lhy,res.ang.mhx]) 

        

    else:

        rospy.loginfo("swing leg is out of reach") 
    rospy.loginfo("Swing Leg IK angles: ay=%f,kny=%f,hip_y=%f,ax=%f,hip_x=%f " %(res.ang.uay,res.ang.kny,res.ang.lhy,res.ang.lax,res.ang.mhx)) 
    #    rospy.loginfo("swing_x,swing_y,swing_z") 
    #    rospy.loginfo([swing_x,swing_y,swing_z])
    rospy.loginfo("Swing Leg position: swing_x=%f,swing_y=%f,swing_z=%f " %(swing_x,swing_y,swing_z))  


    return res


  
###################################################################
#                       Stance leg IK                             #                                   
###################################################################

def stance_leg_ik(req):

    l1=round(0.37700,6) #u_leg
    l2=round(0.42200,6) #l_leg
       #eps = 0.00000001
    
    right_leg_is_swing = req.pos.leg
    
    if right_leg_is_swing: 
        hip_frame = 'l_uleg'
    else:
        hip_frame = 'r_uleg'
       
    com_frame = 'com' 
            # while listener.waitForTransform (base_frame, hip_frame, rospy.Time(0), 0.1, 0.01) and not rospy.is_shutdown():
            # rospy.loginfo("Not ready for Forward Kinematics transform")
            
            #try:
    (delta,rot) = ns.listener.lookupTransform(com_frame, hip_frame, rospy.Time(0)) # delta is hip - CoM
            # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
            # print ex
            # continue
  
    delX = 0.020318 -delta[0] # Yuval Comm

    #rospy.loginfo('delta[0]=%f' %(delta[0])) 

    # # Yuval Comm
    if right_leg_is_swing:
        delY = 0.089139 - delta[1]
    else:
        delY = 0.089139 + delta[1]
   # delY = 0 # Yuval added


    #rospy.loginfo('delta[1]=%f' %(delta[1])) 
    delZ = 0#delta[2]
     
    ns.COMx_d = req.pos.COMx
    ns.COMy_d = req.pos.COMy
    ns.COMz_d = req.pos.COMz
    
    hip_x = ns.COMx_d + delX    
    hip_y = ns.COMy_d + delY
    hip_z = ns.COMz_d + delZ + l1 +l2
   
    # rospy.loginfo('delX=%f' %delX) 
    # rospy.loginfo('delY=%f' %delY) 
    # rospy.loginfo('delZ=%f' %delZ) 


    L1=hip_x**2+hip_y**2+hip_z**2
    L=round(L1,6)

    res = LegIkResponse()
    if round((hip_x**2+hip_y**2+hip_z**2),3)<=round((l1+l2)**2,3):
        res.ang.lax = math.atan2(hip_y,hip_z)

        des_cos_qky = round((L-l1**2-l2**2),3)/round((2*l1*l2),3);
        cos_qky = max(min(0.9998,des_cos_qky),-0.9998)
        res.ang.kny = math.acos(cos_qky);

        #res.ang.kny = math.acos(round((L-l1**2-l2**2),3)/round((2*l1*l2),3))
        ztilda = hip_z*math.cos(res.ang.lax) + hip_y*math.sin(res.ang.lax)

        cosinus = round((-l1*math.sin(res.ang.kny)*hip_x+ztilda*l2+l1*ztilda*math.cos(res.ang.kny))/L,3)
        sinus = round( -(l2*hip_x+l1*math.cos(res.ang.kny)*hip_x+ztilda*l1*math.sin(res.ang.kny))/L,3)

        res.ang.uay = math.atan2(sinus,cosinus)

        th4 = res.ang.kny
        th5 = res.ang.uay
        th6 = res.ang.lax

        res.ang.mhx = -math.atan2(math.sin(th6),math.cos(th4+ th5)*math.cos(th6)) 
        res.ang.lhy = -math.atan2(math.sin(th4 + th5)*math.cos(th6),math.sqrt(( math.cos(th4 + th5) )**2+( math.sin(th4+th5)*math.sin(th6) )**2))
        res.ang.uhz = 0 #need to change

        res.ang.mby =  0#-res.ang.lhy/4
        res.ang.ubx =  0#res.ang.lax
    else:

        rospy.loginfo("stance leg is out of reach") 
    rospy.loginfo("Stance Leg IK angles: ay=%f,kny=%f,hip_y=%f,ax=%f,hip_x=%f, " %(res.ang.uay,res.ang.kny,res.ang.lhy,res.ang.lax,res.ang.mhx))   
   # rospy.loginfo("lax,kny,uay") 
   # rospy.loginfo([res.ang.lax,res.ang.kny,res.ang.uay]) 
 
    return res
###################################################################
#                       Server parameters                         #                                   
###################################################################
def leg_ik_server():
    rospy.init_node('leg_ik_server')
    
    s1 = rospy.Service('swing_leg_ik', LegIk, swing_leg_ik)
    s2 = rospy.Service('stance_leg_ik', LegIk, stance_leg_ik)
    s3 =rospy.Subscriber('/test_stability/CoM', CoM_Array_msg, get_com)
    ns.listener = tf.TransformListener()
    rospy.loginfo("Leg ik server ready") 
    rospy.spin()

if __name__ == "__main__":
    leg_ik_server()
