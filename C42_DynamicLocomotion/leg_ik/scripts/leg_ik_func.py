#!/usr/bin/env python

###############################################################################
####                                                                         ##
####                                                                         ##  
####      computes the IK of the foot in 3D                                  ##
####                                                                         ##
####                                                                         ##
###############################################################################
import roslib
roslib.load_manifest('leg_ik')
import rospy, math , sys  
import tf
import roslib;
from pylab import *
from numpy import *
#import scipy as Sci
#import scipy.linalg   


 
###################################################################
#                       Swing leg IK                              #                                   
###################################################################
 
# swing_foot = desired swing foot position and orientation (x,y,z,r,p,w)
# swing_hip  = measured swing hip position (x,y,z)   
# pelvis_m   = measured pelvis orientation(r,p,w)
  
def swing_leg_ik(swing_foot,swing_hip,pelvis_m):

    l1=round(0.37700,6) #u_leg
    l2=round(0.42200,6) #l_leg

    xankle = array([swing_foot.x,swing_foot.y,swing_foot.z])
    xhip =   array([swing_hip.x,swing_hip.y,swing_hip.z])

    Rrpw_pelvis_m = array([[cos(pelvis_m.w)*cos(pelvis_m.p),cos(pelvis_m.w)*sin(pelvis_m.p)*sin(pelvis_m.r)-sin(pelvis_m.w)*cos(pelvis_m.r),
                                                            cos(pelvis_m.w)*sin(pelvis_m.p)*cos(pelvis_m.r)+sin(pelvis_m.w)*sin(pelvis_m.r)],

                           [sin(pelvis_m.w)*cos(pelvis_m.p),sin(pelvis_m.w)*sin(pelvis_m.p)*sin(pelvis_m.r)+cos(pelvis_m.w)*cos(pelvis_m.r),
                                                            sin(pelvis_m.w)*sin(pelvis_m.p)*cos(pelvis_m.r)-cos(pelvis_m.w)*sin(pelvis_m.r)],

                           [-sin(pelvis_m.p), cos(pelvis_m.p)*sin(pelvis_m.r), cos(pelvis_m.p)*cos(pelvis_m.r)]])

    Rrpw_foot  = array([[cos(swing_foot.w)*cos(swing_foot.p),cos(swing_foot.w)*sin(swing_foot.p)*sin(swing_foot.r)-sin(swing_foot.w)*cos(swing_foot.r),
                                                             cos(swing_foot.w)*sin(swing_foot.p)*cos(swing_foot.r)+sin(swing_foot.w)*sin(swing_foot.r)],
                        
                        [sin(swing_foot.w)*cos(swing_foot.p),sin(swing_foot.w)*sin(swing_foot.p)*sin(swing_foot.r)+cos(swing_foot.w)*cos(swing_foot.r),
                                                             sin(swing_foot.w)*sin(swing_foot.p)*cos(swing_foot.r)-cos(swing_foot.w)*sin(swing_foot.r)],
                        
                        [-sin(swing_foot.p), cos(swing_foot.p)*sin(swing_foot.r),cos(swing_foot.p)*cos(swing_foot.r)]])

    des_pos = dot(linalg.inv(Rrpw_pelvis_m),(xankle-xhip))
    
    
    swing_x = des_pos[0]
    swing_y = des_pos[1]
    swing_z = des_pos[2]
    

    if round((swing_x**2+swing_y**2+swing_z**2),5)<=round((l1+l2)**2,5):

       L1 = swing_x**2+swing_y**2+swing_z**2
       L = round(L1,5)
       kny = math.acos(round((L-l1**2-l2**2),3)/round((2*l1*l2),3))
       mhx = math.atan2(swing_y,-swing_z) # math.atan2(-swing_y,swing_z) - math.pi # gave angle of 6.28 insted of 0
       ztilda = swing_z*math.cos(mhx) - swing_y*math.sin(mhx)
       cosinus = round(- (l1*ztilda+l2*math.cos(kny)*ztilda+swing_x*l2*math.sin(kny))/(swing_x**2+ztilda**2),3)
       sinus = round((-swing_x*l1-swing_x*l2*math.cos(kny)+ztilda*l2*math.sin(kny))/(swing_x**2+ztilda**2),3)
       lhy = math.atan2(sinus,cosinus) 
       uhz = 0 #need to change
       
        
        #Ry(-th_k)*Ry(-th_h_y)*Rx(-th_h_x)*inv(R_pelvis)*R_ankle = Ry(th_a_y)*Rx(th_a_x)
        
       Ry_th_k_minus = array([[cos(kny), 0.0, -sin(kny)],  
                              [ 0.0,     1.0,    0.0   ],
                              [sin(kny), 0.0,  cos(kny)]])

       Ry_th_hip_y_minus = array([[cos(lhy), 0.0, -sin(lhy)],  
                                  [ 0.0,     1.0,    0.0   ],
                                  [sin(lhy), 0.0,  cos(lhy)]])
 
       Rx_th_hip_x_minus = array([[ 1.0,     0.0,    0.0    ],  
                                  [ 0.0, cos(mhx), sin(mhx) ],
                                  [ 0.0,-sin(mhx), cos(mhx) ]])

       Ry_k_hy_hx_Rp_Rf1 = dot(dot(Ry_th_k_minus,Ry_th_hip_y_minus),Rx_th_hip_x_minus)
       Ry_k_hy_hx_Rp_Rf2 =  dot( linalg.inv(Rrpw_pelvis_m),Rrpw_foot)
       Ry_k_hy_hx_Rp_Rf = dot(Ry_k_hy_hx_Rp_Rf1,Ry_k_hy_hx_Rp_Rf2)

       lax = math.atan2(Ry_k_hy_hx_Rp_Rf[2,1],Ry_k_hy_hx_Rp_Rf[2,2])
       uay = math.atan2(-Ry_k_hy_hx_Rp_Rf[2,0],sqrt(Ry_k_hy_hx_Rp_Rf[0,0]**2+Ry_k_hy_hx_Rp_Rf[1,0]**2))

       
    else:

       rospy.loginfo("swing leg is out of reach") 

       rospy.loginfo("swing_x = %f, swing_y = %f, swing_z = %f" %  (swing_x, swing_y, swing_z) ) 
    
    res = [mhx,lhy,uhz,kny,lax,uay]
    #rospy.loginfo("SwingAngl : mhx[0] = %f, lhy[1] = %f, uhz[2] = %f, kny[3] = %f, lax[4] = %f, uay[5] = %f" % (res[0], res[1], res[2], res[3], res[4], res[5]) )
    return res
###################################################################
#                       Server parameters                         #                                   
###################################################################

###################################################################
#                       Stance leg IK                             #                                   
###################################################################
#stance_hip = desired stance hip position (x,y,z)
#pelvis_d   = desired pelvis orientation (r,p,w)
def stance_leg_ik(stance_hip,pelvis_d):

    l1 = round(0.37700,6) #u_leg
    l2 = round(0.42200,6) #l_leg
           
    xhip_stance =   array([stance_hip.x,stance_hip.y,stance_hip.z])

    Rrpw_pelvis_d = array([[math.cos(pelvis_d.w)*math.cos(pelvis_d.p),math.cos(pelvis_d.w)*math.sin(pelvis_d.p)*math.sin(pelvis_d.r)-math.sin(pelvis_d.w)*math.cos(pelvis_d.r),
                                                                      math.cos(pelvis_d.w)*math.sin(pelvis_d.p)*math.cos(pelvis_d.r)+math.sin(pelvis_d.w)*math.sin(pelvis_d.r)],
                           
                           [math.sin(pelvis_d.w)*math.cos(pelvis_d.p),math.sin(pelvis_d.w)*math.sin(pelvis_d.p)*math.sin(pelvis_d.r)+math.cos(pelvis_d.w)*math.cos(pelvis_d.r),
                                                                      math.sin(pelvis_d.w)*math.sin(pelvis_d.p)*math.cos(pelvis_d.r)-math.cos(pelvis_d.w)*math.sin(pelvis_d.r)],
                         
                           [-math.sin(pelvis_d.p), math.cos(pelvis_d.p)*math.sin(pelvis_d.r),math.cos(pelvis_d.p)*math.cos(pelvis_d.r)]])
    hip_x = stance_hip.x    
    hip_y = stance_hip.y 
    hip_z = stance_hip.z 
   
    
    L1 = hip_x**2+hip_y**2+hip_z**2
    L = round(L1,6)
    
    if round((hip_x**2+hip_y**2+hip_z**2),3)<=round((l1+l2)**2,3):

        lax = math.atan2(hip_y,hip_z)

        des_cos_qky = round((L-l1**2-l2**2),3)/round((2*l1*l2),3);
        cos_qky = max(min(0.9998,des_cos_qky),-0.9998)
        kny = math.acos(cos_qky);

        ztilda = hip_z*math.cos(lax) + hip_y*math.sin(lax)
        cosinus = round((-l1*math.sin(kny)*hip_x+ztilda*l2+l1*ztilda*math.cos(kny))/L,3)
        sinus = round( -(l2*hip_x+l1*math.cos(kny)*hip_x+ztilda*l1*math.sin(kny))/L,3)
        uay = math.atan2(sinus,cosinus)


        #Rx(th_ax)*Ry(th_ay)*Ry(th_k)*Rrpw_pelvis_d =Rz(-th_hip_z)Ry(-th_hip_y)*Rx(-th_hip_x)
        
        Ry_th_k = array([[math.cos(kny),  0.0, math.sin(kny) ],  
                         [ 0.0,           1.0,      0.0      ],
                         [-math.sin(kny), 0.0,  math.cos(kny)]])

        Ry_th_a_y = array([[math.cos(uay),  0.0, math.sin(uay) ],  
                           [ 0.0,           1.0,   0.0         ],
                           [-math.sin(uay), 0.0,  math.cos(uay)]])

        Rx_th_a_x = array([[ 1.0,     0.0,      0.0             ],  
                           [ 0.0, math.cos(lax), -math.sin(lax) ],
                           [ 0.0, math.sin(lax),  math.cos(lax)  ]])
        #R_ax_ay_k = Rx_th_a_x*Ry_th_a_y*Ry_th_k*Rrpw_pelvis_d
        R_ax_ay_k1 = dot(Rx_th_a_x,Ry_th_a_y)
        R_ax_ay_k = dot(dot(R_ax_ay_k1,Ry_th_k),Rrpw_pelvis_d)  
        #rospy.loginfo("R_ax_ay_k : R_ax_ay_k[0,2] = %f, " % (R_ax_ay_k[0,2]) )   
        mhx = -math.atan2(R_ax_ay_k[2,1],R_ax_ay_k[2,2])
        lhy = -math.atan2(-R_ax_ay_k[2,0],math.sqrt((R_ax_ay_k[0,0])**2+(R_ax_ay_k[1,0])**2))
        
        uhz = 0 #need to change

 
    else:

        rospy.loginfo("stance leg is out of reach") 
     
   
    res = [mhx,lhy,uhz,kny,lax,uay]
    #rospy.loginfo("StanceAngl : mhx[0] = %f, lhy[1] = %f, uhz[2] = %f, kny[3] = %f, lax[4] = %f, uay[5] = %f" % (res[0], res[1], res[2], res[3], res[4], res[5]) )
    return res


