#!/usr/bin/env python
import roslib; roslib.load_manifest('C42_DynamicLocomotion')
import roslib; roslib.load_manifest('C42_Leg_IK')
from atlas_msgs.msg import AtlasSimInterfaceCommand, AtlasSimInterfaceState, AtlasState, AtlasCommand
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import *
from sensor_msgs.msg import Imu
# from C42_DynamicLocomotion.msg import slop_m
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from C42_DynamicLocomotion.msg import imu_contact
from contact_reflex import contact_reflex
from atlas_msgs.msg import ForceTorqueSensors
import control_primitives
from foot_contact_filter import contact_filter

import math
import rospy
import sys
import tf


class Nasmpace: pass
ns = Nasmpace()

class IMUCh(object):
    def __init__(self):

        rospy.sleep(3)
        self.contact = contact_reflex()

        self.leg_force_z_r = 0
        self.leg_force_z_l = 0
        self.leg_force_x_r = 0
        self.leg_force_x_l = 0
        self.leg_force_y_r = 0
        self.leg_force_y_l = 0

        self.arm_force_z_r = 0
        self.arm_force_z_l = 0
        self.arm_force_x_r = 0
        self.arm_force_x_l = 0
        self.arm_force_y_r = 0
        self.arm_force_y_l = 0

        self.arm_force_r = 0
        self.arm_force_l = 0
        self.leg_force_r = 0
        self.leg_force_l = 0

        self.pitch_acc = 0
        self.roll_acc = 0
        self.yaw_acc = 0
        self.count = 0
        self.pitch_avg = 0
        self.roll_avg = 0
        self.yaw_avg = 0

        
        self.last_start = 0
        self.force_treshold = 800
        self.time_treshold = 0.3
        self.time_treshold_contact = 0.03
        self.time_contact_r = 0.0
        self.time_contact_l = 0.0
        self.second_contact = ''
        self.first_contact = ''


        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.F_av = 0
        self.Sigma = 0

        self.stand_up_flag = 0
        self.turned = 0
        self.slop = 0
        self.count_stand = 0

        self.out = imu_contact()

        self.contact_sub = rospy.Subscriber('/atlas/force_torque_sensors', ForceTorqueSensors, self.get_contact)
        self.imu_check = rospy.Subscriber('/atlas/imu',Imu, self.imu_manipulate)
        a = [1,-3.180638548874721,3.861194348994217,-2.112155355110971,0.438265142261981]
        b = [0.0004165992044065786,0.001666396817626,0.002499595226439,0.001666396817626,0.0004165992044065786]
        self.filter = control_primitives.filter(b = b, a = a)
        self.filter2 = contact_filter(b = b, a = a, use_internal_subscriber = False)

    def reset_angle_acc(self):
        self.pitch_acc = 0
        self.roll_acc = 0
        self.yaw_acc = 0
        self.count = 0

    def compute_angle_avg(self,pitch_acc,roll_acc,yaw_acc,count):
        self.pitch_avg = pitch_acc/count
        self.roll_avg = roll_acc/count
        self.yaw_avg = yaw_acc/count

    def imu_manipulate(self,msg):
        self.roll,self.pitch,self.yaw = euler_from_quaternion([msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w])
        self.out.roll = self.roll
        self.out.pitch = self.pitch
        self.out.yaw = self.yaw

       
    def get_contact(self,msg):

       self.filter2.update(msg)
       buf = self.filter2.get_buffer()
       self.leg_force_z_r = buf[0].r_foot.force.z
       self.leg_force_z_l = buf[0].l_foot.force.z
       self.leg_force_y_r = buf[0].r_foot.force.y
       self.leg_force_y_l = buf[0].l_foot.force.y
       self.leg_force_x_r = buf[0].r_foot.force.x
       self.leg_force_x_l = buf[0].l_foot.force.x


       self.arm_force_z_r = buf[0].r_hand.force.z
       self.arm_force_z_l = buf[0].l_hand.force.z      
       self.arm_force_x_r = buf[0].r_hand.force.x
       self.arm_force_x_l = buf[0].l_hand.force.x
       self.arm_force_y_r = buf[0].r_hand.force.y
       self.arm_force_y_l = buf[0].l_hand.force.y

       self.arm_force_r = (self.arm_force_z_r**2 + self.arm_force_y_r**2 + self.arm_force_x_r**2)**0.5
       self.arm_force_l = (self.arm_force_z_l**2 + self.arm_force_y_l**2 + self.arm_force_x_l**2)**0.5
       self.leg_force_r = (self.leg_force_z_r**2 + self.leg_force_y_r**2 + self.leg_force_x_r**2)**0.5
       self.leg_force_l = (self.leg_force_z_l**2 + self.leg_force_y_l**2 + self.leg_force_x_l**2)**0.5

       self.F_av = (self.arm_force_r+self.arm_force_l+self.leg_force_r+self.leg_force_l)/4
       self.Sigma = ( ((self.arm_force_r-self.F_av)**2 + (self.arm_force_l-self.F_av)**2 + (self.leg_force_r-self.F_av)**2 + (self.leg_force_l-self.F_av)**2)/3 )**0.5
      # Check  tipping with contact forces:

       if (self.arm_force_r > self.force_treshold): 
        self.time_contact_r = rospy.get_time()

       if (self.arm_force_l > self.force_treshold): 
        self.time_contact_l = rospy.get_time()

       delta_t = self.time_contact_r - self.time_contact_l

       if 2 >= abs(delta_t) >= self.time_treshold_contact :
        if delta_t > 0:
          self.first_contact = 'arm_l'
          self.second_contact = 'arm_r'
        else:
          self.first_contact = 'arm_r'
          self.second_contact = 'arm_l'

        print 'first_contact:' ,self.first_contact
        print 'second_contact:',self.second_contact
       




       if ((self.arm_force_r > self.force_treshold) or (self.arm_force_l > self.force_treshold)) and ( (rospy.get_time()-self.last_start) > self.time_treshold ):
        self.last_start = rospy.get_time()
        print 'last_start:',self.last_start

        #compute_angle_avg(self.pitch_acc,self.roll_acc,self.yaw_acc)
        self.pitch_avg = self.pitch_acc/self.count
        self.roll_avg = self.roll_acc/self.count
        self.yaw_avg = self.yaw_acc/self.count
        print 'pitch_avg:',self.pitch_avg


        #Check if robot stoped turning and goes with back forward and rise flag turn==1  
        if abs(self.pitch_avg) >= 0.7 :
           self.turned = 1
           print 'Turn finished'

        #Check if robot goes with back forward and on slop and rise flag slop==1  
        if (self.turned == 1) and (abs(self.pitch_avg) < 0.65) :
           self.slop = 1
           #self.turned = 0
           print 'Slop'
           self.count_stand =0

        #Check if robot on slop and ground slop starts to be flat self.count_stand += 1
        if (self.slop == 1 ) and (abs(self.pitch_avg) > 0.61) :
            print 'count_stand:',self.count_stand
            self.count_stand += 1
        #Check if groung is flat i.e. self. count_stand >= 2 ===> Stand up
        if self.count_stand >=2 and (abs(self.pitch_avg)) > 0.61:
               self.stand_up_flag = 1
               print 'Ground is flat :',self.count_stand
               self.slop = 0
               print 'You can stand up!!!'
         #reset_angle_acc()
        self.pitch_acc = 0
        self.roll_acc = 0
        self.yaw_acc = 0
        self.count = 0

       self.pitch_acc += self.pitch
       self.roll_acc += self.roll
       self.yaw_acc += self.yaw
       self.count += 1 

       self.out.leg_r = self.leg_force_r#self.leg_force_z_r#self.leg_contact_r
       self.out.leg_l = self.leg_force_l#self.leg_force_z_l#self.leg_contact_l
       self.out.arm_r = self.arm_force_r#self.arm_force_z_r#self.arm_contact_r
       self.out.arm_l = self.arm_force_l#self.arm_force_z_l#self.arm_contact_l
       self.out.force_avg = self.F_av
       self.out.sigma = self.Sigma

       ns.pub_imu_contact.publish(self.out)
       imu_data.write('%s %s %s %s %s %s %s %s %s %s %s %s %s %s %s\n'  %(str(rospy.get_time()),self.last_start,self.roll,self.pitch,self.yaw,self.arm_force_r,self.arm_force_l,self.leg_force_r,self.leg_force_l,self.F_av,self.Sigma,self.pitch_avg,self.roll_avg,self.yaw_avg,self.last_start))

if __name__ == '__main__':

    rospy.init_node('IMU_monitoring')
    rospy.loginfo( "IMU monitoring node is ready" )
    ns.pub_imu_contact = rospy.Publisher('imureal_out', imu_contact )
    imu_data =  open('imu_data.txt','w')
    walk = IMUCh()

    rospy.spin()
