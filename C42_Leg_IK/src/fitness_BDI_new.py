import roslib; roslib.load_manifest('C42_Leg_IK')
# import roslib; roslib.load_manifest('C42_ZMPWalk')
# from C42_ZMPWalk.msg import Position, Orientation, Pos_and_Ori
from std_msgs.msg import Float64
from sensor_msgs.msg import *
# from C42_ZMPWalk.msg import walking_trajectory#,Pos_and_Ori
from C42_Leg_IK.msg import zmp_real
from drc2_tools import *
import tf
from std_msgs.msg import Int32
import rospy, math, sys,numpy
import pylab as pl
from atlas_msgs.msg import ForceTorqueSensors
from contact_reflex import contact_reflex
from foot_contact_filter import contact_filter
from sensor_msgs.msg import JointState
import control_primitives
from geometry_msgs.msg import *
import copy

class Fitness(object):
    def __init__(self):
        self.N = 2000
        self.get_fit = 0
        self.reset_buffer_torque()

        self.zmp_real_m = 0
        a = [1,-3.180638548874721,3.861194348994217,-2.112155355110971,0.438265142261981]
        b = [0.0004165992044065786,0.001666396817626,0.002499595226439,0.001666396817626,0.0004165992044065786]
        self.filter = control_primitives.filter(b = b, a = a)

    def reset_buffer_torque(self):
        self.torque_count = 0
        self.buff_torque_l_ankl_x = pl.zeros([self.N,1],float)
        self.buff_torque_l_knee_y = pl.zeros([self.N,1],float)
        self.buff_torque_l_hip_x  = pl.zeros([self.N,1],float)
        self.buff_torque_r_ankl_x = pl.zeros([self.N,1],float)
        self.buff_torque_r_knee_y = pl.zeros([self.N,1],float)
        self.buff_torque_r_hip_x  = pl.zeros([self.N,1],float)

        self.buff_torque_l_ankl_x_t = []
        self.buff_torque_l_knee_y_t = []
        self.buff_torque_l_hip_x_t  = []
        self.buff_torque_r_ankl_x_t = []
        self.buff_torque_r_knee_y_t = []
        self.buff_torque_r_hip_x_t  = []

        self.step_torque_l_ankl_x = []
        self.step_torque_l_knee_y = []
        self.step_torque_l_hip_x  = []
        self.step_torque_r_ankl_x = []
        self.step_torque_r_knee_y = []
        self.step_torque_r_hip_x  = []
        self.torque_fitness = 0

    def get_torque_buffer(self):
        self.step_torque_l_ankl_x = self.buff_torque_l_ankl_x[0:self.torque_count]
        self.step_torque_l_knee_y = self.buff_torque_l_knee_y[0:self.torque_count]
        self.step_torque_l_hip_x  = self.buff_torque_l_hip_x[0:self.torque_count]
     
        self.step_torque_r_ankl_x = self.buff_torque_r_ankl_x[0:self.torque_count]
        self.step_torque_r_knee_y = self.buff_torque_r_knee_y[0:self.torque_count]
        self.step_torque_r_hip_x  = self.buff_torque_r_hip_x[0:self.torque_count]

    def buffer_torques(self,msg):
       # Filtered data:
        left_ankle_x  = self.filter.update(msg.effort[9])
        left_knee_y   = self.filter.update(msg.effort[7])
        left_hip_x    = self.filter.update(msg.effort[5])
        right_ankle_x = self.filter.update(msg.effort[15])
        right_knee_y  = self.filter.update(msg.effort[13])
        right_hip_x   = self.filter.update(msg.effort[11])

        self.buff_torque_l_ankl_x[self.torque_count] = left_ankle_x
        self.buff_torque_l_knee_y[self.torque_count] = left_knee_y
        self.buff_torque_l_hip_x [self.torque_count] = left_hip_x
        
        self.buff_torque_r_ankl_x[self.torque_count] = right_ankle_x
        self.buff_torque_r_knee_y[self.torque_count] = right_knee_y
        self.buff_torque_r_hip_x [self.torque_count] = right_hip_x
        self.torque_count += 1

    def buffer_torques_text(self,msg,contact_r,contact_l):
       # Filtered data:
        left_ankle_x  = self.filter.update(msg.effort[9])
        left_knee_y   = self.filter.update(msg.effort[7])
        left_hip_x    = self.filter.update(msg.effort[5])
        right_ankle_x = self.filter.update(msg.effort[15])
        right_knee_y  = self.filter.update(msg.effort[13])
        right_hip_x   = self.filter.update(msg.effort[11])
        fitness_torq.write('%s %s %s %s %s %s %s %s %s \n'  %(str(rospy.get_time()),left_ankle_x,left_knee_y,left_hip_x,right_ankle_x,right_knee_y,right_hip_x,contact_r,contact_l))

    def fitness_torques(self):
        T_lax_max =  max(self.step_torque_l_ankl_x.max(0),abs(self.step_torque_l_ankl_x.min(0))) 
        T_lky_max = max(self.step_torque_l_knee_y.max(0),abs(self.step_torque_l_knee_y.min(0)))
        T_lhx_max = max(self.step_torque_l_hip_x.max(0),abs(self.step_torque_l_hip_x.min(0)))

        T_rax_max = max(self.step_torque_r_ankl_x.max(0),abs(self.step_torque_r_ankl_x.min(0))) 
        T_rky_max = max(self.step_torque_r_knee_y.max(0),abs(self.step_torque_r_knee_y.min(0)))
        T_rhx_max = max(self.step_torque_r_hip_x.max(0),abs(self.step_torque_r_hip_x.min(0)))
        self.torque_fitness = 1.0/6.0*( (1.0-T_lax_max/90.0)+(1.0-T_lky_max/220.0)+(1.0-T_lhx_max/180.0)+(1.0-T_rax_max/90.0)+(1.0-T_rky_max/220.0)+(1.0-T_rhx_max/180.0) )
        print 'torq_fitt:',self.torque_fitness
        return(self.torque_fitness)

class test(object): 
    def __init__(self):
        self.FT = Fitness()
        rospy.init_node('Fit_function')
        rospy.loginfo( "Fitness node is ready" )
        self.listener = tf.TransformListener()
        self.contact = contact_reflex()
        self.state = 0
        self._last_state = 0
        self.force_z_r = 0
        self.force_z_l = 0
        self.contact_l = 0
        self.contact_r = 0
        contact_sub1 = rospy.Subscriber('/atlas/joint_states', JointState, self.update_torque)
        sub2 = rospy.Subscriber("zmpreal_out", zmp_real, self.update_zmp_real) 
        contact_sub = rospy.Subscriber('/atlas/force_torque_sensors', ForceTorqueSensors, self.update_zmp_state)

        a = [1,-3.180638548874721,3.861194348994217,-2.112155355110971,0.438265142261981]
        b = [0.0004165992044065786,0.001666396817626,0.002499595226439,0.001666396817626,0.0004165992044065786]
        self.filter2 = contact_filter(b = b, a = a, use_internal_subscriber = False)

    def update_zmp_state(self,msg):
        self.filter2.update(msg)
        buf = self.filter2.get_buffer()
        self.force_z_r = buf[0].r_foot.force.z
        self.force_z_l = buf[0].l_foot.force.z
        self.contact_r,self.contact_l = self.contact.update(self.force_z_r,self.force_z_l)
        if ( self.contact_l == 1 ) and ( self.contact_r == 0 ):
            self.state = 1
        if (( self.contact_l == 0 ) and ( self.contact_r == 1 )) and (self.state==1):# stance = left, swing = right 

           self.FT.get_torque_buffer()
           torque_fitness = self.FT.fitness_torques()
           rospy.loginfo("Torque_RMS : torque_fitness = %f" % (torque_fitness) )
           self.get_fit = 0
           self.FT.reset_buffer_torque()
           self.state = 0

    def update_zmp_real(self,msg):
        self.zmp_real_m = msg

    def update_zmp_out(self,msg):
        if self.state == 4:
            self.FT.buffer_zmp(msg.zmp_ref,self.zmp_real_m)
            self.FT.buffer_hip(msg.stance_hip,msg.stance_hip_m)

    def update_torque(self,msg):
        self.FT.buffer_torques_text(msg,self.contact_r,self.contact_l)
        if ( self.contact_l == 1 ) and ( self.contact_r == 0 ):
            self.FT.buffer_torques(msg)
            
if __name__ == '__main__':
    fitness_torq =  open('fitness_torq.txt','w')

    tst = test()
    rospy.spin()
