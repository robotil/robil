#! /usr/bin/env python
import roslib; roslib.load_manifest('C42_DRCSim2_tools')
import rospy
import copy
from atlas_msgs.msg import ForceTorqueSensors
import control_primitives

class contact_filter(control_primitives.filter):
    def __init__(self,a,b, use_internal_subscriber = True):
        control_primitives.filter.__init__(self,a = a,b = b)
        if use_internal_subscriber:
            self.sub = rospy.Subscriber('/atlas/force_torque_sensors',ForceTorqueSensors,self.update)
        self.u_buffer = [ForceTorqueSensors() for k in xrange(len(self.a))]
        self.y_buffer = [ForceTorqueSensors() for k in xrange(len(self.b))]
    def update(self,u):
        control_primitives.filter.update(self,u)
    def _filter_func(self):
        y_filt = ForceTorqueSensors()
        y_filt.l_foot.force.z = copy.copy(self.b[0]*self.u_buffer[0].l_foot.force.z)#ForceTorqueSensors()
        y_filt.r_foot.force.z = copy.copy(self.b[0]*self.u_buffer[0].r_foot.force.z)#ForceTorqueSensors()
        y_filt.l_foot.force.x = copy.copy(self.b[0]*self.u_buffer[0].l_foot.force.x)#ForceTorqueSensors()
        y_filt.r_foot.force.x = copy.copy(self.b[0]*self.u_buffer[0].r_foot.force.x)#ForceTorqueSensors()
        y_filt.l_foot.force.y = copy.copy(self.b[0]*self.u_buffer[0].l_foot.force.y)#ForceTorqueSensors()
        y_filt.r_foot.force.y = copy.copy(self.b[0]*self.u_buffer[0].r_foot.force.y)#ForceTorqueSensors()

        y_filt.l_hand.force.z = copy.copy(self.b[0]*self.u_buffer[0].l_hand.force.z)#ForceTorqueSensors()
        y_filt.r_hand.force.z = copy.copy(self.b[0]*self.u_buffer[0].r_hand.force.z)#ForceTorqueSensors()
        y_filt.l_hand.force.x = copy.copy(self.b[0]*self.u_buffer[0].l_hand.force.x)#ForceTorqueSensors()
        y_filt.r_hand.force.x = copy.copy(self.b[0]*self.u_buffer[0].r_hand.force.x)#ForceTorqueSensors()
        y_filt.l_hand.force.y = copy.copy(self.b[0]*self.u_buffer[0].l_hand.force.y)#ForceTorqueSensors()
        y_filt.r_hand.force.y = copy.copy(self.b[0]*self.u_buffer[0].r_hand.force.y)#ForceTorqueSensors()


        y_filt.l_foot.torque.z = copy.copy(self.b[0]*self.u_buffer[0].l_foot.torque.z)#ForceTorqueSensors()
        y_filt.r_foot.torque.z = copy.copy(self.b[0]*self.u_buffer[0].r_foot.torque.z)#ForceTorqueSensors()
        y_filt.l_foot.torque.x = copy.copy(self.b[0]*self.u_buffer[0].l_foot.torque.x)#ForceTorqueSensors()
        y_filt.r_foot.torque.x = copy.copy(self.b[0]*self.u_buffer[0].r_foot.torque.x)#ForceTorqueSensors()
        y_filt.l_foot.torque.y = copy.copy(self.b[0]*self.u_buffer[0].l_foot.torque.y)#ForceTorqueSensors()
        y_filt.r_foot.torque.y = copy.copy(self.b[0]*self.u_buffer[0].r_foot.torque.y)#ForceTorqueSensors()
        for t in xrange(1,len(self.a)):
            y_filt.l_foot.force.z += self.y_buffer[t-1].l_foot.force.z*(-self.a[t]) + self.u_buffer[t].l_foot.force.z*self.b[t]
            y_filt.r_foot.force.z += self.y_buffer[t-1].r_foot.force.z*(-self.a[t]) + self.u_buffer[t].r_foot.force.z*self.b[t]
            y_filt.l_foot.force.x += self.y_buffer[t-1].l_foot.force.x*(-self.a[t]) + self.u_buffer[t].l_foot.force.x*self.b[t]
            y_filt.r_foot.force.x += self.y_buffer[t-1].r_foot.force.x*(-self.a[t]) + self.u_buffer[t].r_foot.force.x*self.b[t]
            y_filt.l_foot.force.y += self.y_buffer[t-1].l_foot.force.y*(-self.a[t]) + self.u_buffer[t].l_foot.force.y*self.b[t]
            y_filt.r_foot.force.y += self.y_buffer[t-1].r_foot.force.y*(-self.a[t]) + self.u_buffer[t].r_foot.force.y*self.b[t]

            y_filt.l_hand.force.z += self.y_buffer[t-1].l_hand.force.z*(-self.a[t]) + self.u_buffer[t].l_hand.force.z*self.b[t]
            y_filt.r_hand.force.z += self.y_buffer[t-1].r_hand.force.z*(-self.a[t]) + self.u_buffer[t].r_hand.force.z*self.b[t]
            y_filt.l_hand.force.x += self.y_buffer[t-1].l_hand.force.x*(-self.a[t]) + self.u_buffer[t].l_hand.force.x*self.b[t]
            y_filt.r_hand.force.x += self.y_buffer[t-1].r_hand.force.x*(-self.a[t]) + self.u_buffer[t].r_hand.force.x*self.b[t]
            y_filt.l_hand.force.y += self.y_buffer[t-1].l_hand.force.y*(-self.a[t]) + self.u_buffer[t].l_hand.force.y*self.b[t]
            y_filt.r_hand.force.y += self.y_buffer[t-1].r_hand.force.y*(-self.a[t]) + self.u_buffer[t].r_hand.force.y*self.b[t]


            y_filt.l_foot.torque.z += self.y_buffer[t-1].l_foot.torque.z*(-self.a[t]) + self.u_buffer[t].l_foot.torque.z*self.b[t]
            y_filt.r_foot.torque.z += self.y_buffer[t-1].r_foot.torque.z*(-self.a[t]) + self.u_buffer[t].r_foot.torque.z*self.b[t]
            y_filt.l_foot.torque.x += self.y_buffer[t-1].l_foot.torque.x*(-self.a[t]) + self.u_buffer[t].l_foot.torque.x*self.b[t]
            y_filt.r_foot.torque.x += self.y_buffer[t-1].r_foot.torque.x*(-self.a[t]) + self.u_buffer[t].r_foot.torque.x*self.b[t]
            y_filt.l_foot.torque.y += self.y_buffer[t-1].l_foot.torque.y*(-self.a[t]) + self.u_buffer[t].l_foot.torque.y*self.b[t]
            y_filt.r_foot.torque.y += self.y_buffer[t-1].r_foot.torque.y*(-self.a[t]) + self.u_buffer[t].r_foot.torque.y*self.b[t]
        return y_filt

if __name__ == '__main__':
    class test_filter(control_primitives.controller):
        def __init__(self):
            control_primitives.controller.__init__(self,name = 'filter_test',Fs = 1000)
            self.pub = rospy.Publisher('/filtered_contacts',ForceTorqueSensors)
            # a = [1,-2.374094743709352,1.929355669091215,-0.532075368312092]
            # b = [0.002898194633721,0.008694583901164,0.008694583901164,0.002898194633721]
            a = [1,-3.180638548874721,3.861194348994217,-2.112155355110971,0.438265142261981]
            b = [0.0004165992044065786,0.001666396817626,0.002499595226439,0.001666396817626,0.0004165992044065786]
            self.filter = contact_filter(b = b,a = a)
        def on_update(self):
            buf = self.filter.get_buffer()
            buf[0].header.stamp = rospy.Time.now()
            self.pub.publish(buf[0])
            
    test = test_filter()
    rospy.sleep(1)
    test.start()
    rospy.spin()
        
