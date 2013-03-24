#! /usr/bin/env python
import roslib; roslib.load_manifest('DRCSim2_tools')
import rospy
import copy
from atlas_msgs.msg import ForceTorqueSensors
import control_primitives

class contact_filter(control_primitives.filter):
    def __init__(self,a,b):
        control_primitives.filter.__init__(self,a = a,b = b)
        self.sub = rospy.Subscriber('/atlas/force_torque_sensors',ForceTorqueSensors,self.update)
        self.u_buffer = [ForceTorqueSensors() for k in xrange(len(self.a))]
        self.y_buffer = [ForceTorqueSensors() for k in xrange(len(self.b))]
    def update(self,u):
        F = u
        control_primitives.filter.update(self,F)
    def _filter_func(self):
        y_filt = ForceTorqueSensors()
        y_filt.l_foot.force.z = copy.copy(self.b[0]*self.u_buffer[0].l_foot.force.z)#ForceTorqueSensors()
        y_filt.r_foot.force.z = copy.copy(self.b[0]*self.u_buffer[0].r_foot.force.z)#ForceTorqueSensors()
        for t in xrange(1,len(self.a)):
            y_filt.l_foot.force.z += self.y_buffer[t-1].l_foot.force.z*(-self.a[t]) + self.u_buffer[t].l_foot.force.z*self.b[t]
            y_filt.r_foot.force.z += self.y_buffer[t-1].r_foot.force.z*(-self.a[t]) + self.u_buffer[t].r_foot.force.z*self.b[t]
        return y_filt

if __name__ == '__main__':
    class test_filter(control_primitives.controller):
        def __init__(self):
            control_primitives.controller.__init__(self,name = 'filter_test',Fs = 100)
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
        
