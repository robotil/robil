#! /usr/bin/env python
import roslib; roslib.load_manifest('C42_DRCSim2_tools')
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import control_primitives
import copy

class imu_vel_filter(control_primitives.filter):
    def __init__(self,a,b):
        control_primitives.filter.__init__(self,a = a,b = b)
        self.imu_sub = rospy.Subscriber('/atlas/imu',Imu,self._update)
        self.u_buffer = [Vector3() for k in xrange(len(self.a))]
        self.y_buffer = [Vector3() for k in xrange(len(self.b))]
    def _update(self,u):
        vel = u.angular_velocity
        control_primitives.filter.update(self,vel)
    def _filter_func(self):
        y_filt = Vector3()
        y_filt.x = self.b[0]*self.u_buffer[0].x
        y_filt.y = self.b[0]*self.u_buffer[0].y
        y_filt.z = self.b[0]*self.u_buffer[0].z
        for t in xrange(1,len(self.a)):
            y_filt.x += self.y_buffer[t-1].x*(-self.a[t]) + self.u_buffer[t].x*self.b[t]
            y_filt.y += self.y_buffer[t-1].y*(-self.a[t]) + self.u_buffer[t].y*self.b[t]
            y_filt.z += self.y_buffer[t-1].z*(-self.a[t]) + self.u_buffer[t].z*self.b[t]
        return y_filt

if __name__ == '__main__':
    class test_imu(control_primitives.controller):
        def __init__(self):
            control_primitives.controller.__init__(self,name = 'imu_filter_test',Fs = 100)
            self.pub = rospy.Publisher('/filtered_imu',Vector3)
            # a=[1,-0.909929988177738]
            # b=[0.045035005911131,0.045035005911131]
            a = [1,-2.374094743709352,1.929355669091215,-0.532075368312092]
            b = [0.002898194633721,0.008694583901164,0.008694583901164,0.002898194633721]
            self.imu_filter = imu_vel_filter(a,b)
        def on_update(self):
            buf = self.imu_filter.get_buffer()
            self.pub.publish(buf[0])

    imu_test = test_imu()
    rospy.sleep(1)
    imu_test.start()
    rospy.spin()
        
