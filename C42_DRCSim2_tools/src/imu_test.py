#! /usr/bin/env python
import roslib; roslib.load_manifest('C42_DRCSim2_tools')
import rospy
import tf
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
rospy.init_node('IMU_test')
class imu_test():
	def __init__(self):
		self.sub = rospy.Subscriber('/atlas/imu',Imu,self.imu_cb)
		self.pub = rospy.Publisher('/imu_test',Float64)
	def imu_cb(self,msg):
		quat = msg.orientation
		(r, p, y) = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
		self.pub.publish(Float64(r))
IT = imu_test()
rospy.spin()

