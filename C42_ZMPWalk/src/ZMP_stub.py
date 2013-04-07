#! /usr/bin/env python
import roslib; roslib.load_manifest('C42_ZMPWalk')
import rospy
from ZmpLocalPathPlanner import *
from RobilTaskPy import *
from threading import Lock  
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
from C31_PathPlanner.msg import C31_Waypoints
from std_msgs.msg import String
import math;
from zmp_stub_pose import stub_pose

class MyTask(RobilTask):
    def __init__(self, name):
        print "Init FollowPath"
        RobilTask.__init__(self, name)
        stub_pose()
        self._PP = LocalPathPlanner()
        self._odom_sub = rospy.Subscriber('/ground_truth_odom',Odometry,self._odom_cb)
        self._path_sub = rospy.Subscriber('/C31PathPlanner',C31_Waypoints,self._path_cb)
        self._vel_pub = rospy.Publisher('/atlas/cmd_vel',Twist)
        self._mode_pub = rospy.Publisher('/atlas/mode',String)
        self._done = False
        self._Walk = False    
        self._lock = Lock() 
        
    def _odom_cb(self,odom):
        self._done = self._PP.UpdatePosition(odom.pose.pose.position.x,odom.pose.pose.position.y)
        seg = self._PP.GetCurrentSegment()
        des = seg.GetTarget()
        if self._Walk:
            Vel = Twist()
            lin = self._get_vel()
            Vel.linear = lin
            self._vel_pub.publish(Vel)
        
    def _path_cb(self,path):
        rospy.loginfo('got path %s',path)
        p = []
        for wp in path.points:
            p.append(Waypoint(wp.x,wp.y))
        self._PP.SetPath(p)
        
    def _get_vel(self):
        vel = 1.0 #0.5
        segment = self._PP.GetCurrentSegment()
        a = segment.GetSource()
        b = segment.GetTarget()
        u = [b.GetX()-a.GetX(),b.GetY()-a.GetY()]
        norm_u = math.sqrt(u[0]**2+u[1]**2)
        if norm_u == 0:
            V = Vector3(0,0,0)
        else:
            V = Vector3(vel*u[0]/norm_u,vel*u[1]/norm_u,0)
        rospy.loginfo('Vel = %s done: %s',V,self._done)
        return V
        
    def task(self, name, uid, parameters):
        print "Start FollowPath"
        r = rospy.Rate(0.1)
        self._Walk = True
        Vel = Twist()
        while not self._done:
            if self.isPreepted():
                print "Preempt FollowPath"
                self._Walk = False
                Vel.linear = Vector3(0,0,0)
                self._vel_pub.publish(Vel)
                return RTResult_PREEPTED()
            r.sleep()
        self._Walk = False
        Vel.linear = Vector3(0,0,0)
        self._vel_pub.publish(Vel)
        self._mode_pub.publish(String('nominal'))
        stub_pose()
        print "Finish FollowPath"
        return RTResult_SUCCESSED("Finished in Success")

if __name__ == '__main__':
    rospy.init_node('FollowPathNode')
    MyTask("FollowPath")
    rospy.spin()
    print "Node Closed"
