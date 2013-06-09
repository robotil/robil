#! /usr/bin/env python
# import roslib; roslib.load_manifest('DynamicWalk')
import roslib; roslib.load_manifest('C42_Leg_IK')
import rospy
import copy
from atlas_msgs.msg import AtlasState
from std_msgs.msg import Int32, Float64

def none(iterable):
    for k in iterable:
        if k:
            return False
    return True

class contact_reflex():
    def __init__(self,buffer_len = 20):
        self._treshold = 100;#100
        self._state = {'l':1,'r':1}
        self._states_buffer = {'l':[True for k in xrange(buffer_len)],'r':[True for k in xrange(buffer_len)]}

    def update(self,r_foot_z,l_foot_z):
        l_state = (l_foot_z > self._treshold)
        r_state = (r_foot_z > self._treshold)
        for leg in self._state:
            if leg == 'l':
                state = l_state
            else:
                if leg == 'r':
                    state = r_state

            buf = self._states_buffer[leg]
            buf.insert(0,state)
            buf.pop()
            if all(buf):
                self._state[leg] = 1
            else:
                if none(buf):
                    self._state[leg] = 0
        return self._state['r'],self._state['l']  



if __name__ == '__main__':
    rospy.init_node('test')
    class test(object):
        def __init__(self):
            self.sub = rospy.Subscriber('/atlas/atlas_state',AtlasState,self.on_update)
            self.pub = rospy.Publisher('/contact_reflex',Int32)
            self.reflex = contact_reflex()
        def on_update(self,msg):
            # print 1
            r,l = self.reflex.update(msg.r_foot.force.z,msg.l_foot.force.z)
            self.pub.publish(Int32(r))

    testinger = test()
    rospy.spin()
