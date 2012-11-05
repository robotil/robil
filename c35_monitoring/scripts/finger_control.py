#!/usr/bin/env python

import roslib; roslib.load_manifest('c35_monitoring')
import rospy, math



from std_msgs.msg import Float64

class _Getch:
    """Gets a single character from standard input.  Does not echo to the
screen."""
    def __init__(self):
        self.impl = _GetchUnix()
    def __call__(self): return self.impl()


class _GetchUnix:
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class _GetchWindows:
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()


getch = _Getch()

def finger_command():
    # Setup the publishers for each joint
    r_arm_elx = rospy.Publisher('/r_arm_elx_position_controller/command', Float64)
    r_arm_ely = rospy.Publisher('/r_arm_ely_position_controller/command', Float64)

    # Initialize the node
    rospy.init_node('joint_control')

    #Initial pose
    r_arm_elx.publish(-1.17)
    r_arm_ely.publish(0.4)
    elbow_x = -1.17
    elbow_y = 0.4
    # Sleep for 1 second to wait for the home position
    rospy.sleep(1)
    print 'Press a key'
    inkey = _Getch()
    import sys
    for i in xrange(sys.maxint):
      k=inkey()
      if k<'a' or k>'z':break
      if k=='q':elbow_x = elbow_x + 0.1
      if k=='a':elbow_x = elbow_x - 0.1
      if k=='w':elbow_y = elbow_y + 0.1
      if k=='s':elbow_y = elbow_y - 0.1
      print 'you pressed ',k
      r_arm_elx.publish(elbow_x)
      r_arm_ely.publish(elbow_y)

    #This while loop will continue until ROS tells it to shutdown
#    while not rospy.is_shutdown():
#        t = 6 * rospy.get_time()
#        elbow_x = -1.57 + 0.4 * math.cos(t)
#        elbow_y = 0.4 + 0.4 * math.sin(t)

#        r_arm_elx.publish(elbow_x)
#        r_arm_ely.publish(elbow_y)

        # Wait 0.01 second
        #rospy.sleep(0.01)



if __name__ == '__main__': # a little test
    try:
        finger_command()
    except rospy.ROSInterruptException: pass
