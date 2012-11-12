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



getch = _Getch()

def finger_command():
    # Setup the publishers for each joint
    r_f0_j0 = rospy.Publisher('/r_f0_j0_position_controller/command', Float64)
    r_f0_j1 = rospy.Publisher('/r_f0_j1_position_controller/command', Float64)
    r_f0_j2 = rospy.Publisher('/r_f0_j2_position_controller/command', Float64)
    r_f1_j0 = rospy.Publisher('/r_f1_j0_position_controller/command', Float64)
    r_f1_j1 = rospy.Publisher('/r_f1_j1_position_controller/command', Float64)
    r_f1_j2 = rospy.Publisher('/r_f1_j2_position_controller/command', Float64)
    r_f2_j0 = rospy.Publisher('/r_f2_j0_position_controller/command', Float64)
    r_f2_j1 = rospy.Publisher('/r_f2_j1_position_controller/command', Float64)
    r_f2_j2 = rospy.Publisher('/r_f2_j2_position_controller/command', Float64)
    r_f3_j0 = rospy.Publisher('/r_f3_j0_position_controller/command', Float64)
    r_f3_j1 = rospy.Publisher('/r_f3_j1_position_controller/command', Float64)
    r_f3_j2 = rospy.Publisher('/r_f3_j2_position_controller/command', Float64)
    

    # Initialize the node
    rospy.init_node('joint_control')

    #Initial pose
    r_f0_j0.publish(-0.25)
    r_f0_j1.publish(0.08)
    r_f0_j2.publish(0)
    r_f1_j0.publish(-0.24)
    r_f1_j1.publish(0.08)
    r_f1_j2.publish(0)
    r_f2_j0.publish(-0.22)
    r_f2_j1.publish(0.08)
    r_f2_j2.publish(0)
    r_f3_j0.publish(0.64)
    r_f3_j1.publish(0.08)
    r_f3_j2.publish(0)
    
    r_f0_j0_pos=-0.25
    r_f0_j1_pos=0.08
    r_f0_j2_pos=0
    r_f1_j0_pos=-0.24
    r_f1_j1_pos=0.08
    r_f1_j2_pos=0
    r_f2_j0_pos=-0.22
    r_f2_j1_pos=0.08
    r_f2_j2_pos=0
    r_f3_j0_pos=0.64
    r_f3_j1_pos=0.08
    r_f3_j2_pos=0
    

    # Sleep for 1 second to wait for the home position
    rospy.sleep(1)
    print 'Press a key'
    inkey = _Getch()
    import sys
    for i in xrange(sys.maxint):
      k=inkey()
      if k<'a' or k>'z':break
      if k=='q':
		#r_f0_j0_pos = r_f0_j0_pos + 0.02
		#r_f1_j0_pos = r_f1_j0_pos + 0.02
		#r_f2_j0_pos = r_f2_j0_pos + 0.02
		r_f3_j0_pos = r_f3_j0_pos + 0.02
      if k=='a':
		#r_f0_j0_pos = r_f0_j0_pos - 0.02
		#r_f1_j0_pos = r_f1_j0_pos - 0.02
		#r_f2_j0_pos = r_f2_j0_pos - 0.02
		r_f3_j0_pos = r_f3_j0_pos - 0.02
      if k=='w':
		r_f0_j1_pos = r_f0_j1_pos + 0.02
		r_f1_j1_pos = r_f1_j1_pos + 0.02
		r_f2_j1_pos = r_f2_j1_pos + 0.02
		r_f3_j1_pos = r_f3_j1_pos + 0.02
      if k=='s':
		r_f0_j1_pos = r_f0_j1_pos - 0.02
		r_f1_j1_pos = r_f1_j1_pos - 0.02
		r_f2_j1_pos = r_f2_j1_pos - 0.02
		r_f3_j1_pos = r_f3_j1_pos - 0.02
      if k=='e':
		r_f0_j2_pos = r_f0_j2_pos + 0.02
		r_f1_j2_pos = r_f1_j2_pos + 0.02
		r_f2_j2_pos = r_f2_j2_pos + 0.02
		r_f3_j2_pos = r_f3_j2_pos + 0.02
      if k=='d':
		r_f0_j2_pos = r_f0_j2_pos - 0.02
		r_f1_j2_pos = r_f1_j2_pos - 0.02
		r_f2_j2_pos = r_f2_j2_pos - 0.02
		r_f3_j2_pos = r_f3_j2_pos - 0.02
      
      
      r_f0_j0.publish(r_f0_j0_pos)
      r_f0_j1.publish(r_f0_j1_pos)
      r_f0_j2.publish(r_f0_j2_pos)
      r_f1_j0.publish(r_f1_j0_pos)
      r_f1_j1.publish(r_f1_j1_pos)
      r_f1_j2.publish(r_f1_j2_pos)
      r_f2_j0.publish(r_f2_j0_pos)
      r_f2_j1.publish(r_f2_j1_pos)
      r_f2_j2.publish(r_f2_j2_pos)
      r_f3_j0.publish(r_f3_j0_pos)
      r_f3_j1.publish(r_f3_j1_pos)
      r_f3_j2.publish(r_f3_j2_pos)

      print 'Positions: r_f0_j1=%f,r_f0_j2=%f' % (r_f0_j1_pos,r_f0_j2_pos)

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
