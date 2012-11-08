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

def leg_command():
    # Setup the publishers for each joint
    #move the knee
    l_leg_kny = rospy.Publisher('/l_leg_kny_position_controller/command', Float64)
    #left paw - left and right
    l_leg_lax = rospy.Publisher('/l_leg_lax_position_controller/command', Float64)
    #move the leg forward and backward
    l_leg_lhy = rospy.Publisher('/l_leg_lhy_position_controller/command', Float64)
    #move the leg left and right
    l_leg_mhx = rospy.Publisher('/l_leg_mhx_position_controller/command', Float64)
    #move leg paw forward and backward
    l_leg_uay = rospy.Publisher('/l_leg_uay_position_controller/command', Float64)
    #turn leg - open the crotch
    l_leg_uhz = rospy.Publisher('/l_leg_uhz_position_controller/command', Float64)  

    # Initialize the node
    rospy.init_node('leg_joint_control')

    #Initial pose
    l_leg_kny.publish(0)
    l_leg_lax.publish(0)
    l_leg_lhy.publish(0)
    l_leg_mhx.publish(0)
    l_leg_uay.publish(0)
    l_leg_uhz.publish(0)

    
    l_leg_kny_pos=0
    l_leg_lax_pos=0
    l_leg_lhy_pos=0
    l_leg_mhx_pos=0
    l_leg_uay_pos=0
    l_leg_uhz_pos=0

    

    # Sleep for 1 second to wait for the home position
    #This while loop will continue until ROS tells it to shutdown
    while not rospy.is_shutdown():
      rospy.sleep(1)
      print '##Getting Started:##'
      print '#use control q and a to move the knee '
      print '#use control w and s to move the paw (left/right) '
      print '#use control e and d to move the leg (forward/backward) '
      print '#use control r and f to move the leg (left/right) '
      print '#use control t and g to move the paw (forward/backward) '
      print '#use control y and h to move the turn the leg (open and close the crotch) '
      print 'Press 1(one) for exit '
      inkey = _Getch()
      import sys
      for i in xrange(sys.maxint):
	k=inkey()
	if k=='q':
		  l_leg_kny_pos = l_leg_kny_pos + 0.05
	if k=='a':
		  l_leg_kny_pos = l_leg_kny_pos - 0.05
	if k=='w':
		  l_leg_lax_pos = l_leg_lax_pos + 0.05
	if k=='s':
		  l_leg_lax_pos = l_leg_lax_pos - 0.05
	if k=='e':
		  l_leg_lhy_pos = l_leg_lhy_pos + 0.05
	if k=='d':
		  l_leg_lhy_pos = l_leg_lhy_pos - 0.05
	if k=='r':
		  l_leg_mhx_pos = l_leg_mhx_pos + 0.05
	if k=='f':
		  l_leg_mhx_pos = l_leg_mhx_pos - 0.05
	if k=='t':
		  l_leg_uay_pos = l_leg_uay_pos + 0.05
	if k=='g':
		  l_leg_uay_pos = l_leg_uay_pos - 0.05
	if k=='y':
		  l_leg_uhz_pos = l_leg_uhz_pos + 0.05
	if k=='h':
		  l_leg_uhz_pos = l_leg_uhz_pos - 0.05
	# ends the script
	if k=='1':
		  break

	#publishers
	l_leg_kny.publish(l_leg_kny_pos)
	l_leg_lax.publish(l_leg_lax_pos)
	l_leg_lhy.publish(l_leg_lhy_pos)
	l_leg_mhx.publish(l_leg_mhx_pos)
	l_leg_uay.publish(l_leg_uay_pos)
	l_leg_uhz.publish(l_leg_uhz_pos)
      break
     
    #print 'Bye-Bye'
	
	



if __name__ == '__main__': # a little test
    try:
        leg_command()
    except rospy.ROSInterruptException: pass