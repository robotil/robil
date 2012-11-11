#!/usr/bin/env python

import roslib; roslib.load_manifest('c35_monitoring')
import rospy, math, ros

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





def armControl():
    
    #albow x
    r_arm_elx = rospy.Publisher('/r_arm_elx_position_controller/command', Float64)
    #albow y
    r_arm_ely = rospy.Publisher('/r_arm_ely_position_controller/command', Float64)
    #wrist- SHORES CAF HAYAD
    r_arm_mwx = rospy.Publisher('/r_arm_mwx_position_controller/command', Float64)
    #shoulder
    r_arm_shx = rospy.Publisher('/r_arm_shx_position_controller/command', Float64)
    #shoulder - y (like moment around z axis)
    r_arm_usy = rospy.Publisher('/r_arm_usy_position_controller/command', Float64)
    #upper limb, between the elbow and the wrist - Rotation
    r_arm_uwy = rospy.Publisher('/r_arm_uwy_position_controller/command', Float64)

    # Initialize the node
    rospy.init_node('arm_control')

   
   
    # Initialize position
    mov_arm_x = 0    
    mov_arm_y = 0
    mov_arm_mwx = 0
    mov_arm_shx = 0
    mov_arm_usy = 0
    mov_arm_uwy = 0
    
    
    r_arm_elx.publish(mov_arm_x)
    r_arm_ely.publish(mov_arm_y)
    r_arm_mwx.publish(mov_arm_mwx)
    r_arm_shx.publish(mov_arm_shx)
    r_arm_usy.publish(mov_arm_usy)
    r_arm_uwy.publish(mov_arm_uwy)
    
    # Sleep for 1 second to wait for the home position
    rospy.sleep(1)
    print '##Getting Started:##'
    print '#use control q and a to move the arm, x axis'
    print '#use control w and s to move the arm, y axis'
    print '#use control e and d to move the wrist '
    print '#use control r and f to move the shoulder '
    print '#use control t and g to rotate the shoulder '
    print '#use control y and h to rotate the upper limb '
    #get char from user
    getch = _Getch()
    
    #This while loop will continue until ROS tells it to shutdown
    while not rospy.is_shutdown():

	mode = getch()
	#move arm x
	if mode=='q':
	  mov_arm_x=mov_arm_x+0.1
	elif mode=='a':
	  mov_arm_x=mov_arm_x-0.1
	#move arm y 
	elif mode=='w':
	  mov_arm_y=mov_arm_y+0.1
	elif mode=='s':
	  mov_arm_y=mov_arm_y-0.1
	#move mwx - SHORESH CAF HAYAD  
	elif mode=='e':
	  mov_arm_mwx=mov_arm_mwx+0.1
	elif mode=='d':
	  mov_arm_mwx=mov_arm_mwx-0.1
	#move sholder x  
	elif mode=='r':
	  mov_arm_shx=mov_arm_shx+0.1
	elif mode=='f':
	  mov_arm_shx=mov_arm_shx-0.1
	#move sholder  
	elif mode=='t':
	  mov_arm_usy=mov_arm_usy+0.1
	elif mode=='g':
	  mov_arm_usy=mov_arm_usy-0.1
	#move upper limb Rotation
	elif mode=='y':
	  mov_arm_uwy=mov_arm_uwy+0.1
	elif mode=='h':
	  mov_arm_uwy=mov_arm_uwy-0.1
	elif mode=='\x03':
	  print "\n"
	  break;
	else:
	  print "Not a legal char"
	
        r_arm_elx.publish(mov_arm_x)
        r_arm_ely.publish(mov_arm_y)
        r_arm_mwx.publish(mov_arm_mwx)
        r_arm_shx.publish(mov_arm_shx)
        r_arm_usy.publish(mov_arm_usy)
        r_arm_uwy.publish(mov_arm_uwy)
      
        # Wait 0.01 second
        rospy.sleep(0.01)


if __name__ == '__main__':
    try:
        armControl()
    except rospy.ROSInterruptException: pass
