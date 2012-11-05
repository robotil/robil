#!/usr/bin/env python

import roslib; roslib.load_manifest('joint_control_tutorial')
import rospy, math

from std_msgs.msg import Float64

def jointStateCommand():
    # Setup the publishers for each joint
    r_arm_elx = rospy.Publisher('/r_arm_elx_position_controller/command', Float64)
    r_arm_ely = rospy.Publisher('/r_arm_ely_position_controller/command', Float64)

    # Initialize the node
    rospy.init_node('joint_control')

    #Initial pose
    r_arm_elx.publish(-1.17)
    r_arm_ely.publish(0.4)

    # Sleep for 1 second to wait for the home position
    rospy.sleep(1)

    #This while loop will continue until ROS tells it to shutdown
    while not rospy.is_shutdown():
        t = 6 * rospy.get_time()
        elbow_x = -1.57 + 0.4 * math.cos(t)
        elbow_y = 0.4 + 0.4 * math.sin(t)

        r_arm_elx.publish(elbow_x)
        r_arm_ely.publish(elbow_y)

        # Wait 0.01 second
        rospy.sleep(0.01)


if __name__ == '__main__':
    try:
        jointStateCommand()
    except rospy.ROSInterruptException: pass
