#!/usr/bin/env python

###################################################################################
####                                                                             ##
####  zmp_main.py (based on zmp_main(old).py)                                    ##
####  Created - Israel 25/12/2012                                                ##
####  last updated - version 2.0, Yuval 04/2/2013                                ##
####                                                                             ##
####    run this script to initiate zmp node                                     ##
####    to start walking: rostopic pub /zmp_walk_command std_msgs/Int32 1        ##
####    to stop walking:  rostopic pub /zmp_walk_command std_msgs/Int32 0        ##
####                                                                             ##
####    this node runs with parameters generated by matlab and are stored        ##
####    in the file '/src/parameters'.                                           ##
####    the walking patern in governed by the parameters:                        ##
####    step_length,step_width and step_time                                     ##
####                                                                             ##
###################################################################################     

import roslib; roslib.load_manifest('C42_ZMPWalk')
import rospy, sys #,os.path
from pylab import *
# from numpy import * # no need after line above
from C42_ZMPWalk.msg import walking_trajectory, Position, Orientation, Pos_and_Ori  #traj
from std_msgs.msg import Int32, Float64
from preview_controller import ZMP_Preview_Controller
from swing_trajectory import *
from zmp_profiles import *
from preview_buffer import ZMP_Preview_Buffer
import tf
import copy
from robot_state import Robot_State
from StepStateMachine import *
from ZmpLocalPathPlanner import *
from sensor_msgs.msg import Imu #Odometry

class namespace: pass
ns = namespace()
ns.walk = 0

def listn_to_command(zmp_walk_command): 
    ns.walk = zmp_walk_command.data
    rospy.loginfo("recieved zmp_walk_command: walk = %i" % (ns.walk) )
    rospy.loginfo("time:")
    rospy.loginfo(rospy.get_time())

def listn_to_orientation_command(orientation_command): 
    ns.Des_Orientation = (orientation_command.data)*math.pi/180.0
    rospy.loginfo("recieved orientation command: orientation = %f" % (ns.Des_Orientation) )
    rospy.loginfo("time:")
    rospy.loginfo(rospy.get_time())

# setup node:
rospy.init_node('zmp_movement_plan')  #('ZMP_node')
rospy.loginfo("started ZMP node")

ns.Des_Orientation = 0#-math.pi/2

pub_zmp = rospy.Publisher('zmp_out', walking_trajectory ) #traj)
sub_command = rospy.Subscriber('zmp_walk_command' , Int32 , listn_to_command)
sub_orientation_command = rospy.Subscriber('orientation_command' , Float64 , listn_to_orientation_command)

rospy.sleep(1)
ns.listener = tf.TransformListener()

# init hip_z_orientation_controller
def get_imu(msg):  #listen to /atlas/imu/pose/pose/orientation
    ns.imu_orientation = msg.orientation
imu_ori_z_sub = rospy.Subscriber('/atlas/imu', Imu, get_imu)  #Odometry, get_imu) 

orientation_correction = Orientation_Control()

# sampling time:
rate = 100  # [Hz]
dt = 1.0/rate # [sec] # sample time (was named time_step)
interval = rospy.Rate(rate)

out = walking_trajectory () #traj() # output message of topic 'zmp_out'

max_step_time = 10.0 #[sec] the longest periodstep_length, of step that we plan to do

# Preview Controllers:

Sagital_x_Preview_Controller = ZMP_Preview_Controller('X_sagital','sagital_x',0.0) # name, parameters_folder_name, initial position of COM
Lateral_y_Preview_Controller = ZMP_Preview_Controller('Y_lateral','lateral_y',0.0) # name, parameters_folder_name, initial position of COM

NL = Lateral_y_Preview_Controller.getBufferSize()

# Preview Buffers:
Preview_Sagital_x = ZMP_Preview_Buffer('Sagital X', NL, 4*max_step_time/dt, 0 ) #name, preview_sample_size, max_step_samples, precede_time_samples
Preview_Lateral_y = ZMP_Preview_Buffer('Lateral Y', NL, 4*max_step_time/dt, 0 ) #name, preview_sample_size, max_step_samples, precede_time_samples

# init preview:step_length,
p_ref_x = zeros(NL)
p_ref_y = zeros(NL)

# initialize swing trajectory object
swing_traj = Swing_Trajectory('swing_foot_trajectory')

# Robot State object:
rs = Robot_State('Robot State')

ZmpStateMachine = StepStateMachine(rs,out,ns.listener,Preview_Sagital_x,Preview_Lateral_y,swing_traj,rate,orientation_correction)

ZmpStateMachine.Initialize()

ZmpLpp = LocalPathPlanner()

# Main Loop
while not rospy.is_shutdown():
    
    if(ns.walk):
        ZmpStateMachine.Start()
    else:
        ZmpStateMachine.Stop()
        
#    if (ZmpLpp.UpdatePosition(x,y)):
#        ZmpStateMachine.Stop()
    p_ref_x,p_ref_y = ZmpStateMachine.UpdatePreview()
    [COMx, COMx_dot, p_pre_con_x] = Sagital_x_Preview_Controller.getCOM_ref( p_ref_x )
    [COMy, COMy_dot, p_pre_con_y] = Lateral_y_Preview_Controller.getCOM_ref( p_ref_y )
    rs.getRobot_State(listener = ns.listener)
    ZmpStateMachine.CalculateFootSwingTrajectory()
    requiredYaw = ZmpLpp.GetTargetYaw()
    out = ZmpStateMachine.GetWalkingTrajectory(COMx, COMx_dot, p_pre_con_x,COMy, COMy_dot, p_pre_con_y,p_ref_x,p_ref_y,requiredYaw,ns.imu_orientation)
    pub_zmp.publish(out)
    
    interval.sleep()

#end while not rospy.is_shutdown()
