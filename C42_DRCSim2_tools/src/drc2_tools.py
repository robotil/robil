#! /usr/bin/env python
import roslib; roslib.load_manifest('C42_DRCSim2_tools')
import math, rospy, os, rosparam
import tf
from atlas_msgs.msg import AtlasState, AtlasCommand
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, Quaternion, Wrench
from numpy import zeros, array, linspace, arange
import numpy as np
from math import ceil
import yaml
from copy import copy

class AtlasCommand_msg_handler(object):
    # A class for composing and sending AtlasCommand messages
    #
    # osrf empty message: 
    #        string[] name  - names of the joints to control (need to contain *all* atlas joints in order to work)
    #
    #     commands (order should be same as name[]):
    #        float64[] position
    #        float64[] velocity
    #        float64[] effort
    #
    #     gains (order should be same as name[]):
    #        float64[] kp_position
    #        float64[] ki_position
    #        float64[] kd_position
    #        float64[] kp_velocity
    #        float64[] i_effort_min
    #        float64[] i_effort_max
    #
    # object of this class stores one Joint command message (self._command)
    # the class implements method that allow easy command sending to individual joints while remembering previous command to the unaffected joints
    # 
    # the default command is:
    # name[28] = atlasJointNames (a list of all joints names in the drc robot)
    # position velocity and effort [28] all set to 0
    # gains set to default values taken from parameter server
    # the default parameter is '/atlas_controller/gains/' which is loaded automaticly with drcsim
    # 
    # reset_command() : resets self._command to its default gains and commands
    #
    # set_default_gains_from_param(param_name='/atlas_controller'): sets the default gains from parameter server, by default the param is /atlas_controller
    #
    # set_default_gains_from_yaml(yaml_path): sets the default gains from a yaml file, note: gains must be defined in /atlas_controller/gains/, use atlas_controller.yaml as template
    #
    # reset_gains(joints = 'all'): resets the gains of joints to default, resets all joints by default
    #
    # set_pos(joint,pos): sets a position command to the joint with default gains, joint may be specified as string (joint name) or number
    #
    # set_eff(joint,eff): sets an effort command to joint, position and velocity PID gains are set to 0, joint may be specified as string (joint name) or number
    #
    # set_gains(joint,p,i,d): sets PID gains of a joint, joint may be specified as string (joint name) or number
    #
    # send_command(): publishes the command to the controller
    #
    # print_command(): prints the command (useful for debugging)

    atlasJointNames = [
      'atlas::back_lbz', 'atlas::back_mby', 'atlas::back_ubx', 'atlas::neck_ay',
      'atlas::l_leg_uhz', 'atlas::l_leg_mhx', 'atlas::l_leg_lhy', 'atlas::l_leg_kny', 'atlas::l_leg_uay', 'atlas::l_leg_lax',
      'atlas::r_leg_uhz', 'atlas::r_leg_mhx', 'atlas::r_leg_lhy', 'atlas::r_leg_kny', 'atlas::r_leg_uay', 'atlas::r_leg_lax',
      'atlas::l_arm_usy', 'atlas::l_arm_shx', 'atlas::l_arm_ely', 'atlas::l_arm_elx', 'atlas::l_arm_uwy', 'atlas::l_arm_mwx',
      'atlas::r_arm_usy', 'atlas::r_arm_shx', 'atlas::r_arm_ely', 'atlas::r_arm_elx', 'atlas::r_arm_uwy', 'atlas::r_arm_mwx']


    def __init__(self):

        self.Joint_dict = {AtlasCommand_msg_handler.atlasJointNames[i]:i for i in xrange(len(AtlasCommand_msg_handler.atlasJointNames))}
        self.inv_dict = dict(zip(self.Joint_dict.values(), self.Joint_dict.keys()))

        if rospy.get_name() == '/unnamed':
            rospy.init_node('AtlasCommand_msg_handler',anonymous=True)
        self._ac_pub = rospy.Publisher('atlas/atlas_command', AtlasCommand)
        self._command = AtlasCommand()
        self._joint_names = list(self.atlasJointNames)
        self._command.k_effort = [255] * len(self._joint_names)
        self.set_default_gains_from_param()
        self.reset_command()
        

    def set_default_gains_from_yaml(self,yaml_path):
        yaml_file = file(yaml_path)
        self._default_gains = yaml.load(yaml_file)
        self._default_gains = self._default_gains['atlas_controller']['gains']

    def set_default_gains_from_param(self,param_name = '/atlas_controller'):
        self._default_gains = rospy.get_param(param_name)
        self._default_gains = self._default_gains['gains']

    def reset_command(self):
        n = len(self._joint_names)
        self._command.position     = zeros(n)
        self._command.velocity     = zeros(n)
        self._command.effort       = zeros(n)
        self._command.kp_position  = zeros(n)
        self._command.ki_position  = zeros(n)
        self._command.kd_position  = zeros(n)
        self._command.kp_velocity  = zeros(n)
        self._command.i_effort_min = zeros(n)
        self._command.i_effort_max = zeros(n)
        self.reset_gains()

    def reset_gains(self,joints = 'ALL'):
        if joints == 'ALL':
            n = len(self._joint_names)
            lst = range(n)
        else:
            lst = list([joints])
        for i in lst:
          name = self._joint_names[i]
          self._command.kp_position[i]  = self._default_gains[name[7::]]['p']
          self._command.kd_position[i]  = self._default_gains[name[7::]]['d']
          self._command.ki_position[i]  = self._default_gains[name[7::]]['i']
          self._command.i_effort_max[i] = self._default_gains[name[7::]]['i_clamp']
          self._command.i_effort_min[i] = -self._command.i_effort_max[i]

    def set_pos(self,joint, pos):
        if type(joint) == int:
            joint_num = int(joint)
        elif type(joint) == str:
            joint_num = self.Joint_dict['atlas::'+joint]
        else:
            raise TypeError
            return
        name = self._joint_names[joint_num]
        self._command.position[joint_num] = float(pos)
        # self.reset_gains(joint_num)

    def set_vel(self,joint, vel):
        if type(joint) == int:
            joint_num = int(joint)
        elif type(joint) == str:
            joint_num = self.Joint_dict['atlas::'+joint]
        else:
            raise TypeError
            return
        name = self._joint_names[joint_num]
        self._command.velocity[joint_num] = float(vel)

    def set_eff(self,joint,eff,null_gains = True):
        if type(joint) == int:
            joint_num = int(joint)
        elif type(joint) == str:
            joint_num = self.Joint_dict['atlas::'+joint]
        else:
            raise TypeError
            return
        self._command.effort[joint_num] = float(eff)
        if null_gains:
            self.set_gains(joint_num,0.0,0.0,0.0,0.0,set_default = False)

    def set_mixed(self,joint,eff,pos):
        if type(joint) == int:
            joint_num = int(joint)
        elif type(joint) == str:
            joint_num = self.Joint_dict['atlas::'+joint]
        else:
            raise TypeError
            return
        self._command.effort[joint_num] = float(eff)
        self._command.position[joint_num] = float(pos)

    def set_gains(self,joint,p,i,d,i_clamp,set_default = True):
        if type(joint) == int:
            joint_num = joint
            joint_name = self.inv_dict[joint_num][7::]
        elif type(joint) == str:
            joint_name = joint
            joint_num = self.Joint_dict['atlas::'+joint]
        else:
            raise TypeError
            return
        #update defaults
        if set_default:
            self._default_gains[joint_name]['p'] = p
            self._default_gains[joint_name]['i'] = i
            self._default_gains[joint_name]['d'] = d
            self._default_gains[joint_name]['i_clamp'] = d
        #
        self._command.kp_position[joint_num]  = p
        self._command.ki_position[joint_num]  = i
        self._command.kd_position[joint_num]  = d
        self._command.i_effort_max[joint_num] = i_clamp
        self._command.i_effort_min[joint_num] = -i_clamp

    def set_command(self,joint,p,i,d,i_clamp,pos,eff,vel,default = False):
        self.set_mixed(joint,eff,pos)
        self.set_gains(joint,p,i,d,i_clamp,set_default = default)

    def print_command(self):
        for k in xrange(len(self._joint_names)):
            print 'name: ', self._joint_names[k]
            print 'command: pos:', self._command.position[k], 'eff:', self._command.effort[k]
            print 'p: ', self._command.kp_position[k]
            print 'i: ', self._command.ki_position[k]
            print 'd: ', self._command.kd_position[k]

    def send_command(self):
        self._command.header.stamp = rospy.Time.now()
        self._ac_pub.publish(self._command)

    def get_command(self):
        com = copy(self._command)
        return com

    def set_all_pos(self,pos_vec):
        if len(pos_vec) == len(self._joint_names):
            self._command.position = list(pos_vec)
        else:
            print 'position command legth doest fit'

    def send_pos_traj(self,pos1,pos2,T,dt):
        if len(pos1) == len(pos2) == len(self._joint_names):
            N = ceil(T/dt)
            pos1 = array(pos1)
            pos2 = array(pos2)
            for ratio in linspace(0, 1, N):
              interpCommand = (1-ratio)*pos1 + ratio * pos2
              self._command.position = [ float(x) for x in interpCommand ]
              self.send_command()
              rospy.sleep(dt)
        else:
            print 'position command legth doest fit'
#####################################################
#####################################################33
####################################################33

class robot_listner(object):
    def __init__(self, enable_tf_Listener = False):
        if rospy.get_name() == '/unnamed':
            rospy.init_node('Joint_state_listner',anonymous=True)

        self.joint_state = JointState()
        self.odometry = Odometry()
        sub = rospy.Subscriber('/atlas/joint_states',JointState,self._update_state)
        subOd = rospy.Subscriber('/ground_truth_odom',Odometry,self._update_odom)
        
        self.enable_tf_Listener = enable_tf_Listener
        if self.enable_tf_Listener:
            self.tfListen = tf.TransformListener()        

    def _update_state(self,msg):
        self.joint_state = msg 

    def _update_odom(self,msg):
        self.odometry = msg 

    def current_pos(self):
        pos = copy(self.joint_state.position)
        return pos
        
    def current_vel(self):
        vel = copy(self.joint_state.velocity)
        return vel
        
    def current_odom_pos(self):
        pos = copy(self.odometry.pose.pose.position)
        return pos

    def current_odom_vel(self):
        vel = copy(self.odometry.twist.twist.linear)
        return vel
        
    def current_ypr(self):
		quat = copy(self.odometry.pose.pose.orientation)
		
		(r, p, y) = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
		return (y, p, r)
		
    def current_ypr_vel(self):
		ang_vel = copy(self.odometry.twist.twist.angular)
		
		(r, p, y) = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
		return (y, p, r)
		
    def get_transform(self,Frame1,Frame2,Time = rospy.Time(0)):
        if self.enable_tf_Listener:
            try:
                #self.tfListen.waitForTransform(Frame1, Frame2, Time, rospy.Duration(1), rospy.Duration(0.01))
                (trans,rot) = self.tfListen.lookupTransform(Frame1,Frame2,Time)
                return (trans, rot)
            except tf.Exception as ex:
                print ex.args
                return
        else:
            (trans,rot) = ([0,0,0], [0,0,0])   
            return (trans, rot)




