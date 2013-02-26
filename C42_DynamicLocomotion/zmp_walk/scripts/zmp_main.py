#!/usr/bin/env python

###################################################################################
####	                                                                           ##
####	zmp_main.py (based on zmp_main(old).py)                                    ##
####	Created - Israel 25/12/2012 	                                             ##
####  last updated - version 2.0, Yuval 04/2/2013                                ##
####                                                                             ##
####    run this script to initiate zmp node           	                         ##
####    to start walking: rostopic pub /zmp_walk_command std_msgs/Int32 1        ##
####    to stop walking:  rostopic pub /zmp_walk_command std_msgs/Int32 0	       ##
####                                         	                                   ##
####    this node runs with parameters generated by matlab and are stored	       ##
####    in the file '/src/parameters'.	                                         ##
####    the walking patern in governed by the parameters: 			                 ##
####    step_length,step_width and step_time		                                 ##
####	                                                                  	       ##
###################################################################################			

import roslib; roslib.load_manifest('zmp_walk') #roslib.load_manifest('leg_ik')
import rospy, sys #,os.path
from pylab import *
# from numpy import * # no need after line above
from zmp_walk.msg import walking_trajectory, Position, Orientation, Pos_and_Ori  #traj
from std_msgs.msg import Int32
from preview_controller import ZMP_Preview_Controller
from zmp_profiles import *
from preview_buffer import ZMP_Preview_Buffer
import tf
import copy
from robot_state import Robot_State

class namespace: pass
ns = namespace()
ns.walk = 0

def listn_to_command(zmp_walk_command): 
    ns.walk = zmp_walk_command.data
    rospy.loginfo("recieved zmp_walk_command: walk = %i" % (ns.walk) )
    rospy.loginfo("time:")
    rospy.loginfo(rospy.get_time())

# setup node:
rospy.init_node('zmp_movement_plan')  #('ZMP_node')
rospy.loginfo("started ZMP node")

pub_zmp = rospy.Publisher('zmp_out', walking_trajectory ) #traj)
sub_command = rospy.Subscriber('zmp_walk_command' , Int32 , listn_to_command)
ns.listener = tf.TransformListener()

# sampling time:
rate = 100  # [Hz]
dt = 1.0/rate # [sec] # sample time (was named time_step)
interval = rospy.Rate(rate)

out = walking_trajectory () #traj() # output message of topic 'zmp_out'

#zc = 0#0.8455 # [m] COM height

# Walking Parameters 
step_length = 0.03 #0.01  # [m]
step_width  = 0.165  # 0.178  # [m]
step_time   = 1 #1   # [sec]
bend_knees  = 0.04  # [m]    
step_height = 0.05 #0.03 #0.05  # [m] 
trans_ratio_of_step = 1.0 #0.8 # units fraction: 0-1.0 ; fraction of step time to be used for transition. 1.0 = all of step time is transition 
trans_slope_steepens_factor = 8/step_time #2 # 1 transition Sigmoid slope (a)
# Robot State object:
rs = Robot_State('Robot State')
pelvis_des = rs.place_in_Ori( 0, 0, 0 ) # pelvis desired rotation (0,0,0)<=>stand up straight

max_step_time = 10.0 #[sec] the longest period of step that we plan to do

# TODO: make sure robot is static (in start position) and tf is running
# init Robot State using tf data getting hip to com relative position:
rs.static_init_with_tf_data(ns.listener,bend_knees)

# Preview Controllers:
Sagital_x_Preview_Controller = ZMP_Preview_Controller('X_sagital','sagital_x',0.0) # name, parameters_folder_name, initial position of COM 
Lateral_y_Preview_Controller = ZMP_Preview_Controller('Y_lateral','lateral_y',0.0) # name, parameters_folder_name, initial position of COM

NL = Lateral_y_Preview_Controller.getBufferSize() 

# Preview Buffers:
Preview_Sagital_x = ZMP_Preview_Buffer('Sagital X', NL, 4*max_step_time/dt, 0 ) #name, preview_sample_size, max_step_samples, precede_time_samples
Preview_Lateral_y = ZMP_Preview_Buffer('Lateral Y', NL, 4*max_step_time/dt, 0 ) #name, preview_sample_size, max_step_samples, precede_time_samples

# init preview:
p_ref_x = zeros(NL)
p_ref_y = zeros(NL)

# init output message (before starting to walk)
out.stance_hip = copy.deepcopy( rs.Get_l_stance_hip_0() )
out.pelvis_d = pelvis_des
out.swing_foot = rs.swing_foot
out.swing_hip = rs.place_in_Pos( out.stance_hip.x, rs.swing_hip.y, out.stance_hip.z) # constraint: hips at same hight and advance together
out.pelvis_m = rs.pelvis_m

out.zmp_ref = rs.place_in_Pos(0,0,0)
out.zmp_pc = rs.place_in_Pos(0,0,0)
out.com_ref = rs.place_in_Pos(0,0,0)
out.com_dot_ref = rs.place_in_Pos(0,0,0)
out.com_m = rs.com_m

out.step_phase = rs.Get_step_phase()
# for debug use
out.stance_hip_m = rs.stance_hip
out.swing_hip_m = rs.swing_hip
out.swing_foot_m = rs.swing_foot

# rospy.loginfo("zmp_main, stance foot: hip_x = %f, hip_y = %f, hip_z = %f, swing l-foot: foot_x = %f, foot_y = %f, foot_z = %f" \
#                % (l_stance_hip_0.x, l_stance_hip_0.y, l_stance_hip_0.z, rs_from_r_foot.swing_foot.x, rs_from_r_foot.swing_foot.y, rs_from_r_foot.swing_foot.z) )
# Wait 1 second
# rospy.sleep(1)


# Main Loop

go = 0

while not rospy.is_shutdown():
  
  pub_zmp.publish(out) 
  interval.sleep()

  if ns.walk == 1:
      
      rospy.loginfo("started walking")
      rospy.loginfo("time:")
      rospy.loginfo(rospy.get_time())
      D     = 0.0

      k     = 1
      samples_in_step = ceil (step_time / dt)

      step_length_z = step_length;
      step_time_z = step_time;
      pre_step = 1
      first_step = 0
      full_step = 0
      #last_step_mes = 0
      go = 1
      steps_count = 0 #-1
      last_step = 0
      step_done = 0
      swing_x_v = []
      swing_z_v = []

      swing_z_t_more_double_support = 0
      swing_x_t_more_double_support = 0

      distance_x_ref = 0 # the accumelated ZMP ref distance past, is updated at the end of each step 
                         # and used as initial starting point for the next step
      
      # create ZMP_ref profiles: 
      # 1) to start walking (pre_step + first_step)
      p_ref_x_start = Start_sagital_x(0, step_length, trans_ratio_of_step, trans_slope_steepens_factor, step_time, dt)
      p_ref_y_start = Start_lateral_y_weight_to_left_foot(0, step_width, trans_ratio_of_step, trans_slope_steepens_factor, step_time, dt)
      # 2) preceeding steps (full_step)
      p_ref_x_forward_step = Step_forward_x(0, step_length, trans_ratio_of_step, trans_slope_steepens_factor, step_time, dt)
      p_ref_y_step_right = Step_onto_right_foot(0, step_width, trans_ratio_of_step, trans_slope_steepens_factor, step_time, dt)
      p_ref_y_step_left = Step_onto_left_foot(0, step_width, trans_ratio_of_step, trans_slope_steepens_factor, step_time, dt)

      # Load the starting step to the preview buffer:
      Preview_Sagital_x.load_NewStep( p_ref_x_start, p_ref_x_forward_step )
      Preview_Lateral_y.load_NewStep( p_ref_y_start, r_[ p_ref_y_step_right, p_ref_y_step_left ] ) 

  while go == 1:

  #######################################################
  #                                                     #
  # sagital control   (x)                               #
  #                                                     #
  #######################################################

      p_ref_x = Preview_Sagital_x.update_Preview()
      [COMx, COMx_dot, p_pre_con_x] = Sagital_x_Preview_Controller.getCOM_ref( p_ref_x )


  #######################################################
  #                                                     #
  # lateral control  (y)                                #
  #                                                     #
  #######################################################

      p_ref_y = Preview_Lateral_y.update_Preview()
      [COMy, COMy_dot, p_pre_con_y] = Lateral_y_Preview_Controller.getCOM_ref( p_ref_y )

  #######################################################
  #                                                     #
  # foot trajectory                                     #
  #                                                     #
  #######################################################

      k_total = step_time*100
      k_start_swing =  round(k_total/3)
      k_stop_swing =   round(2*k_total/3)

      if pre_step:
         swing_pre_step = abs(sin(pi/(step_time_z)*(k-1)*(dt)))
         swing_x_t = 0 
         swing_y = 0
         swing_z_t = 0 

      if full_step:  
         #swing_x_t = -step_length_z + 2*(k-1)*dt*step_length_z/step_time_z
         swing_y = 0
         swing_z_t = 0.05*abs(sin(pi/step_time_z*(k-1)*dt))


         if(k>k_start_swing  and k<k_stop_swing ):
              if rs.Get_step_phase() == 1:    # comment  to stay in step phase 1
                  rs.Set_step_phase(2)
              elif rs.Get_step_phase() == 3:
                  rs.Set_step_phase(4)
              swing_x_t_more_double_support = -step_length_z +2*(k-k_start_swing)*step_length/(k_stop_swing - k_start_swing)
              swing_z_t_more_double_support = step_height*abs( (sin(pi/(k_stop_swing -k_start_swing)*(k - k_start_swing)))**2 ) #step_height*abs(sin(pi/(k_stop_swing -k_start_swing)*(k - k_start_swing)))  
         elif k<k_start_swing:
              swing_z_t_more_double_support = 0
              swing_x_t_more_double_support = -step_length
         elif k>k_stop_swing:
              swing_z_t_more_double_support = 0
              swing_x_t_more_double_support = step_length



      if first_step:  

         #swing_x_t = (k-1)*dt*step_length/step_time
         #rospy.loginfo(swing_x_t)
         swing_y = 0#-0.02*(k-1)*dt/step_time 
         swing_z_t = 0.06*abs(sin(pi/step_time*(k-1)*dt))
 
         if(k>k_start_swing  and k<k_stop_swing ):
              if rs.Get_step_phase() == 1:  # comment  to stay in step phase 1
                  rs.Set_step_phase(2)
              elif rs.Get_step_phase() == 3:
                  rs.Set_step_phase(4)
              swing_x_t_more_double_support = (k-k_start_swing)*step_length/(k_stop_swing - k_start_swing)
              swing_z_t_more_double_support = step_height*abs( (sin(pi/(k_stop_swing -k_start_swing)*(k - k_start_swing)))**2 )  
 
         elif k<k_start_swing:
              swing_z_t_more_double_support = 0
              swing_x_t_more_double_support = 0 #-step_length
         elif k>k_stop_swing:
              swing_z_t_more_double_support = 0
              swing_x_t_more_double_support = step_length
      if last_step:  
         swing_x_t = -step_length_z + (k-1)*dt*step_length_z/step_time_z
         swing_y = 0
         swing_z_t = 0.05*abs(sin(pi/step_time_z*(k-1)*dt))

      
      swing_x_v = r_[ swing_x_v , swing_x_t_more_double_support ]
      swing_z_v = r_[ swing_z_v , swing_z_t_more_double_support ]


      swing_x_k = swing_x_t_more_double_support # for use of IK 
      swing_z_k = swing_z_t_more_double_support # for use of IK

  #######################################################
  #                                                     #
  # Update Robot State and publish zmp                  #
  #                                                     #
  #######################################################
      
      # Update Robot State: getting joints locations using tf to process and store them in Robot State object 
      rs.getRobot_State( listener = ns.listener )

      stance_hip_0 = rs.place_in_Pos(0,0,0)
      if ( rs.Get_step_phase() == 1 ) or ( rs.Get_step_phase() == 2 ): 
          # stance = left, swing = right
          stance_hip_0 = copy.deepcopy( rs.Get_l_stance_hip_0() )
          swing_y_0 = -step_width 
      elif ( rs.Get_step_phase() == 3 ) or ( rs.Get_step_phase() == 4 ): 
          # stance = right, swing = left
          stance_hip_0 = copy.deepcopy( rs.Get_r_stance_hip_0() )
          swing_y_0 = step_width

      if ( rs.Get_step_phase() == 2 ) or ( rs.Get_step_phase() == 4 ): # if swing foot is in the air
          out.swing_foot = rs.place_in_Pos_Ori( swing_x_k, swing_y_0, swing_z_k, 0, 0, 0 )
      elif ( rs.Get_step_phase() == 1 ) or ( rs.Get_step_phase() == 3 ): # if swing foot is on the ground
          out.swing_foot = rs.swing_foot  

      out.stance_hip.x = COMx + stance_hip_0.x-D 
      out.stance_hip.y = COMy + stance_hip_0.y
      out.stance_hip.z = stance_hip_0.z

      out.pelvis_d = pelvis_des #place_in_Orientation( 0, 0, 0 )
      
      out.swing_hip = rs.place_in_Pos( out.stance_hip.x, rs.swing_hip.y, out.stance_hip.z) # constraint: hips at same hight and advance together
      out.pelvis_m = rs.pelvis_m

      out.zmp_ref = rs.place_in_Pos( p_ref_x[0], p_ref_y[0], 0)
      out.zmp_pc = rs.place_in_Pos( p_pre_con_x, p_pre_con_y, 0)
      out.com_ref = rs.place_in_Pos( COMx, COMy, 0)
      out.com_dot_ref = rs.place_in_Pos( COMx_dot, COMy_dot, 0)
      out.com_m = rs.com_m

      out.step_phase = rs.Get_step_phase()
      # for debug use
      out.stance_hip_m = rs.stance_hip
      out.swing_hip_m = rs.swing_hip
      out.swing_foot_m = rs.swing_foot

      # rospy.loginfo("zmp_main go=1, stance foot: COMy = %f, out.hip_y = %f, stance_hip_0.y = %f, l_stance_hip_0 = %f " \
      #          % (COMy, out.stance_hip.y, stance_hip_0.y, rs.Get_l_stance_hip_0() ) )

      pub_zmp.publish(out)

  #######################################################
  #                                                     #
  # command control - Steps state machine               #
  #                                                     #
  ####################################################### 
      
      # pre_step:  
          
      if pre_step:
          if (samples_in_step <= k): # swing_pre_step < 0.00001 and pre_step and k>1:
            # completed pre_step
            rospy.loginfo("done pre step")
            rospy.loginfo("time:")
            rospy.loginfo(rospy.get_time())

            # start first step:
            pre_step = 0
            first_step = 1
            rs.Set_step_phase(1) # Double-Support left leg in front
            k = 1
            
      elif first_step:
            if (samples_in_step <= k):   #swing_z_t < 0.00001 and k>1 and not pre_step:
              # completed start steps (pre_step + first_step)
              step_done = 1
              steps_count = steps_count + 1
              D = out.com_ref.x+step_length/2#step_length
              rospy.loginfo("done first step, number = %d, walk = %d" % (steps_count, ns.walk) )
              rospy.loginfo("time:")
              rospy.loginfo(rospy.get_time())
          
              first_step = 0
              k = 1
              distance_x_ref = p_ref_x[0]
              rs.Set_step_phase(3) # Double-Support right leg in front
              #rospy.sleep(4)
              #exit()######################################################
              if ns.walk:
                # make a full step:
                #exit()
                full_step = 1
                rospy.loginfo("starting full step, walk = %d" % (ns.walk) )
                # TODO: update ZMP_profiles with new parameters: step time,length...
                # Load new step to preview buffer:
                Preview_Sagital_x.load_NewStep( p_ref_x_forward_step + distance_x_ref, p_ref_x_forward_step )
                Preview_Lateral_y.load_NewStep( p_ref_y_step_right, r_[ p_ref_y_step_left, p_ref_y_step_right ] )
              else:
                # last step:
                last_step = 1
                # TODO: add stop ZMP_profiles

      
      else:
          #exit()######################################################
          if full_step:
              if (samples_in_step <= k):
                # completed a full step
                step_done = 1
                steps_count = steps_count + 1
                D = out.com_ref.x+step_length/2#D + step_length##############################################################
                rospy.loginfo("done step number = %d" % (steps_count) )
                rospy.loginfo("time:")
                rospy.loginfo(rospy.get_time())
                #rospy.sleep(4)
                k = 1
                distance_x_ref = p_ref_x[0]
                if rs.Get_step_phase() >= 3:
                  rs.Set_step_phase(1) # Double-Support left leg in front
                else:
                  rs.Set_step_phase(3) # Double-Support right leg in front
                rospy.loginfo( "step phase = %d" % ( rs.Get_step_phase() ) )

                if ns.walk:
                  # make a full step:
                  full_step = 1
                  # TODO: update ZMP_profiles with new parameters: step time,length...
                  # Load new step to preview buffer:
                  Preview_Sagital_x.load_NewStep( p_ref_x_forward_step + distance_x_ref, p_ref_x_forward_step )
                  if rs.Get_step_phase() == 1:
                    Preview_Lateral_y.load_NewStep( p_ref_y_step_left, r_[ p_ref_y_step_right,p_ref_y_step_left ] )
                  else:
                    Preview_Lateral_y.load_NewStep( p_ref_y_step_right, r_[ p_ref_y_step_left, p_ref_y_step_right ] )
                
                else:
                  # last step:
                  last_step = 1
                  full_step = 0

          elif last_step:
              if (samples_in_step <= k):
                # ?? TODO: allow another step cycle for stop step -> if (2*samples_in_step <= k):
                distance_x_ref = p_ref_x[0]
                go = 0
                rospy.loginfo("stoped walking")

          else:
             rospy.loginfo("Error: Problem step state not found. step phase = %d" % (rs.Get_step_phase()))

      #rospy.loginfo(k)
      step_done = 0
      interval.sleep()
      k  = k + 1
      
  #end while go

#end while not rospy.is_shutdown()
