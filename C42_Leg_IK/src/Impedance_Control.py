#!/usr/bin/env python

###############################################################################
####                                                                         ##
####    Impedance Control: contains classes of controllers to be used with   ##
####                       joint position PID controllers.                   ##
####                                                                         ##
####    1) class "Joint_Stiffness_Controller":                               ##
####       The purpose of this controller is to limit the effort of the      ## 
####       join. Because of uncertainty of the structure* the desired angle  ##
####       might not fit the structure which can cause a build up of moment. ##   
####       This controller regulates the input of a joint position           ##
####       controller by aspiring to minimize its error while compromising   ##
####       the error from the desired angle so that we will get a better fit ##
####       to the structure. (* the structure can be built from a few        ##
####       controllers e.g: a parallelogram with a joint controller at each  ## 
####       corner).                                                          ##
####       The controller will kick-in (effect the set point) only if the    ##
####       INPUT set point is constant (with some noise that can be adjusted)##
####                                                                         ##
###############################################################################

import roslib; roslib.load_manifest('C42_Leg_IK')
import rospy

#################################################################################
#                     Joint_Stiffness_Controller                                #
#################################################################################


class Joint_Stiffness_Controller:
    'This controller regulates the input of a joint position controller to achieve disired effort output'
    # Class parameters:
    limit_command_diff = 0.01 # 0.02 # units [rad]
    command_resolution = 0.002 #0.0001 # Command will change with steps greater than command_resolution
                                       # Should be greater than steady state noise (PID error) 

    def __init__(self, name, stiffness, update_period):
        self.name = name
        #self.limit_command_diff = limit_command_diff
        self.K_current = stiffness # parameter to tune
        self.last_update_stamp = rospy.Time()
        self.avg_start_time = rospy.Time()
        self.update_period = update_period
        # Joint State parameters:
        self.latest_position = 0
        self.position_sum = 0
        self.latest_velocity = 0
        self.velocity_sum = 0
        self.latest_effort = 0
        self.effort_sum = 0
        self.reset_avg_flag = True

        self.JS_i = -1 # index of joint state in JointState msg, is update with first call of joint_states Subscriber 

        self.last_set_point = 0 # set point
        self.last_command = 0

        self.num_of_samples = 0      
        
        
        
    def UpdateState(self, position, velocity, effort, time_stamp):
        self.num_of_samples += 1
        self.latest_position = position
        self.position_sum += position
        self.latest_velocity = velocity
        self.velocity_sum += velocity
        self.latest_effort = effort
        self.effort_sum += effort
        self.last_update_stamp = time_stamp
        if self.reset_avg_flag:
            self.avg_start_time = time_stamp
            self.reset_avg_flag = False


    def ResetStateSum(self):
        self.num_of_samples = 0
        self.position_sum = 0
        self.velocity_sum = 0
        self.effort_sum = 0
        self.reset_avg_flag = True

    def getAvgPosition(self):
        if self.num_of_samples != 0:
            return (self.position_sum/self.num_of_samples)
        else:
            return (self.latest_position)

    def getAvgEffort(self):
        if self.num_of_samples != 0:
            return (self.effort_sum/self.num_of_samples)
        else:
            return (self.effort_sum)

    def getCMD(self, set_point): # set_point = original input to joint position controller 
        
        time_from_avg_start = self.last_update_stamp.to_sec() - self.avg_start_time.to_sec()
        #rospy.loginfo("Stiffness C = '%s' method getCMD: time from starting to avg = %f " %(self.name,time_from_avg_start))

        if abs(set_point-self.last_set_point) < self.command_resolution: # if set_point doesn't change from previous set_point command 

            command = self.last_command

            if (time_from_avg_start >= self.update_period):

                position_avg = self.getAvgPosition()

                # rospy.loginfo("SC_'%s' method getCMD: update time = %f, position_sum = %f, effort_sum = %f" %  \
                #              (self.name,time_from_avg_start,self.position_sum,self.effort_sum))

                #J =  set_point - position_avg #self.latest_position ##self.latest_effort # units [Nm] J~=err*PID can try to use err = ??? 
                J =  self.getAvgEffort() # units [Nm] J~=err*PID can try to use err = ??? 
                
                correction_factor = J/self.K_current
                if correction_factor > self.limit_command_diff:
                    correction_factor = self.limit_command_diff
                # elif -1*correction_factor > self.limit_command_diff: # added without checking need to check that it works
                #     correction_factor = -1*self.limit_command_diff
      
                command = self.last_set_point - correction_factor  # last_set_point ~= set_point, we use it so that set point will not include noise 

                self.ResetStateSum()
                self.avg_start_time = self.last_update_stamp # it's not enough to ResetStateSum() because getCMD might be called again before UpdateState

        else:
            if self.last_set_point*set_point < 0: # if set_point command is jittering around zero
                command = 0
                #rospy.loginfo("JSC_'%s' method getCMD: set_point = %f, last_set_point = %f, JITTER" %(self.name,set_point,self.last_set_point))
            else:
                command = set_point
                self.ResetStateSum()
                #rospy.loginfo("JSC_'%s' method getCMD: set_point = %f, last_set_point = %f, CHANGED" %(self.name,set_point,self.last_set_point))

            self.last_set_point = set_point            
            

        self.last_command = command

        return command 


#################################################################################
#     Joint_Stiffness_Controller 2 - using ground reaction feedback             #
#################################################################################


class Joint_Stiffness_Controller_2:
    'This controller regulates the input of a joint position controller to limit effort output so that foot stays on ground'
    # Class parameters:
    limit_command_diff = 0.02 # units [rad]
    # command_resolution = 0.002 #0.0001 # Command will change with steps greater than command_resolution
    #                                    # Should be greater than steady state noise (PID error)

    # leg_phase_type:
    stance = 'stance'
    swing = 'swing' 

    def __init__(self, name, stance_stiffness, swing_stiffness, activation_ZMP_point):
        self.name = name
        self.K_stance = stance_stiffness  # parameter to tune
        self.K_swing = swing_stiffness    # parameter to tune
        self.K_current = swing_stiffness 
        self.activation_ZMP_point = activation_ZMP_point # when the torque/force is greater than this value we start to reduce the effort of the joint
        
        self.last_set_point = 0.0 # set point
        self.last_command = 0.0

        # FeedBack data:
        self.FB_force_avg = 0.0
        self.FB_torque_avg = 0.0

        self.FB_force_sum = 0.0
        self.FB_torque_sum = 0.0

        self.num_of_FB_samples = 0.0
        self.reset_sum_flag = False

    # External auxiliary method:
    def UpdateFeedBack(self, force, torque):
        self.num_of_FB_samples += 1.0

        if self.reset_sum_flag or (self.num_of_FB_samples <= 0.0) or (self.num_of_FB_samples > 150) :
            self.ResetFB_Sum()
            self.num_of_FB_samples = 1.0
            self.reset_sum_flag = False


        self.FB_force_sum += force
        self.FB_torque_sum += torque

        self.FB_force_avg = self.FB_force_sum/self.num_of_FB_samples
        self.FB_torque_avg = self.FB_torque_sum/self.num_of_FB_samples

    # External auxiliary method:
    def getAvgForce(self):
        return (self.FB_force_avg)

    # External auxiliary method:
    def getAvgTorque(self):
        return (self.FB_torque_avg )

    def ResetFB_Sum(self):
        self.FB_force_sum = 0.0
        self.FB_torque_sum = 0.0
        self.num_of_FB_samples = 0.0

    # External auxiliary method:
    def ChangeStiffness(self, leg_phase_type):
        if leg_phase_type == Joint_Stiffness_Controller_2.stance:
            self.K_current = self.K_stance
        elif leg_phase_type == Joint_Stiffness_Controller_2.swing:
            self.K_current = self.K_swing

    # External auxiliary method:
    def getCMD(self, set_point): # set_point = original input to joint position controller 

        if self.FB_force_avg == 0:
            force_avg = 10.0  # units [N]
        else:
            force_avg = self.FB_force_avg


        J_extra =  abs(self.FB_torque_avg) - self.activation_ZMP_point*force_avg # units [Nm], excess torque beyond the ZMP activation point
        if J_extra > 0.0: # if measered torque is greater than activation ZMP point we start to reduce joint effort

            # rospy.loginfo("SC_'%s' method getCMD: update time = %f, position_sum = %f, effort_sum = %f" %  \
            #              (self.name,time_from_avg_start,self.position_sum,self.effort_sum))
                
            correction_factor = J_extra/self.K_current             # may want to normalize using force_avg?
            # limit controllers response:
            if correction_factor > self.limit_command_diff:
                correction_factor = self.limit_command_diff
            # determine sign of correction factor:
            if self.FB_torque_avg < 0.0:
                correction_factor = -1.0*correction_factor
      
            command = set_point + correction_factor  
            
        else:
            command = set_point
        
        # rospy.loginfo("JSC2_'%s' method getCMD: set_point = %f, cmd = %f, J_extra = %f, Fb J = %f, Fb F = %f, N_samples = %f, force_avg = %f"\
        #             %(self.name, set_point, command, J_extra, self.FB_torque_avg, self.FB_force_avg, self.num_of_FB_samples, force_avg ))

        self.reset_sum_flag = True
        self.last_set_point = set_point
        self.last_command = command

        return (command) 

#################################################################################
#                     Position_Stiffness_Controller                             #
#################################################################################


class Position_Stiffness_Controller:
    'This controller modifies a position command to achieve the desired stiffness of the system using force feedback'
    # Class parameters:
    minimum_update_period = 0.02 #0.05 #units [sec]; minimal time to average feedback below this value OUTPUT command 
                                 #             will not be modified    
    limit_command_diff = 0.1 # # units [meters]
    command_resolution = 0.002 #0.0001 # Command will change with steps greater than command_resolution
                                       # Should be greater than steady state noise (PID error) 

    def __init__(self, name, stiffness, triggered_controller, bypass_input2output):
        self.name = name
        self.K_current_m = stiffness # parameter to tune
        #self.B_m = damping   # parameter to tune
        self.triggered_controller = triggered_controller
        self.update_command = not(triggered_controller) # flag to enable update of controller output command when trigger is disabled
        self.bypass_in2out = bypass_input2output
        # self.last_update_stamp = rospy.Time()
        self.avg_start_time = rospy.Time()
        # parameters:
        self.num_of_samples = 0
        self.reset_avg_flag = True
        # Desired State (INPUT):
        self.last_X_0 = 0
        #last_Xdot_0 = 0

        # Model State :
        self.last_X_m = 0  # OUTPUT command   
        #last_Xdot_m = 0
        #last_model_stamp = rospy.Time() # time to use for integration


        # Feedback: Interaction Force
        self.last_Fint = 0  
        self.Fint_sum = 0

        # Triggered Controller:
        self.trigger_event = False
        self.trigger_value = 200 # units [Nm] under this value the trigger will be set 

        self.is_swing = 0

    def UpdateForce(self, force):
        self.num_of_samples += 1
        self.last_Fint = force
        self.Fint_sum += force

        #self.last_update_stamp = time_stamp
        if self.reset_avg_flag:
            self.avg_start_time = rospy.get_rostime() #time_stamp
            self.reset_avg_flag = False


    def ResetSum(self):
        self.num_of_samples = 0
        self.Fint_sum = 0
        self.reset_avg_flag = True

    def getAvgForce(self):
        if self.num_of_samples != 0:
            return (self.Fint_sum/self.num_of_samples)
        else:
            return (self.last_Fint) # (0)

    def ResetTrigger(self):
        self.trigger_event = False 

    def SetTrigger(self):
        self.trigger_event = True    

    def checkTriggerEvent(self, force, input_cmd_delta):
        if (self.trigger_value >= force) and (force > 0) or (input_cmd_delta > 0):
            self.SetTrigger()
            rospy.loginfo( "PSC_'%s' method checkTriggerEvent: force = %f, input_cmd_delta = %f" %  \
                          (self.name,force, input_cmd_delta) )
        else:
            self.ResetTrigger()

    def ByPassON(self):
        self.bypass_in2out = True 

    def ByPassOFF(self):
        self.bypass_in2out = False

    def ByPassStatus(self):
        return (self.bypass_in2out) 

    def getCMD(self, X_0,zmp_ref_y,step_phase ,step_width,zmp_width,step_time,des_l_force_pub,des_r_force_pub): # X_0 = original position input command
       

        [desired_force_L , desired_force_R]  = self.desired_force(zmp_ref_y,step_phase ,step_width,zmp_width,step_time)

        des_l_force_pub.publish( desired_force_L )
        des_r_force_pub.publish( desired_force_R )




        if (step_phase == 1) or (step_phase == 2) : # left leg is stance
           force_des = desired_force_R
        if (step_phase == 3) or (step_phase == 4) : # right leg is stance
           force_des = desired_force_L

        current_time = rospy.get_rostime().to_sec() #self.last_update_stamp.to_sec()

        time_from_avg_start = current_time - self.avg_start_time.to_sec()
        #rospy.loginfo("Stiffness C = '%s' method getCMD: time from starting to avg = %f " %(self.name,time_from_avg_start))

        # if feedback had enough time to update generate new output command with feedback 
        # else use previous command with feedback adding to it change of input command (same as using current input command with old feedback)
        if (time_from_avg_start >= self.minimum_update_period) and (self.Fint_sum != 0):

            force_avg = self.getAvgForce() # Assumption that force will be able to get update in minimum_update_period=0.05[sec] 
                                           # does not work all the time. Added to if statement to check Fint_sum value.
                                           # Problem may have occured because rxplot of contact force was open!!! 

            # rospy.loginfo("PSC_'%s' method getCMD: update interval = %f, force samples = %d, force_avg = %f" %  \
            #               (self.name,time_from_avg_start,self.num_of_samples,force_avg))
            
            # Check trigger event if event has not yet occured and updates trigger_event accordingly
            if self.triggered_controller and not(self.trigger_event):
                self.checkTriggerEvent(force_avg,  X_0 - self.last_X_0)

         #   if X_0 > self.last_X_0: # if lifting swing foot we don't want any force on the swing leg 
         #       force_des = 0
       #     else:
        #        force_des = force_desired

            # OUTPUT COMMAND:
            if (self.update_command or self.trigger_event) and ( not self.bypass_in2out): # if update of output command is enabled 
                     
                correction_factor = (force_des - force_avg)/self.K_current_m
                # output cmd saturation (clamp):
                if correction_factor > self.limit_command_diff: 
                    correction_factor = self.limit_command_diff
                elif -1*correction_factor > self.limit_command_diff: 
                    correction_factor = -1*self.limit_command_diff

                pos_command_out = X_0 - correction_factor 
            else:
                pos_command_out = X_0

            # handle feedback filter:
            self.ResetSum()
            # self.avg_start_time = rospy.get_rostime() # it's not enough to ResetSum() because getCMD might be called again before Update

        else:
            input_command_delta_update = X_0 - self.last_X_0 
            pos_command_out = self.last_X_m + input_command_delta_update          

        self.last_X_m = pos_command_out
        self.last_X_0 = X_0
        # rospy.loginfo("PSC_'%s' method getCMD: bypass_in2out-%s, X_0 = %f, output cmd = %f, force_des = %f, force_avg = %f " %  \
        #                   (self.name, self.bypass_in2out, X_0, pos_command_out, force_des, self.getAvgForce()))

        return ( pos_command_out, desired_force_L , desired_force_R )

    def desired_force(self,zmp_ref_y,step_phase ,step_width,zmp_width,step_time):

        Mtot = 91.4
        g = 9.81
        Mass_tr = 0.05*Mtot
        ZMP_tr = 0.1*zmp_width
        alpha_min = 0.1
        dt = 0.01

        foot_length = 0.0825+0.1775
        foot_height = 0.05
        foot_width = 0.124

        foot_x = foot_length
        foot_y = foot_width/3

        left_foot_y = step_width/2 - foot_y/2
        left_foot_x = foot_x
        left_zmp_tr = zmp_width/2 - ZMP_tr/2

        right_foot_y = -step_width/2 + foot_y/2
        right_foot_x = foot_x
        right_zmp_tr = -zmp_width/2 + ZMP_tr/2
        
     #   alpha = 0.5


        pl = zmp_ref_y - left_foot_y
        pr = zmp_ref_y - right_foot_y
        

        if (zmp_ref_y - left_foot_y > 0) or (zmp_ref_y -left_zmp_tr > 0):
            alpha = alpha_min 
        elif  (right_foot_y - zmp_ref_y  > 0) or (right_zmp_tr-zmp_ref_y  > 0):
            alpha = 1-alpha_min
        else:
            alpha = max(min(abs(-pl)/abs(pl-pr),1-alpha_min),alpha_min);
        

        # #Detect when swinging and set alpha to ZERO (in phase 2) or ONE (in phase 4)
        if (step_phase== 2) & (self.is_swing < round(step_time/3/dt)) :  #phase 2: left is stance

             alpha = 0
             self.is_swing = self.is_swing + 1
     
        if (step_phase== 4) and (self.is_swing < round(step_time/3/dt)):   #phase 2: left is stance

             alpha =1
             self.is_swing = self.is_swing + 1

        if (step_phase== 1) or (step_phase== 3):
            self.is_swing = 0

        f_R =  alpha*Mtot*g
        f_L =  (1-alpha)*Mtot*g

          
 #       pl = zmp_ref_y - left_foot_y
 #       pr = zmp_ref_y - right_foot_y
        
 #       if (zmp_ref_y - left_foot_y > 0) or (zmp_ref_y -left_zmp_tr > 0) :
 #           alpha =0
 #       elif  (right_foot_y - zmp_ref_y > 0) or (right_zmp_tr - zmp_ref_y > 0) :
 #           alpha=1
  #      else:
  #          alpha = abs(-pl)/abs(pl-pr)
        
            
 #       f_R =  alpha*Mtot*g
  #      f_L =  (1-alpha)*Mtot*g


        return ([f_L , f_R])


#################################################################################
#  Position_Stiffness_Controller 2 - force update rate same as position command #
#################################################################################


class Position_Stiffness_Controller_2:
    'This controller modifies a position command to achieve the desired stiffness of the system using force feedback from ground reaction'
    # Class parameters:
    #minimum_update_period = 0.05 #units [sec]; minimal time to average feedback below this value OUTPUT command 
                                 #             will not be modified    
    limit_command_diff = 0.1 # # units [meters]
    # command_resolution = 0.002 #0.0001 # Command will change with steps greater than command_resolution
    #                                    # Should be greater than steady state noise (PID error) 

    def __init__(self, name, stiffness, triggered_controller, bypass_input2output):
        self.name = name
        self.K_current_m = stiffness # parameter to tune
        #self.B_m = damping   # parameter to tune
        self.triggered_controller = triggered_controller
        self.update_command = not(triggered_controller) # flag to enable update of controller output command when trigger is disabled
        self.bypass_in2out = bypass_input2output

        # parameters:
        self.num_of_samples = 0
        self.reset_avg_flag = True
        # Desired State (INPUT):
        self.last_X_0 = 0
        #last_Xdot_0 = 0

        # Model State :
        self.last_X_m = 0  # OUTPUT command   
        #last_Xdot_m = 0
        #last_model_stamp = rospy.Time() # time to use for integration

        # FeedBack data:
        self.FB_force_avg = 0.0
        self.FB_force_sum = 0.0
        self.num_of_FB_samples = 0.0
        self.reset_sum_flag = False

        self.FB_force_filtered = 0.0

        # Triggered Controller:
        self.trigger_event = False
        self.trigger_value = 200 # units [Nm] under this value the trigger will be set 

        self.is_swing = 0    


    # External auxiliary method:
    def UpdateFeedBack(self, force, filtered_force):
        ## Average force between getCMD calls:
        self.num_of_FB_samples += 1.0

        if self.reset_sum_flag or (self.num_of_FB_samples <= 0.0) or (self.num_of_FB_samples > 150) :
            self.ResetFB_Sum()
            self.num_of_FB_samples = 1.0
            self.reset_sum_flag = False

        self.FB_force_sum += force

        self.FB_force_avg = self.FB_force_sum/self.num_of_FB_samples

        ## Filtered force:
        self.FB_force_filtered = filtered_force

    # External auxiliary method:
    def getAvgForce(self):
        return (self.FB_force_avg)

    # External auxiliary method:
    def getFilteredForce(self):
        return (self.FB_force_filtered)

    def ResetFB_Sum(self):
        self.FB_force_sum = 0.0
        self.num_of_FB_samples = 0.0 

    def ResetTrigger(self):
        self.trigger_event = False 

    def SetTrigger(self):
        self.trigger_event = True    

    def checkTriggerEvent(self, force, input_cmd_delta):
        if (self.trigger_value >= force) and (force > 0) or (input_cmd_delta > 0):
            self.SetTrigger()
            rospy.loginfo( "PSC_'%s' method checkTriggerEvent: force = %f, input_cmd_delta = %f" %  \
                          (self.name,force, input_cmd_delta) )
        else:
            self.ResetTrigger()

    def ByPassON(self):
        self.bypass_in2out = True 

    def ByPassOFF(self):
        self.bypass_in2out = False

    def ByPassStatus(self):
        return (self.bypass_in2out) 

    def getCMD(self, X_0,zmp_ref_y,step_phase ,step_width,zmp_width,step_time,des_l_force_pub,des_r_force_pub): # X_0 = original position input command
        
        # Desired force profile of swing leg:
        force_des = self.desired_force(zmp_ref_y,step_phase ,step_width, zmp_width, step_time, des_l_force_pub, des_r_force_pub)

        force_FB = self.FB_force_filtered # self.FB_force_avg
       
        # Check trigger event if event has not yet occured and updates trigger_event accordingly
        if self.triggered_controller and not(self.trigger_event):
            self.checkTriggerEvent(force_FB,  X_0 - self.last_X_0)

        # OUTPUT COMMAND:
        if (self.update_command or self.trigger_event) and ( not self.bypass_in2out): # if update of output command is enabled 
                     
            correction_factor = (force_des - force_FB)/self.K_current_m
            # output cmd saturation (clamp):
            if correction_factor > self.limit_command_diff: 
                correction_factor = self.limit_command_diff
            elif -1*correction_factor > self.limit_command_diff: 
                correction_factor = -1*self.limit_command_diff

            pos_command_out = X_0 - correction_factor 
        else:
            pos_command_out = X_0      

        # rospy.loginfo("PSC2_'%s' method getCMD: bypass_in2out-%s, X_0 = %f, output cmd = %f, force_des = %f, force_avg = %f " %  \
        #                   (self.name, self.bypass_in2out, X_0, pos_command_out, force_des, self.getAvgForce()))

        # rospy.loginfo("PSC2_'%s' method getCMD: set_point = %f, cmd = %f, cmd correction = %f, force_des = %f, Fb force_avg = %f, N_samples = %f, force_FB = %f"\
        #             %(self.name, X_0, pos_command_out, correction_factor, force_des, self.FB_force_avg, self.num_of_FB_samples, force_FB ))

        self.reset_sum_flag = True
        self.last_X_0 = X_0
        self.last_X_m = pos_command_out

        return pos_command_out 

    def desired_force(self,zmp_ref_y,step_phase ,step_width, zmp_width, step_time, des_l_force_pub, des_r_force_pub):

        Mtot = 91.4
        g = 9.81
        Mass_tr = 0.05*Mtot
        ZMP_tr = 0.1*zmp_width
        alpha_min = 0.1
        dt = 0.01

        foot_length = 0.0825+0.1775
        foot_height = 0.05
        foot_width = 0.124

        foot_x = foot_length
        foot_y = foot_width/3

        left_foot_y = step_width/2 - foot_y/2
        left_foot_x = foot_x
        left_zmp_tr = zmp_width/2 - ZMP_tr/2

        right_foot_y = -step_width/2 + foot_y/2
        right_foot_x = foot_x
        right_zmp_tr = -zmp_width/2 + ZMP_tr/2
        
     #   alpha = 0.5


        pl = zmp_ref_y - left_foot_y
        pr = zmp_ref_y - right_foot_y
        

        if (zmp_ref_y - left_foot_y > 0) or (zmp_ref_y -left_zmp_tr > 0):
            alpha = alpha_min 
        elif  (right_foot_y - zmp_ref_y  > 0) or (right_zmp_tr-zmp_ref_y  > 0):
            alpha = 1-alpha_min
        else:
            alpha = max(min(abs(-pl)/abs(pl-pr),1-alpha_min),alpha_min);
        

        # #Detect when swinging and set alpha to ZERO (in phase 2) or ONE (in phase 4)
        if (step_phase== 2) & (self.is_swing < round(step_time/3/dt)) :  #phase 2: left is stance

             alpha = 0
             self.is_swing = self.is_swing + 1
     
        if (step_phase== 4) and (self.is_swing < round(step_time/3/dt)):   #phase 2: left is stance

             alpha =1
             self.is_swing = self.is_swing + 1

        if (step_phase== 1) or (step_phase== 3):
            self.is_swing = 0

        f_R =  alpha*Mtot*g
        f_L =  (1-alpha)*Mtot*g

          
 #       pl = zmp_ref_y - left_foot_y
 #       pr = zmp_ref_y - right_foot_y
        
 #       if (zmp_ref_y - left_foot_y > 0) or (zmp_ref_y -left_zmp_tr > 0) :
 #           alpha =0
 #       elif  (right_foot_y - zmp_ref_y > 0) or (right_zmp_tr - zmp_ref_y > 0) :
 #           alpha=1
  #      else:
  #          alpha = abs(-pl)/abs(pl-pr)
        
            
 #       f_R =  alpha*Mtot*g
  #      f_L =  (1-alpha)*Mtot*g

        des_l_force_pub.publish( f_L )
        des_r_force_pub.publish( f_R )

        if (step_phase == 1) or (step_phase == 2) : # left leg is stance
           force_des_swing_leg = f_R
        if (step_phase == 3) or (step_phase == 4) : # right leg is stance
           force_des_swing_leg = f_L

        return (force_des_swing_leg)
