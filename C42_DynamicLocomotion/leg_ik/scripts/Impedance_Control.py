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
####                                                                         ##
###############################################################################

import roslib; roslib.load_manifest('leg_ik')
import rospy

class Joint_Stiffness_Controller:
    'This controller regulates the input of a joint position controller to achieve disired effort output'
    # Joint State parameters:
    latest_position = 0
    position_sum = 0
    latest_velocity = 0
    velocity_sum = 0
    latest_effort = 0
    effort_sum = 0
    reset_avg_flag = True

    JS_i = -1 # index of joint state in JointState msg, is update with first call of joint_states Subscriber 

    last_set_point = 0 # set point
    last_command = 0
    limit_command_diff = 0.01 # 0.02
    command_resolution = 0.002 #0.0001 # Command will change with steps greater than command_resolution
                                       # Should be greater than steady state noise (PID error) 

    num_of_samples = 0

    def __init__(self, name, stiffness, update_period):
        self.name = name
        #self.limit_command_diff = limit_command_diff
        self.K = stiffness # parameter to tune
        self.avg_stamp = rospy.Time()
        self.avg_start_time = rospy.Time()
        self.update_period = update_period       

    def UpdateState(self, position, velocity, effort, time_stamp):
        self.num_of_samples += 1
        self.latest_position = position
        self.position_sum += position
        self.latest_velocity = velocity
        self.velocity_sum += velocity
        self.latest_effort = effort
        self.effort_sum += effort
        self.avg_stamp = time_stamp
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
        
        time_from_avg_start = self.avg_stamp.to_sec() - self.avg_start_time.to_sec()
        #rospy.loginfo("Stiffness C = '%s' method getCMD: time from starting to avg = %f " %(self.name,time_from_avg_start))

        if abs(set_point-self.last_set_point) < self.command_resolution: # if set_point doesn't change from previous set_point command 

            command = self.last_command

            if (time_from_avg_start >= self.update_period):

                position_avg = self.getAvgPosition()

                # rospy.loginfo("SC_'%s' method getCMD: update time = %f, position_sum = %f, effort_sum = %f" %  \
                #              (self.name,time_from_avg_start,self.position_sum,self.effort_sum))

                #J =  set_point - position_avg #self.latest_position ##self.latest_effort # units [Nm] J~=err*PID can try to use err = ??? 
                J =  self.getAvgEffort() # units [Nm] J~=err*PID can try to use err = ??? 
                #K =  10000 #Stiffness - parameter to tune
                correction_factor = J/self.K
                if correction_factor > self.limit_command_diff:
                    correction_factor = self.limit_command_diff
                # error = last_set_point-latest_position
                # if error == 0:
                #     sign = 0
                # else:
                #     sign = abs(error)/error
                command = self.last_set_point - correction_factor  #correction_factor*sign

                self.ResetStateSum()
                self.avg_start_time = self.avg_stamp # it's not enough to ResetStateSum() because getCMD might be called again before UpdateState

        else:
            if self.last_set_point*set_point < 0: # if set_point command is jittering around zero
                command = 0
                rospy.loginfo("JSC_'%s' method getCMD: set_point = %f, last_set_point = %f, JITTER" %(self.name,set_point,self.last_set_point))
            else:
                command = set_point
                self.ResetStateSum()
                rospy.loginfo("JSC_'%s' method getCMD: set_point = %f, last_set_point = %f, CHANGED" %(self.name,set_point,self.last_set_point))

            self.last_set_point = set_point            
            

        self.last_command = command

        return command 

class Position_Impedance_Controller:
    'This controller modifies a position command to achieve better damping of the system using force feedback'
    # parameters:
    num_of_samples = 0
    reset_avg_flag = True       
    limit_command_diff = 0.01 # 0.02
    command_resolution = 0.002 #0.0001 # Command will change with steps greater than command_resolution
                                       # Should be greater than steady state noise (PID error) 

    # Desired State:
    last_X_0 = 0
    last_Xdot_0 = 0

    # Model State:
    last_X_m = 0
    last_Xdot_m = 0
    last_model_stamp = rospy.Time() # time to use for integration


    # Feedback: Interaction Force
    last_Fint = 0  
    Fint_sum = 0

    def __init__(self, stiffness, damping):
        self.K_m = stiffness # parameter to tune
        self.B_m = damping   # parameter to tune
        self.avg_stamp = rospy.Time()
        self.avg_start_time = rospy.Time()     

    def UpdateForce(self, force, time_stamp):
        self.num_of_samples += 1
        self.last_Fint = force
        self.Fint_sum += force

        self.avg_stamp = time_stamp
        if self.reset_avg_flag:
            self.avg_start_time = time_stamp
            self.reset_avg_flag = False


    def ResetSum(self):
        self.num_of_samples = 0
        self.Fint_sum = 0
        self.reset_avg_flag = True

    def getAvgForce(self):
        if self.num_of_samples != 0:
            return (self.Fint_sum/self.num_of_samples)
        else:
            return (0)


    def getCMD(self, X_0, Xdot_0): # set_point = original position 
        
        time_from_avg_start = self.avg_stamp.to_sec() - self.avg_start_time.to_sec()
        #rospy.loginfo("Stiffness C = '%s' method getCMD: time from starting to avg = %f " %(self.name,time_from_avg_start))

        if abs(set_point-self.last_set_point) < self.command_resolution: # if set_point doesn't change from previous set_point command 

            command = self.last_command

            if (time_from_avg_start >= self.update_period):

                position_avg = self.getAvgPosition()

                # rospy.loginfo("SC_'%s' method getCMD: update time = %f, position_sum = %f, effort_sum = %f" %  \
                #              (self.name,time_from_avg_start,self.position_sum,self.effort_sum))

                #J =  set_point - position_avg #self.latest_position ##self.latest_effort # units [Nm] J~=err*PID can try to use err = ??? 
                J =  self.getAvgEffort() # units [Nm] J~=err*PID can try to use err = ??? 
                #K =  10000 #Stiffness - parameter to tune
                correction_factor = J/self.K
                if correction_factor > self.limit_command_diff:
                    correction_factor = self.limit_command_diff
                # error = last_set_point-latest_position
                # if error == 0:
                #     sign = 0
                # else:
                #     sign = abs(error)/error
                command = self.last_set_point - correction_factor  #correction_factor*sign

                self.ResetStateSum()
                self.avg_start_time = self.avg_stamp # it's not enough to ResetStateSum() because getCMD might be called again before UpdateState

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


