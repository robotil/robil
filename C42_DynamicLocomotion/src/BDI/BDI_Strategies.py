#!/usr/bin/env python

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

import math
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from atlas_msgs.msg import AtlasCommand, AtlasSimInterfaceCommand, AtlasSimInterfaceState, AtlasState, AtlasBehaviorStepData

###################################################################################
#--------------------------- Exception -------------------------------------------
###################################################################################

class BDI_Strategy_Exception(Exception):
    """
        For information as to why the exception was raised, see StateMachineError.Message attribute
    """
    def __init__(self,strMessage):
        self.Message = strMessage

###################################################################################
#--------------------------- Strategies -------------------------------------------
###################################################################################

class BDI_Strategy(object):
    """
    """
    def __init__(self,odometer):

        self._Odometer = odometer

        self._k_effort = [0] * 28
        self._StepLength = 0.2
        self._StepWidth = 0.15
        self._alpha = 0.0

        self._command = AtlasSimInterfaceCommand()
        self._command.behavior = AtlasSimInterfaceCommand.WALK

        for i in range(4):
            step_index = i
            self._command.walk_params.step_data[i].step_index = step_index

            self._command.walk_params.step_data[i].foot_index = step_index%2
            
            # A duration of 0.63s is a good default value
            self._command.walk_params.step_data[i].duration = 0.63
            
            # As far as I can tell, swing_height has yet to be implemented
            self._command.walk_params.step_data[i].swing_height = 0.2

            # Determine pose of the next step based on the step_index
            # Right foot occurs on even steps, left on odd
            x = self._StepLength
            y = self._StepWidth if (step_index%2==0) else -self._StepWidth
            x,y = self._Odometer.GetInGlobalCoordinates(x,y)
            self._command.walk_params.step_data[i].pose.position.x = x
            # Step 0.15m to either side of center, alternating with feet
            self._command.walk_params.step_data[i].pose.position.y = y
            
            # The z position is observed for static walking, but the foot
            # will be placed onto the ground if the ground is lower than z
            self._command.walk_params.step_data[i].pose.position.z = 0.0
            
            # Point those feet straight ahead
            self._command.walk_params.step_data[i].pose.orientation.x = 0.0
            self._command.walk_params.step_data[i].pose.orientation.y = 0.0
            self._command.walk_params.step_data[i].pose.orientation.z = 0.0
            self._command.walk_params.step_data[i].pose.orientation.w = 1.0

    def GetAtlasSimInterfaceCommand(self,index):       
        # A walk behavior command needs to know three additional steps beyond the current step needed to plan
        # for the best balance
        print("step Idle")
        for i in range(4):
            step_index = index + i
            is_right_foot = step_index % 2
            
            self._command.walk_params.step_data[i].step_index = step_index
            self._command.walk_params.step_data[i].foot_index = step_index%2

            x = 0
            y = self._StepWidth if (step_index%2==0) else -self._StepWidth
            x,y = self._Odometer.GetInGlobalCoordinates(x,y)
            self._command.walk_params.step_data[i].pose.position.x = x
            # Step 0.15m to either side of center, alternating with feet
            self._command.walk_params.step_data[i].pose.position.y = y

            # Calculate orientation quaternion
            Q = quaternion_from_euler(0, 0, self._Odometer.GetYaw())

            self._command.walk_params.step_data[i].pose.orientation.x = Q[0]
            self._command.walk_params.step_data[i].pose.orientation.y = Q[1]
            self._command.walk_params.step_data[i].pose.orientation.z = Q[2]
            self._command.walk_params.step_data[i].pose.orientation.w = Q[3]

        return self._command

    def StepDone(self,bIsRight):
        pass

class BDI_StrategyIdle(BDI_Strategy):
    """
    """
    def __init__(self,odometer):
        BDI_Strategy.__init__(self,odometer)
        self._counter = 0
        self._StepLength = 0.0
        self._StepWidth = 0.15

    def Initialize(self):
        self._counter = 0

    def StepDone(self,bIsRight):
        if(self._counter > 2):
            raise BDI_Strategy_Exception("Done")
        self._counter += 1

class BDI_StrategyForward(BDI_Strategy):
    """
    """
    def __init__(self,odometer):
        BDI_Strategy.__init__(self,odometer)
        self._StepLength = 0.3
        self._StepWidth = 0.15
        self._ErrorCorrection = 0.05
        self._ErrorCorrected = 0.0
        self._MinimalCorrection = 0.02

    def GetAtlasSimInterfaceCommand(self,index):       
        print("step forward")  
        # Correct Lateral Error
        self._ErrorCorrected = 0
        # Correct only when needed
        if (math.fabs(self._Error) > self._MinimalCorrection):
            # # Correct only with outer foot
            # if (self._Error > 0):
            #     # We are to the left
            #     if (index%2==0):
            #         # Left foot is the next step, correct to the right
            #         if(math.fabs(self._Error) < self._ErrorCorrection):
            #             self._ErrorCorrected = -self._Error
            #         else:
            #             self._ErrorCorrected = -self._ErrorCorrection                   
            # else:
            #     # We are to the right
            #     if (index%2==1):
            #         # Right foot is the next step, correct to the right
            #         if(math.fabs(self._Error) < self._ErrorCorrection):
            #             self._ErrorCorrected = -self._Error
            #         else:
            #             self._ErrorCorrected = self._ErrorCorrection    
            if(math.fabs(self._Error) < self._ErrorCorrection):
                self._ErrorCorrected = -self._Error
            else:
                self._ErrorCorrected = -self._ErrorCorrection*(self._Error/math.fabs(self._Error)) 

        for i in range(4):
            step_index = index + i
            is_right_foot = step_index % 2
            
            self._command.walk_params.step_data[i].step_index = step_index
            self._command.walk_params.step_data[i].foot_index = step_index%2

            x = self._StepLength*(i+1)
            y = self._StepWidth if (step_index%2==0) else -self._StepWidth\

            y += self._ErrorCorrected*(i+1)

            x,y = self._Odometer.GetInGlobalCoordinates(x,y)
            #print(x,y)
            self._command.walk_params.step_data[i].pose.position.x = x
            # Step 0.15m to either side of center, alternating with feet
            self._command.walk_params.step_data[i].pose.position.y = y

            # Calculate orientation quaternion
            Q = quaternion_from_euler(0, 0, self._Odometer.GetYaw())

            self._command.walk_params.step_data[i].pose.orientation.x = Q[0]
            self._command.walk_params.step_data[i].pose.orientation.y = Q[1]
            self._command.walk_params.step_data[i].pose.orientation.z = Q[2]
            self._command.walk_params.step_data[i].pose.orientation.w = Q[3]

            #print(self._command)

        return self._command

    def StepDone(self,bIsRight):
        x = self._StepLength
        print("Correction",self._ErrorCorrected)
        self._Odometer.AddLocalPosition(x,self._ErrorCorrected)

    def SetPathError(self,pathError):
        self._Error = pathError

class BDI_StrategyLeft(BDI_Strategy):
    """
    """
    def __init__(self,odometer):
        BDI_Strategy.__init__(self,odometer)

        self._TurnRadius = 3.25

        self._InnerRadius = self._TurnRadius - self._StepWidth
        self._OuterRadius = self._TurnRadius + self._StepWidth
        #self._alpha = self._StepLength/self._TurnRadius
        self._alpha = math.acos(1-self._StepLength**2/(2*self._TurnRadius**2))

    def GetAtlasSimInterfaceCommand(self,index): 
        print("step Left ",self._Odometer.GetYaw())
        for i in range(4):
            step_index = index + i
            is_right_foot = step_index % 2
            
            self._command.walk_params.step_data[i].step_index = step_index
            self._command.walk_params.step_data[i].foot_index = step_index%2

            # a left step is in the inner circle
            r = self._InnerRadius if (step_index%2==0) else self._OuterRadius
            R = self._InnerRadius if (step_index%2==1) else self._OuterRadius

            theta = self._alpha*(i+1)
            #print(theta)

            # (Translate(dot)Rotate(dot)Translate)
            x = r*math.sin(theta)
            y = R - r*math.cos(theta)

            #print(x,y)
            x,y = self._Odometer.GetInGlobalCoordinates(x,y)
            self._command.walk_params.step_data[i].pose.position.x = x
            self._command.walk_params.step_data[i].pose.position.y = y

            # Calculate orientation quaternion
            Q = quaternion_from_euler(0, 0, self._Odometer.GetYaw()+theta)

            self._command.walk_params.step_data[i].pose.orientation.x = Q[0]
            self._command.walk_params.step_data[i].pose.orientation.y = Q[1]
            self._command.walk_params.step_data[i].pose.orientation.z = Q[2]
            self._command.walk_params.step_data[i].pose.orientation.w = Q[3]

        return self._command

    def StepDone(self,bIsRight):
        if (bIsRight):
            r = self._OuterRadius
            R = self._InnerRadius
        else:
            r = self._InnerRadius
            R = self._OuterRadius

        # (Translate(dot)Rotate(dot)Translate)
        x = r*math.sin(self._alpha)
        y = R - r*math.cos(self._alpha)
        self._Odometer.AddLocalPosition(x,y)
        self._Odometer.AddYaw(self._alpha)

class BDI_StrategyRight(BDI_Strategy):
    """
    """
    def __init__(self,odometer):
        BDI_Strategy.__init__(self,odometer)

        self._TurnRadius = 3.25

        self._InnerRadius = self._TurnRadius - self._StepWidth
        self._OuterRadius = self._TurnRadius + self._StepWidth
        #self._alpha = self._StepLength/self._TurnRadius
        self._alpha = -math.acos(1-self._StepLength**2/(2*self._TurnRadius**2))

    def GetAtlasSimInterfaceCommand(self,index):
        print("step Right") 
        for i in range(4):
            step_index = index + i
            is_right_foot = step_index % 2
            
            self._command.walk_params.step_data[i].step_index = step_index
            self._command.walk_params.step_data[i].foot_index = step_index%2

            # a left step is in the outer circle
            r = self._InnerRadius if (step_index%2==1) else self._OuterRadius
            R = self._InnerRadius if (step_index%2==0) else self._OuterRadius

            theta = self._alpha*(i+1)

            x = r*math.sin(-theta)
            y = r*math.cos(theta) - R

            #print(x,y)
            x,y = self._Odometer.GetInGlobalCoordinates(x,y)
            self._command.walk_params.step_data[i].pose.position.x = x
            self._command.walk_params.step_data[i].pose.position.y = y

            # Calculate orientation quaternion
            Q = quaternion_from_euler(0, 0, self._Odometer.GetYaw()+theta)

            self._command.walk_params.step_data[i].pose.orientation.x = Q[0]
            self._command.walk_params.step_data[i].pose.orientation.y = Q[1]
            self._command.walk_params.step_data[i].pose.orientation.z = Q[2]
            self._command.walk_params.step_data[i].pose.orientation.w = Q[3]

        return self._command

    def StepDone(self,bIsRight):
        if (bIsRight):
            r = self._OuterRadius
            R = self._InnerRadius
        else:
            r = self._InnerRadius
            R = self._OuterRadius

        # (Translate(dot)Rotate(dot)Translate)
        x = r*math.sin(-self._alpha)
        y = r*math.cos(self._alpha) - R
        self._Odometer.AddLocalPosition(x,y)
        self._Odometer.AddYaw(self._alpha)

