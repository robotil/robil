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
        self._StepLength = 0.2
        self._StepWidth = 0.15
        self._alpha = 0.0
        self._Duration = 0.63
        self._SwingHeight  = 0.2
        
        self._counterTarget = 0
        self._counter = 0

    def Initialize(self):
        self._counter = 0

    def GetStepData(self,index):
        self._counter += 1
        command = AtlasSimInterfaceCommand()
        
        stepData = command.walk_params.step_data[0]
        stepData.step_index = index
        stepData.foot_index = index%2

        stepData.duration = self._Duration
        stepData.swing_height = self._SwingHeight

        stepData.pose.position.z = 0.0

        return stepData
    
    def SetTarget(self,targetYaw):
        delatYaw = targetYaw - self._Odometer.GetYaw()
        # get the angle in +-pi
        delatYaw = math.asin(math.sin(delatYaw))
        print("SetTarget: alpha ",self._alpha,"targetYaw ",targetYaw,"delatYaw ",delatYaw)
        if (math.fabs(self._alpha) > 0.01):
            self._counterTarget = math.fabs(delatYaw)/math.fabs(self._alpha)
        else:
            self._counterTarget = 1
        print("SetTarget: CounterTarget",self._counterTarget)
            
    def IsDone(self):
        return (self._counterTarget<self._counter)        

class BDI_StrategyIdle(BDI_Strategy):
    """
    """
    def __init__(self,odometer):
        BDI_Strategy.__init__(self,odometer)
        self._counter = 0
        self._StepLength = 0.0
        self._StepWidth = 0.15
        
        self._counterTarget = 5

    def GetStepData(self,index):
        stepData = BDI_Strategy.GetStepData(self,index)
        x = 0
        y = self._StepWidth if (index%2==0) else -self._StepWidth
        self._Odometer.AddLocalPosition(x,y)
        stepData.pose.position.x,stepData.pose.position.y = self._Odometer.GetGlobalPosition()

        # Calculate orientation quaternion
        Q = quaternion_from_euler(0, 0, self._Odometer.GetYaw())

        stepData.pose.orientation.x = Q[0]
        stepData.pose.orientation.y = Q[1]
        stepData.pose.orientation.z = Q[2]
        stepData.pose.orientation.w = Q[3]

        return stepData

class BDI_StrategyForward(BDI_Strategy):
    """
    """
    def __init__(self,odometer):
        BDI_Strategy.__init__(self,odometer)
        self._StepLength = 0.3
        self._StepWidth = 0.15
        self._ErrorCorrection = self._StepWidth
        self._ErrorCorrected = 0.0
        self._MinimalCorrection = 0.02
        self._Error = 0.0

    def GetStepData(self,index):
        stepData = BDI_Strategy.GetStepData(self,index)
        # Correct Lateral Error
        self._ErrorCorrected = 0
        # Correct only when needed
        if (math.fabs(self._Error) > self._MinimalCorrection):   
            if(math.fabs(self._Error) < self._ErrorCorrection):
                self._ErrorCorrected = -self._Error/4
            else:
                self._ErrorCorrected = -self._ErrorCorrection*(self._Error/math.fabs(self._Error))/4 

        x = self._StepLength
        y = self._StepWidth if (index%2==0) else -self._StepWidth
        y += self._ErrorCorrected
        self._Odometer.AddLocalPosition(x,y)
        stepData.pose.position.x,stepData.pose.position.y = self._Odometer.GetGlobalPosition()

        # Calculate orientation quaternion
        Q = quaternion_from_euler(0, 0, self._Odometer.GetYaw())

        stepData.pose.orientation.x = Q[0]
        stepData.pose.orientation.y = Q[1]
        stepData.pose.orientation.z = Q[2]
        stepData.pose.orientation.w = Q[3]

        return stepData

    def SetPathError(self,pathError):
        self._Error = pathError

    def IsDone(self):
        # Forward does not finish alone, it is either preempted to turn or to stop
        return False 
    
class BDI_StrategyLeft(BDI_Strategy):
    """
    """
    def __init__(self,odometer):
        BDI_Strategy.__init__(self,odometer)

        self._TurnRadius = 1.5

        self._InnerRadius = self._TurnRadius - self._StepWidth
        self._OuterRadius = self._TurnRadius + self._StepWidth
        #self._alpha = self._StepLength/self._TurnRadius
        self._alpha = math.acos(1-self._StepLength**2/(2*self._TurnRadius**2))


    def GetStepData(self,index):
        stepData = BDI_Strategy.GetStepData(self,index)

        # a left step is in the inner circle
        r = self._InnerRadius if (index%2==0) else self._OuterRadius
        R = self._InnerRadius if (index%2==1) else self._OuterRadius

        # (Translate(dot)Rotate(dot)Translate)
        x = r*math.sin(self._alpha)
        y = R - r*math.cos(self._alpha)
        self._Odometer.AddLocalPosition(x,y)
        stepData.pose.position.x,stepData.pose.position.y = self._Odometer.GetGlobalPosition()

        self._Odometer.AddYaw(self._alpha)

        # Calculate orientation quaternion
        Q = quaternion_from_euler(0, 0, self._Odometer.GetYaw())

        stepData.pose.orientation.x = Q[0]
        stepData.pose.orientation.y = Q[1]
        stepData.pose.orientation.z = Q[2]
        stepData.pose.orientation.w = Q[3]

        return stepData

class BDI_StrategyRight(BDI_Strategy):
    """
    """
    def __init__(self,odometer):
        BDI_Strategy.__init__(self,odometer)

        self._TurnRadius = 1.5

        self._InnerRadius = self._TurnRadius - self._StepWidth
        self._OuterRadius = self._TurnRadius + self._StepWidth
        #self._alpha = self._StepLength/self._TurnRadius
        self._alpha = -math.acos(1-self._StepLength**2/(2*self._TurnRadius**2))


    def GetStepData(self,index):
        stepData = BDI_Strategy.GetStepData(self,index)

        # a left step is in the inner circle
        r = self._InnerRadius if (index%2==0) else self._OuterRadius
        R = self._InnerRadius if (index%2==1) else self._OuterRadius

        # (Translate(dot)Rotate(dot)Translate)
        x = r*math.sin(-self._alpha)
        y = r*math.cos(self._alpha) - R
        self._Odometer.AddLocalPosition(x,y)
        stepData.pose.position.x,stepData.pose.position.y = self._Odometer.GetGlobalPosition()

        self._Odometer.AddYaw(self._alpha)

        # Calculate orientation quaternion
        Q = quaternion_from_euler(0, 0, self._Odometer.GetYaw())

        stepData.pose.orientation.x = Q[0]
        stepData.pose.orientation.y = Q[1]
        stepData.pose.orientation.z = Q[2]
        stepData.pose.orientation.w = Q[3]

        return stepData

