#!/usr/bin/env python

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

from Abstractions.PathPlanner import *
from Abstractions.Odometer import *

from CD_PathPlanner import *

from atlas_msgs.msg import AtlasSimInterfaceCommand
from tf.transformations import quaternion_from_euler

###################################################################################
#----------------------------------- CD Bot ---------------------------------------
###################################################################################

class CD_Robot(object):
    """
        The CD_ActualRobot represents the real actual robot as it walks on the path
    """
    def __init__(self,pathPlanner,stepQueue):
        self._LPP = pathPlanner
        self._StepQueue = stepQueue

    def Initialize(self,stepQueue):
        self._StepQueue = stepQueue

    def SetPosition(self,x,y):
        self._LPP.UpdatePosition(x,y)

    def SetPath(self,path):
        self._LPP.SetPath(path)

###################################################################################
#-------------------------------- Actual Bot --------------------------------------
###################################################################################

class CD_ActualRobot(CD_Robot):
    """
        The CD_ActualRobot represents the real actual robot as it walks on the path
    """
    def __init__(self,pathPlanner,stepQueue):
        CD_Robot.__init__(self,pathPlanner,stepQueue)
        self._Yaw = 0

    def Step(self):
        command = AtlasSimInterfaceCommand()
        command.behavior = AtlasSimInterfaceCommand.WALK
        for i in range(4):
            command.walk_params.step_queue.append(self._StepQueue[i])
        self._StepQueue.popleft()
        return command
    

    def IsEndOfPath(self):
        return self._LPP.IsEndOfPath()
    
    def GetPathError(self):
        return self._LPP.GetPathError()
    
    def SetYaw(self,yaw):
        self._Yaw = yaw
    
###################################################################################
#-------------------------------- Phantom Bot -------------------------------------
###################################################################################

class CD_PhantomRobot(CD_Robot):
    """
        The CD_PhantomRobot represents the future robot as it walks on the path in the future
    """
    def __init__(self,pathPlanner,odometer,stepQueue):
        CD_Robot.__init__(self,pathPlanner,stepQueue)
        self._Odometer = odometer
        self._index = 0
        
        # Parameters
        self._StepLength = 0.3
        self._StepWidth = 0.15
        self._Duration = 0.63
        self._SwingHeight  = 0.2
        self._MinimalCorrection = 0.02
        self._ErrorCorrection = self._StepWidth

    def Initialize(self,stepQueue,index):
        CD_Robot.Initialize(self,stepQueue)
        self._index = index
        self._Error = 0

    def Step(self):
        self._StepQueue.append(self._ForwardStep())

    def EndOfSegment(self):
        return self._LPP.IsEndOfSegment()

    def PrepareNextSegment(self):
        if (self._LPP.IsEndOfPath()):
            self._AddIdleSteps()
        else:
            self._Turn()
            self._AddIdleSteps()
            self._LPP.PromoteSegment()

    def GetIndex(self):
        self._index 
    
    def SetPathError(self,error):
        self._Error = error

    def SetYaw(self,yaw):
        self._Odometer.SetYaw(yaw)
        
    def _PrepareStepData(self):
        self._index  += 1
        command = AtlasSimInterfaceCommand()
        
        stepData = command.walk_params.step_queue[0]
        stepData.step_index = self._index
        stepData.foot_index = self._index%2

        stepData.duration = self._Duration
        stepData.swing_height = self._SwingHeight

        stepData.pose.position.z = 0.0
        
        return stepData
        
    def _ForwardStep(self):
        stepData = self._PrepareStepData()        
        # Correct Lateral Error
        errorCorrected = 0
        # Correct only when needed
        if (math.fabs(self._Error) > self._MinimalCorrection):   
            if(math.fabs(self._Error) < self._ErrorCorrection):
                errorCorrected = -self._Error/4
            else:
                errorCorrected = -self._ErrorCorrection*(self._Error/math.fabs(self._Error))/4 

        x = self._StepLength
        y = self._StepWidth if (self._index%2==0) else -self._StepWidth
        y += errorCorrected
        self._Odometer.AddLocalPosition(x,y)
        stepData.pose.position.x,stepData.pose.position.y = self._Odometer.GetGlobalPosition()

        # Calculate orientation quaternion
        Q = quaternion_from_euler(0, 0, self._Odometer.GetYaw())

        stepData.pose.orientation.x = Q[0]
        stepData.pose.orientation.y = Q[1]
        stepData.pose.orientation.z = Q[2]
        stepData.pose.orientation.w = Q[3]

        return stepData
                
    def _IdleStep(self):
        stepData = self._PrepareStepData()
        
        x = 0
        y = self._StepWidth if (self._index%2==0) else -self._StepWidth
        self._Odometer.AddLocalPosition(x,y)
        stepData.pose.position.x,stepData.pose.position.y = self._Odometer.GetGlobalPosition()

        # Calculate orientation quaternion
        Q = quaternion_from_euler(0, 0, self._Odometer.GetYaw())

        stepData.pose.orientation.x = Q[0]
        stepData.pose.orientation.y = Q[1]
        stepData.pose.orientation.z = Q[2]
        stepData.pose.orientation.w = Q[3]
        
        return stepData
    
    def _AddIdleSteps():
        for i in range(4):
            self._StepQueue.append(self._IdleStep())
            
    def _Turn(self):
        pass
    