#!/usr/bin/env python

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

from Abstractions.StateMachine import *
from Abstractions.Odometer import *

from CD_Bots import *

from collections import deque

###################################################################################
#--------------------------- CD State Machine -------------------------------------
###################################################################################

class CD_StateMachine(StateMachine):
    """
        The CD_StateMachine class is a finite state machine that handles the stepping logic for the CD walking mode
    """
    
    def __init__(self,pathPlanner):

        #StateMachine
        StateMachine.__init__(self,State('Idle'))
        self._bIsDone = False

        self._StepQueue = deque([])
        self._actualRobot = CD_ActualRobot(pathPlanner,self._StepQueue)
        self._phantomRobot = CD_PhantomRobot(CD_PathPlanner(),Odometer(),self._StepQueue)

        # Add states
        StateMachine.AddState(self,State('WaitingForPhantom'))
        StateMachine.AddState(self,State('BothWalking'))
        StateMachine.AddState(self,State('WaitingForActual'))
        StateMachine.AddState(self,State('WindDown'))
        
        # Add transitions
        StateMachine.AddTransition(self,'Idle',                 'Start',        'WaitingForPhantom')
        StateMachine.AddTransition(self,'WaitingForPhantom',    'Both',         'BothWalking')
        StateMachine.AddTransition(self,'BothWalking',          'Actual',       'WaitingForActual')
        StateMachine.AddTransition(self,'WaitingForActual',     'WindingDown',  'WindDown')
        StateMachine.AddTransition(self,'WaitingForActual',     'Both',         'BothWalking')
        StateMachine.AddTransition(self,'WindDown',             'Done',         'Idle')

    def Start(self):
        self._phantomRobot.AlignToPath()
        if (StateMachine.PerformTransition(self,'Start')):
            print("CD_StateMachine::Start")

    def Step(self):
        command = 0
        if ('Idle' == self._CurrentState.Name):
            #print("CD StateMachine - Idle")
            pass
        elif ('WaitingForPhantom' == self._CurrentState.Name):
            #print("CD StateMachine - WaitingForPhantom")
            self._phantomRobot.Step()
            if (3 < len(self._StepQueue)):
                if (StateMachine.PerformTransition(self,'Both')):
                    print("CD_StateMachine::BothWalking")
                else:
                    raise StateMachineError("CD_StateMachine::Step() - could not perform transition 'Both'") 
        elif ('BothWalking' == self._CurrentState.Name):
            #print("CD StateMachine - BothWalking")
            self._phantomRobot.SetPathError(self._actualRobot.GetPathError())
            self._phantomRobot.Step()
            command = self._actualRobot.Step()
            if (self._phantomRobot.IsEndOfSegment()):
                self._phantomRobot.PrepareNextSegment()
                if (StateMachine.PerformTransition(self,'Actual')):
                    print("CD_StateMachine::WaitingForActual")
                else:
                    raise StateMachineError("CD_StateMachine::Step() - could not perform transition 'Actual'") 
        elif ('WaitingForActual' == self._CurrentState.Name):
            print("CD StateMachine - WaitingForActual")
            print(len(self._StepQueue))
            if (5 > len(self._StepQueue)):
                if (self._actualRobot.IsEndOfPath()):
                    if (StateMachine.PerformTransition(self,'WindingDown')):
                        print("CD_StateMachine::WindingDown")
                        self._phantomRobot.AddFinalSteps()
                    else:
                        raise StateMachineError("CD_StateMachine::Step() - could not perform transition 'WindingDown'")
                else:
                    self._actualRobot.PrepareNextSegment()
                    if (StateMachine.PerformTransition(self,'Both')):
                        print("CD_StateMachine::BothWalking")
                    else:
                        raise StateMachineError("CD_StateMachine::Step() - could not perform transition 'Both'")
            else:
                command = self._actualRobot.Step()
        elif ('WindDown' == self._CurrentState.Name):
            print("CD StateMachine - WindDown")
            command = self._actualRobot.Step()
            print(len(self._StepQueue))
            if (1 >= len(self._StepQueue)):
                if (StateMachine.PerformTransition(self,'Done')):
                    print("CD_StateMachine::Done")
                    self._bIsDone = True
                else:
                    raise StateMachineError("CD_StateMachine::Step() - could not perform transition 'Done'")
        else:
            raise StateMachineError("CD_StateMachine::Step() - unknown state")
        return command

    def Initialize(self,index):
        self._StepQueue = deque([])
        self._actualRobot.Initialize(self._StepQueue,index)
        self._phantomRobot.Initialize(self._StepQueue,index)
        print("Initialize to index ",index)
        self._bIsDone = False

    def IsDone(self):
        return self._bIsDone

    def UpdateOdometryPosition(self,x,y):
        self._actualRobot.SetPosition(x,y)
        self._OdomX = x
        self._OdomY = y

    def UpdateOdometryYaw(self,yaw):
        self._actualRobot.SetYaw(yaw)
        self._OdomYaw = yaw

    def SetStatePosition(self,x,y):
        self._phantomRobot.SetPosition(self._OdomX,self._OdomY)
        self._phantomRobot.SetPosDelta(x-self._OdomX,y-self._OdomY)
        
    def SetStateYaw(self,yaw):
        self._phantomRobot.SetYaw(self._OdomYaw)
        self._phantomRobot.SetYawDelta(yaw-self._OdomYaw)

    def SetPath(self,p):
        self._phantomRobot.SetPath(p)
        self._actualRobot.SetPath(p)

    def GetIndex(self):
        return self._actualRobot.GetIndex()
