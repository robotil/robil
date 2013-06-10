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
        
        # Add transitions
        StateMachine.AddTransition(self,'Idle',                 'Start',        'WaitingForPhantom')
        StateMachine.AddTransition(self,'WaitingForPhantom',    'Both',         'BothWalking')
        StateMachine.AddTransition(self,'BothWalking',          'Actual',       'WaitingForActual')
        StateMachine.AddTransition(self,'WaitingForActual',     'None',         'Idle')
        StateMachine.AddTransition(self,'WaitingForActual',     'Both',         'BothWalking')

    def Start(self):
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
            if (self._phantomRobot.EndOfSegment()):
                self._phantomRobot.PrepareNextSegment()
                if (StateMachine.PerformTransition(self,'Actual')):
                    print("CD_StateMachine::WaitingForActual")
                else:
                    raise StateMachineError("CD_StateMachine::Step() - could not perform transition 'Actual'") 
        elif ('WaitingForActual' == self._CurrentState.Name):
            print("CD StateMachine - WaitingForActual")
            command = self._actualRobot.Step()
            if (5 > len(self._StepQueue)):
                if (self._actualRobot.IsEndOfPath()):
                    if (StateMachine.PerformTransition(self,'None')):
                        print("CD_StateMachine::Idle")
                        self._bIsDone = True
                    else:
                        raise StateMachineError("CD_StateMachine::Step() - could not perform transition 'None'")
        else:
            raise StateMachineError("CD_StateMachine::Step() - unknown state")
        return command

    def Initialize(self,index):
        self._StepQueue = deque([])
        self._actualRobot.Initialize(self._StepQueue)
        self._phantomRobot.Initialize(self._StepQueue,index)
        self._bIsDone = False

    def IsDone(self):
        return self._bIsDone

    def UpdateActualPosition(self,x,y):
        self._actualRobot.SetPosition(x,y)

    def UpdateActualYaw(self,yaw):
        self._actualRobot.SetYaw(yaw)

    def SetPhantomPosition(self,x,y):
        self._phantomRobot.SetPosition(x,y)

    def SetPhantomYaw(self,yaw):
        self._actualRobot.SetYaw(yaw)

    def SetPath(self,p):
        self._phantomRobot.SetPath(p)
        self._actualRobot.SetPath(p)

    def GetIndex(self):
        return self._phantomRobot.GetIndex()
