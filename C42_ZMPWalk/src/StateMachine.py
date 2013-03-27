#!/usr/bin/env python

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

class StateMachineError(Exception):
    """
        For information as to why the exception was raised, see StateMachineError.Message attribute
    """
    def __init__(self,strMessage):
        self.Message = strMessage
    
class State(object):
    """
        The State class is intended to be used with the StateMachine class,
        to be more precise, subclasses of State should be used as a strategy design pattern for subclasses of StateMachine
    """
    def __init__(self,strStateName):
        self.Name = strStateName
        
    def OnEnter(self):
        pass
    def OnExit(self):
        pass

class StateMachine(object):
    """
        The StateMachine class is an abstraction of a finite state machine
    """
    
    def __init__(self,stateDefaultState):
        if(False == isinstance(stateDefaultState,State)):
            raise StateMachineError("Default State not an instance of class State")
        self._States = {stateDefaultState.Name:stateDefaultState}
        self._Transitions = {}
        self._CurrentState = stateDefaultState
    
    def GetCurrentState(self):
        """
            Returns the current state
        """
        return self._CurrentState
    
    def GetStateAtTransition(self,strStartState,strTransition):
        """
            Returns the state transitioned to from strStartState by strTransition
        """
        if((strStartState,strTransition) in self._Transitions):
            state = self._States[self._Transitions[(strStartState,strTransition)]]
        else:
            state = self._States[strStartState]
        return state
    
    def AddState(self,cState):
        """
            Add cState to the state machine
            Throws StateMachineError if cState already exists in the state machine
        """
        if(cState.Name in self._States):
            raise StateMachineError(cState.Name + " already exists in state machine")
        self._States[cState.Name] = cState
    
    def AddTransition(self,strStartState,strTransition,strEndState):
        """
            Add transition strTransition to the state machine from state strStartState to state strEndState
            Throws StateMachineError if strStartState already has a strTransition transition
        """
        if((strStartState,strTransition) in self._Transitions):
            raise StateMachineError(strStartState + " already has a " + strTransition + " transition")
        self._Transitions[(strStartState,strTransition)] = strEndState
        
    def PerformTransition(self,strTransition):
        """
            Perform transition strTransition
        """
        if((self._CurrentState.Name,strTransition) in self._Transitions):
            self._CurrentState.OnExit()
            self._CurrentState = self._States[self._Transitions[(self._CurrentState.Name,strTransition)]]
            self._CurrentState.OnEnter()

# a little testing script
if __name__ == "__main__":
    Machine = StateMachine(State("State_1"))
    print Machine.GetCurrentState().Name
    Machine.AddState(State("State_2"))
    try:
        Machine.AddTransition("State_1", "1->2", "State_2")
        Machine.AddTransition("State_2", "2->1", "State_1")
    except StateMachineError as error:
        print error.Message
    Machine.PerformTransition("1->2")
    print Machine.GetCurrentState().Name
    print Machine.GetStateAtTransition("State_2", "2->1").Name
