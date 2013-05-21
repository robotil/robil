#!/usr/bin/env python

from collections import deque

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

class StepQueue(object):
    def __init__(self):
    	pass

    def Initialize(self,step1,step2,step3,step4):
    	self._Queue = deque([step1,step2,step3,step4])

    def Push(self,step):
    	self._Queue.append(step)
    	return self._Queue.popleft()

    def Peek(self,i):
    	return self._Queue[i]


class PathQueue(object):
    def __init__(self):
    	self._Queue = deque([])

    def Initialize(self):
    	self._Queue = deque([])

    def Append(self,steps):
    	for i in range(len(steps)):
    		self._Queue.append(steps[i])

    def Pop(self):
    	return self._Queue.popleft()

    def Push(self,step):
    	self._Queue.append(step)
    	return self._Queue.popleft()

    def Peek(self,i):
    	return self._Queue[i]

    def Length(self):
        return len(self._Queue)
    
