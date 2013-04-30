#!/usr/bin/env python

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

class WalkingMode(object):
    
    def __init__(self):
      self._rate = 100  # [Hz]
        
    def Initialize(self):
        pass
    
    def StartWalking(self):
        return self._rate
    
    def Walk(self):
        return true
    
    def Stop(self):
        pass
    
    def EmergencyStop(self):
        pass    

    def IsDone(self):
        return False
