#!/usr/bin/env python

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

class WalkingModeChooserInterface(object):

    def __init__(self):
        pass
        
    def IsCurrentModeAppropriate(self):
        return true;
    
    def GetRecommendedMode(self):
        pass
        