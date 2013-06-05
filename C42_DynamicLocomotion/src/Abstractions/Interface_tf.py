#!/usr/bin/env python

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

import tf

class Interface_tf(object):
    def __init__(self):
    	self._TransformListener = tf.TransformListener()
        self._TransformBroadcaster = tf.TransformBroadcaster()
 
    def TransformListener(self):
        return self._TransformListener
    
    def TransformBroadcaster(self):
        return self._TransformBroadcaster
