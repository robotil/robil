#!/usr/bin/env python

###################################################################################
# File created by David Dovrat, 2013.
# For Robil, under project Robil's license of choice
# The code in this file is provided "as is" and comes with no warranty whatsoever
###################################################################################

import tf

class Interface_tf(object):
    def __init__(self):
    	self._TransformListener = tf.TransformListener() #None
        self._TransformBroadcaster = tf.TransformBroadcaster() #None
 
    def TransformListener(self):
        if (None == self._TransformListener):
            self._TransformListener = tf.TransformListener()
        return self._TransformListener
    
    def TransformBroadcaster(self):
        if (None == self._TransformBroadcaster):
            self._TransformBroadcaster = tf.TransformBroadcaster()
        return self._TransformBroadcaster
