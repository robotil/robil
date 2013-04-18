#!/usr/bin/env python

###############################################################################
####                                                                         ##
####  preview_buffer.py                                                      ##
####  Created - Yuval 07/2/2013                                              ##
####  last updated - version 1.0, Yuval 07/2/2013                            ##
####                                                                         ##
####    ZMP Preview Buffer:   ##
####                                          ##
####                                                                         ##
####    Used to produce the current (updated) ZMP refernce to the preview    ##
####    sagital (x) and lateral (y) plans.                                    ##
####                                                                         ##
####                                                                         ##
###############################################################################

import roslib; roslib.load_manifest('C42_ZMPWalk')
import rospy, sys, os.path
from pylab import *


# # Buffer_Data class: an auxiliary class to be used by class ZMP_Preview_Buffer as a list of objects
# class Buffer_Data:
#     'Buffer_Data - Saves data about a template that is used in the ZMP_Preview_Buffer'
#     # State parameters:

#     def __init__(self, name, template_type,  number_of_samples, step_size, first_value, last_value ):
#         self._name = name



class ZMP_Preview_Buffer:
    'Preview Buffer - Synchronizes and buffer the ZMP profile templates to produce the current (updated) ZMP refernce to the preview controller '
    
    def __init__(self, name, preview_sample_size, max_step_samples, precede_time_samples ):
        self._name = name
        self._preview_size = ceil (preview_sample_size)   # Number of samples in preview 
        self._buffer_size = ceil (preview_sample_size + max_step_samples)                           
        self._buffer = zeros(preview_sample_size + max_step_samples) # init buffer
        self._precede_time_samples = ceil (precede_time_samples) # number of samples to bring forward the step profile
        self._start_index = 0 # points to the current position in the buffer where the preview starts. We increment start_index on each time cycle
        self._data_end = 0 # start_index to where the data in the buffer ends (from where new data can be written into the buffer)   
        self._end_of_step_preview = self._preview_size
        self._NewStep_trigger = False # trigger to indicate that a new step was loaded

    # Handling Buffer Stack:
    def clearBuffer(self):
        self._buffer = zeros(self._buffer_size) # clear buffer
        self._start_index = 0
        self._data_end = 0

    def pushData(self,new_data):
        data_length = len(new_data)
        total_data_size = self._data_end + data_length
        if total_data_size <= self._buffer_size:  # if new data fits into buffer
            self._buffer[self._data_end : total_data_size] = new_data
            self._data_end = total_data_size
        else:  # load part of new data according to the available space
            self._buffer[self._data_end : self._buffer_size] = new_data[0 : self._buffer_size - self._data_end]
            self._data_end = self._buffer_size
            rospy.loginfo("ZMP_Preview_Buffer %s: unable to load all data because buffer is full. Buffer size = data_end = %f" % (self._name, self._data_end) )

    def buffer_Is_notFull(self):
        if self._data_end < self._buffer_size:  # if the is available space in buffer
            return (True)
        else:  
            return (False)


    def load_NewStep(self, new_step, following_steps_cycle): 
        # new_step is the step that we are starting to perform now.
        # following_steps_cycle is a/are step/s to perform after new_step that will repeat 
        #  them selves until filling the required space in the buffer.
        # is handled by off_set <-- !!!Attention!!! continuity should be kept from new_step to 
        #        following_steps_cycle and between begining and end of following_steps_cycle      
        step_length = len(new_step)
        self._end_of_step_preview = step_length+self._preview_size

        #rospy.loginfo("ZMP_Preview_Buffer %s: end_of_step_preview = %f, buffer_size = %f, preview_size = %f, precede_time_samples = %f " % (self._name, self._end_of_step_preview, self._buffer_size, self._preview_size, self._precede_time_samples) )

        self.clearBuffer() # clear buffer
        #rospy.loginfo("ZMP_Preview_Buffer %s: data_end = %f, step_length = %f" % (self._name, self._data_end, step_length) )

        self.pushData(new_step[self._precede_time_samples :]) # Insert into buffer new step with (lead) time shift  
        # Insert into buffer cycles of step sequence (need to fill buffer with at least step_length+preview_size samples)
        while self._data_end <= (self._end_of_step_preview) and self.buffer_Is_notFull():
            off_set = self._buffer[self._data_end-1] - 2*following_steps_cycle[0] + following_steps_cycle[1]# we add off_set to following_steps_cycle to insure continuity
            self.pushData( following_steps_cycle + off_set )
            #rospy.loginfo("ZMP_Preview_Buffer %s: data_end = %f" % (self._name, self._data_end) )

        self._NewStep_trigger = True

        return

    def load_PreviewStep(self, new_preview_step): 
        # Loads new preview of (future) step into end of buffer according to the following steps:
        # 1) Shifts preview of previous step (last_preview) to begining of buffer. (last preview becomes current step to be excuted)
        # 2) Load buffer with new_preview_step after last_preview.

        end_index = self._start_index + self._preview_size
        last_preview = self._buffer[self._start_index : end_index]

        self.clearBuffer() # clear buffer
        self.pushData( last_preview )
        following_preview = new_preview_step[self._precede_time_samples :] # new_preview_step with (lead) time shift
        off_set = self._buffer[self._data_end-1] - 2*following_preview[0] + following_preview[1]# we add off_set to following_preview to insure continuity
        self.pushData( following_preview + off_set )

        self._NewStep_trigger = True

        return

    def update_Preview(self): 
        # return current step sequence preview and increment preview (one time cycle)
        end_index = self._start_index + self._preview_size
        if end_index > self._end_of_step_preview: # make sure not to exceed steps preview
            # may want to do some thing alse in this case like: preview = zeros(self._preview_size) ???
            end_index = self._end_of_step_preview # freeze preview (preview is not updated, stays on end_of_step_preview values)
            self._start_index = self._end_of_step_preview - self._preview_size
        if end_index > self._buffer_size: # make sure not exceed buffer size
            end_index = self._buffer_size # freeze preview (preview is not updated, stays on end of buffer values)
            self._start_index = self._buffer_size - self._preview_size

        preview = self._buffer[self._start_index : end_index]
        # if self._start_index > 675:
        #     rospy.loginfo("ZMP_Preview_Buffer %s: start_index = %f, end_index = %f " % (self._name, self._start_index, end_index ) )

        if self._start_index >= 1 : # On the second update of preview (after one loop iteration) the trigger of loading a new step is returned to false
            self._NewStep_trigger = False

        self._start_index += 1

        return preview,self._NewStep_trigger