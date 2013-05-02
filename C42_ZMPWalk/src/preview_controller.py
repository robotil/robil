#!/usr/bin/env python

###############################################################################
####                                                                         ##
####  preview_controller.py (based on zmp_main(old).py)                      ##
####  Created - Yuval 04/2/2013                                              ##
####  last updated - version 1.0, Yuval 05/2/2013                            ##
####                                                                         ##
####    ZMP Preview Controller:   ##
####                       .                   ##
####                                                                         ##
####    Used for sagital (x) and lateral (y) control                         ##
####                                                                         ##
####                                                                         ##
###############################################################################

import roslib; roslib.load_manifest('C42_ZMPWalk')
import rospy, sys, os.path
from pylab import *
import copy



class ZMP_Preview_Controller:
    'Preview controller - input: ZMP reference trajectory, output: COM reference trajectory; based on Linear Inverted Pendulum Model'
    # Class parameters:

    def __init__(self, name, parameters_folder_name, COM ):
        self.name = name
        self._initCOM = COM
        # rospy.loginfo("ZMP_Preview_Controller: init %s controller" % (self.name) )
        # rospy.sleep(1)

        self.init_values()
        
        self._COM_at_step_begining = copy.copy(self.x[0]) # accumulated COM until begining of step. Used to return relative COM for each step that starts from zero.   

        # Load Controller Parameters
        [pathname, Script_File_Name] = os.path.split(sys.argv[0])
        [Sub_pathname, Script_Folder] = os.path.split(pathname)

        parameters_path = os.path.join(Sub_pathname, r"src/parameters/", parameters_folder_name + '/')
        # rospy.loginfo("ZMP_Preview_Controller %s: pathname = %s, Script_Folder = %s, Sub_pathname = %s, parameters path = %s" %\
        #  (self.name, pathname, Script_Folder, Sub_pathname, parameters_path + 'A.txt') )

        #self.A = genfromtxt('/home/yuval/Projects/Robil/C42_ZMPWalk/src/parameters/sagital_x/A.txt') #parameters_path + 'A.txt')
        self.A = genfromtxt(parameters_path + 'A.txt')
        self.B = genfromtxt(parameters_path + 'B.txt')
        self.C = genfromtxt(parameters_path + 'C.txt')

        self.Gi = genfromtxt(parameters_path + 'Gi.txt')
        self.Gx = genfromtxt(parameters_path + 'Gx.txt')
        self.Gd = genfromtxt(parameters_path + 'Gd.txt')
        self.BufferSize = genfromtxt(parameters_path + 'NL.txt') + 1

        #self.PreviewBuffer = zeros( self.BufferSize )

        # for i in range(len(self.Gd)) :
        #     rospy.loginfo("Gd coefficient in place %i) %.8f" % (i, self.Gd[i]) )
        # # Wait 1 second
        # rospy.sleep(1)

    def init_values(self):
        # assumption: we start movement from a static position => COM = ZMP point
        self.x = array([self._initCOM , 0.0 , 0.0])[:,newaxis]                              
        self.p = self._initCOM # initial_ZMP_point
        self.sum_e = 0.0 # sum of ZMP error


    def getBufferSize(self):
        return (self.BufferSize)

    def getAccumulatedCOM_ref (self):
        return (self.x[0])   

    def getCOM_ref(self, p_ref_Buffer, loaded_new_step_trigger): 
        # returns COM relative to position of COM at begining of step (COM starts from zero for each step)  

        if loaded_new_step_trigger:
            self._COM_at_step_begining = copy.copy(self.x[0])
        
        SumGd = dot(self.Gd, p_ref_Buffer[1:self.BufferSize])
 
        e = self.p - p_ref_Buffer[0]
  
        self.sum_e = self.sum_e + e
  
        u =  - self.Gi*self.sum_e  - SumGd - dot( self.Gx , self.x )
        
        self.x = dot(self.A, self.x) + (self.B*u)[:,newaxis] # COM state: x[0]-position, x[1]-velocity x[2]-acceleration
        
        self.p = dot(self.C, self.x) # scalar, ZMP point (of model)

        return (self.x[0]-self._COM_at_step_begining, self.x[1], self.p) 