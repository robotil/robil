# -*- coding: utf-8 -*-
"""

@author: polak
"""

from Node import node

class tskNode (node):
    def __init__(self,treeInst,mytree):
        node.__init__(self,treeInst,mytree,"tsk")
    
    def run (self):
        print ("please implemnt run func in tskNode")