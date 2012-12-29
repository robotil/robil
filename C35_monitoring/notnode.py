# -*- coding: utf-8 -*-
"""
Created on Thu Dec 13 15:44:32 2012

@author: polak
"""


from Node import node

class NotNode (node):
    def __init__(self,treeInst,mytree,parent):
        node.__init__(self,treeInst,mytree,"not",parent)

    
    def run (self, index):
        tmpIndex = index 
        if (node.debugMode):
            if not(self.hasDebugChild()):
                res = self.runAsBaseCase(index)
                if res!=None:
                    return res
            else:
                if not(self.reset):
                    self.clear()
        
        debug = node.run(self, index)
        if (debug!=None):
            return debug             
        
        a = [True, 0]
#        index = 1
        child = self.getChildren()
        b = child[0].run(index)
        a[0] = not(b[0])
        a[1] = b[1]

            
        if (self.monitor):    
            if a[0]:
                self.setDistTableSuccAtIndex(tmpIndex, a[1])
            else:
                self.setDistTableFailAtIndex(tmpIndex, a[1])    
            self.updateProbTableAtIndex(tmpIndex, a[0])
        return a    