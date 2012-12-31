# -*- coding: utf-8 -*-
"""
ParallelNode class inherits from node class.
type of node- parallel
has it own run function
"""

from Node import node

class ParallelNode (node):
    def __init__(self,treeInst,mytree,parent):
        #call to super constracture
        node.__init__(self,treeInst,mytree,"par",parent)
    
    #run-parallel
    def run(self, index):
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
        
        a = [False, 0]       
        for i in self.getChildren():
            b = i.run(index)
            if (a[0]):
                if b[0]:
                    a[1] = (min(b[1], a[1]))
            else:
                if b[0]:
                    a[1] = (b[1])
                else:
                    a[1] = (max(b[1], a[1]))
            a[0] = b[0] or a[0]
            
        if (self.monitor):    
            if a[0]:
                self.setDistTableSuccAtIndex(tmpIndex, a[1])
            else:
                self.setDistTableFailAtIndex(tmpIndex, a[1])          
            self.updateProbTableAtIndex(tmpIndex, a[0])
        return a