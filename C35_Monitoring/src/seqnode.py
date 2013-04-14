# -*- coding: utf-8 -*-
"""
SeqNode class inherits from node class.
type of node- seq
has it own run function
"""
from Node import node

class SeqNode (node):
    
    def __init__(self,treeInst,mytree,parent):
        #call to super constracture
        node.__init__(self,treeInst,mytree,"seq",parent)
    
    #seq- run
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
        for i in self.getChildren():                     
            b = i.run(index)  
            a[0] = a[0] and b[0]
            a[1] = a[1] + b[1]           
            if not b[0]:	  
                break
            
        if (self.monitor):    
            if a[0]:
                self.setDistTableSuccAtIndex(tmpIndex, a[1])
            else:
                self.setDistTableFailAtIndex(tmpIndex, a[1])    
            self.updateProbTableAtIndex(tmpIndex, a[0])
        return a    
        
        
        