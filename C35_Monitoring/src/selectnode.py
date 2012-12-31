# -*- coding: utf-8 -*-
"""
SelectNode class inherits from node class.
type of node- sel
has it own run function
"""

from Node import node

class SelectNode (node):
    def __init__(self,treeInst,mytree,parent):
        #call to super constracture
        node.__init__(self,treeInst,mytree,"sel",parent)
    
    #run-select
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
       
        a = [False, 0]
        for i in self.getChildren():                        
            b = i.run(index)           
            a[0] = a[0] or b[0]
            a[1] = a[1] + b[1]
            if b[0]:	  
                break
            

            
        if (self.monitor):    
            if a[0]:
                self.setDistTableSuccAtIndex(tmpIndex, a[1])
            else:
                self.setDistTableFailAtIndex(tmpIndex, a[1])       
            self.updateProbTableAtIndex(tmpIndex, a[0]) 
        return a    