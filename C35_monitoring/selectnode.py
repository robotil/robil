# -*- coding: utf-8 -*-
"""

@author: polak
"""

from Node import node

class SelectNode (node):
    def __init__(self,treeInst,mytree,parent):
        node.__init__(self,treeInst,mytree,"select",parent)
    
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
            #self.setSucc(a[0])
            a[1] = a[1] + b[1]
            #self.setTime(a[1])
            if b[0]:	  
                break
            
#        if (self.getNot()):
#            a[0] = not(a[0]) 
            
        if (self.monitor):    
            if a[0]:
                self.setDistTableSuccAtIndex(tmpIndex, a[1])
            else:
                self.setDistTableFailAtIndex(tmpIndex, a[1])       
            self.updateProbTableAtIndex(tmpIndex, a[0]) 
        return a    