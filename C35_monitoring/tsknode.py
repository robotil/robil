# -*- coding: utf-8 -*-
"""

@author: polak
"""

from Node import node



class TskNode (node):
    def __init__(self,treeInst,mytree,parent):
        node.__init__(self,treeInst,mytree,"tsk",parent)
        #self.getDistFromAttrib()                
        # not working yet..
        self.distTableSucc = self.createDistTable("Successdistribution")
        self.distTableFail = self.createDistTable("Failuredistribution")
        #self._readDebugFromAttrib()
        
        
        
        
        
    def run (self, index):

        debug = node.run(self, index)
        if (debug!=None):
            return debug  
            
        a = [True, 0]        
        a[0]= self.getRandomProb(index)
#        if (self.getNot()):
#            a[0] = not(a[0])        
        if a[0]:
            a[1] = round(self.getDistSuccByIndex(index).calcProb())
        else:
            a[1] = round(self.getDistFailByIndex(index).calcProb())   
        #print "Task:[%r %f]" %(a[0] ,a[1])    
        return a
        
    #override the node func- tsk dosn't have children   
    def getChild(self,index):
         return None
         
    #override the node func- tsk dosn't have children      
    def getChildren (self):
        return None
        
        
     #override the node func   
    def setDEBUGnode(self,sSucc=None,sTime=None):
        node.DEBUGnode(None,None)
        self.DEBUG = [sSucc,sTime]
        #would you like to update success ans time success?
    
#######################-----Adi changes(23/12/2012)-----####################

   
            