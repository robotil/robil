# -*- coding: utf-8 -*-
"""

@author: polak
"""
import sys
from lxml import etree


class node:
    #constractur- treeInstance-node in the etree, the etree itself, and prep-type(seq,plan etc.)
    def __init__(self,treeInstance = None,mytree = None,prep="plan"):
        if treeInstance.tag != prep :
            print ( "error. check node")
            
        # you can't have multiple __init__ functions in Python so we use mytree = None
        if mytree == None :
            self.treeInst =  etree.Element("plan")
            self.myTree = self.treeInst
        else:
            self.myTree = mytree
            self.treeInst = treeInstance
    
   
   #return parent. if it's the root- return None   
    def getParent(self):
        return self.treeInst.getparent()

    #return childs-list
    def getChildList(self):
        return list(self.treeInst)
    
    #get branch-factor        
    def getBF(self):
        return (len(self.treeInst))
    
    #input:string-tagtype, create a new node with tag-type and add it to the node direct children
    #output - return the newNode
   def addNode(self,tag):
       return  self.createChildByTag(etree.SubElement(self.treeInst,tag))
        
       # while True :
        #    print("would you like to add attributes to the new node?(Y/N)")
         #   ans = sys.stdin.read(1)
          #  if ans == 'N':
           #     break
            
            #parm = raw_input("Enter parm Name")
            #value =  raw_input("Enter parm value :")
            
            #child.attrib[parm]=value
            
    #input: parmeter and his value, add parm and set value or just set value  
    def setAttrib(self,parm,value):
        self.treeInst.attrib[parm] = value
    #input: paramter name. output: return the value as a string or None   
    def getAttrib(self,attribParm):
        return self.treeInst.get(attribParm)
    #input: node, output: boolean if this node is monitore        
    def isMonitored (self):
        return (self.treeInst.tag == "monitor")
    #input: node, output: boolean if this node is not
    def isNot (self):
        return (self.treeInst.tag == "not")
    #input: node, output: boolean if this node is tsk   
    def isTask(self):
        return(self.treeInst.tag == "tsk")
    #return list of the node children
    def getchildren (self):
        return self.createChildList()
                
        
    #create the childs list    
    def createChildList(self):
        childlist = []
        for element in list(self.treeInst):
            childlist.append(self.createChildByTag(element))
            
        return childlist
    
    #output: disType as a string, or None
    def getDisttribuationType(self):
        return self.getAttrib("distribution")
    
    #output: return prob string or None
    def getProbString(self):
        prob = self.getAttrib("probability")
        if prob == None :
            print ("can't get probability for")
            print (self.treeInst.tag)
            print (self.getAttrib("name"))
            print("please check if there is a probString")
        return prob
    
    #input: child num in the list , output: a new child now- not a deepcopy    
    def getChild(self,number):
        if number > self.getBF():
            print ("there is no such a child number")
      
        return self.createChildByTag(self.treeInst[number] )
        
        
    #run the node. each subclass should implement this
    def run (self):
        raise NotImplementedError("Subclasses should implement this!")    

    #input xml tree elem, create the node wrap    
    def createChildByTag(self,elem):
        #create the new node accordint to the type
        if elem.tag == "seq":
            from seqnode import seqNode
            return seqNode(elem,self.myTree)
        if elem.tag == "tsk":
            from tsknode import tskNode
            return tskNode(elem,self.myTree)
        if elem.tag == "monitor":
            from monitornode import monitorNode
            return monitorNode(elem,self.myTree)
        if elem.tag == "loop":
            from loopnode import loopNode
            return loopNode(elem,self.myTree)
            #need to continue implementing the rest..