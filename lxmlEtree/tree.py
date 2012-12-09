# -*- coding: utf-8 -*-
"""

@author: polak
"""
from lxml import etree
from Node import node

class xmlTree:
    #constructor
    def __init__(self,fileName):
        tree = etree.parse(fileName)
        self.root = tree.getroot()
        self.fileName = fileName
        self.rootNode = node(self.root,self,self.root.tag)
        
    #print the tree to xml file. if non is given, prints to the original file
    def treeToXml(self,fileName = None):
        strT = etree.tostring(self.root,pretty_print = True)
        
       
        if fileName != None :
            with open(fileName, "w") as text_file:
                text_file.write(strT)
        else:
             with open(self.fileName, "w") as text_file:
                text_file.write(strT)
                
    def boolFindNodeInTree(self,nodeToComp,node = None):
        if node == None:
            node = self.root
        for element in node.iter(nodeToComp.tag):
            if element == nodeToComp:
                return True
                
        return False
        
    def getRoot(self):
        return self.rootNode
        
    
        
        
        
    
            
        
                
    
            
            
            
        
       