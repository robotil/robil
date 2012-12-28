# -*- coding: utf-8 -*-
"""

@author: polak
"""
from lxml import etree
from Node import node

class xmlTree:
    #constructor
    def __init__(self, fileName=None ,root=None):
        if fileName != None:
            tree = etree.parse(fileName)
        #if we don't get a file to parse
        if root == None:
            self.root = tree.getroot()
        else:
            self.root= root
            
        
        self.fileName = fileName
        self.rootNode = node(self.root,self,self.root.tag)
        self._getUpdateTree()
                
    #this func build the wrap for all the tree.
    def _getUpdateTree(self):
        self.rootNode._updateEtreeToPrintXmlFile(self.rootNode)
            
    #print the tree to xml file. if non is given, prints to the original file
    def treeToXml(self,fileName = None):
        
        self. _getUpdateTree()
        
        strT = etree.tostring(self.root,pretty_print = True)
        if fileName != None :
            with open(fileName, "w") as text_file:
                text_file.write(strT)
        else:
            if self.fileName != None:
                with open(self.fileName, "w") as text_file:
                    text_file.write(strT)
                
    def boolFindNodeInTree(self,nodeToComp,node = None):
        if node == None:
            node = self.root
        for element in node.iter(nodeToComp.tag):
            if id(element) == id(nodeToComp):
                return True
                
        return False
        
    def getRoot(self):
        return self.rootNode
        
    
        
        
        
    
            
        
                
    
            
            
            
        
       