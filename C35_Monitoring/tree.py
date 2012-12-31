# -*- coding: utf-8 -*-
"""
xmlTree wrap etree from lxml lib
hold the tree root.
read from xml file into a tree and write the tree to xml file after changes/calculations.
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
            
        #hold the name of the file.
        self.fileName = fileName
        #create a root node- type plan
        self.rootNode = node(self.root,self,self.root.tag)
        #update the whole tree- create the wrap for the whole tree.
        self._getUpdateTree()
                
    #this func build the wrap for all the tree.
    def _getUpdateTree(self):
        #update the tree from root.
        self.rootNode._updateEtreeToPrintXmlFile(self.rootNode)
            
    #print the tree to xml file. if non is given, prints to the original file
    def treeToXml(self,fileName = None):
        #update etree in order to print the updated value to xml
        self. _getUpdateTree()
        #turn the whole tree to an xml-string
        strT = etree.tostring(self.root,pretty_print = True)
        #if we didn't get a file to write to. will try to write to the file we read the tree from.
        if fileName != None :
            with open(fileName, "w") as text_file:
                text_file.write(strT)
        else:
        #print the tree to the fileName.
            if self.fileName != None:
                with open(self.fileName, "w") as text_file:
                    text_file.write(strT)
    #try to find a node in the tree. check for the same instance- uses python func- id.           
    def boolFindNodeInTree(self,nodeToComp,node = None):
        if node == None:
            node = self.root
        #node.iter- turn the tree into a list, and itereats over it.
        for element in node.iter(nodeToComp.tag):
            if id(element) == id(nodeToComp):
                return True
                
        return False
    #return a pointer to root wrapped, type- node- plan
    def getRoot(self):
        return self.rootNode
        
    
        
        
        
    
            
        
                
    
            
            
            
        
       