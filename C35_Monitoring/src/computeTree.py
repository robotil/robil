# -*- coding: utf-8 -*-
"""
Created on Fri Apr  5 13:11:54 2013

@author: adi
"""
from tree import xmlTree
from Node import node


class Ctree:
    myTree = None
    filePath =""
def __init__(self,inputTree):
    pass


def nodeDataInDebugMode(nodeTime, nodeSuccFail, nodeID, monitordNodeID, numOfIter):
    Ctree.myTree.createWrapperTreeMap("id")
   # print "E="+ str(Ctree.myTree.getWrappedNode(monitordNodeID).getAverageSuccTime(0))  
   # print "prob="+str(Ctree.myTree.getWrappedNode(monitordNodeID).getProbAtIndex(0))
    if monitordNodeID == "":
        monitordNode = Ctree.myTree.getRoot()
        monitordNode = monitordNode.getChild(0)
    else:
        monitordNode= Ctree.myTree.getWrappedNode(nodeID)
		
    finished_node = Ctree.myTree.getWrappedNode(nodeID)
    finished_node.setDebug(str(nodeSuccFail) + " " + str(nodeTime))
   # finished_node.clear()
    Ctree.myTree.treeToXml("output/run_temp.xml")
    node.debugMode = True
    Ctree.myTree  = xmlTree("output/run_temp.xml")
    root = Ctree.myTree.getRoot()
    for i in range(numOfIter):  
        			root.runPlan(0)
    Ctree.myTree.treeToXml("output/"+Ctree.filePath[6:-4]+"_after_run_debug_true.xml")
    Ctree.myTree  = xmlTree("output/"+Ctree.filePath[6:-4]+"_after_run_debug_true.xml")
    Ctree.myTree.createWrapperTreeMap("id")
    E =  Ctree.myTree.getWrappedNode(monitordNodeID).getAverageSuccTime(0)
    prob = Ctree.myTree.getWrappedNode(monitordNodeID).getProbAtIndex(0)
        
# Liat Here!  
    sd = monitordNode.getSDSuccTime(0)
    
    return (prob, sd, E)


def constructTree(event_file):
    # if the tree is not null then don't calculate the tree.
    if Ctree.myTree != None:
        return
    Ctree.filePath = event_file
#    event_file_full_path = event_file
    #tsk_attributes_full_path = event_file_full_path[0:-4] + "_tsk_attribs.xml"
    Ctree.myTree = xmlTree(Ctree.filePath, None,event_file[0:-4]+"_tsk_attribs.xml")
    Ctree.myTree.createWrapperTreeMap("id")
    root = Ctree.myTree.getRoot()
    
    node.debugMode = False	
    for i in range(1000):
        		root.runPlan(0)
    Ctree.myTree.treeToXml("output/"+Ctree.filePath[6:-4]+"_after_run_debug_false.xml")
    Ctree.myTree  = xmlTree("output/"+Ctree.filePath[6:-4]+"_after_run_debug_false.xml")
    Ctree.myTree.createWrapperTreeMap("id")
    
def getNodeInfo(nodeID):
    Ctree.myTree.createWrapperTreeMap("id")
    if nodeID == "":
        infoNode = Ctree.myTree.getRoot()
        infoNode =  infoNode.getChild(0)
    else:
        infoNode = Ctree.myTree.getWrappedNode(nodeID)
    E =  infoNode.getAverageSuccTime(0)
    prob = infoNode.getProbAtIndex(0)
        
# Liat Here!  
    sd =infoNode.getSDSuccTime(0)
    
    return (prob, sd, E)