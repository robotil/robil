from tree import xmlTree
from Node import node





#please run test 3 before test 5: - check attrib func/method    

 #check the monitor set/get func/method   
def test6():
    tree = node()
    root = tree
    #first child
    firstChild = root.addNode("par")
    if firstChild == None:
        print ("error creating par node")
        print("test 7: failed :-(")
        return None
    dist_succ = _createUniformDist(2,5)
    dist_fail = _createUniformDist(6,10)   
    
    firstChild.DEBUGchild = True 
    for j in range(3): 
      if j==0:  
          tempN = firstChild.addNode("seq")
          if tempN == None:
              print ("error creating seq node")
      if j==1:  
          tempN = firstChild.addNode("sel")
          if tempN == None:
              print ("error creating sel node")
      if j==2:  
          tempN = firstChild.addNode("loop")
          if tempN == None:
              print ("error creating seq node")        
      
      for i in range(5):
          if ((j==1) and (i==2)):
              tempN1 = tempN.addNode("seq")
              tempN.DEBUGchild = True
              tempN1.DEBUGchild = True
              if tempN1 == None:
                  print ("error creating seq node")
              else:                
                  for i in range(4):
                      tempN2 = tempN1.addNode("tsk")
                      if tempN2 == None:
                          print ("error creating seq node")
                      else:
                          tempN2.setProbTable([0.1, 0.5])
                          for i in range(2):
                              dist_fail = _createUniformDist(6,10-i)  
                              tempN2.addDistToSuccTable(dist_succ)
                              tempN2.addDistToFailTable(dist_fail)
                          tempN2.setAttrib("Successdistribution",tempN2._distTableToString(tempN2.distTableSucc))
                          tempN2.setAttrib("Failuredistribution",tempN2._distTableToString(tempN2.distTableFail))
                          tempN2.setDebug(True, 100)
                          tempN2.setAttrib("DEBUG", tempN2.DEBUG)
                              
          else:
              tempN1 = tempN.addNode("tsk")
              if tempN1 == None:
                  print ("error creating seq node")
              else:
                  tempN1.setProbTable([0.3, 0.5])
                  for i in range(2):
                      tempN1.addDistToSuccTable(dist_succ)
                      tempN1.addDistToFailTable(dist_fail)
                      
                  tempN1.setAttrib("Successdistribution",tempN1._distTableToString(tempN1.distTableSucc))
                  tempN1.setAttrib("Failuredistribution",tempN1._distTableToString(tempN1.distTableFail))
          if j==2:
              break
	    
        

    
    #iterate over firstChild children: 
    firstChildList = firstChild.getChildren()
    node.debugMode = False
    for i in range(5):
        firstChild.run(0)
    root.treeToXml("output/test4.xml") 
    
    print "phase 2"
    node.debugMode = True
    for i in range(5):
        firstChild.run(0)
    root.treeToXml("output/test5.xml") 
    
        
        
    count = 0
    for childNode in firstChildList:
        count += 1
    if count == 3:
        print("test 6: success! please check the file test4.xml - every tag need to have the same attrib.")
    else:
        print("test 6: failed :-(")
    
     
 #empty test - will be implemented- feeling creative? :-)    
def test7():
  
    tree = node()
    root = tree
    #first child
    firstChild = root.addNode("par")
    if firstChild == None:
        print ("error creating parallel node")
        print("test 7: failed :-(")
        return None
    dist_succ = _createUniformDist(2,5)
    dist_fail = _createUniformDist(6,10)   
    
    firstChild.DEBUGchild = True 
    for j in range(3): 
      tempN = firstChild.addNode("seq")
      if tempN == None:
	  print ("error creating seq node")
      
      for i in range(5):
          if ((j==1) and (i==2)):
              tempN1 = tempN.addNode("seq")
              tempN.DEBUGchild = True
              tempN1.DEBUGchild = True
              if tempN1 == None:
                  print ("error creating seq node")
              else:                
                  for i in range(4):
                      tempN2 = tempN1.addNode("tsk")
                      if tempN2 == None:
                          print ("error creating seq node")
                      else:
                          tempN2.setProbTable([0.8, 0.5])
                          for i in range(2):
                              dist_fail = _createUniformDist(6,10-i)  
                              tempN2.addDistToSuccTable(dist_succ)
                              tempN2.addDistToFailTable(dist_fail)
                          tempN2.setAttrib("Successdistribution",tempN2._distTableToString(tempN2.distTableSucc))
                          tempN2.setAttrib("Failuredistribution",tempN2._distTableToString(tempN2.distTableFail))
                          tempN2.setDebug(True, 100)
                          tempN2.setAttrib("DEBUG", tempN2.DEBUG)
                              
          else:
              tempN1 = tempN.addNode("tsk")
              if tempN1 == None:
                  print ("error creating seq node")
              else:
                  tempN1.setProbTable([0.7, 0.5])
                  for i in range(2):
                      tempN1.addDistToSuccTable(dist_succ)
                      tempN1.addDistToFailTable(dist_fail)
                      
                  tempN1.setAttrib("Successdistribution",tempN1._distTableToString(tempN1.distTableSucc))
                  tempN1.setAttrib("Failuredistribution",tempN1._distTableToString(tempN1.distTableFail))
             
	    
        

    
    #iterate over firstChild children: 
    firstChildList = firstChild.getChildren()
    node.debugMode = False
    for i in range(5):
        firstChild.run(0)
    root.treeToXml("output/test4.xml") 
    
    print "phase 2"
    node.debugMode = True
    for i in range(5):
        firstChild.run(0)
    root.treeToXml("output/test5.xml") 
    
        
        
    count = 0
    for childNode in firstChildList:
        count += 1
    if count == 3:
        print("test 7: success! please check the file test4.xml - every tag need to have the same attrib.")
    else:
        print("test 7: failed :-(")
    
    #print the tree we built from scratch to xml file.
    #please check the file- every tag need to have the same attrib.
#    root.treeToXml("test4.xml")
    
    
def test8():
   tree = xmlTree("tests/test3.xml")
   root = tree.getRoot()
   
   #this child is type- tsk
   child = root.getChild(0)
   
   ### create a new dist - and 
   dist_succ = _createNormalDist(5,2)
   dist_fail = _createNormalDist(4,1)   
   #add to succ table
   child.addDistToSuccTable(dist_succ)
   #add to fail table/
   child.addDistToFailTable(dist_fail)
   #get distribute from the node by it's index (p1,p2,p3..)
   dist_get_succ = child.getSuccDistAtIndex(0)
   dist_get_fail = child.getFailDistAtIndex(0)
   #check that it has the same parms 
   #added by RAZ -- Adi, I made the tests a bit more complex, you should always have the tests as hard a possible, checking all possible cases.   
   if (dist_get_succ != None and dist_get_succ.parmM == float(5) and dist_get_succ.parmG == float(2) and dist_get_fail != None and dist_get_fail.parmM == float(4) and dist_get_fail.parmG == float(1)):
       print ("test 8.1: success!")

   else:
        ("test 8.1: failed :-(")
        
    # try to create computed dist.
   #added by RAZ -- Adi, I made the tests a bit more complex, you should always have the tests as hard a possible, checking all possible cases.
   dist = _createComputedDist()
   dist.setValueToTime(0.1,1)
   dist.setValueToTime(0.1, dist.getCountByTime(0.1)+1 )
   dist.setValueToTime(0.2,1)   
   dist.setValueToTime(0.05,1)
   if (dist.getCountByTime(0.1) == 2 and dist.getCountByTime(0.2) == 1 and dist.getCountByTime(0.05) == 1):
       print ("test 8.2: success!")
   else:
        ("test 8.2: failed :-( - check computed dist")
        
        
        
#this test read test9.xml and create distributaion as needed for tskNode 
def test9():
   tree = xmlTree("tests/test9.xml")
   #root it node type plan
   root = tree.getRoot()
   
   #this child is type- seq
   child = root.getChild(0)
   #this child is type- tsk
   tskChild = child.getChild(0)
   #get dist from the distTable
   distC = tskChild.getSuccDistAtIndex(2)
   distU = tskChild.getSuccDistAtIndex(1)
   distN = tskChild.getSuccDistAtIndex(0)
   

       
   if( distC.whoAmI() == "Computed" and float(distC.getCountByTime(0.1)) == 5 and float(distC.getCountByTime(257)) == 977):
       print ("test 9.1: success!")
   else:
       ("test 9: failed :-( - check computed dist")  

   if( distU.whoAmI() == "Uniform" and float(distU.parmA) == 0 and float(distU.parmB) == 5 ):
       print ("test 9.2: success!")
   else:
       ("test 9.2: failed :-( - check uniform dist")  


   if ( distN.whoAmI() == "Normal"):
       print ("test 9.3: success!")
   else:
       ("test 9.3: failed :-( - check normal dist")      
       
       
       
def test10():
   tree = xmlTree("tests/test3.xml")
   root = tree.getRoot()
   
   #this child is type- tsk
   child = root.getChild(0)
   
   ### create a new dist - and 
   dist_succ = _createNormalDist(5,2)
   dist_fail = _createNormalDist(4,1) 
   dist_fail1 = _createUniformDist(5, 8)   
   #add to succ table
   child.addDistToSuccTable(dist_succ)
   #add to fail table/
   child.addDistToFailTable(dist_fail)
   #get distribute from the node by it's index (p1,p2,p3..)
   dist_get_succ = child.getSuccDistAtIndex(0)
   dist_get_fail = child.getFailDistAtIndex(0)
   #check that it has the same parms 
   #added by RAZ -- Adi, I made the tests a bit more complex, you should always have the tests as hard a possible, checking all possible cases.   
   if (dist_get_succ != None and dist_get_succ.parmM == float(5) and dist_get_succ.parmG == float(2) and dist_get_fail != None and dist_get_fail.parmM == float(4) and dist_get_fail.parmG == float(1)):
       print ("test 10.1: success!")

   else:
        ("test 10.1: failed :-(")
        
    # try to create computed dist.
   #added by RAZ -- Adi, I made the tests a bit more complex, you should always have the tests as hard a possible, checking all possible cases.
   dist = _createComputedDist()
   dist.setValueToTime(0.1,1)
   dist.setValueToTime(0.1, dist.getCountByTime(0.1)+1 )
   dist.setValueToTime(0.2,1)   
   dist.setValueToTime(0.05,1)
   dist.printMe()
   print dist.calcProb()
   print "-----------"
   dist_succ.printMe()
   print dist_succ.calcProb()
   dist_fail1.printMe()
   print dist_fail1.calcProb()
   if (dist.getCountByTime(0.1) == 2 and dist.getCountByTime(0.2) == 1 and dist.getCountByTime(0.05) == 1):
       print ("test 10.2: success!")
   else:
        ("test 10.2: failed :-( - check computed dist")
        
        


#changed by RAZ -- we can now import from dist.* files, since the directory has an empty __init__.py file, and python recognizes it as a module.#thanks
def _createComputedDist(string = None):
    from distributions.computed import Computed
    return Computed()
    
#changed by RAZ -- we can now import from dist.* files, since the directory has an empty __init__.py file, and python recognizes it as a module.
def _createNormalDist(parmM,parmG):
   from distributions.normal import Normal
   return Normal(float(parmM),float(parmG))

#changed by RAZ -- we can now import from dist.* files, since the directory has an empty __init__.py file, and python recognizes it as a module.        
def _createUniformDist(parmA,parmB):
   from distributions.uniform import Uniform
   return Uniform(float(parmA),float(parmB))

if __name__ == "__main__":
    #run the 10 tests

    test6()
    test7()
    test8()
    test9()
    test10()