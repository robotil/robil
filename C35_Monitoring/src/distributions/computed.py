# -*- coding: utf-8 -*-
"""

@author: polak
"""
import random
from distribution import Distribution

class Computed(Distribution):
    #constractur
    def __init__(self,Map = {}):
        Distribution.__init__(self)
         #map is a python dictionary{[time,count]..}    
        self.map = Map
        
       
    def calcProb(self):
        x = random.random()
        return self.distNormalize(x)
    
    
    def distNormalize(self, x):
        s = 0.0
        a = 0
        for i in self.map.values():
            if i !='':
                s=s+float(i)
        for i in self.map:
            if  i!='':
                value = self.map.get(i)
                if value!='':
                    a = a + (float(value))/s
                    if (a >= x):
                        return i
            
    #search for the time key in the dictionary- return 0  or count.- value
    def getCountByTime(self,time):
        ans = self.map.get(float(time))        
        if (ans != None):
            return ans
        else:
            ans = self.map.get(str(time))
        if (ans != None):
            return ans
        return 0
    #search for the time key in the dictionary- and update it. or create it with value.   
    #Changed by RAZ - removed casting value to str    
    def setValueToTime(self,time,value):
        if (self.map.get(float(time)) != None):
            self.map[float(time)]=value           
        else:
            self.map.setdefault(float(time) , value)
     
       
    def stringToDictionary(self,string):
        pass
    
   #for debaug     
    def whoAmI(self):
        return "Computed"
        
    def printMe (self):
        if len(self.map) == 0:
            print "[]"
        else:
            for i in self.map:
                print i, self.map.get(i)
                
                
 #   def toString(self):
  #      string ="C["
  #      i=0
  #      for index in self.map:
            #print i
  #          i=i+1
  #          string+= str("["+str(index)+","+str(self.map.get(index))+"]")
  #          #print string
  #      string+="]"
  #      return string           
     
    def toString(self):
        string ="C["
        
        sortKeyList = []
        for key in self.map :
            if key == '':
                break
            sortKeyList.append(float(key))
        sortKeyList = sorted(sortKeyList)
        
        for key in sortKeyList :
        #for index in self.map:
            #print i
            string+= str("["+str(key)+","+str(self.getCountByTime((key)))+"]")
            
            #print string
        string +="]"
       
        return string     
  
     
    def calcAverageTime(self):
        numOfValues = 0.0
        totalOfValues = 0.0
        timeValueMap = self.map
        for key in timeValueMap:
            numOfValues = numOfValues + timeValueMap.get(key)
            totalOfValues = totalOfValues + (key * timeValueMap.get(key))
        if  numOfValues==0:
            return float('Inf')
        return totalOfValues / numOfValues   