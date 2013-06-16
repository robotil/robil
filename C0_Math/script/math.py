#! /usr/bin/env python

import roslib; roslib.load_manifest('C0_Math')
import rospy
import actionlib
from RobilTaskPy import *

import subprocess
import os
import threading

#def calc(ex):
	#return os.popen("echo '"+ex+"' | bc -l").read().strip()
def calc(ex):
	return [x.strip() for x in os.popen("echo '"+ex+"' | bc -l").readlines()]

gp_mutex = threading.RLock()
glob_parameters={}

def process(parameters):
	global glob_parameters, gp_mutex	
	with gp_mutex:
		fex=[]
		for v in glob_parameters:
			fex.append(v+"="+glob_parameters[v])
		for v in parameters:
			fex.append(v+"=("+parameters[v]+")")
		for v in parameters:
			glob_parameters[v]=parameters[v]
		for v in glob_parameters:
			fex.append(v)
		fex = ';'.join(fex)
		print fex
		res = calc(fex)
		if len(glob_parameters) != len(res) :
			return res 
		for v,e in zip(glob_parameters,res):
			glob_parameters[v]=e
		return "ok"
	
def getList(par):
	global glob_parameters, gp_mutex
	with gp_mutex:
		r=[]
		for v in par:
			r.append(v+"="+glob_parameters[v])
		sres = ';'.join(r)
		return sres

class MathTask(RobilTask):
	def __init__(self, name):
		print "Init MathTask"
		RobilTask.__init__(self, name)
	
	def task(self, name, uid, parameters):
		print "Start MathTask"
		if self.isPreepted():
			print "Preempt MathTask"
			return RTResult_PREEPTED()
		res = process(parameters)
		if self.isPreepted():
			print "Preempt MathTask"
			return RTResult_PREEPTED()
		if res=='ok':
			print glob_parameters
			print "SUCCESS : "+getList(parameters)
			return RTResult_SUCCESSED("Finished in Success: "+getList(parameters))
		else:
			print "calculation problems: "+res
			error_code = RobilTask_FAULT + 0
			return RTResult_ABORT(error_code,"Some Error detected: code="+str(error_code)+", "+res);
		
class MathWhile(RobilTask):
	def __init__(self, name):
		print "Init MathWhile"
		RobilTask.__init__(self, name)
	
	def task(self, name, uid, parameters):
		print "Start MathWhile"
		r = rospy.Rate(1)
		while True:
			if self.isPreepted():
				print "Preempt MathWhile"
				return RTResult_PREEPTED()
			res = process(parameters)
			if res=='ok':
				print glob_parameters
				for v in parameters:
					if glob_parameters[v]=='0':
						print "SUCCESS : "+getList(parameters)
						return RTResult_SUCCESSED("MathWhile Finished in Success: "+getList(parameters))
			else:
				print "calculation problems: "+str(res)
				error_code = RobilTask_FAULT + 0
				return RTResult_ABORT(error_code,"MathWhile Some Error detected: code="+str(error_code)+", "+res);
			r.sleep()
		print "MathWhile"

class MathUntil(RobilTask):
	def __init__(self, name):
		print "Init MathUntil"
		RobilTask.__init__(self, name)
	
	def task(self, name, uid, parameters):
		print "Start MathUntil"
		r = rospy.Rate(1)
		while True:
			if self.isPreepted():
				print "Preempt MathUntil"
				return RTResult_PREEPTED()
			res = process(parameters)
			if res=='ok':
				s=True
				for v in parameters:
					if parameters[v]=='0':
						s=False
				if s : 
					print "SUCCESS : "+getList(parameters)
					return RTResult_SUCCESSED("MathUntil Finished in Success: "+getList(parameters))
			else:
				print "calculation problems: "+res
				error_code = RobilTask_FAULT + 0
				return RTResult_ABORT(error_code,"MathUntil Some Error detected: code="+str(error_code)+", "+res);
			r.sleep()
		print "MathUntil"
		
class MathIF(RobilTask):
	def __init__(self, name):
		print "Init MathIF"
		RobilTask.__init__(self, name)
	
	def task(self, name, uid, parameters):
		print "Start MathIF"
		if self.isPreepted():
			print "Preempt MathIF"
			return RTResult_PREEPTED()
		res = process(parameters)
		if res=='ok':
			s=True
			for v in parameters:
				if parameters[v]=='0':
					s=False
			if s : 
				print "SUCCESS : "+getList(parameters)
				return RTResult_SUCCESSED("MathIF Finished in Success: "+getList(parameters))
			else:
				error_code = RobilTask_FAULT + 1
				return RTResult_ABORT(error_code,"MathIF Finished with False: code="+str(error_code)+", "+getList(parameters));
		else:
			print "calculation problems: "+res
			error_code = RobilTask_FAULT + 0
			return RTResult_ABORT(error_code,"MathIF Some Error detected: code="+str(error_code)+", "+res);
		print "MathIF"  

class MathWait(RobilTask):
	def __init__(self, name):
		print "Init MathWait"
		RobilTask.__init__(self, name)
	
	def task(self, name, uid, parameters):
		print "Start MathWait"
		r = rospy.Rate(1)
		lim = int(parameters["time"])
		sec=0;
		while True:
			if self.isPreepted():
				print "Preempt MathWait"
				return RTResult_PREEPTED()
			
			r.sleep();
			sec+=1
			if sec>=lim:
				return RTResult_SUCCESSED("MathWait Finished in Success")
		
		print "MathWait"  

  
def test():
	print "OK"
      
if __name__ == '__main__':
	print "Start Node with MathTask"
	rospy.init_node('MathTask')
	
	MathTask("math")
	MathWhile("while")
	MathUntil("until")
	MathIF("if")
	MathWait("wait")

	rospy.spin()
	print "MathTask Closed"
