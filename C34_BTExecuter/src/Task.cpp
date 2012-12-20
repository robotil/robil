/*
 * Task.cpp
 *
 *  Created on: Aug 20, 2012
 *      Author: dan
 */

#include "Task.h"

namespace TASK_UTILS{

std::string extractFunctionName(std::string line){
	using namespace std;
	size_t f = line.find('(');
	if(f==string::npos) return line;
	return line.substr(0, line.find('('));
}
std::string extractParameters(std::string line){
	using namespace std;
	size_t f = line.find('(');
	if(f==string::npos) return "";
	size_t p = line.find(')',f);
	size_t pp = p;
	while( p != string::npos ){ pp = p; p=line.find(')',pp+1); /*cout<<"p="<<p<<" pp="<<pp<<endl;*/}
	//cout<<"f="<<f<<", pp="<<pp<<", pp-f="<<(pp-f)<<endl;
	return line.substr(f+1, pp-f-1);
}

}using namespace TASK_UTILS;

Result::Ref Task::run(){

	IF_TASK_ENERGY energyForTask->getStep();
	boost::mutex::scoped_lock l(_node_lock);
	NODE_RETURN_IF_TERMINATED

	std::string fullname = bt.getRootName();
	std::string tname = extractFunctionName( fullname );
	std::string params = extractParameters( fullname );

	bool taskproxy_contains_this_task = taskproxy.get() && taskproxy->contains(tname);
	IF_DEBUG log<<"tsk["<<bt.getRootName()<<"] taskproxy_contains_this_task = "<<taskproxy_contains_this_task;

	if(debug_energy.get() && !taskproxy_contains_this_task){
		long time = bt.getDBGTimeInterval();
		bool result = bt.getDBGResult();
		IF_DEBUG log<<"tsk["<<bt.getRootName()<<"] debug energy: time="<<time<<", ret="<<(result?"true":"false")
				<<", proxy table "<<(!taskproxy_contains_this_task?"does not contain":"contains")<<" task name";

		if(time>=0){
			debug_energy->setContinuously(time);
		}else{
			debug_energy->setStepMode();
		}
		debug_energy->getStep(l);

		NODE_RETURN_IF_TERMINATED
		IF_DEBUG log<<"tsk["<<bt.getRootName()<<"] done";
		return Result::New(result, info("Debugging simulation"));

	}else{
		IF_DEBUG log<<"tsk["<<bt.getRootName()<<"]: search proxy for task";
		if(taskproxy_contains_this_task){
			IF_DEBUG log<<"tsk["<<bt.getRootName()<<"]: proxy found.";
			BTTask::Ref proxy = taskproxy->get(tname)->clone();
			BTTaskGoal goal;
			goal.name = tname;
			goal.uid = bt.getID();
			goal.parameters = params;
			IF_DEBUG log<<"tsk["<<bt.getRootName()<<"]: set goal: "<<goal.name<<", "<<goal.uid<<", "<<goal.parameters<<".";
			proxy->setGoal(goal);
			while(proxy->isActive() && !_terminateSignaled){
				Unlock ul(l);
				proxy->waitResult(0.1);
			}
			if(_terminateSignaled){
				IF_DEBUG log<<"tsk["<<bt.getRootName()<<"]: task terminated by Executer.";
				proxy->terminate();
				return Result::New(false, info(_terminateDescription) );
			}else{
				IF_DEBUG log<<"tsk["<<bt.getRootName()<<"]: task done.";
				BTTaskResult task_result = proxy->getResult();
				IF_DEBUG log<<"tsk["<<bt.getRootName()<<"]: result: success="<<task_result.success<<", plan="<<task_result.plan<<", description="<<task_result.description;
				_terminateDescription = task_result.description;
				switch(task_result.success){
					case BTTaskResult::SUCCESS_PLAN:{

						std::stringstream xmlstream; xmlstream<<task_result.plan;
						BT rbt(xmlstream);

						NODE_RETURN_IF_TERMINATED

						if(rbt.getRootType()!="plan"){
							_terminateDescription = "gotten plan xml does not valid.";
							return Result::New(false, info(_terminateDescription) );
						}

						_runningNode = createChildNode(rbt);
						IF_DEBUG _runningNode->setOutputDebugStream(*_debug);

						Result::Ref res = runChildNode(_runningNode,l);

						NODE_RETURN_IF_TERMINATED
						return Result::New(res->value(), info(), res);

					}break;
					default:
						return Result::New(task_result.success!=BTTaskResult::SUCCESS_FAULT, info(_terminateDescription) );
				}
			}
		}else{
			IF_DEBUG log<<"tsk["<<bt.getRootName()<<"]: no proxy found.";
		}
	}

	NODE_RETURN_IF_TERMINATED
	return Result::New(true, info());

}



