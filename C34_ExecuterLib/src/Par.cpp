/*
 * Par.cpp
 *
 *  Created on: Aug 20, 2012
 *      Author: dan
 */

#include "Par.h"

void Par::runTask(Node::Ref _node){
	boost::mutex::scoped_lock l(_node_lock);
	Result::Ref res = runChildNode(_node,l);
	if(_result.get()==NULL){
		_result = res; //I am finished first.
		//IF_DEBUG log<<"Par::runTask: the first task is finished.";
	}
	_finished.notify_one();
}

Result::Ref Par::run(){
	Result::Ref res;
	{
		boost::mutex::scoped_lock l(_node_lock);
		NODE_RETURN_IF_TERMINATED

		std::vector<BT> subs = bt.getSubtree();
		if(subs.size()==0){
			return Result::New(false, info("parallel does not have children"));
		}

		checkTerminateSignal(l);
		NODE_RETURN_IF_TERMINATED

		res.reset();
		for(size_t i=0; i<subs.size(); i++){
			Node::Ref node = createChildNode(subs[i]);
			_runningNodes.push_back(node);
			_threads.add_thread(new boost::thread(boost::bind(&Par::runTask, this, node)));
		}

		_finished.wait(l);
		res = _result;

		for(size_t i=0; i<subs.size(); i++){
			//IF_DEBUG { log<<"terminate node #"<<i; }
			_runningNodes[i]->terminate();
		}
	}
	_threads.join_all();

	NODE_RETURN_IF_TERMINATED
	if(res.get())
		return Result::New(res->value(), info(), res);
	else
		return Result::New(false, info("No results gotten from children"));

}

