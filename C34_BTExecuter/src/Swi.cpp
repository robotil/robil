/*
 * Swi.cpp
 *
 *  Created on: Aug 20, 2012
 *      Author: dan
 */

#include "Swi.h"


Result::Ref Swi::run(){
	boost::mutex::scoped_lock l(_node_lock);
	NODE_RETURN_IF_TERMINATED

	//IF_DEBUG log<<"seq:"<<bt.getRootName();

	std::vector<BT> subs = bt.getSubtree();
	if(subs.size()==0){
		return Result::New(false, Result::SYSTEM_ERROR+1, info("switch does not have children"));
	}

	checkTerminateSignal(l);
	NODE_RETURN_IF_TERMINATED

	Result::Ref last_res;
	for(size_t i=0; 0<=i && i<subs.size(); /*i++*/){
		_runningNode = createChildNode(subs[i]);
		IF_DEBUG _runningNode->setOutputDebugStream(*_debug);

		Result::Ref res = runChildNode(_runningNode,l);
		last_res = res;

		NODE_RETURN_IF_TERMINATED
		if( res->value() == true ){
			return Result::New(true, 0, info(), res);
		}

		i = res->error_code();

	}
	return Result::New(false, last_res->error_code(), info());
}



