/*
 * Seq.cpp
 *
 *  Created on: Aug 20, 2012
 *      Author: dan
 */

#include "Seq.h"


Result::Ref Seq::run(){
	boost::mutex::scoped_lock l(_node_lock);
	NODE_RETURN_IF_TERMINATED

	//IF_DEBUG log<<"seq:"<<bt.getRootName();

	std::vector<BT> subs = bt.getSubtree();
	if(subs.size()==0){
		return Result::New(false, info("sequencer does not have children"));
	}

	checkTerminateSignal(l);
	NODE_RETURN_IF_TERMINATED

	for(size_t i=0; i<subs.size(); i++){
		_runningNode = createChildNode(subs[i]);
		IF_DEBUG _runningNode->setOutputDebugStream(*_debug);

		Result::Ref res = runChildNode(_runningNode,l);

		NODE_RETURN_IF_TERMINATED
		if( res->value() == false ){
			return Result::New(false, info(), res);
		}
	}
	return Result::New(true, info());
}



