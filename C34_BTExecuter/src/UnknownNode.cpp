/*
 * UnknownNode.cpp
 *
 *  Created on: Aug 20, 2012
 *      Author: dan
 */

#include "UnknownNode.h"

Result::Ref UnknownNode::run(){
	boost::mutex::scoped_lock l(_node_lock);
	NODE_RETURN_IF_TERMINATED

	//IF_DEBUG log<<"unknown:"<<bt.getRootName();

	std::vector<BT> subs = bt.getSubtree();
	if(subs.size()>1){
		return Result::New(false, Result::SYSTEM_ERROR+1, info());
	}
	if(subs.size()<1){
		return Result::New(true, 0, info());
	}

	checkTerminateSignal(l);
	NODE_RETURN_IF_TERMINATED

	_runningNode = Node::createChildNode(subs[0]);
	IF_DEBUG _runningNode->setOutputDebugStream(*_debug);

	Result::Ref res = runChildNode(_runningNode,l);

	NODE_RETURN_IF_TERMINATED
	return Result::New(res->value(), res->error_code(), info(), res);
}
