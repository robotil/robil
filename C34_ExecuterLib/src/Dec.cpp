/*
 * Dec.cpp
 *
 *  Created on: Aug 20, 2012
 *      Author: dan
 */

#include "Dec.h"

Result::Ref Dec::run(){
	boost::mutex::scoped_lock l(_node_lock);
	NODE_RETURN_IF_TERMINATED

	//IF_DEBUG log<<"dec:"<<bt.getRootName();

	std::vector<BT> subs = bt.getSubtree();
	if(subs.size()!=1)
		return Result::New(false, info("decorator has to have just one child"));

	checkTerminateSignal(l);
	NODE_RETURN_IF_TERMINATED;

	_runningNode = createChildNode(subs[0]);

	while(true){
		NODE_RETURN_IF_TERMINATED;
		if(doit()){

			Result::Ref res = runChildNode(_runningNode, l);

			NODE_RETURN_IF_TERMINATED;
			if(done(res)){
				return result(res);
			}
		}
	}

	//Unreachable code.
	return Result::New(false, info());
}
bool Dec::doit(){
	return true;
}
bool Dec::done(Result::Ref res){
	if(bt_name=="!") return true;
	else
	if(bt_name=="L" || bt_name=="L!"){
		if(res->value()) return false;
		return true;
	}else
	if(bt_name=="!L!" || bt_name=="!L"){
		if(res->value()==false) return false;
		return true;
	}
	return true;
}
Result::Ref Dec::result(Result::Ref res){
	if(bt_name=="!") return Result::New(!res->value(), info(), res);
	else
	if(bt_name=="L"||bt_name=="!L"){
		return Result::New(false, info(), res);
	}
	if(bt_name=="!L!"||bt_name=="L!"){
		return Result::New(true, info(), res);
	}
	return Result::New(res->value(), info(), res);
}

