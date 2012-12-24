/*
 * Node.h
 *
 *  Created on: Aug 20, 2012
 *      Author: dan
 */

#ifndef NODE_H_
#define NODE_H_

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "Unlock.h"
#include "Logger.h"

#include "BT.h"
#include "Info.h"
#include "Result.h"
#include "ExeStack.h"
#include "ExeEnergy.h"
#include "Lookup.h"
#include "TaskProxyTable.h"

class Node;
boost::shared_ptr<Node> static_Node_createNode(BT& bt);
boost::shared_ptr<Node> static_Node_createNode(Lookup::Ref lookup,BT& bt);

class Node{
public:
	typedef boost::shared_ptr<Node> Ref;

protected:
	boost::mutex _node_lock;
	Node::Ref _runningNode;
	bool _terminateSignaled;
	std::string _terminateDescription;
	std::string _executer_id;

	bool* _debug;
	bool debug(){bool b= _debug && *_debug; /*{log<<"debug="<<b<<" addr="<<_debug;}*/ return b;}
	#define IF_DEBUG if(debug())

	ExeStack::Ref stack;
	ExeEnergy::Ref energy;
	ExeEnergy::Ref energyForTask;
	#define IF_ENERGY if(energy.get())
	#define IF_TASK_ENERGY if(energyForTask.get())

	Lookup::Ref lookup;
	TaskProxyTable::Ref taskproxy;

protected:
	void setRunningNode(Node::Ref node){
		boost::mutex::scoped_lock l(_node_lock);
		_runningNode = node;
	}
	Node::Ref getRunningNode(){
		boost::mutex::scoped_lock l(_node_lock);
		return _runningNode;
	}
	void checkTerminateSignal(boost::mutex::scoped_lock& plocker){
		Unlock ul(plocker);
	}

protected:
	virtual void terminateSignal(){
		_terminateSignaled = true;
		if(_runningNode.get()!=NULL) _runningNode->terminate();
	}
	virtual void terminateSignal(std::string desc){
		_terminateSignaled = true;
		_terminateDescription = desc;
		if(_runningNode.get()!=NULL) _runningNode->terminate();
	}

	Node::Ref createChildNode(BT bt){
		Node::Ref ch = createNode(lookup, bt);
		ch->setExecuterId(_executer_id);
		ch->lookup = lookup;
		ch->taskproxy = taskproxy;
		ch->setOutputDebugStream(*_debug);
		ch->setEnergy(energy);
		ch->setEnergyForTask(energyForTask);
		return ch;
	}
	void deleteChildNode(Node::Ref& ch){
		ch->removeFromStack();
	}

	Result::Ref runChildNode(Node::Ref& node, boost::mutex::scoped_lock& l){
		if(stack.get()) node->setStack(stack);
		Result::Ref res = node->run(l);
		deleteChildNode(node);
		return res;
	}

public:
	void terminate(){
		boost::mutex::scoped_lock l(_node_lock);
		terminateSignal();
	}
	void terminate(std::string desc){
		boost::mutex::scoped_lock l(_node_lock);
		terminateSignal(desc);
	}
	void setOutputDebugStream(bool& cout){
		_debug = &cout;
	}
	void setLookup(Lookup::Ref lu){
		lookup = lu;
	}
	void setTaskProxy(TaskProxyTable::Ref tbl){
		taskproxy = tbl;
	}
	void setStack(ExeStack::Ref st){
		ExeStack* e = new ExeStack(str(), st);
		stack = e->ref();
	}
	void removeFromStack(){ if(this->stack.get()) this->stack->remove(); }
	void setEnergy(ExeEnergy::Ref en){
		energy = en;
	}
	void setEnergyForTask(ExeEnergy::Ref en){
		energyForTask = en;
	}
	void setExecuterId(std::string exeid){
		_executer_id = exeid;
	}

	Node():_terminateSignaled(false), _debug(0){

	}
	virtual ~Node(){}
	virtual Result::Ref run()=0;
	Result::Ref run(boost::mutex::scoped_lock& plocker){
		if(debug() && stack.get()) {Logger l(_executer_id); stack->printFromRoot(l);}
		IF_ENERGY energy->getStep(plocker);
		Unlock ul(plocker);
		return run();
	}
	virtual Info info()=0;
	virtual Info info(std::string desc)=0;


//	static Node::Ref createNode(BT bt){
//		return static_Node_createNode(bt);
//	}
	static Node::Ref createNode(Lookup::Ref lookup, BT bt){
		if(lookup.get()) lookup->analize(bt);
		if(lookup.get() && lookup->contains(bt))
			return static_Node_createNode(lookup,bt);
		return static_Node_createNode(bt);
	}


	virtual std::string str()const=0;

};


//#define NODE_REMOVE_FROM_STACK(N) N->removeFromStack();
//#define NODE_ADD_TO_STACK(N) {if(stack.get()){ N->setStack(stack); }}
#define NODE_RETURN_IF_TERMINATED {if(_terminateSignaled){ return Result::New(false, Result::SYSTEM_ERROR_TERMINATED, info(_terminateDescription) );}}
#define EMPTY if(false){}
#define NODE_DESTRUCTOR(N) struct stract_##N{Node *self; Node::Ref node; public:stract_##N(Node* s, Node::Ref _n):self(s),node(_n){ } ~stract_##N(){ self->deleteChildNode(node); }} instance_##N(this, N);

#endif /* NODE_H_ */
