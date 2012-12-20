/*
 * Par.h
 *
 *  Created on: Aug 20, 2012
 *      Author: dan
 */

#ifndef PAR_H_
#define PAR_H_

#include "Node.h"
#include <boost/thread.hpp>
#include <boost/bind.hpp>

class Par:public Node {
	BT bt;
	boost::thread_group _threads;
	std::vector<Node::Ref> _runningNodes;
	boost::condition_variable _finished;
	Result::Ref _result;

protected:
	void terminateSignal(){
		Node::terminateSignal();
		for(size_t i=0;i<_runningNodes.size();i++){
			if(_runningNodes[i].get()!=NULL) _runningNodes[i]->terminate();
		}
		_finished.notify_one();
	}

public:
	void runTask(Node::Ref _node);


	Par(BT bt):bt(bt){

	}
	~Par(){
		_threads.join_all();
	}

	std::string str()const{
		return "Parallel("+bt.getRootName()+")"+(bt.hasID()?std::string(" [id=")+bt.getID()+"]":std::string(""));
	}


	Result::Ref run();


	virtual Info info(){ return Info("Parallel", bt, ""); };
	virtual Info info(std::string desc){ return Info("Parallel", bt, desc); };


};

#endif /* PAR_H_ */
