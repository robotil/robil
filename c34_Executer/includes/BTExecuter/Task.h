/*
 * Task.h
 *
 *  Created on: Aug 20, 2012
 *      Author: dan
 */

#ifndef TASK_H_
#define TASK_H_

#include "Node.h"
#include <boost/thread.hpp>
#include "ExeEnergy.h"

class Task: public Node {
	BT bt;
	ExeEnergy::Ref debug_energy;
protected:

	void terminateSignal(){
		Node::terminateSignal();
		releaseEnergy();
	}
	void terminateSignal(std::string desc){
		Node::terminateSignal(desc);
		releaseEnergy();
	}
	void releaseEnergy(){
		if(energyForTask.get() && energyForTask->stepByStep()) energyForTask->setStep();
		if(debug_energy.get()) debug_energy->setStep();
	}

public:
	Task(BT bt):
		bt(bt), debug_energy(new ExeEnergy())
	{

	}

	std::string str()const{ return "Task("+bt.getRootName()+")"+(bt.hasID()?std::string(" [id=")+bt.getID()+"]":std::string("")); }


	Result::Ref run();


	virtual Info info(){ return Info("Task", bt, ""); };
	virtual Info info(std::string desc){ return Info("Task", bt, desc); };
};

#endif /* TASK_H_ */
