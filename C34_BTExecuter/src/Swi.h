/*
 * Seq.h
 *
 *  Created on: Aug 20, 2012
 *      Author: dan
 */

#ifndef SWI_H_
#define SWI_H_

#include "Node.h"

class Swi: public Node{
	BT bt;
public:
	Swi(BT bt):bt(bt){

	}

	std::string str()const{ return "Switch("+bt.getRootName()+")"+(bt.hasID()?std::string(" [id=")+bt.getID()+"]":std::string("")); }

	Result::Ref run();

	virtual Info info(){ return Info("Switch", bt, ""); };
	virtual Info info(std::string desc){ return Info("Switch", bt, desc); };
};

#endif /* SWI_H_ */
