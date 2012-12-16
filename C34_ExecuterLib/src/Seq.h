/*
 * Seq.h
 *
 *  Created on: Aug 20, 2012
 *      Author: dan
 */

#ifndef SEQ_H_
#define SEQ_H_

#include "Node.h"

class Seq: public Node{
	BT bt;
public:
	Seq(BT bt):bt(bt){

	}

	std::string str()const{ return "Sequence("+bt.getRootName()+")"+(bt.hasID()?std::string(" [id=")+bt.getID()+"]":std::string("")); }

	Result::Ref run();

	virtual Info info(){ return Info("Sequence", bt, ""); };
	virtual Info info(std::string desc){ return Info("Sequence", bt, desc); };
};

#endif /* SEQ_H_ */
