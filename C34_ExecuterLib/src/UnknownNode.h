/*
 * UnknownNode.h
 *
 *  Created on: Aug 20, 2012
 *      Author: dan
 */

#ifndef UNKNOWNNODE_H_
#define UNKNOWNNODE_H_

#include "Node.h"

class UnknownNode: public Node {
	BT bt;
public:
	UnknownNode(BT bt):bt(bt){

	}

//	std::string str()const{ return "Unknown("+bt.getRootName()+")"; }
	std::string str()const{ return "Unknown("+bt.getRootName()+")"+(bt.hasID()?std::string(" [id=")+bt.getID()+"]":std::string("")); }

	Result::Ref run();

	virtual Info info(){ return Info("Unknown", bt.getRootName(), ""); };
	virtual Info info(std::string desc){ return Info("Unknown", bt.getRootName(), desc); };
};

#endif /* UNKNOWNNODE_H_ */
