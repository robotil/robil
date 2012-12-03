/*
 * Sel.h
 *
 *  Created on: Aug 20, 2012
 *      Author: dan
 */

#ifndef SEL_H_
#define SEL_H_

#include "Node.h"

class Sel: public Node{
	BT bt;
public:
	Sel(BT bt):bt(bt){

	}

	std::string str()const{ return "Selection("+bt.getRootName()+")"+(bt.hasID()?std::string(" [id=")+bt.getID()+"]":std::string("")); }

	Result::Ref run();

	virtual Info info(){ return Info("Selection", bt, ""); };
	virtual Info info(std::string desc){ return Info("Selection", bt, desc); };
};

#endif /* SEL_H_ */
