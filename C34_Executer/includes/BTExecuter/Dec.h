/*
 * Dec.h
 *
 *  Created on: Aug 20, 2012
 *      Author: dan
 */

#ifndef DEC_H_
#define DEC_H_

#include "Node.h"

class Dec: public Node{
	BT bt;
	std::string bt_name;
public:
	Dec(BT bt):bt(bt){
		bt_name = bt.getRootName();
	}

	std::string str()const{
		return "Decorator("+bt_name+")";
	}

	Result::Ref run();
	bool doit();
	bool done(Result::Ref res);
	Result::Ref result(Result::Ref res);

	virtual Info info(){ return Info("Decorator", bt_name, ""); };
	virtual Info info(std::string desc){ return Info("Decorator", bt_name, desc); };

};

#endif /* DEC_H_ */
