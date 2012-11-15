/*
 * ExeStack.h
 *
 *  Created on: Aug 21, 2012
 *      Author: dan
 */

#ifndef EXESTACK_H_
#define EXESTACK_H_

#include <string>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <map>
#include <boost/thread/mutex.hpp>
#include "Logger.h"

class ExeStack {
public:
	typedef boost::shared_ptr<ExeStack> Ref;
	typedef std::map<ExeStack*,Ref> Children;

	typedef void (*OnChangeCallback)(std::string sid, int code, std::string node, std::string state);
	static const int OnChangeCallback_CHILD_ADDED = 0;
	static const int OnChangeCallback_CHILD_REMOVED = 1;

private:
	Ref _parent;
	Children _children;
	boost::mutex* mtx;

	std::string _title;

	OnChangeCallback _onChange;
	std::string _stack_id;


	void printUp(Logger& cout, std::string & tab);
	void printDown(Logger& cout, std::string tab);
	void printUp(std::ostream& cout, std::string & tab);
	void printDown(std::ostream& cout, std::string tab);

	void setChild(ExeStack* ch){
		//boost::mutex::scoped_lock l(*mtx);
		_children[ch]=Ref(ch);
	}
	void removeChild(ExeStack* ch){
		//boost::mutex::scoped_lock l(*mtx);
		_children.erase(ch);
	}

	void changed(int code, std::string title){
		if(_onChange){
			std::stringstream st;
			printDown(st,"");
			_onChange(_stack_id, code, title, st.str());
		}
		if(!isRoot()) _parent->changed(code, title);
	}


public:

	ExeStack(std::string title, Ref parent = Ref());
	virtual ~ExeStack();

	void printUp(Logger& cout);
	void printDown(Logger& cout);
	void printFromRoot(Logger& cout);
	void printUp(std::ostream& cout);
	void printDown(std::ostream& cout);
	void printFromRoot(std::ostream& cout);

	bool isRoot()const{ return _parent.get()==NULL; }

	void remove();

	ExeStack::Ref ref(){ if(isRoot()==false) return _parent->_children[this]; else return Ref(); }

	void setOnChangeNotification(std::string sid, OnChangeCallback cb){
		_stack_id = sid;
		_onChange = cb;
	}
};

#endif /* EXESTACK_H_ */
