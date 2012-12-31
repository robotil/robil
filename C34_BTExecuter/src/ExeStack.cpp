/*
 * ExeStack.cpp
 *
 *  Created on: Aug 21, 2012
 *      Author: dan
 */

#include "ExeStack.h"

class ExeStackException:public std::exception{
	std::string w;
public:
	ExeStackException(std::string w):w(w){};
	virtual ~ExeStackException() throw(){};
	const char* what() const throw(){ return w.c_str(); }
};

ExeStack::ExeStack(std::string title, Ref parent):
	_parent(parent),
	mtx(parent.get()==NULL?new boost::mutex():parent->mtx),
	_title(title),
	_onChange(0)
{
	boost::mutex::scoped_lock l(*mtx);
	if(isRoot()==false){
		_parent->setChild(this);
	}
	changed(OnChangeCallback_CHILD_ADDED, _title);
}

void ExeStack::remove(const std::string& res_desc){ if(!mtx) return;
	boost::mutex::scoped_lock l(*mtx);
	if(_children.size()>0) throw ExeStackException("ExeStack::remove. attempt to destroy not terminal item.");
	changed(OnChangeCallback_CHILD_REMOVED, _title+"($"+res_desc+"$)");
	if(isRoot()==false){
		Ref self = ref();
		_parent->removeChild(this);
		_parent = Ref();
		mtx=0;
	}
}

ExeStack::~ExeStack() {
	if(mtx) delete mtx;
	mtx = 0;
}

void ExeStack::printUp(Logger& cout){
	boost::mutex::scoped_lock l(*mtx);
	std::string tab;
	printUp(cout, tab);
	if(_children.size()>0) cout<<tab+"  "<<"..."<<'\n';
}
void ExeStack::printUp(Logger& cout, std::string& tab){
	if(isRoot()==false) _parent->printUp(cout, tab);
	tab+="  ";
	cout<<tab<<_title<<'\n';
}
void ExeStack::printDown(Logger& cout){
	boost::mutex::scoped_lock l(*mtx);
	std::string tab="";
	if(isRoot()==false){ cout<<"..."<<'\n'; tab="  "; }
	printDown(cout, tab);
}
void ExeStack::printDown(Logger& cout, std::string tab){
	cout<<tab<<_title;
	if(_children.size()>0){
		cout<<"{"<<'\n';
		for(Children::iterator ch=_children.begin();ch!=_children.end();ch++){
			ch->second->printDown(cout, tab+"  ");
		}
		cout<<tab<<"}";
	}
	cout<<'\n';
}

void ExeStack::printFromRoot(Logger& cout){
	Ref r;
	{boost::mutex::scoped_lock l(*mtx);
	r = ref();
	while(r->isRoot()==false) r = r->_parent;}
	r->printDown(cout);
}

void ExeStack::printUp(std::ostream& cout){
	boost::mutex::scoped_lock l(*mtx);
	std::string tab;
	printUp(cout, tab);
	if(_children.size()>0) cout<<tab+"  "<<"..."<<'\n';
}
void ExeStack::printUp(std::ostream& cout, std::string& tab){
	if(isRoot()==false) _parent->printUp(cout, tab);
	tab+="  ";
	cout<<tab<<_title<<'\n';
}
void ExeStack::printDown(std::ostream& cout){
	boost::mutex::scoped_lock l(*mtx);
	std::string tab="";
	if(isRoot()==false){ cout<<"..."<<'\n'; tab="  "; }
	printDown(cout, tab);
}
void ExeStack::printDown(std::ostream& cout, std::string tab){
	cout<<tab<<_title;
	if(_children.size()>0){
		cout<<"{"<<'\n';
		for(Children::iterator ch=_children.begin();ch!=_children.end();ch++){
			ch->second->printDown(cout, tab+"  ");
		}
		cout<<tab<<"}";
	}
	cout<<'\n';
}

void ExeStack::printFromRoot(std::ostream& cout){
	Ref r;
	{boost::mutex::scoped_lock l(*mtx);
	r = ref();
	while(r->isRoot()==false) r = r->_parent;}
	r->printDown(cout);
}
