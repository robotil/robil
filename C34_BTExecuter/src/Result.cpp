/*
 * Result.cpp
 *
 *  Created on: Aug 20, 2012
 *      Author: dan
 */

#include "Result.h"

//const int leanthOfDescription = 7;
const int leanthOfDescription = 255;

void Result::print(Logger& cout){
	Result* r = this;
	std::string tab="";
	while(r){
		cout<<tab;
		r->_info.printFullName(cout);
		cout<<":"<<(r->_val?"OK":"FAILURE");
		if( r->_info.error_desc().length()>0){
			cout<<":"; r->_info.printShortDescription(cout, leanthOfDescription);
		}
		cout<<";"<<'\n';

		if(r->hasNext()){
			tab+=" ";
			r = r->next().get();
		}else{
			r = 0;
		}
	}
}
void Result::print(std::ostream& cout){
	Result* r = this;
	std::string tab="";
	while(r){
		cout<<tab;
		r->_info.printFullName(cout);
		cout<<":"<<(r->_val?"OK":"FAILURE");
		if( r->_info.error_desc().length()>0){
			cout<<":"; r->_info.printShortDescription(cout, leanthOfDescription);
		}
		cout<<";"<<'\n';

		if(r->hasNext()){
			tab+=" ";
			r = r->next().get();
		}else{
			r = 0;
		}
	}
}






