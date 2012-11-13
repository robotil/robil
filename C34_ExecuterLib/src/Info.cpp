/*
 * Info.cpp
 *
 *  Created on: Aug 20, 2012
 *      Author: dan
 */

#include "Info.h"

Info::Info(std::string node_type,std::string node_name,std::string error_description)
:_node_type(node_type), _node_name(node_name),_error_description(error_description)
{

}
Info::Info(BT bt,std::string error_description)
:_node_type(bt.getRootType()), _node_name(bt.getRootName()),_error_description(error_description), bt(bt)
{

}
Info::Info(std::string node_type,BT bt,std::string error_description)
:_node_type(node_type), _node_name(bt.getRootName()),_error_description(error_description), bt(bt)
{

}

Info::~Info() {

}

void Info::printFullName(Logger& cout){

	cout<<_node_type<<"("<<_node_name<<")"<<(bt.hasID()?std::string(" [id=")+bt.getID()+"]":"");

}
void Info::printShortDescription(Logger& cout, int len){

	if(_error_description.length()<=(unsigned int)len) cout<<_error_description;
	else{
		cout<<_error_description.substr(0,len-3)<<"...";
	}
}
void Info::printFullName(std::ostream& cout){

	cout<<_node_type<<"("<<_node_name<<")"<<(bt.hasID()?std::string(" [id=")+bt.getID()+"]":"");

}
void Info::printShortDescription(std::ostream& cout, int len){

	if(_error_description.length()<=(unsigned int)len) cout<<_error_description;
	else{
		cout<<_error_description.substr(0,len-3)<<"...";
	}
}



