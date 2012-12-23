/*
 * ExecuterStatus.cpp
 *
 *  Created on: Nov 28, 2012
 *      Author: dan
 */

#include "ExecuterStatus.h"

ExecuterStatus::ExecuterStatus() {
	state = ST_STOPPED;

}

ExecuterStatus::~ExecuterStatus() {
}


void ExecuterStatus::setBTSource(std::string fname){
	boost::mutex::scoped_lock l(mtx);
	bt_source = fname;
}
void ExecuterStatus::setLookupSource(std::string fname){
	boost::mutex::scoped_lock l(mtx);
	lookup_source = fname;
}
void ExecuterStatus::setAddressSource(std::string fname){
	boost::mutex::scoped_lock l(mtx);
	address_source = fname;
}
void ExecuterStatus::updateState(STATE st){
	boost::mutex::scoped_lock l(mtx);
	std::cout<<"status changed: "<<stateStr()<<" -> ";
	state = st;
	std::cout<<stateStr()<<std::endl;
}

ExecuterStatus::STATE ExecuterStatus::getState(){
	boost::mutex::scoped_lock l(mtx);
	return state;
}

std::string ExecuterStatus::stateStr(){
	std::stringstream out;
	switch(state){
	case ST_FINISHED: out<<"FINISHED"; break;
	case ST_RUNNING: out<<"RUNNING"; break;
	case ST_PAUSED: out<<"PAUSED"; break;
	case ST_STOPPED: out<<"STOPPED"; break;
	case ST_ERROR: out<<"ERROR"; break;
	}
	return out.str();
}


#include <boost/filesystem.hpp>
namespace {

	std::string getFileName(const std::string& path){
		namespace fs = boost::filesystem;
		using namespace std;
		using namespace fs;
		if(path[0]=='<') return "[text]";
		fs::path p(path);
		return p.filename().string();
	}

}


std::string ExecuterStatus::str(){
	boost::mutex::scoped_lock l(mtx);
	std::stringstream out;

	out<<stateStr();

	out<<"\t"<<getFileName(bt_source);

	out<<"\t lookup:";
	if(lookup_source==""){
		out<<"empty";
	}else{
		out<<getFileName(lookup_source);
	}

	out<<"\t address:";
	if(address_source==""){
		out<<"empty";
	}else{
		out<<getFileName(address_source);
	}

	return out.str();
}

