/*
 * TaskProxyTableXML.cpp
 *
 *  Created on: Oct 2, 2012
 *      Author: dan
 */

#include "TaskProxyTableXML.h"
#include "MapReader.h"
#include <boost/thread.hpp>
#include <iostream>
#include "Logger.h"
#define PRINT(X) logID("TaskProxyTableXML")<<X;
#define PRINT_COUT(X) //std::cout<<("TaskProxyTableXML(cout): ")<<X<<std::endl;



TaskProxyTableXML::TaskProxyTableXML(std::string pathToTaskList, BTTaskProxyCreator::Ref creator):
	creator(creator)
{

	PRINT("read map from file : "<<pathToTaskList)
	src = pathToTaskList+std::string("+Creator[")+creator->nameOfCreator()+"]";
	MapReader mr(pathToTaskList,
			"tasks","task","name","address");
	PRINT("... "<<mr.getMap().size()<<" elements have been read.")
	typedef MapReader::MAP Map;
	for(Map::const_iterator i = mr.getMap().begin(); i!=mr.getMap().end(); i++){
		//if(creator->canCreate(i->second)==false || creator->isDynamic(i->second)) continue;
		add(i->first, BTTask::Ref(creator->create(i->second)));
		PRINT("ADDRESS("<<i->first<<" , "<<i->second<<")")
		PRINT_COUT("ADDRESS("<<i->first<<" , "<<i->second<<")")
	}

}
TaskProxyTableXML::TaskProxyTableXML(std::istream& pathToTaskList, BTTaskProxyCreator::Ref creator):
			creator(creator)
{

	src = std::string("istream")+std::string("+Creator[")+creator->nameOfCreator()+"]";
	MapReader mr(pathToTaskList,
			"tasks","task","name","address");
	typedef MapReader::MAP Map;
	for(Map::const_iterator i = mr.getMap().begin(); i!=mr.getMap().end(); i++){
		//if(creator->canCreate(i->second)==false || creator->isDynamic(i->second)) continue;
		add(i->first, BTTask::Ref(creator->create(i->second)));
		PRINT("ADDRESS("<<i->first<<" , "<<i->second<<")")
		PRINT_COUT("ADDRESS("<<i->first<<" , "<<i->second<<")")
	}

}
TaskProxyTableXML::~TaskProxyTableXML() {

}

// void TaskProxyTableXML::add(std::string taskname, RobilTask::Ref taskaccess){
//	TaskProxyTable::add(taskname,taskaccess);
//	PRINT("TaskProxyTableXML::add("<<taskname<<",taskaccess)");
//}
 bool TaskProxyTableXML::contains(std::string tn){
	bool res = TaskProxyTable::contains(tn);
	if(res) return tbl[tn].get()!=NULL;
	res = creator->canCreate(tn);
	return res;
}
// void TaskProxyTableXML::remove(std::string tn){
//	PRINT("TaskProxyTableXML::remove("<<tn<<")");
//	if(contains(tn)){
//		tbl.erase(tn);
//		PRINT("TaskProxyTableXML::remove: ... done");
//	}
//}
 BTTask::Ref TaskProxyTableXML::get(std::string tn){
	if(TaskProxyTable::contains(tn)){
		return tbl[tn];
	}
	if(creator->canCreate(tn)){
		add(tn, BTTask::Ref( creator->create(tn) ));
		PRINT("ADDRESS("<<tn<<" , "<<tn<<")")
		PRINT_COUT("ADDRESS("<<tn<<" , "<<tn<<")")
		return tbl[tn];
	}
	return BTTask::Ref();
}

#undef PRINT
#undef PRINT_COUT
