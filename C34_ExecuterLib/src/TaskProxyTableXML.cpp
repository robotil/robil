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



TaskProxyTableXML::TaskProxyTableXML(std::string pathToTaskList, BTTaskProxyCreator::Ref creator) {

	PRINT("read map from file : "<<pathToTaskList)
	MapReader mr(pathToTaskList,
			"tasks","task","name","address");
	PRINT("... "<<mr.getMap().size()<<" elements have been read.")
	typedef MapReader::MAP Map;
	for(Map::const_iterator i = mr.getMap().begin(); i!=mr.getMap().end(); i++){
		add(i->first, BTTask::Ref(creator->create(i->second)));
		PRINT("ADDRESS("<<i->first<<" , "<<i->second<<")")
		PRINT_COUT("ADDRESS("<<i->first<<" , "<<i->second<<")")
	}

}
TaskProxyTableXML::TaskProxyTableXML(std::istream& pathToTaskList, BTTaskProxyCreator::Ref creator) {

	MapReader mr(pathToTaskList,
			"tasks","task","name","address");
	typedef MapReader::MAP Map;
	for(Map::const_iterator i = mr.getMap().begin(); i!=mr.getMap().end(); i++){
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
// bool TaskProxyTableXML::contains(std::string tn){
//	bool res = TaskProxyTable::contains(tn);
//	PRINT("TaskProxyTableXML::contains("<<tn<<") = "<<res);
//	return res;
//}
// void TaskProxyTableXML::remove(std::string tn){
//	PRINT("TaskProxyTableXML::remove("<<tn<<")");
//	if(contains(tn)){
//		tbl.erase(tn);
//		PRINT("TaskProxyTableXML::remove: ... done");
//	}
//}
// RobilTask::Ref TaskProxyTableXML::get(std::string tn){
//	PRINT("TaskProxyTableXML::get("<<tn<<")");
//	if(contains(tn)){
//		PRINT("TaskProxyTableXML::get: ... done");
//		return tbl[tn];
//	}logID
//	return RobilTask::Ref();
//}

#undef PRINT
#undef PRINT_COUT
