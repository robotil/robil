/*
 * TaskProxyTable.h
 *
 *  Created on: Oct 2, 2012
 *      Author: dan
 */

#ifndef TASKPROXYTABLE_H_
#define TASKPROXYTABLE_H_

#include <string>
#include <boost/shared_ptr.hpp>
#include <sstream>
#include <map>

class BTTaskGoal{
public:
	std::string name;
	std::string uid;
	std::string parameters;
};
class BTTaskFeedback{
public:
	float complete;
};
class BTTaskResult{
public:
	static const int SUCCESS_FAULT = 0;
	static const int SUCCESS_OK = 1;
	static const int SUCCESS_PLAN = 2;
	int32_t success;
	std::string description;
	std::string plan;
};
class BTTask{
public:
	BTTask(){}
	virtual ~BTTask(){}
	typedef boost::shared_ptr<BTTask> Ref;
	virtual void setGoal (const BTTaskGoal& goal)=0;
	virtual BTTaskFeedback getFeedback ()=0;
	virtual BTTaskResult getResult ()=0;
	virtual void waitResult(double time)=0;
	virtual void terminate()=0;
	virtual bool isActive()=0;
	virtual std::string address()const=0;
	virtual BTTask::Ref clone()const=0;
};
class TaskProxyTable{
protected:
	typedef std::map<std::string, BTTask::Ref> table;
	table tbl;
public:
	typedef boost::shared_ptr<TaskProxyTable> Ref;
	TaskProxyTable(){}
	virtual ~TaskProxyTable(){}
	virtual void add(std::string taskname, BTTask::Ref taskaccess){
		tbl[taskname] = taskaccess;
	}
	virtual bool contains(std::string tn){ return tbl.find(tn)!=tbl.end(); }
	virtual void remove(std::string tn){ if(contains(tn)) tbl.erase(tn); }
	virtual BTTask::Ref get(std::string tn){
		if(contains(tn)) return tbl[tn];
		return BTTask::Ref();
	}
	std::string str()const{
		std::stringstream s;
		for(table::const_iterator i = tbl.begin(); i!= tbl.end(); i++){
			s<<"("<<i->first<<", "<<i->second->address()<<") ";
		}
		return s.str();
	}
};

class BTTaskProxyCreator{
public:
	typedef boost::shared_ptr<BTTaskProxyCreator> Ref;
	virtual BTTask* create(std::string BTNAME)=0;
	virtual ~BTTaskProxyCreator(){}
};

#endif /* TASKPROXYTABLE_H_ */
