/*
 * TestTaskProxyTable.h
 *
 *  Created on: Oct 2, 2012
 *      Author: dan
 */

#ifndef TEST2TASKPROXYTABLEXML_H_
#define TEST2TASKPROXYTABLEXML_H_

#include "TaskProxyTable.h"


class TaskProxyTableXML: public TaskProxyTable {
	std::string src;
	BTTaskProxyCreator::Ref creator;
public:
	TaskProxyTableXML(std::string pathToTaskList, BTTaskProxyCreator::Ref creator);
	TaskProxyTableXML(std::istream& pathToTaskList, BTTaskProxyCreator::Ref creator);
	virtual ~TaskProxyTableXML();

	virtual std::string source(){ return src; }

//	virtual void add(std::string taskname, BTTask::Ref taskaccess);
	virtual bool contains(std::string tn);
//	virtual void remove(std::string tn);
	virtual BTTask::Ref get(std::string tn);

};

#endif /* TEST2TASKPROXYTABLE_H_ */
