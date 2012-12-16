/*
 * TestTaskProxyTable.h
 *
 *  Created on: Oct 2, 2012
 *      Author: dan
 */

#ifndef TESTTASKPROXYTABLE_H_
#define TESTTASKPROXYTABLE_H_

#include "TaskProxyTable.h"

class TestTaskProxyTable: public TaskProxyTable {
public:
	TestTaskProxyTable();
	virtual ~TestTaskProxyTable();


//	virtual void add(std::string taskname, BTTask::Ref taskaccess);
//	virtual bool contains(std::string tn);
//	virtual void remove(std::string tn);
//	virtual BTTask::Ref get(std::string tn);

};

#endif /* TESTTASKPROXYTABLE_H_ */
