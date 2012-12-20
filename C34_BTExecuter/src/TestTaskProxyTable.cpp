/*
 * TestTaskProxyTable.cpp
 *
 *  Created on: Oct 2, 2012
 *      Author: dan
 */

#include "TestTaskProxyTable.h"
#include <boost/thread.hpp>
#include <iostream>

#define PRINT(X) std::cout<<"TEST: "<<X<<std::endl;

class TestTask:public BTTask{
private:
	BTTaskGoal goal;
	bool active;
	bool terminatesig;
	int c;

	int predef_c;
	bool predef_res;

public:
	TestTask(int pc, bool pr){
		predef_c = pc;
		predef_res = pr;
		active = false;
		terminatesig = false;
		c=0;
	}
	virtual void setGoal (const BTTaskGoal& goal){
		this->goal = goal;
		active = true;
		c=0;
	}
	virtual BTTaskFeedback getFeedback (){
		return BTTaskFeedback();
	}
	virtual BTTaskResult getResult (){
		BTTaskResult res;
		res.description="test result";
		res.plan="";
		res.success = predef_res;
		PRINT("result{success="<<res.success<<",plan="<<res.plan<<",desc="<<res.description);
		return res;
	}
	virtual void waitResult(double time){
		if(active==false){
			PRINT("waitResult: active==false");
			return;
		}
		if(c++ > predef_c){
			active = false;
			PRINT("waitResult: c++>100");
			return;
		}
		if(terminatesig){
			active = false;
			PRINT("waitResult: terminatesig");
			return;
		}
		boost::this_thread::sleep(boost::posix_time::milliseconds(time*1000));
	}
	virtual void terminate(){
		terminatesig = true;
	}
	virtual bool isActive(){
		return active;
	}
	virtual std::string address()const{ std::stringstream s; s<< "TestTask(pref_c="<<predef_c<<",pref_res="<<predef_res<<")";  return s.str();}

	virtual BTTask::Ref clone()const {return BTTask::Ref(new TestTask(predef_c, predef_res)); }
};







TestTaskProxyTable::TestTaskProxyTable() {

	add("t2", BTTask::Ref(new TestTask(100, false)));

}

TestTaskProxyTable::~TestTaskProxyTable() {

}

// void TestTaskProxyTable::add(std::string taskname, BTTask::Ref taskaccess){
//	TaskProxyTable::add(taskname,taskaccess);
//	PRINT("TestTaskProxyTable::add("<<taskname<<",taskaccess)");
//}
// bool TestTaskProxyTable::contains(std::string tn){
//	bool res = TaskProxyTable::contains(tn);
//	PRINT("TestTaskProxyTable::contains("<<tn<<") = "<<res);
//	return res;
//}
// void TestTaskProxyTable::remove(std::string tn){
//	PRINT("TestTaskProxyTable::remove("<<tn<<")");
//	if(contains(tn)){
//		tbl.erase(tn);
//		PRINT("TestTaskProxyTable::remove: ... done");
//	}
//}
// BTTask::Ref TestTaskProxyTable::get(std::string tn){
//	PRINT("TestTaskProxyTable::get("<<tn<<")");
//	if(contains(tn)){
//		PRINT("TestTaskProxyTable::get: ... done");
//		return tbl[tn];
//	}
//	return BTTask::Ref();
//}


