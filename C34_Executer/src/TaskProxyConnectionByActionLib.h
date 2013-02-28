/*
 * Test2TaskProxyTable.cpp
 *
 *  Created on: Oct 2, 2012
 *      Author: dan
 */

#include "TaskProxyTable.h"
#include <boost/thread.hpp>
#include <iostream>
#define PRINT(X) std::cout<<"TaskProxyConnectionByActionLib: "<<X<<std::endl;

#include <C0_RobilTask/RobilTaskAction.h>
#include <actionlib/client/simple_action_client.h>
#include <C0_RobilTask/RobilTask.h>

namespace TaskProxyConnectionByActionLib{

typedef actionlib::SimpleActionClient<C0_RobilTask::RobilTaskAction> Client;
using namespace C0_RobilTask;

class RobilTaskProxy:public BTTask{
private:
	bool active;
	bool serverFound;
	Client client;
	std::string tname;

public:
	RobilTaskProxy(std::string taskName);
	virtual ~RobilTaskProxy();
	virtual void setGoal (const BTTaskGoal& goal);
	virtual BTTaskFeedback getFeedback ();
	virtual BTTaskResult getResult ();
	virtual void waitResult(double time);
	virtual void terminate();
	virtual bool isActive();
	virtual std::string address()const;
	virtual Ref clone()const{return Ref( new RobilTaskProxy(tname) ); }
};

class RobilTaskProxyCreator:public BTTaskProxyCreator{
public:
	virtual std::string nameOfCreator(){return "RobilTask";}
	virtual BTTask* create(std::string TNAME){
		PRINT("[RobilTask]: create "<<TNAME<<" task");
		if(TNAME=="TEST"||TNAME=="test") return BTTask::Ref().get();
		return new RobilTaskProxy(TNAME);
	}
	virtual bool canCreate(std::string TNAME){
		PRINT("[RobilTask]: is can create "<<TNAME<<" task?");
		if(TNAME=="TEST"||TNAME=="test"){
			PRINT("[RobilTask]: ... not, I can't");
			return false;
		}
		PRINT("[RobilTask]: ... yes, I can.");
		return true;
	}
};

}using namespace TaskProxyConnectionByActionLib;

#undef PRINT
