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

#include <RobilTask/RobilTaskAction.h>
#include <actionlib/client/simple_action_client.h>
#include <RobilTask/RobilTask.h>

namespace TaskProxyConnectionByActionLib{

typedef actionlib::SimpleActionClient<RobilTask::RobilTaskAction> Client;
using namespace RobilTask;

class RobilTaskProxy:public BTTask{
private:
	bool active;
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
};

class RobilTaskProxyCreator:public BTTaskProxyCreator{
public: virtual BTTask* create(std::string TNAME){ return new RobilTaskProxy(TNAME); }
};

}using namespace TaskProxyConnectionByActionLib;

#undef PRINT
