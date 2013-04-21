#ifndef _C11_UNTIL_OPERATOR_INTERVENTION_H_
#define _C11_UNTIL_OPERATOR_INTERVENTION_H_

#include <actionlib/server/simple_action_server.h>
#include <C0_RobilTask/RobilTask.h>
#include <C0_RobilTask/RobilTaskAction.h>
#include <C0_RobilTask/StringOperations.h>
#include "C10_Common/HMIResponse.h"

using namespace std;
using namespace C0_RobilTask;

class UntilOperatorIntervention:public RobilTask{

public:
	UntilOperatorIntervention(string name = "/untilOperatorIntervention"):
    	RobilTask(name)
    {

    }

	TaskResult task(const string& name, const string& uid, Arguments& args){

	    	while(isPreempt())
	    	{
	    	    sleep(100);
	    	}

	    	return TaskResult::SUCCESS();
	    }

};

#endif //_C11_UNTIL_OPERATOR_INTERVENTION_H_
