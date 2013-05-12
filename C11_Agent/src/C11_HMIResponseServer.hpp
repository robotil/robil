#ifndef _C11_HMI_RESPONSE_SERVER_H_
#define _C11_HMI_RESPONSE_SERVER_H_

#include <actionlib/server/simple_action_server.h>
#include <C0_RobilTask/RobilTask.h>
#include <C0_RobilTask/RobilTaskAction.h>
#include <C0_RobilTask/StringOperations.h>
#include "C10_Common/HMIResponse.h"

using namespace std;
using namespace C0_RobilTask;

class IHMIResponseInterface
{
public:
  virtual void HMIResponse() = 0;
};

class HMIResponseServer:public RobilTask{

public:
	HMIResponseServer(string name = "/HMIResponse"):
    	RobilTask(name)
    {
	  pIHMIResponseInterface = NULL;
	  IsWaitForResponse = false;
    }

	TaskResult task(const string& name, const string& uid, Arguments& args)
	{

	    	ROS_INFO("C11_Agent: HMIResponse called!\n");
	    	if(!args.empty())
	    	{
	    		ROS_INFO("%s: %s\n","data",args["data"].data());
	    	}
//	    	ros::ServiceClient c11Client = _node.serviceClient<C10_Common::HMIResponse>("C11/HMIResponse");
//
//	    	C10_Common::HMIResponse srv11;
//
//	    	if (!c11Client.call(srv11))
//			{
//				ROS_ERROR("couldn't get a C11 reply, exiting\n");
//				return TaskResult::FAULT();
//			}
	    	if(pIHMIResponseInterface != NULL)
	    	  {
	    	    IsWaitForResponse = true;
	    	    pIHMIResponseInterface->HMIResponse();
	    	    while(IsWaitForResponse)
                    {
                      sleep(100);
                    }
	    	  }
	    	else
	    	  {
	    	    return TaskResult::FAULT();
	    	  }
	    	ROS_INFO("C11_Agent: HMIResponse sent!\n");

	    	return TaskResult::SUCCESS();
	    }

	void SetHMIResponseInterface(IHMIResponseInterface* pihmiResponseInterface)
	{
	  pIHMIResponseInterface = pihmiResponseInterface;
	}
	void ResponseReceived()
	{
	  IsWaitForResponse = false;
	}

private:
	IHMIResponseInterface* pIHMIResponseInterface;
	bool IsWaitForResponse;

};

#endif //_C11_HMI_RESPONSE_SERVER_H_
