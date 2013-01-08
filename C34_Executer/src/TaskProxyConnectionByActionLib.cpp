/*
 * Test2TaskProxyTable.cpp
 *
 *  Created on: Oct 2, 2012
 *      Author: dan
 */

#include "TaskProxyConnectionByActionLib.h"
#define PRINT(X) std::cout<<"TaskProxyConnectionByActionLib: "<<X<<std::endl;

namespace TaskProxyConnectionByActionLib{

typedef actionlib::SimpleActionClient<C0_RobilTask::RobilTaskAction> Client;
using namespace C0_RobilTask;


	RobilTaskProxy::RobilTaskProxy(std::string taskName):
		active(0),
		client(taskName.c_str(), true),
		tname(taskName)
	{

	}
	RobilTaskProxy::~RobilTaskProxy(){
		//client.shutdown();
	}
	void RobilTaskProxy::setGoal (const BTTaskGoal& goal){
		C0_RobilTask::RobilTaskGoal msg_goal;
		msg_goal.name = goal.name;
		msg_goal.uid = goal.uid;
		msg_goal.parameters = goal.parameters;
		PRINT("goal{name="<<msg_goal.name<<",uid="<<msg_goal.uid<<",parameters="<<msg_goal.parameters<<"}");
		PRINT("actionlib client waits for server");
		serverFound = false;
		client.waitForServer(ros::Duration(1.0));
		if(client.isServerConnected()){
			PRINT("actionlib client finds server");
			serverFound = true;
			client.sendGoal(msg_goal);
			active = true;
		}else{
			PRINT("actionlib client does not find server");
		}
	}
	BTTaskFeedback RobilTaskProxy::getFeedback (){
		return BTTaskFeedback();
	}
	namespace{
		inline std::string success2str(int val){
			if(val==BTTaskResult::SUCCESS_FAULT) return "FAULT";
			if(val==BTTaskResult::SUCCESS_OK) return "OK";
			if(val==BTTaskResult::SUCCESS_PLAN) return "PLAN";
			if(val > 0) return "ERROR_CODE";
			return "UNKNOWN";
		}
	}
	BTTaskResult RobilTaskProxy::getResult (){
		BTTaskResult res;
		if(serverFound==false){
			res.description="Task server is not connected.";
			res.plan="";
			res.success = BTTaskResult::SUCCESS_FAULT;			
		}else{
			actionlib::SimpleClientGoalState state = client.getState();
			/*if(state==actionlib::SimpleClientGoalState::SUCCEEDED){*/
				res.description = client.getResult()->description;
				res.plan = client.getResult()->plan;
				res.success = client.getResult()->success;
			/*}else{
				res.description="actionlib::SimpleClientGoalState != actionlib::SimpleClientGoalState::SUCCEEDED";
				res.plan="";
				res.success = BTTaskResult::SUCCESS_FAULT;
			}*/
		}
		PRINT("result{success="<<success2str(res.success)<<",plan="<<res.plan<<",desc="<<res.description<<"}");
		return res;
	}
	void RobilTaskProxy::waitResult(double time){
		if(!active) return;
		   
		if(client.isServerConnected()==false){
			active = false;
			return;
		}

		bool finished = client.waitForResult(ros::Duration(time));
//		actionlib::SimpleClientGoalState::StateEnum state = client.getState();
//		switch( state ){
//			case actionlib::SimpleClientGoalState::PENDING:
//			case actionlib::SimpleClientGoalState::ACTIVE :
//				active = true;
//				break;
//			case actionlib::SimpleClientGoalState::RECALLED:
//			case actionlib::SimpleClientGoalState::REJECTED :
//			case actionlib::SimpleClientGoalState::PREEMPTED:
//			case actionlib::SimpleClientGoalState::ABORTED :
//			case actionlib::SimpleClientGoalState::SUCCEEDED:
//			case actionlib::SimpleClientGoalState::LOST :
//				active = true;
//				break;
//		}
		active = !finished;
	}
	void RobilTaskProxy::terminate(){
		client.cancelGoal();
	}
	bool RobilTaskProxy::isActive(){
		return active;
	}
	std::string RobilTaskProxy::address()const{
		return "RobilTaskProxy("+tname+")";
	}


}

#undef PRINT
