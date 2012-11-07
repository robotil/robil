/*
 * Test2TaskProxyTable.cpp
 *
 *  Created on: Oct 2, 2012
 *      Author: dan
 */

#include "TaskProxyConnectionByActionLib.h"
#define PRINT(X) std::cout<<"TaskProxyConnectionByActionLib: "<<X<<std::endl;

namespace TaskProxyConnectionByActionLib{

typedef actionlib::SimpleActionClient<c0_RobilTask::RobilTaskAction> Client;
using namespace c0_RobilTask;


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
		client.waitForServer();
		c0_RobilTask::RobilTaskGoal msg_goal;
		msg_goal.name = goal.name;
		msg_goal.uid = goal.uid;
		msg_goal.parameters = goal.parameters;
		PRINT("goal{name="<<msg_goal.name<<",uid="<<msg_goal.uid<<",parameters="<<msg_goal.parameters<<"}");
		client.sendGoal(msg_goal);
		active = true;
	}
	BTTaskFeedback RobilTaskProxy::getFeedback (){
		return BTTaskFeedback();
	}
	BTTaskResult RobilTaskProxy::getResult (){
		BTTaskResult res;
		actionlib::SimpleClientGoalState state = client.getState();
		if(state==actionlib::SimpleClientGoalState::SUCCEEDED){
			res.description=client.getResult()->description;
			res.plan=client.getResult()->plan;
			res.success = client.getResult()->success;
		}else{
			res.description="Result does not available";
			res.plan="";
			res.success = 0;
		}
		PRINT("result{success="<<res.success<<",plan="<<res.plan<<",desc="<<res.description<<"}");
		return res;
	}
	void RobilTaskProxy::waitResult(double time){
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
