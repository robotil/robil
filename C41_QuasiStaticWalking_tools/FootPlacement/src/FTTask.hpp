
#include "FootPlacementService.h"

#include <actionlib/server/simple_action_server.h>
#include <C0_RobilTask/RobilTask.h>
#include <C0_RobilTask/RobilTaskAction.h>
#include <C0_RobilTask/StringOperations.h>

using namespace std;
using namespace C0_RobilTask;

class TaskCheckExtremeSlope:public RobilTask{
protected:

public:
	TaskCheckExtremeSlope(string name="/checkExtremeSlope"):
		RobilTask(name)
	{
		
	}
	TaskResult task(const string& name, const string& uid, Arguments& args){

		ros::ServiceClient client = _node.serviceClient<FootPlacement::FootPlacement_Service>("foot_placement");
		FootPlacement::FootPlacement_Service srv;

		while(true){
			if (isPreempt()){
				return TaskResult::Preempted();
			}
			
			init(srv);
			
			if (client.call(srv)) {
				ROS_INFO("foot_placement call is successed");
			}else{
				ROS_INFO("foot_placement call is fault");
			}
			
			sleep(1000);	
		}
		return TaskResult::FAULT();
	}
	
private:
	void init( FootPlacement::FootPlacement_Service& srv ){
		srv.request.start_pose.foot_index = LEFT;
		srv.request.start_pose.pose.position.x = 0.0;
		srv.request.start_pose.pose.position.y = 0.0;
		srv.request.start_pose.pose.position.z = 0.0;
		srv.request.start_pose.pose.ang_euler.x = 0.0;
		srv.request.start_pose.pose.ang_euler.y = 0.0;
		srv.request.start_pose.pose.ang_euler.z = 0.0;
		srv.request.other_foot_pose.foot_index = RIGHT;
		srv.request.other_foot_pose.pose.position.x = 0.0;
		srv.request.other_foot_pose.pose.position.y = 0.0;
		srv.request.other_foot_pose.pose.position.z = 0.0;
		srv.request.other_foot_pose.pose.ang_euler.x = 0.0;
		srv.request.other_foot_pose.pose.ang_euler.y = 0.0;
		srv.request.other_foot_pose.pose.ang_euler.z = 0.0;
	}
};
