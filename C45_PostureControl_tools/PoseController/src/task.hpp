
#include <actionlib/server/simple_action_server.h>
#include <C0_RobilTask/RobilTask.h>
#include <C0_RobilTask/RobilTaskAction.h>
#include <C0_RobilTask/StringOperations.h>

#include <PoseController/back_lbz_neck_ay.h>

using namespace std;
using namespace C0_RobilTask;

class TaskResetHead:public RobilTask{
protected:

public:
	TaskResetHead(string name="/resetHead"):
		RobilTask(name)
	{
		
	}
	TaskResult task(const string& name, const string& uid, Arguments& args){
		try{

			ros::ServiceClient srv_client = _node.serviceClient<PoseController::back_lbz_neck_ay>("/PoseController/back_lbz_neck_ay");
			PoseController::back_lbz_neck_ay srv;

			//while(true)
			{
				if (isPreempt()){
					return TaskResult::Preempted();
				}

				
				srv.request.back_lbz=0;
				if(args.find("neck")==args.end()){
					srv.request.neck_ay=0;
				}else{
					std::stringstream o; o<<args["neck"];
					o>>srv.request.neck_ay;
				}
					
			
				if (srv_client.call(srv)) {
					ROS_INFO("TaskResetHead: /PoseController/back_lbz_neck_ay call is successed");
				}else{
					ROS_INFO("TaskResetHead: /PoseController/back_lbz_neck_ay call is fault");
				}
			

				sleep(100);	
			}
		}catch(...){
			ROS_INFO("TaskResetHead: WARNING: some exception is chatched.");
		}

		ROS_INFO("TaskResetHead: finished");
		return TaskResult::SUCCESS();
	}

};
