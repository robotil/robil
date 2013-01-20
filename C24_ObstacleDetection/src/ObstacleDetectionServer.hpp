
#include <actionlib/server/simple_action_server.h>
#include <C0_RobilTask/RobilTask.h>
#include <C0_RobilTask/RobilTaskAction.h>

using namespace std;
using namespace C0_RobilTask;

class ObstacleDetectionServer: public RobilTask{
public:
    ObstacleDetectionServer():
        RobilTask("/ObstacleDetection")
    {
	    
    }

    TaskResult task(const string& name, const string& uid, Arguments& args){
       /* HERE PROCESS TASK PARAMETERS */
	    
	while(true){
		if (isPreempt()){

			/* HERE PROCESS PREEMPTION OR INTERAPT */

			return TaskResult::Preempted();
		}
		 /* HERE PROCESS TASK */

		 ROS_INFO(STR(_name<<": in progress..."));
		 
		 //ON SUCCESS : return TaskResult(SUCCESS, "Alright");
		 //ON FALURE  : return TaskResult(FAULT, "Some problem detected");
		 
		 /* SLEEP BETWEEN LOOP ITERATIONS */
		sleep(100);
	}
	return TaskResult::FAULT();
    }

};
