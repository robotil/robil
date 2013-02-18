
#include <actionlib/server/simple_action_server.h>
#include <C0_RobilTask/RobilTask.h>
#include <C0_RobilTask/RobilTaskAction.h>
#include <C0_RobilTask/StringOperations.h>

using namespace std;
using namespace C0_RobilTask;

class TrackObjectServer:public RobilTask{
	C23_Node& myNode;
public:
    TrackObjectServer(C23_Node& nodeRef, string name ="/TrackObject"):
        RobilTask(name), myNode(nodeRef)
    {

    }

    template<class A> castArg(const std::string& p){
		A a; std::stringstream s(p); s>>a; return a;
	}
    TaskResult task(const string& name, const string& uid, Arguments& args){
        
        /* GET TASK PARAMETERS */
        // check if argument exitsts : args.find("arg_name")!=args.end()
        // get argument : args["arg_name"]
        // cast argument to int : castArg<int>(args["arg_name"]);
        // cast argument to double : castArg<double>(args["arg_name"]);
        
        /* HERE PROCESS TASK PARAMETERS */

        while(true){
            if (isPreempt()){

                /* HERE PROCESS PREEMPTION OR INTERAPT */

            	return TaskResult::Preempted();
            }
            
            /* HERE PROCESS TASK */
			
			//return TaskResult(SUCCESS, "OK");
			//return TaskResult(FAULT, "Unexpected ..... ");

            /* SLEEP BETWEEN LOOP ITERATIONS */
            sleep(1000);
        }

        return TaskResult::FAULT();
    }

};
