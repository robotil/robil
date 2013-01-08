
#include <actionlib/server/simple_action_server.h>
#include <C0_RobilTask/RobilTask.h>
#include <C0_RobilTask/RobilTaskAction.h>

using namespace std;
using namespace C0_RobilTask;

class ObstacleDetectionServer{

    typedef C0_RobilTask::RobilTaskGoalConstPtr GOAL;
    typedef C0_RobilTask::RobilTaskFeedback FEEDBACK;
    typedef C0_RobilTask::RobilTaskResult RESULT;
    typedef actionlib::SimpleActionServer<C0_RobilTask::RobilTaskAction> Server;
    int PLAN;
    int FAULT;
    
protected:
    ros::NodeHandle _node;
    string _name;
    Server _server;
    FEEDBACK _feedback;
    RESULT _result;

public:
    ObstacleDetectionServer():
    	_name("/ObstacleDetection"),
    	_server(_node, _name, boost::bind(&ObstacleDetectionServer::task, this, _1), false)
    {
        _server.start();
        ROS_INFO("instance of ObstacleDetectionServer started.");
        PLAN=0;
        FAULT=1;
    }

    void task(const GOAL &goal){
        int32_t success = PLAN;
        string plan ="";
        
        /* GET TASK PARAMETERS */
        ROS_INFO("%s: Start: task name = %s", _name.c_str(), goal->name.c_str());
        ROS_INFO("%s: Start: task id = %s", _name.c_str(), goal->uid.c_str());
        ROS_INFO("%s: Start: task params = %s", _name.c_str(), goal->parameters.c_str());
        
        /* HERE PROCESS TASK PARAMETERS */

        /* NUMBER OF ITERATIONS IN TASK LOOP */
        for(int times =0; times < 100; times++){
            if (_server.isPreemptRequested() || !ros::ok()){
            
                /* HERE PROCESS PREEMPTION OR INTERAPT */
            
                ROS_INFO("%s: Preempted", _name.c_str());
                _server.setPreempted();
                success = FAULT;
                break;
            }
            
            /* HERE PROCESS TASK */

            /* SLEEP BETWEEN LOOP ITERATIONS */
            boost::this_thread::sleep(boost::posix_time::millisec(100));
        }

        if(success-1)
        {
            _result.success = success;
            ROS_INFO("%s: Succeeded", _name.c_str());
            if(success == PLAN){
                ROS_INFO("%s: New plan", _name.c_str());
                _result.plan = plan;
            }
            _server.setSucceeded(_result);
        }else{
            ROS_INFO("%s: Aborted", _name.c_str());
            _server.setAborted(_result);
        }
    }

};
