#ifndef _PATH_PLANNING_SERVER_H_
#define _PATH_PLANNING_SERVER_H_


#include <actionlib/server/simple_action_server.h>
#include <C0_RobilTask/RobilTask.h>
#include <C0_RobilTask/RobilTaskAction.h>
#include <sstream>
#include <iostream>

using namespace std;
using namespace C0_RobilTask;
using namespace RobilTask;



class PathPlanningServer{

    typedef RobilTaskGoalConstPtr GOAL;
    typedef RobilTaskFeedback FEEDBACK;
    typedef RobilTaskResult RESULT;
    typedef actionlib::SimpleActionServer<RobilTaskAction> Server;
    
protected:
    ros::NodeHandle _node;
    Server _server;
    string _name;
    FEEDBACK _feedback;
    RESULT _result;

public:
    PathPlanningServer(string name = "/PathPlanning"):
        _server(_node, name, boost::bind(&PathPlanningServer::task, this, _1), false),
        _name(name)
    {
        _server.start();
        ROS_INFO("instance of %s started.",_name.c_str());
    }

    void finish(const int32_t& success, const string& plan){
    	if(success <= 0)
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

    void task(const GOAL &goal){
        int32_t success = SUCCESS; //FAULT, SUCCESS, PLAN
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
            ROS_INFO("%s: ITERATION %i", _name.c_str(), times);

            /* SLEEP BETWEEN LOOP ITERATIONS */
            boost::this_thread::sleep(boost::posix_time::millisec(100));
        }

        finish( success, plan );
    }

};

class PathPlanningFocusServer{

    typedef RobilTaskGoalConstPtr GOAL;
    typedef RobilTaskFeedback FEEDBACK;
    typedef RobilTaskResult RESULT;
    typedef actionlib::SimpleActionServer<RobilTaskAction> Server;

protected:
    ros::NodeHandle _node;
    Server _server;
    string _name;
    FEEDBACK _feedback;
    RESULT _result;

public:
    PathPlanningFocusServer(string name = "/PathPlanningFocus"):
        _server(_node, name, boost::bind(&PathPlanningFocusServer::task, this, _1), false),
        _name(name)
    {
        _server.start();
        ROS_INFO("instance of %s started.",_name.c_str());
    }

    void finish(const int32_t& success, const string& plan){
    	if(success <= 0)
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

    void task(const GOAL &goal){
        int32_t success = SUCCESS; //FAULT, SUCCESS, PLAN
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

        finish( success, plan );
    }

};



#endif //_PATH_PLANNING_SERVER_H_
