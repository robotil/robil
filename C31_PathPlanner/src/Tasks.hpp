#ifndef _PATH_PLANNING_SERVER_H_
#define _PATH_PLANNING_SERVER_H_


#include <actionlib/server/simple_action_server.h>
#include <C0_RobilTask/RobilTask.h>
#include <C0_RobilTask/RobilTaskAction.h>
#include <C0_RobilTask/StringOperations.h>
#include <sstream>
#include <iostream>

#include "cogniteam_pathplanning.h"

using namespace std;
using namespace C0_RobilTask;
using namespace RobilTask;


struct Editable_Constraints{
	RobotDimentions& dimentions;
	Transits& transits;
	Attractors& attractors;
	Editable_Constraints(RobotDimentions& dimentions, Transits& transits, Attractors& attractors)
	:dimentions(dimentions),transits(transits),attractors(attractors){}
};

class PlanningArguments{
public:
	const Map& map;
	const Waypoint& start;
	const Waypoint& finish;
	PlanningArguments(const Map& map, const Waypoint& start, const Waypoint& finish)
	:map(map), start(start), finish(finish){}
};
class Editable_PlanningArguments{
public:
	Map& map;
	Waypoint& start;
	Waypoint& finish;
	Editable_PlanningArguments(Map& map, Waypoint& start, Waypoint& finish)
	:map(map), start(start), finish(finish){}
};

class PathPlanning{
	boost::mutex _mtx;
#define SYNCHRONIZED boost::mutex::scoped_lock l(_mtx);

	Map map;
	Waypoint start;
	Waypoint finish;

	RobotDimentions dimentions;
	Transits transits;
	Attractors attractors;

	PlanningArguments arguments;
	Constraints constraints;
	Editable_PlanningArguments ed_arguments;
	Editable_Constraints ed_constraints;
public:

	PathPlanning():
		map(0,0),
		arguments(map, start, finish), constraints(dimentions, transits, attractors),
		ed_arguments(map, start, finish), ed_constraints(dimentions, transits, attractors)
	{

	}

	Path plan(){ SYNCHRONIZED

		Path path = searchPath(arguments.map, arguments.start, arguments.finish, constraints);

		return path;
	}

	class EditSession{
		boost::shared_ptr<boost::mutex::scoped_lock> l;
	public:
		Editable_Constraints& constraints;
		Editable_PlanningArguments& arguments;
		EditSession(boost::mutex& _mtx, Editable_Constraints& cons, Editable_PlanningArguments& arguments):l(new boost::mutex::scoped_lock(_mtx)),constraints(cons),arguments(arguments) {}
		EditSession(const EditSession& e):l(e.l),constraints(e.constraints),arguments(e.arguments)  {}
	};

	EditSession startEdit(){
		return EditSession(_mtx, ed_constraints, ed_arguments);
	}

#undef SYNCHRONIZED
};


class PathPlanningTask{
protected:
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

    PathPlanning& _planner;

    template <typename A>
    PathPlanningTask(PathPlanning& planner, string name, A taskFunc):
		_server(_node, name, taskFunc, false),
		_name(name), _planner(planner)
	{
		_server.start();
		ROS_INFO("instance of %s started.",_name.c_str());
	}

	void finish(const int32_t& success, const std::string& description, const string& plan){
		_result.success = success;
		_result.description = description;
		if(success <= 0)
		{
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

class PathPlanningServer:public PathPlanningTask{



public:
    PathPlanningServer(PathPlanning& planner, string name = "/PathPlanning"):
    	PathPlanningTask(planner, name, boost::bind(&PathPlanningServer::task, this, _1))
    {

    }



    void task(const GOAL &goal){
        int32_t success = SUCCESS; //FAULT, SUCCESS, PLAN
        string plan ="";
        string desc = "";

        /* GET TASK PARAMETERS */
        ROS_INFO("%s: Start: task name = %s", _name.c_str(), goal->name.c_str());
        ROS_INFO("%s: Start: task id = %s", _name.c_str(), goal->uid.c_str());
        ROS_INFO("%s: Start: task params = %s", _name.c_str(), goal->parameters.c_str());

        /* HERE PROCESS TASK PARAMETERS */
        Arguments args = parseArguments(goal->parameters);

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

        finish( success, desc, plan );
    }

};

class PathPlanningFocusServer:public PathPlanningTask{

public:
    PathPlanningFocusServer(PathPlanning& planner, string name = "/PathPlanningFocus"):
    	PathPlanningTask(planner, name, boost::bind(&PathPlanningFocusServer::task, this, _1))
    {

    }

    void task(const GOAL &goal){
        int32_t success = SUCCESS; //FAULT, SUCCESS, PLAN
        string desc = "";
        string plan ="";
        
        /* GET TASK PARAMETERS */
        ROS_INFO("%s: Start: task name = %s", _name.c_str(), goal->name.c_str());
        ROS_INFO("%s: Start: task id = %s", _name.c_str(), goal->uid.c_str());
        ROS_INFO("%s: Start: task params = %s", _name.c_str(), goal->parameters.c_str());
        
        /* HERE PROCESS TASK PARAMETERS */

        Arguments args = parseArguments(goal->parameters);

        /* NUMBER OF ITERATIONS IN TASK LOOP */
        for(int times =0; times < 1; times++){
            if (_server.isPreemptRequested() || !ros::ok()){
            
                /* HERE PROCESS PREEMPTION OR INTERAPT */
            
                ROS_INFO("%s: Preempted", _name.c_str());
                _server.setPreempted();
                success = FAULT;
                break;
            }
            
            /* HERE PROCESS TASK */

            if( args.find("x")!=args.end() && args.find("y")!=args.end() ){
            	std::stringstream numbers; numbers<<args["x"]<<','<<args["y"];
            	char c; double x, y;
            	numbers>>x>>c>>y;
            	ROS_INFO("%s: set planning goal [x,y] = %f, %f", _name.c_str(), x, y);
            	PathPlanning::EditSession session = _planner.startEdit();
            	session.arguments.finish.x=x;
            	session.arguments.finish.y=y;
            }else{
            	success = FAULT;
            	desc = "I don't know to set path planner arguments from current parameters";
            	ROS_INFO("%s: ERROR: %s", _name.c_str(), desc.c_str());
            	break;
            }

            /* SLEEP BETWEEN LOOP ITERATIONS */
            boost::this_thread::sleep(boost::posix_time::millisec(100));
        }

        finish( success, desc, plan );
    }

};



#endif //_PATH_PLANNING_SERVER_H_
