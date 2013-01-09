/*
 * SimpleTask.cpp
 *
 *  Created on: Sep 27, 2012
 *      Author: dan
 */

#ifndef _DEF_ROBIL_TASK
#define _DEF_ROBIL_TASK



namespace RobilTask{
	static const int32_t FAULT = 1;
	static const int32_t SUCCESS = 0;
	static const int32_t PLAN=-1;
}


#include <actionlib/server/simple_action_server.h>
#include <C0_RobilTask/RobilTaskAction.h>
#include <C0_RobilTask/StringOperations.h>
#include <sstream>
#include <iostream>

namespace RobilTask{

	using namespace std;
	using namespace C0_RobilTask;

	class AbstractTask{
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

	    AbstractTask(string name):
			_server(_node, name, boost::bind(&AbstractTask::abstract_task, this, _1), false),
			_name(name)
		{
			_server.start();
			ROS_INFO("instance of %s started.",_name.c_str());
		}

	    virtual ~AbstractTask(){}

		void finish(const int32_t& success, const std::string& description, const string& plan, bool AbortForUnsuccess = true){
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
				if(AbortForUnsuccess){
					ROS_INFO("%s: Aborted", _name.c_str());
					_server.setAborted(_result);
				}else{
					ROS_INFO("%s: Preempted", _name.c_str());
					//_server.setPreempted();
				}
			}
		}

		class TaskResult{
			friend class AbstractTask;
		protected:
			int32_t success;
			string plan;
			string description;
			bool preempted;

			TaskResult(bool preempted)
			:success(RobilTask::FAULT), plan(""), description("Preempted"), preempted(preempted){}

		public:
			TaskResult(const string& plan, const std::string& description )
			:success(RobilTask::PLAN), plan(plan), description(description), preempted(false){}

			TaskResult(const int32_t success, const std::string& description)
			:success(success), plan(""), description(description), preempted(false){}


			TaskResult(const TaskResult& o)
			:success(o.success), plan(o.plan), description(o.description), preempted(o.preempted){ }

			static TaskResult Preempted(){ return TaskResult(true); }
			static TaskResult FAULT(){ return TaskResult(RobilTask::FAULT,"Unexpected exception"); }
		};

		virtual AbstractTask::TaskResult task(const string& name, const string& uid, Arguments& args)=0;

		bool isPreempt(){
			return (_server.isPreemptRequested() || !ros::ok());
		}

		void sleep(long millisec){
			boost::this_thread::sleep(boost::posix_time::millisec(millisec));
		}

	    void abstract_task(const GOAL &goal){
	        /* GET TASK PARAMETERS */
	        ROS_INFO("%s: Start: task name = %s", _name.c_str(), goal->name.c_str());
	        ROS_INFO("%s: Start: task id = %s", _name.c_str(), goal->uid.c_str());
	        ROS_INFO("%s: Start: task params = %s", _name.c_str(), goal->parameters.c_str());

	        /* HERE PROCESS TASK PARAMETERS */
	        Arguments args;
	        try{
	        	args = parseArguments(goal->parameters);
	        }catch(...){
	        	ROS_INFO("%s: WARNING: argument parser report about parameters problem.", _name.c_str());
	        }

	        AbstractTask::TaskResult res = task(goal->name, goal->uid, args);

	        if(res.preempted){

	        	_server.setPreempted();
	        }
	        finish( res.success, res.description, res.plan, !res.preempted);
	    }
	};


}

class SSTREAM{
	typedef const char* c_str_t;
	boost::shared_ptr<std::stringstream> stream;
public:
	operator c_str_t()const{ return c_str(); }
	c_str_t c_str()const{ if(stream.get()) return stream->str().c_str(); return 0; }
	operator std::string()const{ return str(); }
	std::string str()const{ if(stream.get()) return stream->str(); return ""; }
	SSTREAM(const SSTREAM& str):stream(str.stream){}
	SSTREAM():stream(new std::stringstream()){}
	template <typename T>
	SSTREAM& operator<<(const T& t){ if(stream.get()) *(stream.get()) << t; return *this; }
};
#define STR(X) ((SSTREAM()<<X).c_str())

#endif // _DEF_ROBIL_TASK
