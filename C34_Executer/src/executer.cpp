/*
 * executer.cpp
 *
 *  Created on: Sep 27, 2012
 *      Author: dan
 */

#include "ros/ros.h"
#include "BTExecuter.h"
#include "TaskProxyConnectionByActionLib.h"

#include "boost/shared_ptr.hpp"

#include "C34_Executer/pwd.h"
#include "C34_Executer/cd.h"
#include "C34_Executer/ls.h"
#include "C34_Executer/run.h"
#include "C34_Executer/lookup.h"
#include "C34_Executer/stop.h"
#include "C34_Executer/step.h"
#include "C34_Executer/pause.h"
#include "C34_Executer/resume.h"
#include "C34_Executer/btstack.h"
#include "C34_Executer/help_msg.h"
#include "C34_Executer/whoIsRunning.h"
#include "C34_Executer/show_table_msg.h"
#include "C34_Executer/version.h"
#include "C34_Executer/save_file.h"
#include "C34_Executer/read_file.h"

#include "std_msgs/String.h"

using namespace C34_Executer;

ServerActions* actions;

bool command_help(help_msg::Request& req, help_msg::Response& res){
	if(!actions) return false;
	res.text = actions->help();
	return true;
}
bool command_show_lookup(show_table_msg::Request& req, show_table_msg::Response& res){
	if(!actions) return false;
	res.content = actions->show_lookup();
	return true;
}
bool command_show_address(show_table_msg::Request& req, show_table_msg::Response& res){
	if(!actions) return false;
	res.content = actions->show_address();
	return true;
}
bool command_pwd(pwd::Request& req, pwd::Response& res){
	if(!actions) return false;
	res.location = actions->pwd();
	return true;
}
bool command_read_file(read_file::Request& req, read_file::Response& res){
	if(!actions) return false;
	res.content = std::string("\n") + actions->readFile(req.filename);
	return true;
}
bool command_save_file(save_file::Request& req, save_file::Response& res){
	if(!actions) return false;
	actions->saveFile(req.filename, req.content);
	return true;
}
bool command_version(version::Request& req, version::Response& res){
	if(!actions) return false;
	res.version = actions->version();
	return true;
}
bool command_whoIsRunning(whoIsRunning::Request& req, whoIsRunning::Response& res){
	if(!actions) return false;
	//std::cout<<"command_whoIsRunning"<<std::endl;
	res.runningList = actions->whoIsRunning();
	return true;
}

bool command_cd(cd::Request& req, cd::Response& res){
	if(!actions) return false;
	try{
		res.location = actions->cd(req.path);
	}catch(std::string& err){
		res.location = std::string("ERROR: ")+err;
	}
	return true;
}

bool command_ls(ls::Request& req, ls::Response& res){
	if(!actions) return false;
	try{
		res.content = actions->ls();
	}catch(std::string& err){
		res.content = std::string("ERROR: ")+err;
	}
	return true;
}

bool command_lookup(lookup::Request& req, lookup::Response& res){
	if(!actions) return false;
	try{
		//res.output =
			actions->lookup(req.filename);
	}catch(std::string& err){
		res.output = std::string("ERROR: ")+err;
	}
	return true;
}

bool command_address(lookup::Request& req, lookup::Response& res){
	if(!actions) return false;
	try{
		//res.output =
			actions->address(req.filename, BTTaskProxyCreator::Ref(new RobilTaskProxyCreator()));
	}catch(std::string& err){
		res.output = std::string("ERROR: ")+err;
	}
	return true;
}

bool command_run(run::Request& req, run::Response& res){
	if(!actions) return false;
	try{
		//res.output =
			actions->run(req.tree_id, req.filename);
	}catch(std::string& err){
		res.output = std::string("ERROR: ")+err;
	}
	return true;
}

bool command_stop(stop::Request& req, stop::Response& res){
	if(!actions) return false;
	try{
		res.output = actions->stop(req.tree_id);
	}catch(std::string& err){
		res.output = std::string("ERROR: ")+err;
	}
	return true;
}
bool command_pause(pause::Request& req, pause::Response& res){
	if(!actions) return false;
	try{
		//res.output =
			actions->pause(req.tree_id);
	}catch(std::string& err){
		res.output = std::string("ERROR: ")+err;
	}
	return true;
}
bool command_step(step::Request& req, step::Response& res){
	if(!actions) return false;
	try{
	//res.output =
			actions->step(req.tree_id);
	}catch(std::string& err){
		res.output = std::string("ERROR: ")+err;
	}
	return true;
}
bool command_resume(resume::Request& req, resume::Response& res){
	if(!actions) return false;
	try{
	//res.output =
			actions->resume(req.tree_id);
	}catch(std::string& err){
		res.output = std::string("ERROR: ")+err;
	}
	return true;
}
bool command_stack(btstack::Request& req, btstack::Response& res){
	if(!actions) return false;
	try{
	res.content = actions->stack(req.tree_id);
	}catch(std::string& err){
		res.content = std::string("ERROR: ")+err;
	}
	return true;
}
bool command_dump(resume::Request& req, resume::Response& res){
	if(!actions) return false;
	try{
	//res.output =
			actions->dump(req.tree_id);
	}catch(std::string& err){
		res.output = std::string("ERROR: ")+err;
	}
	return true;
}

ros::Publisher* log_publisher=0;
void LogCallback(const std::string& message){
	if(!log_publisher || log_publisher->getNumSubscribers()==0) return;
	std_msgs::String msg; msg.data = message;
	log_publisher->publish(msg);
}

ros::Publisher* stop_publisher=0;
void ExeFinished(std::string exe_id, std::string result){
	std::stringstream message;
	message<<"ExeFinished: notification from "<<exe_id<<"\n"<<result;
	{logSystem<<message.str();}
	if(!stop_publisher || stop_publisher->getNumSubscribers()==0) return;
	std_msgs::String msg; msg.data = message.str();
	stop_publisher->publish(msg);
}

ros::Publisher* stack_publisher=0;
void ExeStack(std::string exe_id, int code, std::string node, std::string result){
	std::stringstream message;
	message<<"ExeStack: changed : "<<exe_id<<" code="<<code<<", node="<<node<<"\n"<<result;
	{logSystem<<message.str();}
	if(!stack_publisher || stack_publisher->getNumSubscribers()==0) return;
	std_msgs::String msg; msg.data = message.str();
	stack_publisher->publish(msg);
}


int main(int argc, char** argv){
	Params params(argc,argv);

	ros::init(argc, argv, "executer");
	ros::NodeHandle n;

	ros::Publisher log_publisher = n.advertise<std_msgs::String>("executer/dump_stream",100); ::log_publisher=&log_publisher;
	ros::Publisher stop_publisher = n.advertise<std_msgs::String>("executer/stop_stream",100); ::stop_publisher=&stop_publisher;
	ros::Publisher stack_publisher = n.advertise<std_msgs::String>("executer/stack_stream",100); ::stack_publisher=&stack_publisher;

	logSystem.setLogCallback(LogCallback);
	BTServer server;
	server.start();
	ServerActions actions(server); ::actions = &actions;
	server.finishCallback = ExeFinished;
	server.stackCallback = ExeStack;

#define RUN_SERVICE(N) ros::ServiceServer ss_##N = n.advertiseService("executer/"#N, command_##N);
	RUN_SERVICE(help)
	RUN_SERVICE(pwd)
	RUN_SERVICE(whoIsRunning)
	RUN_SERVICE(cd)
	RUN_SERVICE(ls)
	RUN_SERVICE(lookup)
	RUN_SERVICE(address)
	RUN_SERVICE(run)
	RUN_SERVICE(stop)
	RUN_SERVICE(step)
	RUN_SERVICE(pause)
	RUN_SERVICE(resume)
	RUN_SERVICE(stack)
	RUN_SERVICE(dump)
	RUN_SERVICE(show_lookup)
	RUN_SERVICE(show_address)
	RUN_SERVICE(version)
	RUN_SERVICE(read_file)
	RUN_SERVICE(save_file)
#undef RUN_SERVICE

	if(params.contains("help")){
		help_msg::Request req; help_msg::Response res;
		command_help(req,res);
	}
	if(params.contains("lookup")){
		std::cout<<"Set lookup to "<<params.get<std::string>("lookup")<<std::endl;
		lookup::Request req; lookup::Response res;
		req.filename = params.get<std::string>("lookup");
		command_lookup(req,res);
	}
	if(params.contains("address")){
		std::cout<<"Set address to "<<params.get<std::string>("address")<<std::endl;
		lookup::Request req; lookup::Response res;
		req.filename = params.get<std::string>("address");
		command_address(req,res);
	}else{
		std::cout<<"Set empty address list"<<std::endl;
		lookup::Request req; lookup::Response res;
		req.filename = "<tasks> </tasks>";
		command_address(req,res);
	}
	if(params.contains("show_lookup")){
		show_table_msg::Request req; show_table_msg::Response res;
		command_show_lookup(req,res);
	}
	if(params.contains("show_address")){
		show_table_msg::Request req; show_table_msg::Response res;
		command_show_address(req,res);
	}

	ros::spin();

	return 0;
}
