#include "ros/ros.h"
#include "ros/package.h"
// #include "BTExecuter.h"
// #include "TaskProxyConnectionByActionLib.h"

#include "boost/shared_ptr.hpp"

// #include "C34_Executer/pwd.h"
// #include "C34_Executer/cd.h"
// #include "C34_Executer/ls.h"
// #include "C34_Executer/run.h"
// #include "C34_Executer/lookup.h"
// #include "C34_Executer/stop.h"
// #include "C34_Executer/step.h"
// #include "C34_Executer/pause.h"
// #include "C34_Executer/resume.h"
// #include "C34_Executer/btstack.h"
// #include "C34_Executer/help_msg.h"
// #include "C34_Executer/whoIsRunning.h"
// #include "C34_Executer/show_table_msg.h"
// #include "C34_Executer/version.h"
// #include "C34_Executer/save_file.h"
// #include "C34_Executer/read_file.h"

#include "std_msgs/String.h"
#include <boost/regex.hpp>

#include <fstream>

#define NEWLINE printf("\n")

#include <map>
#include <set>

std::map<std::string,std::string> sent;
std::set<std::string> filter;
bool ignore_filter = false;


ros::Publisher* stack_publisher=0;
void ExeStack(std::string exe_id, std::string code, std::string node){
	std::stringstream message;
	message<<""<<code<<":"<<exe_id<<":"<<node;
	
	std::string out_mes = message.str();
	ROS_INFO("OUT: %s",out_mes.c_str());

	if(!stack_publisher || stack_publisher->getNumSubscribers()==0) return;
	
	if(!ignore_filter){
		if(filter.find(node) == filter.end()){
			ROS_INFO("MESSAGE IS FILTERED OUT");
			return;
		}
	}
	
	if(sent.find( node ) != sent.end() && sent[node]==code) return;
	
	sent[node] = code;

	std_msgs::String msg; msg.data = out_mes;
	stack_publisher->publish(msg);
}

void OnExeStackMessage(const std_msgs::String::ConstPtr & msg){
	ROS_INFO("MESSAGE: %s",msg->data.c_str());
	
	const boost::regex e(".*changed : (\\w+) code=(\\w+),.*\\[id=((\\w+[-,])+\\w+)\\].*");
	boost::smatch w;
	std::string m1;
	if(boost::regex_match(msg->data, w, e)){
		ExeStack(w[1], w[2], w[3]);
	}else{
	}	
}

void readFilters(std::string filename){
	std::ifstream file(filename.c_str());
	std::string line;
	while( std::getline(file, line) ){
		std::cout<<"Filter: "<<line<<std::endl;
		filter.insert(line);
	}
	file.close();
}

std::string replace(std::string s){
	std::stringstream ss;
	for(size_t i=0;i<s.size();i++){
		if(s[i]==' ' || s[i]=='/') ss<<'_';
		else ss<<s[i];
	}
	return ss.str();
}

std::string node_name = "stack_stream_filter";

int main(int argc, char** argv){
	std::string topicname="executer/stack_stream_filtered";
	if(argc>2){
		std::string f(argv[1]);
		if(f!="ignore")
			readFilters(argv[1]);
		else
			ignore_filter = true;
		topicname = std::string(argv[2]);
	}else{
		std::cout<<"Syntax: filter_file_or_ignore topic_name"<<std::endl;
		return 0;
	}
	
	node_name = node_name +"_"+ replace(topicname);
	
	ROS_INFO("Start node %s", node_name.c_str());
	
	ros::init(argc, argv, node_name);
	ros::NodeHandle node;
	
	
	ros::Subscriber stack_stream = node.subscribe("/executer/stack_stream", 1000, &OnExeStackMessage);
	ros::Publisher stack_publisher = node.advertise<std_msgs::String>(topicname.c_str(),100); ::stack_publisher=&stack_publisher;

	ros::spin();
	
	ROS_INFO("Stop node %s", node_name.c_str());
	return 0;
}

