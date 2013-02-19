#ifndef __DUMMYTASK__HPP
#define __DUMMYTASK__HPP

#include <actionlib/server/simple_action_server.h>
#include <C0_RobilTask/RobilTask.h>
#include <C0_RobilTask/RobilTaskAction.h>
using namespace std;
using namespace C0_RobilTask;

ostream& operator<<(ostream& o, std::vector<string>& s){
	for(size_t i=0;i<s.size()-1;i++)
		o<<s[i]<<',';
	if(s.size()!=0)
		return o<<s[s.size()-1];
	return o;
}

class DummyTaskServer: public RobilTask {
protected:

	std::vector<string> params;
	
	int time;
	int retValue;
	
public:
    DummyTaskServer(std::string name, std::vector<string> par):
        RobilTask(name), params(par)
    {
		cout<<"params: "<<params<<endl;
		cout<<"  time: "<<find(params, "time=")<<" value="<<cast<int>(value(params,"time="))<<endl;
		cout<<"  return: "<<find(params, "return=")<<" value="<<cast<int>(value(params,"return="))<<endl;
		
		time = -1;
		if( find(params, "time=").size()>0 ){
			time = cast<int>(value(params,"time="));
		}
		
		retValue = 0;
		if( find(params, "return=").size()>0 ){
			retValue = cast<int>(value(params,"return="));
		}
    }

    bool exists(Arguments& args, std::string name){
		return args.find(name) != args.end();
	}
	
	std::string find(std::vector<string>& s, std::string key){
		for(size_t i=0;i<s.size();i++){
			if(s[i].find(key)==0) return s[i];
		}
		return "";
	}
	std::string value(std::vector<string>& s, std::string key){
		for(size_t i=0;i<s.size();i++){
			if(s[i].find(key)==0) return s[i].substr(s[i].find('=')+1);
		}
		return "";
	}
	template<class A> A cast(std::string name){
		std::stringstream n;n<<name;
		A a; n>>a;
		return a;
	}	
 
	template<class A> A cast(Arguments& args, std::string name){
		std::stringstream n;n<<args[name];
		A a; n>>a;
		return a;
	}
    
    TaskResult task(const string& name, const string& uid, Arguments& args) {
		int time = this->time;
		
		while(time != 0) {
			time -- ;
			
			if(isPreempt()){
				
				return TaskResult::Preempted();
			}
			
			ROS_INFO("%s: process...", _name.c_str());
			sleep(1);
		}
                
        
        //return TaskResult::Preempted();
        if( retValue > 0 ){
			return TaskResult(retValue, "ERROR");
		}
        return TaskResult(SUCCESS, "OK");
    }

};
#endif


#include "ros/ros.h"

int main(int argc, char **argv)
{
	if(argc<0) return 1;
	
	ROS_INFO("Start DummyTaskNode with Task %s", argv[1]);
	
	ros::init(argc, argv, "DummyTaskNode");
	ros::NodeHandle node;
	
	ROS_INFO("craete task");
	
	std::string tname (argv[1]);
	std::vector<string> params;
	if(argc>2){
		for(size_t i=2; i<argc; i++){
			params.push_back(argv[i]);
		}
	}
	DummyTaskServer task(tname, params);

	ros::spin();

	return 0;
}
