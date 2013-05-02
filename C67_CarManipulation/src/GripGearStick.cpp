#ifndef __GRIPGEARSTICK__HPP
#define __GRIPGEARSTICK__HPP

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

class GripGearStickServer: public RobilTask {
protected:
	enum Consts { Time = 5 };
	enum Errors { NoParams = 1 };
	std::vector<string> params;
	int operation;
	int time;
	int retValue;
	string outputstr;
	
public:
	GripGearStickServer(std::string name, std::vector<string> par):
        RobilTask(name), params(par)
    {
		cout<<"params: "<<params<<endl;
		
//		time = -1;
//		if( find(params, "time=").size()>0 ){
//			time = cast<int>(value(params,"time="));
//		}
		time = (int)Time;
		
		retValue = 0;
		if( find(params, "return=").size()>0 ){
			retValue = cast<int>(value(params,"return="));
		}
		
		outputstr="process...";
		if( find(params, "print=").size()>0 ){
			outputstr = value(params,"print=");
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
		// initiate return value
		retValue = 0;
		
		if (!exists(args,"operation"))
		{
			ROS_INFO("%s: No operation Defined!", _name.c_str());
			retValue  = NoParams;
			return TaskResult(retValue, "ERROR");
		}

		operation = cast<int>(args["operation"]);
		ROS_INFO("%s: Target Operation %d - %s", _name.c_str(), operation, (operation == 1? "Drive":(operation == 2? "Park": "Reverse")));

		while(time >= 0) {
			time -- ;
			
			if(isPreempt()){
				
				return TaskResult::Preempted();
			}
			
			if(outputstr!="no_print"){
				ROS_INFO("%s: %s", _name.c_str(), outputstr.c_str());
			}
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
	std::string tname ("GripGearStick");
	
	if(tname=="-h" || tname=="--help"){
		cout<<"GripGearStick Server"<<endl;
		cout<<"Syntax: GripGearStickServer [ARG1=VALUE] [ARG2=VALUE] ..."<<endl;
		cout<<"Arguments:"<<endl;
		//cout<<"    time=INTEGER    : time (in mS) for task running. if -1, wait forever"<<endl;
		cout<<"    return=INTEGER  : return value of task: 0 is OK, -1 is PLAN, 0< is a ERROR CODE "<<endl;
		cout<<"    print=STRING    : print progress text. default is 'process...' and 'no_print' suppress printing."<<endl;
		return 0;
	}
	
	stringstream snodename; snodename<<"GripGearStickServerNode_"<<time(NULL);
	ROS_INFO("Start %s", snodename.str().c_str());
	
	ros::init(argc, argv, snodename.str().c_str());
	ros::NodeHandle node;
	
	ROS_INFO("create task");
	

	std::vector<string> params;
	if(argc>1){
		for(int i=1; i<argc; i++){
			params.push_back(argv[i]);
		}
	}
	if(argc == 1) params.push_back("No Parameters");
	GripGearStickServer task(tname, params);

	ros::spin();

	return 0;
}
