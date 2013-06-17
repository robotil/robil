#ifndef __PLACEFEETONPEDALS__HPP
#define __PLACEFEETONPEDALS__HPP

#include <actionlib/server/simple_action_server.h>
#include <C0_RobilTask/RobilTask.h>
#include <C0_RobilTask/RobilTaskAction.h>
using namespace std;
using namespace C0_RobilTask;


#include <atlas_msgs/AtlasSimInterfaceCommand.h>
#include <PedalManipulation/PedalsCalibration.h>
#include <std_srvs/Empty.h>


ostream& operator<<(ostream& o, std::vector<string>& s){
	for(size_t i=0;i<s.size()-1;i++)
		o<<s[i]<<',';
	if(s.size()!=0)
		return o<<s[s.size()-1];
	return o;
}

class PlaceFeetOnPedalsServer: public RobilTask {
protected:
	enum Consts { Time = 5 };
	enum Errors { NoParams = 1 };
	std::vector<string> params;
	double angle;
	int time;
	int retValue;
	string outputstr;
	
public:
	PlaceFeetOnPedalsServer(std::string name, std::vector<string> par):
        RobilTask(name), params(par)
	{

	}

	bool exists(Arguments& args, std::string name){
		return args.find(name) != args.end();
	}
/*	
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
	}*/
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
		try{
		
		ros::Publisher sic = _node.advertise<atlas_msgs::AtlasSimInterfaceCommand>("/atlas/atlas_sim_interface_command", 100);
		atlas_msgs::AtlasSimInterfaceCommand sic_msg;
		
		ros::ServiceClient pms = _node.serviceClient<PedalManipulation::PedalsCalibration>("/PedalsManipulation/calibrate");
		PedalManipulation::PedalsCalibration pms_msg;

		ros::ServiceClient st_start = _node.serviceClient<PedalManipulation::PedalsCalibration>("/PoseController/start");
		//ros::ServiceClient st_stop = n.serviceClient<PedalManipulation::PedalsCalibration>("/PedalsManipulation/calibrate");
		std_srvs::Empty st_msg;
		
		ROS_INFO("PlaceFeetOnPedalsServer: wait for subscribers for /atlas/atlas_sim_interface_command");
		while(sic.getNumSubscribers()<1 && !isPreempt());
		if(isPreempt()) ROS_INFO("PlaceFeetOnPedalsServer: break (isPreempt)");
		else ROS_INFO("PlaceFeetOnPedalsServer: continue");
		
		int counter=0;
		while(true) {
			
			if(isPreempt()){
				ROS_INFO("PlaceFeetOnPedalsServer: stop task (isPreempt)");
				return TaskResult::Preempted();
			}
			
			init(sic_msg);
			sic.publish(sic_msg);
			
			if(counter==0){
				ROS_INFO("PlaceFeetOnPedalsServer: time = 1");
				if( st_start.call(st_msg) ){
					ROS_INFO("PlaceFeetOnPedalsServer: /PoseController/start - ok");
				}else{
					ROS_INFO("PlaceFeetOnPedalsServer: /PoseController/start - fault");
				}
				
				init(pms_msg);
				if( pms.call(pms_msg) ){
					ROS_INFO("PlaceFeetOnPedalsServer: /PedalsManipulation/calibrate - ok");
				}else{
					ROS_INFO("PlaceFeetOnPedalsServer: /PedalsManipulation/calibrate - fault");
				}
			}
			
			if(counter==1){
				ROS_INFO("PlaceFeetOnPedalsServer: time = 2");
				init(pms_msg);
				if( pms.call(pms_msg) ){
					ROS_INFO("PlaceFeetOnPedalsServer: /PedalsManipulation/calibrate - ok");
				}else{
					ROS_INFO("PlaceFeetOnPedalsServer: /PedalsManipulation/calibrate - fault");
				}
			}

			if(counter==2){
				ROS_INFO("PlaceFeetOnPedalsServer: time = 3");
				if( st_start.call(st_msg) ){
					ROS_INFO("PlaceFeetOnPedalsServer: /PoseController/start - ok");
				}else{
					ROS_INFO("PlaceFeetOnPedalsServer: /PoseController/start - fault");
				}
				break;
			}
			
			sleep(100); counter++;
		}
		
		}catch(...){
			ROS_INFO("PlaceFeetOnPedalsServer: WARNING: exception catched.");
		}
		ROS_INFO("PlaceFeetOnPedalsServer: stop task");
		return TaskResult(SUCCESS, "OK");
    }
    
    void init(atlas_msgs::AtlasSimInterfaceCommand& sic_msg){
	sic_msg.behavior = 1;
    }
    void init(PedalManipulation::PedalsCalibration& msg){
	msg.request.right_pedal_location.x= 0.50;
	msg.request.right_pedal_location.y= -0.2;
	msg.request.right_pedal_location.z= -0.4;
	msg.request.left_pedal_location.x= 0.51;
	msg.request.left_pedal_location.y= 0.0;
	msg.request.left_pedal_location.z= -0.4;
    }

};
#endif


#include "ros/ros.h"

int main(int argc, char **argv)
{
	if(argc<0) return 1;
	std::string tname ("PlaceFeetOnPedals");
	
	if(tname=="-h" || tname=="--help"){
		cout<<"PlaceFeetOnPedals Server"<<endl;
// 		cout<<"Syntax: PlaceFeetOnPedalsServer [ARG1=VALUE] [ARG2=VALUE] ..."<<endl;
// 		cout<<"Arguments:"<<endl;
// 		//cout<<"    time=INTEGER    : time (in mS) for task running. if -1, wait forever"<<endl;
// 		cout<<"    return=INTEGER  : return value of task: 0 is OK, -1 is PLAN, 0< is a ERROR CODE "<<endl;
// 		cout<<"    print=STRING    : print progress text. default is 'process...' and 'no_print' suppress printing."<<endl;
		return 0;
	}
	
	stringstream snodename; snodename<<"PlaceFeetOnPedalsServerNode_"<<time(NULL);
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
	PlaceFeetOnPedalsServer task(tname, params);

	ros::spin();

	return 0;
}
