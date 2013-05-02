
#include <ros/ros.h>

#include "Tasks_stub.hpp"


int main_for_qual1_1(int argc, char** argv){
	  ros::init(argc, argv, "C31_GlobalPathPlanner_stub11");
	  ros::NodeHandle node;

	  ROS_INFO("Start node with stubs for qual1.1");

	  //Stubs for planning tasks
	  PathPlanning task_pathplanning;
	  PathPlanningFocus task_pathplanningfocus;

	  ROS_INFO("spin node");
	  ros::spin();
	  return 0;

}

int main_for_qual1_2(int argc, char** argv){
	  ros::init(argc, argv, "C31_GlobalPathPlanner_stub12");
	  ros::NodeHandle node;

	  ROS_INFO("Start node with stubs for qual1.2");

	  //Stub for walking tasks
	  WhileInQSArea task_whileQS;
	  WhileInZMPArea task_whileZmp;
	  ObjectRecognition task_searchObject;

	  ROS_INFO("spin node");
	  ros::spin();
	  return 0;

}


int main_for_qual1_1and2(int argc, char** argv){
	  ros::init(argc, argv, "C31_GlobalPathPlanner_stub112");
	  ros::NodeHandle node;

	  ROS_INFO("Start node with stubs for qual1.1 and qual1.2");

	  //Stubs for planning tasks
	  PathPlanning task_pathplanning;
	  PathPlanningFocus task_pathplanningfocus;

	  //Stub for walking tasks
	  WhileInQSArea task_whileQS;
	  WhileInZMPArea task_whileZmp;
	  ObjectRecognition task_searchObject;

	  ROS_INFO("spin node");
	  ros::spin();
	  return 0;

}


int main(int argc, char** argv){
	for(int i=0;i<argc;i++){
		string a(argv[i]);
		if(a=="-h" || a=="--help"){
			std::cout<<"Run qualification stub tasks"<<std::endl;
			std::cout<<"Syntax: "<<argv[0]<<" PARAMS"<<std::endl;
			std::cout<<"PARAMS: "<<std::endl;
			std::cout<<"        "<<"qual1.1"<<"  : "<<"for qualification 1.1 ( PathPlanning and PathPlanningFocus )"<<std::endl;
			std::cout<<"        "<<"qual1.1"<<"  : "<<"for qualification 1.2 ( whileQS, whileZmp, searchObject )"<<std::endl;
			std::cout<<"        "<<"qual1.all"<<": "<<"for all"<<std::endl;
			return 0;
		}
		if(a=="qual1.1") return main_for_qual1_1(argc, argv);
		if(a=="qual1.2") return main_for_qual1_2(argc, argv);
		if(a=="qual1.all") return main_for_qual1_1and2(argc, argv);
	}
	return main_for_qual1_1and2(argc, argv);
}

