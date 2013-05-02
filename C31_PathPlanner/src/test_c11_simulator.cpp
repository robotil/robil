#include <ros/ros.h>
#include <vector>
#include <string>
#include <C31_PathPlanner/C31_Waypoints.h>

using namespace std;



#define ADD(XX,YY) \
{\
	C31_PathPlanner::C31_Location l; \
	l.x = XX;\
	l.y = YY;\
	wp.points.push_back(l);\
}
void test1(ros::Publisher& p){
	ROS_INFO("pub: test1");
	C31_PathPlanner::C31_Waypoints wp;
	ADD(5.32660547242, 1.87535567751);
	p.publish(wp);
}
void test2(ros::Publisher& p){
	ROS_INFO("pub: test2");
	C31_PathPlanner::C31_Waypoints wp;
	ADD(2.94779506922, 1.8753001429);
	ADD(-3.42720493078,3.2503001429);
	p.publish(wp);
}
void test3(ros::Publisher& p){
	ROS_INFO("pub: test3");
	C31_PathPlanner::C31_Waypoints wp;
	ADD(-2.52478384641,	0.406098028301);
	ADD(3.56896615359,	1.4373480283);
	ADD(-3.30603384641,	2.4373480283);
	ADD(0.943966153587,	2.4373480283);
	p.publish(wp);
}

int main(int a, char** b){
	ros::init(a, b, "c11_simulator");
	ROS_INFO("Start node : c11_simulator");
	ros::NodeHandle node;
	ros::Publisher p = node.advertise<C31_PathPlanner::C31_Waypoints>("/c11_path_update", 10);
	ros::Rate r(1.0/5.0);
	while(ros::ok()){
		test1(p);r.sleep();
		test2(p);r.sleep();
		test3(p);r.sleep();

		ros::spinOnce();
	}


	return 0;
}
