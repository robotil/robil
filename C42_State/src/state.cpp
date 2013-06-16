#include <ros/ros.h>

#include <C42_State/MotionTypeInfo.h>
#include <C42_State/StandingPositionInfo.h>

C42_State::MotionType motion_type;
C42_State::StandingPosition standing_position;

void on_new_motion_type(const C42_State::MotionTypeConstPtr msg){
	motion_type = *msg;
}
void on_new_standing_position(const C42_State::StandingPositionConstPtr msg){
	standing_position = *msg;
}


bool srv_motion_type(C42_State::MotionTypeInfo::Request& req, C42_State::MotionTypeInfo::Response& res){
	res.info = motion_type;
	return true;
}
bool srv_standing_position(C42_State::StandingPositionInfo::Request& req, C42_State::StandingPositionInfo::Response& res){
	res.info = standing_position;
	return true;
}

int main(int a, char** c){
	ros::init(a, c, "C42_State");
	ROS_INFO("Node C42_State is started");

	motion_type.motion = C42_State::MotionType::motion_continuous_walk;
	standing_position.state=C42_State::StandingPosition::state_standing;

	ros::NodeHandle node;

	ROS_INFO("subscribe to topic /motion_state/motion_type as C42_State::MotionType");
	ros::Subscriber mt = node.subscribe("/motion_state/motion_type",100,on_new_motion_type);

	ROS_INFO("subscribe to topic /motion_state/standing_position as C42_State::StandingPosition");
	ros::Subscriber sp = node.subscribe("/motion_state/standing_position",100,on_new_standing_position);

	ROS_INFO("advertise service /motion_state/info/motion_type as  C42_State::MotionTypeInfo");
	ros::ServiceServer mts = node.advertiseService("/motion_state/info/motion_type", srv_motion_type);

	ROS_INFO("advertise service /motion_state/info/standing_position as C42_State::StandingPositionInfo");
	ros::ServiceServer sps = node.advertiseService("/motion_state/info/standing_position", srv_standing_position);

	ros::spin();

	ROS_INFO("Node C42_State is closed");
	return 0;
}
