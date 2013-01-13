/*
 * This class calculates the center of mass error.
 * Dependencies:
 *		joint_states topic
 *
 * Advertises:
 * 		com_error service
 * 		support_legs service
 * 		CoM topic
 * 		projected_com topic
 * 		support_polygon topic
 * 		stability_visualization topic
 */

#ifndef COM_ERROR_NODE_H_
#define COM_ERROR_NODE_H_

#include "ros/ros.h"
#include <hrl_kinematics/TestStability.h>
#include "C45_PostureControl/com_error.h"
//#include <hrl_kinematics/SupportLegs_Status.h>
#include <string>

//namespace hrl_kinematics{

class com_error_node{

protected:
	ros::NodeHandle nh_,nh2_, nh3_;
	ros::Subscriber pcom_sub_, _com_subsriber;
	ros::Subscriber support_polygon_sub_, _support_polygon_subsriber;
	ros::ServiceServer com_error_srv_;
	ros::ServiceServer support_mode_srv_;
	ros::Subscriber joint_state_sub_;
	//TestStability test_stability_;
	ros::Publisher visualization_pub_, support_polygon_pub_,support_polygon_pub_1, pcom_pub_, com_pub_;
	//Kinematics::FootSupport support_mode_, support_mode_status_;
private:
	geometry_msgs::Point _CoM;
	geometry_msgs::Point _SP;


public:
	com_error_node();
	~com_error_node();
	bool result_com_error(C45_PostureControl::com_error::Request &req, C45_PostureControl::com_error::Response &res);
	void get_com_from_hrl_kinematics(const visualization_msgs::MarkerConstPtr& comMarker);
	void get_support_polygon_from_hrl_kinematics(const geometry_msgs::PolygonStampedConstPtr& support_polygon);

	//void jointStateCb(const sensor_msgs::JointStateConstPtr& state);
	//bool FootSupport_func(hrl_kinematics::SupportLegs_Status::Request &req, hrl_kinematics::SupportLegs_Status::Response &res);
};
//}

#endif /* COM_ERROR_NODE_H_ */
