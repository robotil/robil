/**************************************************************************************
 * This is a template for the C25_GlobalPosition module for the robil project
 * The C25_GlobalPosition module goal is to provide UTM global positioning according to the data
 * Received from GPS and camera sensors. data is relative to the pelvis of the drc robot
 **************************************************************************************/

#include "ros/ros.h"
#include "C25_GlobalPosition/C25.h"
#include "C25_GlobalPosition/C25C0_ROP.h"
#include "ros/service.h"
#include "LocalizationTrackServer.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

using namespace sensor_msgs;
using namespace nav_msgs;
using namespace message_filters;


/**
 * this class represent the C25_Node,
 **/
class C25_Node{

private:
  ros::NodeHandle nh_;
  ros::Publisher c25_publisher;
  ros::ServiceServer c25_service;
  typedef sync_policies::ApproximateTime<Imu, Odometry> MySyncPolicy;
  message_filters::Subscriber<Imu> imu_sub;
  message_filters::Subscriber<Odometry> pos_sub;
  Synchronizer<MySyncPolicy> sync;
  C25_GlobalPosition::C25C0_ROP last_msg;
  LocalizationTrackServer *taskserver;
public:

	 /**
	  * constructor, initializes the ROS node, subscribe it to the given topics and instruct it to provide the service
	  */
	  C25_Node(int argc, char **argv):
		  imu_sub(nh_,"imu",1),
		  pos_sub(nh_,"ground_truth_odom",1),
		  sync(MySyncPolicy(10), imu_sub, pos_sub)
	  {
		  sync.registerCallback( boost::bind( &C25_Node::callback, this, _1, _2 ) );
		  c25_publisher=nh_.advertise<C25_GlobalPosition::C25C0_ROP>("C25/publish",100);
		  c25_service=nh_.advertiseService("C25/service",&C25_Node::proccess,this);
		  ROS_INFO("service on\n");
		boost::thread mythread( &C25_Node::startActionServer,this,argc,argv);
	  }


	  void startActionServer(int argc, char **argv){
		  ros::init(argc, argv, "C24_ObstacleDetectionTaskServer");
		  taskserver=new LocalizationTrackServer();
		  while(ros::ok()){

		  }
	  }

	  /**
	   * The call back function executed when a service is requested
	   */
	  bool proccess(C25_GlobalPosition::C25::Request  &req,
			  C25_GlobalPosition::C25::Response &res )
	 	  {
		  	  res.robotPosition=last_msg;
	 		  return true;
	 	  }
	  /**
	   * The call back function executed when a data is available
	   */
	  void callback(const ImuConstPtr& imu_msg,const OdometryConstPtr& pos_msg){
		  last_msg.imu.angular_velocity=imu_msg->angular_velocity;
		  last_msg.imu.angular_velocity_covariance=imu_msg->angular_velocity_covariance;
		  last_msg.imu.header=imu_msg->header;
		  last_msg.imu.linear_acceleration=imu_msg->linear_acceleration;
		  last_msg.imu.linear_acceleration_covariance=imu_msg->linear_acceleration_covariance;
		  last_msg.imu.orientation=imu_msg->orientation;
		  last_msg.imu.orientation_covariance=imu_msg->orientation_covariance;
		  last_msg.pose.child_frame_id=pos_msg->child_frame_id;
		  last_msg.pose.header=pos_msg->header;
		  last_msg.pose.pose=pos_msg->pose;
		  last_msg.pose.twist=pos_msg->twist;
		  c25_publisher.publish(last_msg);
	  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "C25_GlobalPosition");
  C25_Node *my_node=new C25_Node(argc,argv);
  while(ros::ok()){
	  ros::spin();
  }
  return 0;
}

