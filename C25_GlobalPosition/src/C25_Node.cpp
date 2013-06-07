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
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
//#include <C21_VisionAndLidar/C21_C22.h>
#include <C23_ObjectRecognition/C23C0_ODIM.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
//#include <tf/Matrix3x3.h>
#include <pcl/correspondence.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include "tf/transform_datatypes.h"
#include "pcl/point_types.h"
#include "pcl_ros/transforms.h"
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
  //ros::Publisher object_location_publisher;
  //ros::Subscriber object_detector_subscriber;
  //ros::Subscriber C21_subscriber;
  ros::ServiceServer c25_service;
  /*typedef sync_policies::ApproximateTime<Imu, geometry_msgs::PoseWithCovarianceStamped> MySyncPolicy;
  message_filters::Subscriber<Imu> imu_sub;
  message_filters::Subscriber< geometry_msgs::PoseWithCovarianceStamped> pos_sub;
  Synchronizer<MySyncPolicy> sync;*/
  ros::Subscriber imu_sub,pos_sub,ground_truth_sub;
  bool once;
  C25_GlobalPosition::C25C0_ROP last_msg;
  C23_ObjectRecognition::C23C0_ODIM last_obj_msg;
  bool gotObj;
  LocalizationTrackServer *taskserver;
public:

	 /**
	  * constructor, initializes the ROS node, subscribe it to the given topics and instruct it to provide the service
	  */
	  C25_Node(int argc, char **argv)
		 /* imu_sub(nh_,"/atlas/imu",1),
		  pos_sub(nh_,"/robot_pose_ekf/odom",1),
		  sync(MySyncPolicy(100), imu_sub, pos_sub)*/
	  {
		  once=true;
		  //sync.registerCallback( boost::bind( &C25_Node::callback, this, _1, _2 ) );
		  pos_sub=nh_.subscribe("/robot_pose_ekf/odom",1,&C25_Node::callback,this);
		  c25_publisher=nh_.advertise<C25_GlobalPosition::C25C0_ROP>("C25/publish",100);
		  c25_service=nh_.advertiseService("C25/service",&C25_Node::proccess,this);
		  ground_truth_sub= nh_.subscribe("/ground_truth_odom", 1, &C25_Node::poseCallback, this);
		  ROS_INFO("service on\n");
		  boost::thread mythread( &C25_Node::startActionServer,this,argc,argv);
	  }


	  void startActionServer(int argc, char **argv){
		  ros::init(argc, argv, "C25_LocalizationTrackServer");
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
	  


 	void poseCallback(const nav_msgs::Odometry::ConstPtr &msg){
		 if(once){
			 once=false;
			 pos_sub.shutdown();
		 }
		  last_msg.pose.header=msg->header;
		  last_msg.pose.pose=msg->pose;
		  last_msg.pose.twist=msg->twist;
 	      c25_publisher.publish(last_msg);
	  }


	   /**
	   * The call back function executed when a data is available
	   */
	  void callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pos_msg){
		  double delta=last_msg.pose.header.stamp.toSec()-pos_msg->header.stamp.toSec();
		  double x,y,z,r0,p0,y0,r1,p1,y1;
		  x=pos_msg->pose.pose.position.x-last_msg.pose.pose.pose.position.x;
		  y=pos_msg->pose.pose.position.y-last_msg.pose.pose.pose.position.y;
		  z=pos_msg->pose.pose.position.z-last_msg.pose.pose.pose.position.z;
		  tf::Quaternion oldQ;
		  tf::quaternionMsgToTF(last_msg.pose.pose.pose.orientation, oldQ);
		  tf::Matrix3x3 mOld(oldQ);
		  mOld.getRPY(r0,p0,y0);
		  tf::Quaternion newQ;
		  tf::quaternionMsgToTF(pos_msg->pose.pose.orientation, newQ);
		  tf::Matrix3x3 mNew(newQ);
		  mOld.getRPY(r1,p1,y1);
		  last_msg.pose.header=pos_msg->header;
		  last_msg.pose.pose=pos_msg->pose;
		  last_msg.pose.twist.twist.linear.x=x/(delta);
		  last_msg.pose.twist.twist.linear.y=y/(delta);
		  last_msg.pose.twist.twist.linear.z=z/(delta);
		  last_msg.pose.twist.twist.angular.x=(r1-r0)/(delta);
		  last_msg.pose.twist.twist.angular.y=(p1-p0)/(delta);
		  last_msg.pose.twist.twist.angular.z=(y1-y0)/(delta);
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

