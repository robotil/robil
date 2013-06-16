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
#include "std_msgs/Empty.h"
#include  "std_srvs/Empty.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "C25_GlobalPosition/C25BDI.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
//#include <C21_VisionAndLidar/C21_C22.h>
#include <atlas_msgs/AtlasSimInterfaceState.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
//#include <tf/Matrix3x3.h>
#include <pcl/correspondence.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include "tf/transform_datatypes.h"
#include "pcl/point_types.h"
#include "pcl_ros/transforms.h"
#include "boost/assign.hpp"
using namespace sensor_msgs;
using namespace nav_msgs;
using namespace message_filters;


/**
 * this class represent the C25_Node,
 **/
class C25_Node{

private:
  ros::NodeHandle nh_;
  ros::Publisher c25_publisher,bdi_pub;
  //ros::Publisher object_location_publisher;
  //ros::Subscriber object_detector_subscriber;
  //ros::Subscriber C21_subscriber;
  ros::ServiceServer c25_service;
  ros::ServiceServer ground_truth_service;
  ros::ServiceServer bdi_service;

  typedef sync_policies::ApproximateTime<Imu, atlas_msgs::AtlasSimInterfaceState> MySyncPolicy;
  message_filters::Subscriber<Imu> imu_sub;
  message_filters::Subscriber< atlas_msgs::AtlasSimInterfaceState> bdi_sub;
  Synchronizer<MySyncPolicy> sync;
  ros::Subscriber pos_sub,ground_truth_sub;
  bool switch_,switch2_;
  C25_GlobalPosition::C25C0_ROP last_msg;
  geometry_msgs::PoseWithCovarianceStamped kalman_last_msg;
  atlas_msgs::AtlasSimInterfaceState bdi_last_msg;
  sensor_msgs::Imu imu_last_msg;
  float bdi_delta_x,bdi_delta_y,bdi_delta_z,bdi_delta_time;
  float kalman_delta_x,kalman_delta_y,kalman_delta_z,kalman_delta_time;

  //C23_ObjectRecognition::C23C0_ODIM last_obj_msg;
  bool gotObj;
  LocalizationTrackServer *taskserver;

public:

	 /**
	  * constructor, initializes the ROS node, subscribe it to the given topics and instruct it to provide the service
	  */
	  C25_Node(int argc, char **argv):
		  imu_sub(nh_,"/atlas/imu",1),
		  bdi_sub(nh_,"/atlas/atlas_sim_interface_state",1),
		  sync(MySyncPolicy(100), imu_sub, bdi_sub)
	  {
		  switch_=true;
		  switch2_=true;
		  bdi_delta_x=0;
		  bdi_delta_y=0;
		  bdi_delta_z=0;
		  bdi_delta_time=0;
		  kalman_delta_x=0;
		  kalman_delta_y=0;
		  kalman_delta_z=0;
		  kalman_delta_time=0;
		  sync.registerCallback( boost::bind( &C25_Node::bdi_callback, this, _1, _2 ) );
		  c25_publisher=nh_.advertise<C25_GlobalPosition::C25C0_ROP>("C25/publish",100);
		  c25_service=nh_.advertiseService("C25/service",&C25_Node::proccess,this);
		  ground_truth_service=nh_.advertiseService("C25/ground_truth",&C25_Node::groundtruthProccess,this);
		  bdi_service=nh_.advertiseService("C25/BDIswitch",&C25_Node::BDIProccess,this);
		  //bdi_pub=nh_.advertise<nav_msgs::Odometry>("C25/bdi_stripped",100);
		  //bdi_sub= nh_.subscribe("/atlas/atlas_sim_interface_state", 1, &C25_Node::bdi_callback, this);
		  pos_sub=nh_.subscribe("/robot_pose_ekf/odom",1,&C25_Node::callback,this);
		  ROS_INFO("service on\n");
		  boost::thread mythread( &C25_Node::startActionServer,this,argc,argv);
	  }


	  void startActionServer(int argc, char **argv){
		  ros::init(argc, argv, "C25_LocalizationTrackServer");
		  taskserver=new LocalizationTrackServer();
		  while(ros::ok()){

		  }
	  }


	  bool BDIProccess(C25_GlobalPosition::C25BDI::Request  &req,C25_GlobalPosition::C25BDI::Response &res ){
		  if(switch_){
			  if(req.state.data==0){
					switch2_=false;
					//bdi_sub.shutdown();
					//pos_sub=nh_.subscribe("/robot_pose_ekf/odom",1,&C25_Node::callback,this);
			   }else{
				   if(req.state.data==1){
					switch2_=true;
				   }
					//pos_sub.shutdown();
					//bdi_sub= nh_.subscribe("/atlas/atlas_sim_interface_state", 1, &C25_Node::bdi_callback, this);
			   }
		  }else{
			  return false;
		  }
		  return true;
	  }


	 bool groundtruthProccess(std_srvs::Empty::Response  &req,
			 std_srvs::Empty::Response &res )
	 	  {
		 	 if(switch_){
		 		switch_=false;
		 		/*if(!switch2_)
		 			pos_sub.shutdown();
		 		else
		 			bdi_sub.unsubscribe();*/
		 		ground_truth_sub= nh_.subscribe("/ground_truth_odom", 1, &C25_Node::poseCallback, this);
		 	 }else{
		 		switch_=true;
		 		ground_truth_sub.shutdown();
		 		/*if(!switch2_)
		 			pos_sub=nh_.subscribe("/robot_pose_ekf/odom",1,&C25_Node::callback,this);
		 		else
		 			bdi_sub.subscribe(nh_,"/atlas/atlas_sim_interface_state", 1, &C25_Node::bdi_callback, this);*/

		 	 }
	 		  return true;
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
	  

	  void bdi_callback(const sensor_msgs::Imu::ConstPtr &imu_msg, const atlas_msgs::AtlasSimInterfaceState::ConstPtr &msg){
		  bdi_delta_x=msg->pos_est.position.x-bdi_last_msg.pos_est.position.x;
		  bdi_delta_y=msg->pos_est.position.y-bdi_last_msg.pos_est.position.y;
		  bdi_delta_z=msg->pos_est.position.z-bdi_last_msg.pos_est.position.z;
		  bdi_delta_time=msg->header.stamp.toSec()-bdi_last_msg.header.stamp.toSec();
		  bdi_last_msg.pos_est.position=msg->pos_est.position;
		  bdi_last_msg.header=msg->header;
		  imu_last_msg.orientation=imu_msg->orientation;

		  if(switch2_ && switch_){


			  double r0,p0,y0,r1,p1,y1;
			  tf::Quaternion oldQ;
			  tf::quaternionMsgToTF(last_msg.pose.pose.pose.orientation, oldQ);
			  tf::Matrix3x3 mOld(oldQ);
			  mOld.getRPY(r0,p0,y0);
			  tf::Quaternion newQ;
			  tf::quaternionMsgToTF(imu_msg->orientation, newQ);
			  tf::Matrix3x3 mNew(newQ);
			  mOld.getRPY(r1,p1,y1);

			  last_msg.header=bdi_last_msg.header;
			  last_msg.pose.header=bdi_last_msg.header;
			  last_msg.pose.pose.pose.position.x+=bdi_delta_x;
			  last_msg.pose.pose.pose.position.y+=bdi_delta_y;
			  last_msg.pose.pose.pose.position.z+=bdi_delta_z;
			  last_msg.pose.pose.pose.orientation.x=imu_last_msg.orientation.x;
			  last_msg.pose.pose.pose.orientation.y=imu_last_msg.orientation.y;
			  last_msg.pose.pose.pose.orientation.z=imu_last_msg.orientation.z;
			  last_msg.pose.pose.pose.orientation.w=imu_last_msg.orientation.w;
			  last_msg.pose.twist.twist.angular.x=(r1-r0)/(bdi_delta_time);
			  last_msg.pose.twist.twist.angular.y=(p1-p0)/(bdi_delta_time);
			  last_msg.pose.twist.twist.angular.z=(y1-y0)/(bdi_delta_time);
			  c25_publisher.publish(last_msg);
		  }
	  }

 	void poseCallback(const nav_msgs::Odometry::ConstPtr &msg){

		  last_msg.pose.header=msg->header;
		  last_msg.pose.pose=msg->pose;
		  last_msg.pose.twist=msg->twist;
 	      c25_publisher.publish(last_msg);
	  }


	   /**
	   * The call back function executed when a data is available
	   */
	  void callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pos_msg){
		  kalman_delta_x=pos_msg->pose.pose.position.x-kalman_last_msg.pose.pose.position.x;
		  kalman_delta_y=pos_msg->pose.pose.position.y-kalman_last_msg.pose.pose.position.y;
		  kalman_delta_z=pos_msg->pose.pose.position.z-kalman_last_msg.pose.pose.position.z;
		  kalman_delta_time=pos_msg->header.stamp.toSec()-kalman_last_msg.header.stamp.toSec();
		  kalman_last_msg.pose.pose.position=pos_msg->pose.pose.position;
		  kalman_last_msg.header=pos_msg->header;
		  kalman_last_msg.pose.pose.orientation=pos_msg->pose.pose.orientation;



		  if(!switch2_ && switch_){
			  double r0,p0,y0,r1,p1,y1;
			  tf::Quaternion oldQ;
			  tf::quaternionMsgToTF(last_msg.pose.pose.pose.orientation, oldQ);
			  tf::Matrix3x3 mOld(oldQ);
			  mOld.getRPY(r0,p0,y0);
			  tf::Quaternion newQ;
			  tf::quaternionMsgToTF(pos_msg->pose.pose.orientation, newQ);
			  tf::Matrix3x3 mNew(newQ);
			  mOld.getRPY(r1,p1,y1);
			  last_msg.header=pos_msg->header;
			  last_msg.pose.header=pos_msg->header;
			  last_msg.pose.pose.pose.position.x+=kalman_delta_x;
			  last_msg.pose.pose.pose.position.y+=kalman_delta_y;
			  last_msg.pose.pose.pose.position.z+=kalman_delta_z;
			  last_msg.pose.pose.pose.orientation.x=pos_msg->pose.pose.orientation.x;
			  last_msg.pose.pose.pose.orientation.y=pos_msg->pose.pose.orientation.y;
			  last_msg.pose.pose.pose.orientation.z=pos_msg->pose.pose.orientation.z;
			  last_msg.pose.pose.pose.orientation.w=pos_msg->pose.pose.orientation.w;
			  last_msg.pose.twist.twist.angular.x=(r1-r0)/(kalman_delta_time);
			  last_msg.pose.twist.twist.angular.y=(p1-p0)/(kalman_delta_time);
			  last_msg.pose.twist.twist.angular.z=(y1-y0)/(kalman_delta_time);
			  c25_publisher.publish(last_msg);
		  }
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

