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
#include <C21_VisionAndLidar/C21_C22.h>
#include <C23_ObjectRecognition/C23C0_ODIM.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <pcl/correspondence.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include "pcl/point_types.h"
#include "pcl_ros/transforms.h"
#include "pcl/filters/statistical_outlier_removal.h"
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
  ros::Publisher object_location_publisher;
  ros::Subscriber object_detector_subscriber;
  ros::Subscriber C21_subscriber;
  ros::ServiceServer c25_service;
  typedef sync_policies::ApproximateTime<Imu, Odometry> MySyncPolicy;
  message_filters::Subscriber<Imu> imu_sub;
  message_filters::Subscriber<Odometry> pos_sub;
  Synchronizer<MySyncPolicy> sync;
  C25_GlobalPosition::C25C0_ROP last_msg;
  C23_ObjectRecognition::C23C0_ODIM last_obj_msg;
  bool gotObj;
  LocalizationTrackServer *taskserver;
public:

	 /**
	  * constructor, initializes the ROS node, subscribe it to the given topics and instruct it to provide the service
	  */
	  C25_Node(int argc, char **argv):
		  imu_sub(nh_,"/atlas/imu",1),
		  pos_sub(nh_,"ground_truth_odom",1),
		  sync(MySyncPolicy(10), imu_sub, pos_sub)
	  {
		  sync.registerCallback( boost::bind( &C25_Node::callback, this, _1, _2 ) );
		  c25_publisher=nh_.advertise<C25_GlobalPosition::C25C0_ROP>("C25/publish",100);
		  c25_service=nh_.advertiseService("C25/service",&C25_Node::proccess,this);
		  ROS_INFO("service on\n");
		  object_location_publisher=nh_.advertise<geometry_msgs::Point>("C23/objectLocation",100);
		  object_detector_subscriber=nh_.subscribe("C23/object_deminsions",1,&C25_Node::objectDimentionscallback,this);
		  C21_subscriber=nh_.subscribe("C21/C22",1,&C25_Node::cloudcallback,this);
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

	  void cloudcallback(const C21_VisionAndLidar::C21_C22::Ptr & cloud_msg){
		  if(!gotObj){
			  return;
		  }
		  gotObj=false;

		  pcl::PointCloud<pcl::PointXYZ>cloud;
		  pcl::fromROSMsg<pcl::PointXYZ>(cloud_msg->cloud,cloud);
		  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_backup(cloud.makeShared());
		  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

		  for(int i=last_obj_msg.x;i<last_obj_msg.x+last_obj_msg.width;i++){
		  		for(int j=last_obj_msg.y;j<last_obj_msg.y+last_obj_msg.height;j++){
		  			cloud_filtered->push_back(cloud.at(i,j));
		  		}
		  }
		  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		  sor.setInputCloud (cloud_filtered);
		  sor.setMeanK (50);
		  sor.setStddevMulThresh (1.0);
		  sor.filter (*cloud_filtered);


		 /*
		  * this code segment filters the given cloud and lowers its resolution
		  */
		 //std::cout << "PointCloud before filtering has: " << cloud.points.size () << " data points." << std::endl; //*
		 // Create the filtering object: downsample the dataset using a leaf size of 5cm


		     tf::Transform trans;
			 trans.setOrigin(tf::Vector3(cloud_msg->pose.position.x,cloud_msg->pose.position.y,cloud_msg->pose.position.z));
			 trans.setRotation(tf::Quaternion(cloud_msg->pose.orientation.x,cloud_msg->pose.orientation.y,cloud_msg->pose.orientation.z,cloud_msg->pose.orientation.w));
			 tf::Transform trans2;
			 	 trans2.setOrigin(tf::Vector3(last_msg.pose.pose.pose.position.x,last_msg.pose.pose.pose.position.y,last_msg.pose.pose.pose.position.z));
			 	 trans2.setRotation(tf::Quaternion(last_msg.pose.pose.pose.orientation.x,last_msg.pose.pose.pose.orientation.y,last_msg.pose.pose.pose.orientation.z,last_msg.pose.pose.pose.orientation.w));
			 tf::Transform trans3;
			 	 trans3.setOrigin(tf::Vector3(0.0,-0.002, 0.035 ));
			 	trans3.setRotation(tf::Quaternion(-1.57,3.14,1.57));
			 Eigen::Matrix4f sensorToHead,headTopelvis,pelvisToWorld;
			 pcl_ros::transformAsMatrix(trans3, sensorToHead);
			 pcl_ros::transformAsMatrix(trans, headTopelvis);
			 pcl_ros::transformAsMatrix(trans2, pelvisToWorld);
			 // transform pointcloud from sensor frame to fixed robot frame
			 pcl::transformPointCloud(*cloud_filtered, *cloud_filtered, sensorToHead);
			 pcl::transformPointCloud(*cloud_filtered, *cloud_filtered, headTopelvis);
			 pcl::transformPointCloud(*cloud_filtered, *cloud_filtered, pelvisToWorld);

			 double x=0;
			 double y=0;
			 double z=0;
			 double count=0;
			 for(int i=0;i<cloud_filtered->points.size();i++){
				 if(!(cloud_filtered->points.at(i).x != cloud_filtered->points.at(i).x)){
					 x+=cloud_filtered->points.at(i).x;
					 y+=cloud_filtered->points.at(i).y;
					 z+=cloud_filtered->points.at(i).z;
					 count++;
				 }
			 }
			 geometry_msgs::Point msg;
			 msg.x=x/count;
			 msg.y=y/count;
			 msg.z=z/count;
			 object_location_publisher.publish(msg);
	  }

	  void objectDimentionscallback(const C23_ObjectRecognition::C23C0_ODIM::Ptr& obj_msg){
		  last_obj_msg.height=obj_msg->height;
		  last_obj_msg.width=obj_msg->width;
		  last_obj_msg.x=obj_msg->x;
		  last_obj_msg.y=obj_msg->y;
		  gotObj=true;
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

