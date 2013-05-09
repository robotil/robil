



/**
 * this class represent the C22_Node,
 * it subscribe to two camera/image topics and provide the path
 **/

#include "pclPlane.h"
#include "MPlane.h"
#include "MapMatrix.h"
#include "C22_GroundRecognitionAndMapping/C22.h"
#include <C21_VisionAndLidar/C21_C22.h>
#include "sensor_msgs/PointCloud.h"
#include <pcl/correspondence.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/sample_consensus/sac_model_plane.h>
#include "nav_msgs/Odometry.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/callback_queue.h>
class C22_Node{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh2_;

  typedef message_filters::sync_policies::ApproximateTime<
		  C21_VisionAndLidar::C21_C22, nav_msgs::Odometry
    > MySyncPolicy;
  message_filters::Subscriber<C21_VisionAndLidar::C21_C22> pointCloud_sub;
  message_filters::Subscriber<nav_msgs::Odometry> pos_sub;
    message_filters::Synchronizer< MySyncPolicy > sync;
  ros::ServiceServer service;
  ros::ServiceServer service2;
  MapMatrix * _myMatrix;
  std::vector<pclPlane*>* _myPlanes;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  geometry_msgs::Point robotPos;
  geometry_msgs::Point robotOri;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudRecord;
  ros::Publisher C22_pub;
public:

	/**
	 * constructor, initializes the ROS node, subscribe it to the given topics and instruct it to provide the service
	 * @param left_camera the left camera image topic
	 * @param right_camera the right camera image topic
	 */
	  C22_Node();


	  /**
	   * The call back function executed when a service is requested
	   * it must return true in order to work properly
	   * @param req the request message, generated by the node requesting the service
	   * @param res the response message, generated by the service node when a service is requested
	   */
	  bool proccess(C22_GroundRecognitionAndMapping::C22::Request  &req,
			C22_GroundRecognitionAndMapping::C22::Response &res );

	  bool proccess2(C22_GroundRecognitionAndMapping::C22C24::Request  &req,
	  			C22_GroundRecognitionAndMapping::C22C24::Response &res );

	  /**
	   * The call back function executed when a data is available
	   * @param left_msg ROS mesage with image data from the left camera topic
	   * @param right_msg ROS mesage with image data from the right camera topic
	   */
	  void callback(const C21_VisionAndLidar::C21_C22::ConstPtr& pclMsg,const nav_msgs::Odometry::ConstPtr& pos_msg);
};
