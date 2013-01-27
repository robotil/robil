/*********************************************************8
 * a test module for the C21_Node
 * please lunch only after the C21_Node is running
 *
 */

#include "ros/ros.h"
#include "C21_VisionAndLidar/C21.h"
#include <cstdlib>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <pcl/correspondence.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>



boost::shared_ptr<pcl::visualization::PCLVisualizer> createVisualizer (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud,"reconstruction");
  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "reconstruction");
  viewer->addCoordinateSystem ( 1.0 );
  viewer->initCameraParameters ();
  return (viewer);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "c21_scene_tester");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<C21_VisionAndLidar::C21>("C21");
  C21_VisionAndLidar::C21 srv;
  /*
   *  at the moment the C21_node has no use for the data input,
   *  once called it will reply a with a point cloud generated from the given camera image_raw topics
   *
  	  srv.request.azimuth_msg= ?????????????;
  	  srv.request.camera_sample_rate_msg= ??????????;
  	  srv.request.laser_sample_rate_msg= ?????????????;
  	  srv.request.output_image_size_msg= ??????????;
  	  srv.request.required_resolution_msg= ??????????;
  */
	  if (client.call(srv))
	  {
		  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
		  pcl::PointCloud<pcl::PointXYZ> cloud;
		  pcl::fromROSMsg<pcl::PointXYZ>(srv.response.scene_full_resolution_msg.cloud,cloud);
		  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_ptr(&cloud);
		  ROS_INFO("creating a visualiser\n");
		  viewer = createVisualizer( cloud_ptr );

		  //Main loop
		  while ( !viewer->wasStopped())
		  {
			viewer->spinOnce(100);
			boost::this_thread::sleep (boost::posix_time::microseconds (100000));
		  }
	  }
	  else
	  {
		  ROS_ERROR("couldn't get a reply\n");
		return 1;
	  }

  return 0;
}
