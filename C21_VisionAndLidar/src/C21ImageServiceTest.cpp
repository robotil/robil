/*********************************************************8
 * a test module for the C21_Node
 * please lunch only after the C21_Node is running
 *
 */

#include "ros/ros.h"
#include "C21_VisionAndLidar/C21.h"
#include "C21_VisionAndLidar/C21_Pic.h"
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
#include <image_transport/image_transport.h>
namespace enc=sensor_msgs::image_encodings;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "c21_picture_service_tester");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<C21_VisionAndLidar::C21_Pic>("C21/Pic");
  C21_VisionAndLidar::C21_Pic srv;
  cout<<"choose 0 for left 1 for right";
  cv::namedWindow("Image",cv::WINDOW_NORMAL);
  cvStartWindowThread();
  while(1){
	  cin>> srv.request.req.cmd;

	  if (client.call(srv))
		  {
			 cv_bridge::CvImagePtr pan;
				try
				{
				  pan = cv_bridge::toCvCopy(srv.response.res,enc::RGB8);
				}
				catch (cv_bridge::Exception& e)
				{
				  ROS_ERROR("cv_bridge exception: %s", e.what());
				  return 1;
				}
				cv::imshow("Image",pan->image);
		  }
		  else
		  {
			  ROS_ERROR("couldn't get a reply\n");
			return 1;
		  }
  }
  return 0;
}
