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
#include <image_transport/image_transport.h>
namespace enc=sensor_msgs::image_encodings;

char left[]="LEFT";
char right[]="RIGHT";

void imageView(const sensor_msgs::ImageConstPtr& msg,char window[],std::string encoding)
  {

    cv_bridge::CvImagePtr src;
    try
    {
      src = cv_bridge::toCvCopy(msg, encoding);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
	cv::imshow(window, src->image);
	src->image.release();
  }


 void view(const sensor_msgs::ImageConstPtr& msg){
	 imageView(msg,"LEFT",enc::RGB8);
 }

 void view2(const sensor_msgs::ImageConstPtr& msg){
	 imageView(msg,"RIGHT",enc::RGB8);
 }



int main(int argc, char **argv)
{
  ros::init(argc, argv, "c21_image_tester");
  ros::NodeHandle n;
  image_transport::ImageTransport it_(n);
  image_transport::Subscriber left_image;
  image_transport::Subscriber right_image;
  image_transport::Subscriber panoramic_image;
  left_image = it_.subscribe("C21/left_camera/image",1,&view);
  right_image= it_.subscribe("C21/right_camera/image",1,&view2);
  cv::namedWindow(left,cv::WINDOW_NORMAL);
  cv::namedWindow(right,cv::WINDOW_NORMAL);
  cvStartWindowThread();
  while(ros::ok()){
	  ros::spin();
  }


  return 0;
}
