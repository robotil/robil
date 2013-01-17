/*********************************************************8
 * a test module for the C21_Node
 * please lunch only after the C21_Node is running
 *
 */

#include "ros/ros.h"
#include "C21_VisionAndLidar/C21_Pan.h"
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
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
namespace enc=sensor_msgs::image_encodings;

geometry_msgs::Pose last;

void updatePos(nav_msgs::Odometry::Ptr & msg){
	last=msg->pose.pose;
}

void spinToTarget(){
	while(last.orientation.)
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "c21_panorama_tester");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<C21_VisionAndLidar::C21_Pan>("C21/Panorama");
  ros::Publisher pub=n.advertise<geometry_msgs::Twist>("atlas/cmd_vel",1);
  ros::Subscriber sub=n.subscribe("/ground_truth_odom",1,&updatePos);
  /*
   *  at the moment the C21_node has no use for the data input,
   *  once called it will reply a with a point cloud generated from the given camera image_raw topics
   *
  */
  geometry_msgs::Twist spin;
  geometry_msgs::Twist stop;
  spin.angular.z=0.9;
  for(int j=0;j<10;j++){
	  pub.publish(spin);
	  ros::spinOnce();
  }
  ros::Rate r(0.1);
  for(int i=0;i<8;i++){
	  for(int j=0;j<10;j++){
		  pub.publish(stop);
		  ros::spinOnce();
	  }
	  cout<<"taking picture"<<endl;
	  C21_VisionAndLidar::C21_Pan srv;
	  	  srv.request.req.cmd=C21_VisionAndLidar::C21_PANORAMA::TAKE_PICTURE;
	  	  if (!client.call(srv))
	  	  {
	  		ROS_ERROR("Something is wrong: exiting\n");
	  		return 1;
	  	  }
	  for(int j=0;j<10;j++){
		  pub.publish(spin);
		  ros::spinOnce();
	  }
	  r.sleep();
  }
  for(int j=0;j<10;j++){
	  pub.publish(stop);
	  ros::spinOnce();
  }

   cout<<"requesting panorama"<<endl;
  /*std::cout<< "enter 0 to take pictures, when you are done enter 1 (or any other number) to return a panorama" <<std::endl;
  int cmd;
  std::cin >>cmd;
  while(cmd==0){
	  C21_VisionAndLidar::C21_Pan srv;
	  srv.request.req.cmd=C21_VisionAndLidar::C21_PANORAMA::TAKE_PICTURE;
	  if (!client.call(srv))
	  {
		ROS_ERROR("Something is wrong: exiting\n");
		return 1;
	  }
	  std::cin >>cmd;
  }
*/
  C21_VisionAndLidar::C21_Pan srv;
  srv.request.req.cmd=C21_VisionAndLidar::C21_PANORAMA::RETURN_PANORAMA;
  if (client.call(srv))
	  {
		 cv_bridge::CvImagePtr pan;
		 std::cout<<srv.response.res.encoding<<std::endl;
		try
		{
		  pan = cv_bridge::toCvCopy(srv.response.res,enc::RGB8);
		}
		catch (cv_bridge::Exception& e)
		{
		  ROS_ERROR("cv_bridge exception: %s", e.what());
		  return 1;
		}
		cv::namedWindow("panorama",cv::WINDOW_NORMAL);	  //Main loop
		cv::imshow("panorama",pan->image);
		cv::waitKey(0);
		cv::imwrite("pan.jpg",pan->image);
	  }
	  else
	  {
		  ROS_ERROR("couldn't get a reply\n");
		return 1;
	  }

  return 0;
}

