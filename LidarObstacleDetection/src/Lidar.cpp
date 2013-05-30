#include "geometry_msgs/Polygon.h"
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include <nav_msgs/Odometry.h>
#include <pcl/correspondence.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <boost/thread/thread.hpp>
#include <cv_bridge/cv_bridge.h>



class LaserScanToPointCloud{

public:

  ros::NodeHandle n_;
  image_transport::ImageTransport it_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  tf::TransformListener listener;
  cv::Mat leftImage;
  image_transport::Subscriber image_sub;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
  ros::Publisher scan_pub_;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr basic_cloud_ptr;
  LaserScanToPointCloud(ros::NodeHandle n) :
    n_(n),
    it_(n_),
    laser_sub_(n_, "/multisense_sl/laser/scan", 10),
    laser_notifier_(laser_sub_,listener_, "pelvis", 10)
    //basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>)
  {
    image_sub = it_.subscribe("/multisense_sl/camera/left/image_color", 1, &LaserScanToPointCloud::imageCallback,this);
    laser_notifier_.registerCallback(
      boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
    laser_notifier_.setTolerance(ros::Duration(0.01));
    scan_pub_ = n_.advertise<geometry_msgs::Polygon>("/my_cloud",1);
	std::cout << "Genarating example point clouds.\n\n";
	// We're going to make an ellipse extruded along the z-axis.
	/*for (float z(-1.0); z <= 1.0; z += 0.05)
	{
		for (float angle(0.0); angle <= 360.0; angle += 5.0)
		{
			pcl::PointXYZRGB basic_point;
			basic_point.x = 0.5 * std::cos (pcl::deg2rad(angle));
			basic_point.y = std::sin (pcl::deg2rad(angle));
			basic_point.z = z;
			basic_cloud_ptr->points.push_back(basic_point);
		}
	}
	basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
	basic_cloud_ptr->height = 1;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> myviewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer=myviewer;
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZRGB> (basic_cloud_ptr, "reconstruction");
	viewer->addCoordinateSystem ( 1.0 );
	viewer->initCameraParameters ();
*/
  }


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  	cv_bridge::CvImagePtr left;
	try
	{
	  left = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::RGB8);
	}
	catch (cv_bridge::Exception& e)
	{
	  ROS_ERROR("cv_bridge exception: %s", e.what());
	  return;
	}
	left->image.copyTo(leftImage);
}

  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
 
    
     tf::StampedTransform transform;
	  		try{
	  		  listener.lookupTransform("/pelvis","/left_camera_frame",
	  								   ros::Time(0), transform);
	  		}
	  		catch (tf::TransformException ex){
	  		   return;
	  		}
		  sensor_msgs::PointCloud2 cloud;
		try{
		  projector_.transformLaserScanToPointCloud("/left_camera_frame",*scan_in,
		          cloud,listener_);
		}
		catch (tf::TransformException ex){
		   return;
		}

		pcl::PointCloud<pcl::PointXYZ> temp;
		pcl::fromROSMsg<pcl::PointXYZ>(cloud,temp);
		pcl::PointCloud<pcl::PointXYZRGB> cloud2;
		for(int i=0;i<temp.points.size();i++){
			if(scan_in->ranges.at(i)<29 && scan_in->ranges.at(i)>2.5)
				{
					pcl::PointXYZRGB p;
					p.x=temp.points.at(i).x;
					p.y=temp.points.at(i).y;
					p.z=temp.points.at(i).z;
					int xpix,ypix;
					xpix=400+(int)(-482*p.z/p.x);
					ypix=400+(int)(-482*p.y/p.x);
					//stay in frame
					xpix=xpix>0?xpix:-1;
					xpix=xpix<799?xpix:800;
					ypix=ypix>0?ypix:-1;
					ypix=ypix<799?ypix:800;
					
					p.r=leftImage.at<cv::Vec3b>(xpix,ypix)[0];
		
				
					p.g=leftImage.at<cv::Vec3b>(xpix,ypix)[1];
			
					p.b=leftImage.at<cv::Vec3b>(xpix,ypix)[2];

					if(ypix>799 || ypix<0 || xpix>799 ||xpix<0)
						{p.r=255;p.b=255;p.g=255;}
					if(p.z >0)
						continue;
					if(p.z<-0.6&& (p.b <100) && (p.g <150) && (p.r <150))
						continue;
					if(p.z<-0.6&& p.x<3)
					   continue;
					  //{p.r=255;p.b=255;p.g=255;}
					cloud2.points.push_back(p);//temp.points.at(i));
				}
		}	
		Eigen::Matrix4f sensorTopelvis;
		pcl_ros::transformAsMatrix(transform, sensorTopelvis);
		pcl::transformPointCloud(cloud2, cloud2, sensorTopelvis);
		//sensor_msgs::PointCloud2 cloud3;
		//pcl::toROSMsg(cloud2,cloud3);

		//C21_VisionAndLidar::C21_C22 msg;
		//cloud3.header=scan_in->header;
		//cloud3.header.frame_id="/pelvis";

		geometry_msgs::Polygon s_msg;
		for(int i=0;i<cloud2.points.size();i++){
			if(cloud2.points.at(i).x!=cloud2.points.at(i).x)
				continue;
				geometry_msgs::Point32 p;
				p.x=cloud2.points.at(i).x;
				p.y=cloud2.points.at(i).y;
				p.z=cloud2.points.at(i).z;
			s_msg.points.push_back(p);
		}
		
		scan_pub_.publish(s_msg);
		
//*basic_cloud_ptr+=cloud2;
//basic_cloud_ptr->swap(cloud2);
/*	viewer->spinOnce (100);
	if(!viewer->updatePointCloud(basic_cloud_ptr,"reconstruction"))	//problem only in eclipse, works->ignore
		viewer->addPointCloud(basic_cloud_ptr, "reconstruction");	//problem only in eclipse, works->ignore*/

  }
};

int main(int argc, char** argv)
{

  ros::init(argc, argv, "my_scan_to_cloud");
  ros::NodeHandle n;
  LaserScanToPointCloud lstopc(n);

  ros::spin();

  return 0;
}


