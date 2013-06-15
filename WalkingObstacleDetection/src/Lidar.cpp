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
#include <image_transport/subscriber_filter.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <pcl/visualization/pcl_visualizer.h>
class LaserScanToPointCloud{

public:

  ros::NodeHandle n_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
  ros::Publisher scan_pub_;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr;
  LaserScanToPointCloud(ros::NodeHandle n) :
    n_(n),
    laser_sub_(n_, "/multisense_sl/laser/scan", 10),
    laser_notifier_(laser_sub_,listener_, "pelvis", 10)
    //basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>)
  {
    laser_notifier_.registerCallback(
      boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
    laser_notifier_.setTolerance(ros::Duration(0.01));
    scan_pub_ = n_.advertise<geometry_msgs::Polygon>("/LaserCloudForWalking",1);
	/*std::cout << "Genarating example point clouds.\n\n";
	// We're going to make an ellipse extruded along the z-axis.
	for (float z(-1.0); z <= 1.0; z += 0.05)
	{
		for (float angle(0.0); angle <= 360.0; angle += 5.0)
		{
			pcl::PointXYZ basic_point;
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
	viewer->addPointCloud<pcl::PointXYZ> (basic_cloud_ptr, "reconstruction");
	viewer->addCoordinateSystem ( 1.0 );
	viewer->initCameraParameters ();*/

  }

  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
    sensor_msgs::PointCloud2 cloud;
    try
    {
        projector_.transformLaserScanToPointCloud(
          "pelvis",*scan_in, cloud,listener_);
    }
    catch (tf::TransformException& e)
    {
        std::cout << e.what();
        return;
    }
   	pcl::PointCloud<pcl::PointXYZ> temp;
   	pcl::fromROSMsg<pcl::PointXYZ>(cloud,temp);
	pcl::PointCloud<pcl::PointXYZ> cloud2;
	geometry_msgs::Polygon ms;
	for(int i=0;i<temp.points.size();i++){
		if(scan_in->ranges.at(i)<15 && temp.points.at(i).z>-0.5  && temp.points.at(i).z<1){		   
			if (temp.points.at(i).x<0.25 && temp.points.at(i).y>-0.2 && temp.points.at(i).y<0.2)
		   		continue;
		   	if (temp.points.at(i).x<0.15 && temp.points.at(i).y>-0.5 && temp.points.at(i).y<0.5)
		   		continue;
			geometry_msgs::Point32 m;
			m.x=temp.points.at(i).x;
			m.y=temp.points.at(i).y;
			m.z=temp.points.at(i).z;
			ms.points.push_back(m);		
			cloud2.points.push_back(temp.points.at(i));
		}
	}
    scan_pub_.publish(ms);
/*
basic_cloud_ptr->swap(cloud2);
	viewer->spinOnce (100);
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


/*
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
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
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <image_transport/subscriber_filter.h>
#include <pcl_ros/point_cloud.h>
#include "opencv2/stitching/stitcher.hpp"
#include "std_msgs/Empty.h"
#include <tf/tf.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/time_synchronizer.h>
using namespace message_filters;




	  ros::Publisher cloud_pub;
		 laser_geometry::LaserProjection *projector_;


	  void callback(const sensor_msgs::LaserScan::ConstPtr &scan_in,const nav_msgs::Odometry::ConstPtr& pos_msg){
	  		  std::cout<<"print\n";
		  		tf::TransformListener listener_;


	  		  sensor_msgs::PointCloud2 cloud;
	  		try{

	  		  projector_->transformLaserScanToPointCloud("pelvis",*scan_in,cloud,listener_);
	  		}
	  		catch (tf::TransformException ex){
	  		   std::cout<<"exeption\n";
	  		   return;
	  		}
	  	  std::cout<<"continue\n";
	  		pcl::PointCloud<pcl::PointXYZ> temp;
	  		pcl::fromROSMsg<pcl::PointXYZ>(cloud,temp);
	  		pcl::PointCloud<pcl::PointXYZ> cloud2;
	  		for(int i=0;i<temp.points.size();i++){
	  			if(scan_in->ranges.at(i)<29 && scan_in->ranges.at(i)>1)
	  				cloud2.points.push_back(temp.points.at(i));
	  		}

	  		tf::Transform trans2;
	  		trans2.setOrigin(tf::Vector3(pos_msg->pose.pose.position.x,pos_msg->pose.pose.position.y,pos_msg->pose.pose.position.z));
	  		trans2.setRotation(tf::Quaternion(pos_msg->pose.pose.orientation.x,pos_msg->pose.pose.orientation.y,pos_msg->pose.pose.orientation.z,pos_msg->pose.pose.orientation.w));
	  		Eigen::Matrix4f sensorToHead,headTopelvis,pelvisToWorld;
	  		pcl_ros::transformAsMatrix(trans2, pelvisToWorld);
	  		pcl::transformPointCloud(cloud2, cloud2, pelvisToWorld);
	  		sensor_msgs::PointCloud2 cloud3;
	  		pcl::toROSMsg(cloud2,cloud3);
	  		cloud_pub.publish(cloud3);
	  	  }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Lidar");

   ros::NodeHandle nh;
   projector_= new laser_geometry::LaserProjection();
   Subscriber<sensor_msgs::LaserScan> laser(nh,"multisense_sl/laser/scan",1);
   Subscriber< nav_msgs::Odometry> pos_sub(nh,"ground_truth_odom",1);
   cloud_pub=nh.advertise<sensor_msgs::PointCloud2>("C51/detection",1);
   TimeSynchronizer<sensor_msgs::LaserScan, nav_msgs::Odometry> sync( laser, pos_sub, 10);
   sync.registerCallback(boost::bind(&callback, _1, _2));

  while(ros::ok()){
	  ros::spin();
  }
  return 0;
}

*/
