/**************************************************************************************
 * This is a basic prototype for the C21_vision_and_Lidar module for the robil project
 * The C21_vision_and_Lidar module goal is to provide a 3D reconstruction of a scene.
 **************************************************************************************/

#include "ros/ros.h"
#include <vector>
#include "C21_VisionAndLidar/C21.h"
#include "C21_VisionAndLidar/C21_Pan.h"
#include "C21_VisionAndLidar/C21_Pic.h"
#include "C21_VisionAndLidar/C21_C22.h"
#include "C21_VisionAndLidar/C21_obj.h"
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
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
#include <pcl_ros/transforms.h>
#include "opencv2/stitching/stitcher.hpp"
#include "std_msgs/Empty.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <std_msgs/Float64.h>
#include "tf/message_filter.h"
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
namespace enc=sensor_msgs::image_encodings;





/**
 * this class represent the C21_Node,
 * it subscribe to two camera/image topics and provide the 3D reconstruction service
 **/
class C21_Node{

public:

	/**
	 * constructor, initializes the ROS node, subscribe it to the given topics and instruct it to provide the service
	 * @param left_camera the left camera image topic
	 * @param right_camera the right camera image topic
	 */
	  C21_Node(std::string left_camera,std::string right_camera) :
		it_(nh_),
		//the purpose of the following 3 lines is to synchronize the data from the cameras using a message filter
		//more on filters and how to use them can be found on http://www.ros.org/wiki/message_filters
		left_image_sub_( it_, left_camera, 1 ),
		right_image_sub_( it_, right_camera, 1 ),
		pointcloud(nh_,"/multisense_sl/camera/points2",1),
		laser_sub_(nh_, "/multisense_sl/laser/scan", 10),
		laser_notifier_(laser_sub_,listener_, "pelvis", 10),
		sync( MySyncPolicy( 10 ), left_image_sub_, right_image_sub_ ,pointcloud)
	  {
		leftpub = it_.advertise("C21/left_camera/image", 1);
		rightpub = it_.advertise("C21/right_camera/image", 1);
		c22Cloudpub= nh_.advertise<sensor_msgs::PointCloud2>("C21/C22Cloud", 1);
		c22Lidarpub= nh_.advertise<sensor_msgs::PointCloud2>("C21/C22Lidar", 1);
		c22Cloudsub= nh_.subscribe("/multisense_sl/camera/points2", 1, &C21_Node::cloudCallback, this);
		laser_notifier_.registerCallback(
		      boost::bind(&C21_Node::LidarCallback, this, _1));
		    laser_notifier_.setTolerance(ros::Duration(0.01));
		//c25Sub=nh_.subscribe("/robot_pose_ekf/odom", 1,&C21_Node::C25Callback, this);
		ground_truth_sub= nh_.subscribe("ground_truth_odom", 1, &C21_Node::poseCallback, this);
		//set compression data to png
		sync.registerCallback( boost::bind( &C21_Node::HMIcallback, this, _1, _2,_3 ) );  //Specifying what to do with the data
		_panMutex=new boost::mutex();
		_cloudMutex=new boost::mutex();
		_detectionMutex=new boost::mutex();
		_posMutex=new boost::mutex();
		pcl_service = nh_.advertiseService("C21", &C21_Node::proccess, this); //Specifying what to do when a reconstructed 3d scene is requested
		pano_service = nh_.advertiseService("C21/Panorama", &C21_Node::pano_proccess, this);
		pic_service= nh_.advertiseService("C21/Pic", &C21_Node::pic_proccess, this);
		object_service=nh_.advertiseService("C21/C23", &C21_Node::obj_proccess, this);
		pan_imgs=new std::vector<cv::Mat>();
		ROS_INFO("C21 Online\n");
		//boost::thread panorama(&C21_Node::publishPanorama,this);
	  }



	  bool obj_proccess(C21_VisionAndLidar::C21_obj::Request  &req,
			C21_VisionAndLidar::C21_obj::Response &res )
	  {
		  //ROS_INFO("recived request, tying to fetch data\n");
		  int rounds=10;
		  for(int k=0;k<rounds;k++){
		  _posMutex->lock();
		  tf::Transform trans2;
		  trans2.setOrigin(tf::Vector3(c25msg.position.x,c25msg.position.y,c25msg.position.z));
		  trans2.setRotation(tf::Quaternion(c25msg.orientation.x,c25msg.orientation.y,c25msg.orientation.z,c25msg.orientation.w));
		  _posMutex->unlock();
		  /*tf::Transform trans;
		  	 	 trans.setOrigin(tf::Vector3(0.0,-0.002, 0.035 ));
		  	 	 trans.setRotation(tf::Quaternion(-1.57,3.14,1.57));
		  */Eigen::Matrix4f sensorToHead,pelvisToWorld;
		  //pcl_ros::transformAsMatrix(trans, sensorToHead);
		  pcl_ros::transformAsMatrix(trans2, pelvisToWorld);


		  int xMin=std::min(req.sample.x1,req.sample.x2);
		  int yMin=std::min(req.sample.y1,req.sample.y2);
		  int xMax=std::max(req.sample.x1,req.sample.x2);
		  int yMax=std::max(req.sample.y1,req.sample.y2);
		  _detectionMutex->lock();
		  double x=0;
		  double y=0;
		  double z=0;
		  double counter=0;
		  pcl::PointCloud<pcl::PointXYZ> t;
		   for(int i=xMin;i<=xMax;i++){
			   for(int j=yMin;j<=yMax;j++){
				   pcl::PointXYZ p=detectionCloud.at(i,j);
				   if(p.x!=p.x)
					   continue;
				   //if(p.x>0.3 && p.y>0.3)
					   t.points.push_back(p);
			   }
		   }

		   _detectionMutex->unlock();
		   //pcl::transformPointCloud(t, t, sensorToHead);
		   pcl::transformPointCloud(t, t, pelvisToWorld);
		   for(int i=0;i<t.points.size();i++){
			   if(t.points.at(i).x!=t.points.at(i).x)
				   continue;
			   x+=t.points.at(i).x;
			   y+=t.points.at(i).y;
			   z+=t.points.at(i).z;
			   counter++;
		   }
		   res.point.x+=((double)x)/(counter*rounds);
		   res.point.y+=((double)y)/(counter*rounds);
		   res.point.z+=((double)z)/(counter*rounds);
		  }
		  return true;
	  }

	  /**
	   * The call back function executed when a service is requested
	   * it must return true in order to work properly
	   * @param req the request message, generated by the node requesting the service
	   * @param res the response message, generated by the service node when a service is requested
	   */
	  bool proccess(C21_VisionAndLidar::C21::Request  &req,
			C21_VisionAndLidar::C21::Response &res )
	  {
		  //ROS_INFO("recived request, tying to fetch data\n");
		  _cloudMutex->lock();
		  pcl::toROSMsg(my_answer,res.scene_full_resolution_msg.cloud);
		  _cloudMutex->unlock();
		  return true;
	  }

	  bool pic_proccess(C21_VisionAndLidar::C21_Pic::Request  &req,
	  			  C21_VisionAndLidar::C21_Pic::Response &res )
	  	  {
		  	  _panMutex->lock();
	  			cv_bridge::CvImage cvi;
			    cvi.header.stamp = ros::Time::now();
			    cvi.header.frame_id = "image";
			    cvi.encoding = "rgb8";
			    if(req.req.cmd==C21_VisionAndLidar::C21_PICTURE::LEFT){
			    	cvi.image = leftImage;
			    }else{
			    	cvi.image = rightImage;
			    }
			    cvi.toImageMsg(res.res);
	  	      _panMutex->unlock();

	  		  return true;
	  	  }



	  bool pano_proccess(C21_VisionAndLidar::C21_Pan::Request  &req,
			  C21_VisionAndLidar::C21_Pan::Response &res )
	  {

		  if(req.req.cmd==C21_VisionAndLidar::C21_PANORAMA::TAKE_PICTURE){
			  _panMutex->lock();
			  cv::Mat im;
			  leftImage.copyTo(im);
			  pan_imgs->push_back(im);
			  _panMutex->unlock();
		  }else{
			  if(pan_imgs->size()==0)
				  return false;
			  cv::Mat pano;
			  cv::Stitcher stitcher = cv::Stitcher::createDefault(false);
			  stitcher.stitch(*pan_imgs, pano);
			  cv_bridge::CvImage cvi;
			  cvi.header.stamp = ros::Time::now();
			  cvi.header.frame_id = "image";
			  cvi.encoding = "rgb8";
			  cvi.image = pano;
			  cvi.toImageMsg(res.res);
			  while(pan_imgs->size()>0){
				  cv::Mat im=pan_imgs->back();
				  pan_imgs->pop_back();
				  im.release();
			  }
		  }
		  return true;
	  }


	  void C25Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg){
		  _posMutex->lock();
		  c25msg.position=msg->pose.pose.position;
		  c25msg.orientation=msg->pose.pose.orientation;
		  _posMutex->unlock();
	  }

	  void poseCallback(const nav_msgs::Odometry::ConstPtr &msg){
		  _posMutex->lock();
		  c25msg.position=msg->pose.pose.position;
		  c25msg.orientation=msg->pose.pose.orientation;
		  _posMutex->unlock();
	  }

	  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud){
	  		tf::StampedTransform transform;
	  		try{
	  		  listener.lookupTransform("/pelvis","/left_camera_optical_frame",
	  								   ros::Time(0), transform);
	  		}
	  		catch (tf::TransformException ex){
	  		   return;
	  		}
	  		pcl::PointCloud<pcl::PointXYZ> out;
	  		pcl::fromROSMsg(*cloud,out);
			Eigen::Matrix4f sensorTopelvis;
			pcl_ros::transformAsMatrix(transform, sensorTopelvis);
			pcl::transformPointCloud(out, out, sensorTopelvis);
	  		sensor_msgs::PointCloud2 msg;
	  		pcl::toROSMsg(out,msg);
	  		c22Cloudpub.publish(msg);
	  		/*pcl::PointCloud<pcl::PointXYZ> out;
	  		pcl::fromROSMsg(*cloud,out);
			tf::Transform trans;
			trans.setOrigin(orig);
			trans.setRotation(rot);*/
			/*Eigen::Matrix4f sensorTopelvis,pelvisToWorld;
			pcl_ros::transformAsMatrix(trans, sensorTopelvis);*/
			_detectionMutex->lock();
			//pcl::transformPointCloud(out, detectionCloud, sensorTopelvis);
			detectionCloud.swap(out);
			_detectionMutex->unlock();
	  	  }


	  void HMIcallback(const sensor_msgs::ImageConstPtr& left_msg,const sensor_msgs::ImageConstPtr& right_msg,const sensor_msgs::PointCloud2::ConstPtr &cloud){
	  		 cv_bridge::CvImagePtr left;
	  		 cv_bridge::CvImagePtr right;
	  		try
	  		{
	  		  left = cv_bridge::toCvCopy(left_msg,enc::RGB8);
	  		  right =cv_bridge::toCvCopy(right_msg,enc::RGB8);
	  		}
	  		catch (cv_bridge::Exception& e)
	  		{
	  		  ROS_ERROR("cv_bridge exception: %s", e.what());
	  		  return;
	  		}

	  		//left_msg->header.stamp=ros::Time::now();
	  		//right_msg->header.stamp=ros::Time::now();
	  		leftpub.publish(left_msg);
	  		rightpub.publish(right_msg);
	  		/*
	  		 *saving frames for HMI use
	  		 */
	  		_panMutex->lock();
	  		left->image.copyTo(leftImage);
	  		right->image.copyTo(rightImage);
	  		_panMutex->unlock();
	  		pcl::PointCloud<pcl::PointXYZ> out;
	  		pcl::fromROSMsg(*cloud,out);
	  		_cloudMutex->lock();
	  		my_answer.swap(out);
	  		//pcl::io::savePCDFile("cloud.pcd",out,true);
	  		_cloudMutex->unlock();
	  		//std::cout<<" position x:"<<msg.pose.position.x<<" y:"<<msg.pose.position.y<<" z:"<<msg.pose.position.z<<"\n";
	  	  }


	  /**
	   * The call back function executed when a data is available
	   * @param left_msg ROS mesage with image data from the left camera topic
	   * @param right_msg ROS mesage with image data from the right camera topic
	   */
	  void LidarCallback(const sensor_msgs::LaserScan::ConstPtr &scan_in){
		  tf::StampedTransform transform;
		  sensor_msgs::PointCloud2 cloud;
		try{
		  projector_.transformLaserScanToPointCloud("/pelvis",*scan_in,
		          cloud,listener_);
		}
		catch (tf::TransformException ex){
		   return;
		}

		pcl::PointCloud<pcl::PointXYZ> temp;
		pcl::fromROSMsg<pcl::PointXYZ>(cloud,temp);
		pcl::PointCloud<pcl::PointXYZ> cloud2;
		for(int i=0;i<temp.points.size();i++){
			if(scan_in->ranges.at(i)<29 && scan_in->ranges.at(i)>0.5)
				cloud2.points.push_back(temp.points.at(i));
		}
		sensor_msgs::PointCloud2 cloud3;
		pcl::toROSMsg(cloud2,cloud3);

		//C21_VisionAndLidar::C21_C22 msg;
		cloud3.header=scan_in->header;
		cloud3.header.frame_id="/pelvis";

		c22Lidarpub.publish(cloud3);
		_detectionMutex->lock();
		//detectionCloud+=cloud2;
		_detectionMutex->unlock();

	  }

private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  cv::Mat Q;
  int counter;
  bool request;
  boost::mutex * _panMutex;
  boost::mutex * _cloudMutex;
  boost::mutex * _detectionMutex;
  boost::mutex * _posMutex;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  typedef image_transport::SubscriberFilter ImageSubscriber;
  pcl::PointCloud<pcl::PointXYZ> my_answer;
  pcl::PointCloud<pcl::PointXYZ> detectionCloud;
  cv::Mat leftImage;
  cv::Mat rightImage;
  ImageSubscriber left_image_sub_;
  ImageSubscriber right_image_sub_;
  image_transport::Publisher leftpub;
  image_transport::Publisher rightpub;
  ros::Publisher c22Cloudpub;
  ros::Publisher c22Lidarpub;
  ros::Subscriber c22Cloudsub;
  ros::Subscriber ground_truth_sub;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
  ros::Subscriber c25Sub;
  image_transport::Publisher smallPanoramicPublisher;
  message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser;
  tf::TransformListener listener;
  ros::ServiceServer pcl_service;
  ros::ServiceServer object_service;
  std::vector<cv::Mat> *pan_imgs;
  ros::ServiceServer pano_service;
  ros::ServiceServer pic_service;
  geometry_msgs::Pose c25msg;
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image, sensor_msgs::Image,sensor_msgs::PointCloud2
  > MySyncPolicy;
  message_filters::Synchronizer< MySyncPolicy > sync;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "C21_VisionAndLidar");
  ros::AsyncSpinner spinner(7); // Use 7 threads
  spinner.start();
  C21_Node my_node("/multisense_sl/camera/left/image_rect_color","/multisense_sl/camera/right/image_rect_color");
  ros::waitForShutdown();
  while(ros::ok()){
	  ros::spin();
  }
  return 0;
}

