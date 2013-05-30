#ifndef __C23_DETECTOR__
#define __C23_DETECTOR__

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>


#include <image_transport/image_transport.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
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
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_ros/transforms.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <math.h>
#include "GeneralDetector.hpp"

//#include <columnDetect/COLUMN_DETECT_GATE.h>
//#include <columnDetect/COLUMN_DETECT_GATES.h>



namespace enc = sensor_msgs::image_encodings;
using namespace std;
using namespace cv;
using namespace tf;

#define GATE_DISTANCE_THRESHOLD 1


typedef enum targets { 
  CAR,
  GATE,
  PATH,
  PICTURE,
  VALVE,
  FIREHOSE,
  INSIDE_STEERINGWHEEL,
  OUTSIDE_STEERINGWHEEL,
  HANDBRAKE,
  GEAR,
  PUSHBUTTON,
  NONE
} TARGETS;

class C23_Detector{
public:
	C23_Detector(const char* left_cam, const char* right_cam, const char* pointc);

  bool detect(string target);
	void callback(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::PointCloud2::ConstPtr &cloud);
public:
	bool is_search;
	int x;
	int  y;
	int width;
	int height;
private:

  	bool detectGate(Mat img, const sensor_msgs::PointCloud2::ConstPtr &cloud);
    void publishMessage(bool isFound);
    bool detectPath(Mat srcImg);
    bool detectCar(Mat srcImg, const sensor_msgs::PointCloud2::ConstPtr &cloud);
    //bool compareContourAreas ( vector<cv::Point> contour1, vector<cv::Point> contour2) ;
    bool detectPassengerDriver(Mat srcImg, int x1,int y1,int x2,int y2);
    bool detectValve(Mat srcImg, const sensor_msgs::PointCloud2::ConstPtr &cloud);
    bool detectFirehose(Mat srcImg, const sensor_msgs::PointCloud2::ConstPtr &cloud);
    
    bool detectSteeringWheel(Mat srcImg,const sensor_msgs::PointCloud2::ConstPtr &cloud,int location);
    bool detectHandbrake(Mat srcImg,const sensor_msgs::PointCloud2::ConstPtr &cloud,int location);
    bool detectGear(Mat srcImg,const sensor_msgs::PointCloud2::ConstPtr &cloud,int location);
    bool detectPushButton(Mat srcImg,const sensor_msgs::PointCloud2::ConstPtr &cloud,int location);
    
    bool pictureCoordinatesToGlobalPosition(int x1, int y1, int x2, int y2, int* x, int* y, int *z);
	ros::NodeHandle nh;
    bool takePictures(Mat srcImg);
    bool templateMatching( Mat img, Mat templateImage, int matching_method );

	ros::Publisher objectDetectedPublisher;
  ros::Publisher objectDeminsionsPublisher;
	//vector<Gate*>* gates;
	  //Register a service
	  ros::ServiceClient c21client;
	  ros::ServiceClient c23_start_posecontroller;
	  ros::ServiceClient c23_stop_posecontroller;
	  geometry_msgs::Point point;
	  
  
	  image_transport::ImageTransport it_;
	  typedef image_transport::SubscriberFilter ImageSubscriber;
	  ImageSubscriber left_image_sub_;
	  message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud;
	  tf::TransformListener listener;
	  typedef message_filters::sync_policies::ApproximateTime<
	    sensor_msgs::Image,sensor_msgs::PointCloud2
	    > MySyncPolicy;
	  message_filters::Synchronizer< MySyncPolicy > sync;
	  
	  TARGETS _target;
	  GeneralDetector _generalDetector;
};


#endif
