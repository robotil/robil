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

#include "C23_ObjectRecognition/C23_orient.h"

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

#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>

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
  FIREHOSE_GRIP,
  STANDPIPE,
  INSIDE_STEERINGWHEEL,
  OUTSIDE_STEERINGWHEEL,
  HANDBRAKE,
  GEAR,
  FORWARD_GEAR,
  REVERSE_GEAR,
  BRAKE_PEDAL,
  GAS_PEDAL,  
  TABLE,
  ARROW,
  NONE
} TARGETS;

typedef enum gear_status{
 FORWARD_GEAR_STATUS,
 REVERSE_GEAR_STATUS
}GEAR_STATUS;

typedef enum handbrake_status{
 RELEASED_HANDBRAKE_STATUS,
 ENGAGED_HANDBRAKE_STATUS
}HANDBRAKE_STATUS;

class C23_Detector{
public:
	C23_Detector(const char* left_cam, const char* right_cam, const char* pointc);

  bool detect(string target);
	void callback(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::PointCloud2::ConstPtr &cloud);
    void stopDetection();
public:
	bool is_search;
	double x;
    double x2;
    double y2;
	double y;
	double z;
    bool _found;
	int width;
	int height;
    int last_x;
    int last_y;
    double orient_x;
    double orient_y;
    double orient_z;
    double orient_P;
    double orient_Y;
    double orient_R;
    string current_status;
    GEAR_STATUS current_gear_status;
    HANDBRAKE_STATUS current_handbrake_status;
    pcl::PointCloud<pcl::PointXYZ> lastCloud;
    
private:

  	bool detectGate(Mat img, const sensor_msgs::PointCloud2::ConstPtr &cloud);
    void publishMessage(bool isFound);
    bool detectPath(Mat srcImg);
    bool detectCar(Mat srcImg, const sensor_msgs::PointCloud2::ConstPtr &cloud);
    //bool compareContourAreas ( vector<cv::Point> contour1, vector<cv::Point> contour2) ;
    bool detectPassengerDriver(Mat srcImg, int x1,int y1,int x2,int y2, pcl::PointXYZ minPoint, pcl::PointCloud<pcl::PointXYZ> pclcloud, GeneralDetector::CAR_TARGET *car_target);
    bool detectRearCar(Mat srcImg, int x1,int y1,int x2,int y2, pcl::PointXYZ minPoint, pcl::PointCloud<pcl::PointXYZ> pclcloud, GeneralDetector::CAR_TARGET *car_target);
    bool detectFrontCar(Mat srcImg, int x1,int y1,int x2,int y2, pcl::PointXYZ minPoint, pcl::PointCloud<pcl::PointXYZ> pclcloud, GeneralDetector::CAR_TARGET *car_target);
    bool detectValve(Mat srcImg, const sensor_msgs::PointCloud2::ConstPtr &cloud);
    bool detectFirehose(Mat srcImg, const sensor_msgs::PointCloud2::ConstPtr &cloud);
    bool detectFirehoseGrip(Mat srcImg, const sensor_msgs::PointCloud2::ConstPtr &cloud);
    bool detectStandpipe(Mat srcImg, const sensor_msgs::PointCloud2::ConstPtr &cloud);
    bool detectTable(Mat srcImg, const sensor_msgs::PointCloud2::ConstPtr &cloud);
    bool detectSteeringWheel(Mat srcImg,const sensor_msgs::PointCloud2::ConstPtr &cloud,int location);
    bool detectHandbrake(Mat srcImg,const sensor_msgs::PointCloud2::ConstPtr &cloud,int location);
    bool detectGear(Mat srcImg,const sensor_msgs::PointCloud2::ConstPtr &cloud,int location);
    bool detectArrowDirection(Mat srcImg,const sensor_msgs::PointCloud2::ConstPtr &cloud);
   
    bool pictureCoordinatesToGlobalPosition(double x1, double y1, double x2, double y2, double * x, double* y, double*z,double offsetx = 0, double offsety = 0);
    bool pointCloudCoordinatesToGlobalPosition(double x, double y, double z, double* px, double* py, double*pz);

    bool averagePointCloudInsideCar(int x1, int y1, int x2, int y2, const sensor_msgs::PointCloud2::ConstPtr &cloud, double* px, double* py, double *pz); 
    bool averagePointCloud(int x1, int y1, int x2, int y2, const sensor_msgs::PointCloud2::ConstPtr &detectionCloud, double* px, double* py, double *pz);
    bool process_orientation(C23_ObjectRecognition::C23_orient::Request  &req,
                                           C23_ObjectRecognition::C23_orient::Response &res );
        
	ros::NodeHandle nh;
    bool takePictures(Mat srcImg);
    bool templateMatching( Mat img, Mat templateImage, int matching_method, cv::Point *matchLoc, const sensor_msgs::PointCloud2::ConstPtr &cloud, double *value = NULL);
    bool templateMatching3D(string templates_file, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filterPointCloud(int x,int y, int width, int height, const pcl::PointCloud<pcl::PointXYZ> &cloud);
    void saveTemplate(int x,int y, int width, int height, const sensor_msgs::PointCloud2::ConstPtr &cloud2, string target);
        
	ros::Publisher objectDetectedPublisher;
    ros::Publisher objectDimensionsPublisher;
    ros::Publisher objectGlobalPositionPublisher;
	//vector<Gate*>* gates;
	  //Register a service
	  ros::ServiceClient c21client;
	  ros::ServiceClient c23_start_posecontroller;
	  ros::ServiceClient c23_stop_posecontroller;
      ros::ServiceServer orientation_service;
	  geometry_msgs::Point point;
      Point2f rect_points[4];
	  
  
	  image_transport::ImageTransport it_;
	  typedef image_transport::SubscriberFilter ImageSubscriber;
	  ImageSubscriber left_image_sub_;
	  message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud;
	  tf::TransformListener listener,listener2;
	  typedef message_filters::sync_policies::ApproximateTime<
	    sensor_msgs::Image,sensor_msgs::PointCloud2
	    > MySyncPolicy;
	  message_filters::Synchronizer< MySyncPolicy > sync;
	  
	  TARGETS _target;
	  GeneralDetector _generalDetector;
};


#endif
