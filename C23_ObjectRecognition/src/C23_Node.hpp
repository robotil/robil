#ifndef __C23_NODE_HPP__
#define __C23_NODE_HPP__


#include <image_transport/image_transport.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <image_transport/subscriber_filter.h>
#include "libopentld/tld/TLD.h"
#include "C23_Node_TLD_Handler.h"

#include "C23_Utils.hpp"

namespace enc=sensor_msgs::image_encodings;
using namespace tld;


class C23_Node {

public:

	  C23_Node(std::string left_camera,std::string right_camera);
	  bool detectAndTrack(MODELS model);
	  void callback(const sensor_msgs::ImageConstPtr& left_msg,const sensor_msgs::ImageConstPtr& right_msg);
	  void startDetection();
	  void stopDetection();
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  typedef image_transport::SubscriberFilter ImageSubscriber;
  ImageSubscriber left_image_sub_;
  ImageSubscriber right_image_sub_;
  ros::ServiceServer service;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  message_filters::Synchronizer< MySyncPolicy > sync;
  TLD tld;
  C23_Node_TLD_Handler *tldh_;
  MODELS currentModel_;
  bool detect;
  bool done_processing;
  ros::Publisher objectDetectedPublisher;
  ros::Publisher objectDeminsionsPublisher;
  
public:
    int x,y,width,height;
    float confident;
};

#endif
