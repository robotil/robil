/**************************************************************************************
 * This is a template for the c11_Operator_control module for the robil project
 * The c11_Operator_control module goal is to provide the operator control
 * There is no actual input/output at this time, this goes according to the current milestone
 * set at the robil management meeting
 *
 **************************************************************************************/

#ifndef C11_NODE_H
#define C11_NODE_H

#include "ros/ros.h"
#include "C11_OperatorControl/C11.h"
#include "C11_Node_Subscriber.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <string>
#include <QThread>
#include <QStringListModel>

namespace enc=sensor_msgs::image_encodings;

/**
 * this class represent the C11_Node,
 * it subscribe to two camera/image topics and provide the path
 **/
class C11_Node : public QThread
{
  Q_OBJECT


public:

        /**
         * constructor, initializes the ROS node, subscribe it to the given topics and instruct it to provide the service
         */
          C11_Node(IC11_Node_Subscriber* subscriber);
          C11_Node(int argc, char** argv , IC11_Node_Subscriber* subscriber);
          virtual ~C11_Node();

          /**
           * The call back function executed when a service is requested
           * it must return true in order to work properly
           * @param req the request message, generated by the node requesting the service
           * @param res the response message, generated by the service node when a service is requested
           */
          bool proccess(C11_OperatorControl::C11::Request  &req,
                        C11_OperatorControl::C11::Response &res );

          bool init();

          void run();

          static void viewImage(const sensor_msgs::ImageConstPtr& msg);

Q_SIGNALS:
        void loggingUpdated();
    void rosShutdown();

private:
  ros::NodeHandle *nh_;
  ros::ServiceServer service;
  image_transport::ImageTransport* it_;
  image_transport::Subscriber panoramic_image;
  int init_argc;
  char** init_argv;
  static IC11_Node_Subscriber* pIC11_Node_Subscriber;
};

#endif // C11_NODE_H
