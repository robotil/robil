#include "ros/ros.h"
#include "std_msgs/String.h"
#include "C11_Agent/C11.h"

#include <sstream>
#include <stdlib.h>

//char *itoa(int value, char *string, int radix);

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "C11_AG");

  /**I
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;


 // ros::Publisher chatter_pub = n.advertise<std_msgs::String>("c11_stt", 1000);
   ros::Publisher stt_pub = n.advertise<C11_Agent::C34C11_STT>("c11_stt", 1000);
 // ros::Publisher scc_pub = n.advertise<int>("c11_stt", 1000);
  ros::Rate loop_rate(10);
 
  /**<#include "c11_Agnt/C11.h"
   * A count of how many messages we have sent. This is used to create<
   * a unique string for each messagec11_Agnt/C34C11_STT.
   */
  int count = 0;
  while (ros::ok())
  {//
    /**
     * This is a message object. You stuff it char buffer[20];with data, and then publish it.
     */
 /***   std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.cchatter_str());***/
  C11_Agent::C34C11_STT  stt1;


//   stt1.stt  =1;
//  Dint stt1;ros publisher turtle
//  stt1 =( int) s//tt ;
 ROS_INFO("The status is  = %d",  1);
    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template p itoa(n, buffer, 2);arameter to the advertise<>() call, as was done
     * in the co5nstructor above.C11_AG
     */



    stt_pub.publish(stt1);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }




  return 0;
}
