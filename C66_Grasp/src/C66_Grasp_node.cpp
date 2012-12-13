#include "ros/ros.h"
#include <std_msgs/Float64.h>
//#include "std_msgs/String.h"
#include <cmath>

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "C66_Grasp_node");
  ros::NodeHandle n;
  ros::Publisher finger0_joint0 = n.advertise<std_msgs::Float64>("/r_f0_j0_position_controller/command", 1000);
  ros::Publisher finger0_joint1 = n.advertise<std_msgs::Float64>("/r_f0_j1_position_controller/command", 1000);
  ros::Publisher finger0_joint2 = n.advertise<std_msgs::Float64>("/r_f0_j2_position_controller/command", 1000);
  ros::Publisher finger1_joint0 = n.advertise<std_msgs::Float64>("/r_f1_j0_position_controller/command", 1000);
  ros::Publisher finger1_joint1 = n.advertise<std_msgs::Float64>("/r_f1_j1_position_controller/command", 1000);
  ros::Publisher finger1_joint2 = n.advertise<std_msgs::Float64>("/r_f1_j2_position_controller/command", 1000);
  ros::Publisher finger2_joint0 = n.advertise<std_msgs::Float64>("/r_f2_j0_position_controller/command", 1000);
  ros::Publisher finger2_joint1 = n.advertise<std_msgs::Float64>("/r_f2_j1_position_controller/command", 1000);
  ros::Publisher finger2_joint2 = n.advertise<std_msgs::Float64>("/r_f2_j2_position_controller/command", 1000);
  ros::Publisher finger3_joint0 = n.advertise<std_msgs::Float64>("/r_f3_j0_position_controller/command", 1000);
  ros::Publisher finger3_joint1 = n.advertise<std_msgs::Float64>("/r_f3_j1_position_controller/command", 1000);
  ros::Publisher finger3_joint2 = n.advertise<std_msgs::Float64>("/r_f3_j2_position_controller/command", 1000);

  ros::Rate loop_rate(10);

//  int x;
//  x=argc;
//  ROS_INFO("%d", x);
//  for(;x!=0;x--){
//	  ROS_INFO("%d - %s", x, argv[x]);
//  }

  double speed;
  if(argc > 1){
	  speed = atof(argv[1]);
  }else{
	  speed = 0.01;
  }
  int count = 0;
  while (ros::ok())
  {
    std_msgs::Float64 msg1, msg2, msg3, msg4;//, f0_j0_msg, f2_j0_msg;
    msg1.data = sin(count*speed)*0.2 + 1.3;
    msg2.data = msg1.data*0.5 + 0.2;
    msg3.data = sin(count*speed)*0.3 + 0.3;
    msg4.data = msg3.data*0.7;
    //f0_j0_msg.data = sin(count*speed)*0.3 - 0.3;
    //f2_j0_msg.data = -sin(count*speed)*0.3 + 0.3;

    ROS_INFO("%f.04", msg1.data);
    finger0_joint1.publish(msg1);
    finger1_joint1.publish(msg1);
    finger2_joint1.publish(msg1);
    finger3_joint1.publish(msg3);
    //finger0_joint0.publish(f0_j0_msg);
    //finger2_joint0.publish(f2_j0_msg);
    finger0_joint2.publish(msg2);
    finger1_joint2.publish(msg2);
    finger2_joint2.publish(msg2);
    finger3_joint2.publish(msg4);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
