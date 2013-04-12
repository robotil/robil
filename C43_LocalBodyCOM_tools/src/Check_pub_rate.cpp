#include "ros/ros.h"
#include "std_msgs/String.h"
// #include <hrl_kinematics/CoM_Array_msg.h>
#include <hrl_kinematics/TestStability.h>


static ros::Publisher pub;

void pub_COM_callback(const sensor_msgs::JointStateConstPtr& state)
{

  ROS_INFO("I heard: joints");
  hrl_kinematics::CoM_Array_msg com_point;
  com_point.header.stamp.sec = state->header.stamp.sec;
  com_point.header.stamp.nsec = ros::Time::now().nsec - state->header.stamp.nsec; //
  /* if (!com_Arr_.empty()) {
      for (unsigned i = 0; i < 3; ++i){
          com_point.x[i] = com_Arr_[i].x();
          com_point.y[i] = com_Arr_[i].y();
          com_point.z[i] = com_Arr_[i].z();
      }
  } */

  pub.publish(com_point); //Yuval added
  // ros::spinOnce();

} //Yuval added


int main(int argc, char **argv)
{

  ros::init(argc, argv, "test_pub_rate");

  ros::spinOnce();

  ros::NodeHandle n;

  pub = n.advertise<hrl_kinematics::CoM_Array_msg>("PubR_CoM", 100);
  ros::Subscriber sub = n.subscribe("joint_states", 100, pub_COM_callback);

  ros::spin();

  return 0;
}
