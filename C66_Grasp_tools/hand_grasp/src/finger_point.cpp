#include <ros/ros.h>
#include <math.h>
#include <osrf_msgs/JointCommands.h>
#include <atlas_msgs/ForceTorqueSensors.h>
#include <hand_grasp/grasp.h>

int main(int argc, char **argv) {
  ros::init(argc, argv,"point_finger");
  ros::NodeHandle nh_;
  ros::Publisher cmd_pub;
  osrf_msgs::JointCommands command_msg;
  command_msg.name.push_back("atlas::right_f0_j0");
        command_msg.name.push_back("atlas::right_f0_j1");
        command_msg.name.push_back("atlas::right_f0_j2");
        command_msg.name.push_back("atlas::right_f1_j0");
        command_msg.name.push_back("atlas::right_f1_j1");
        command_msg.name.push_back("atlas::right_f1_j2");
        command_msg.name.push_back("atlas::right_f2_j0");
        command_msg.name.push_back("atlas::right_f2_j1");
        command_msg.name.push_back("atlas::right_f2_j2");
        command_msg.name.push_back("atlas::right_f3_j0");
        command_msg.name.push_back("atlas::right_f3_j1");
        command_msg.name.push_back("atlas::right_f3_j2");
        cmd_pub = nh_.advertise<osrf_msgs::JointCommands>("/sandia_hands/r_hand/joint_commands",1,true);
        command_msg.position.resize(12);
                command_msg.position[0] = 0;
                command_msg.position[1] = 1.5;
                command_msg.position[2] = 1.7;
                command_msg.position[3] = 0;
                command_msg.position[4] = 0;
                command_msg.position[5] = 0;
                command_msg.position[6] = 0;
                command_msg.position[7] = 1.5;
                command_msg.position[8] = 1.7;
                command_msg.position[9] = -0.2;
                command_msg.position[10] = 0.8;
                command_msg.position[11] = 1.2;
                cmd_pub.publish(command_msg);
  ros::spin();
  return 0;
}
