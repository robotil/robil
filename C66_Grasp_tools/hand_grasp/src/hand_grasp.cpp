#include <ros/ros.h>
#include <math.h>
#include <osrf_msgs/JointCommands.h>
#include <atlas_msgs/ForceTorqueSensors.h>
#include <hand_grasp/grasp.h>
//#include <boost/thread.hpp>
//#include <boost/algorithm/string.hpp>

class graspper{
private:
  ros::NodeHandle nh_;
  ros::Publisher cmd_pub;
  ros::Subscriber sub_;
  ros::Subscriber grap_sub_;
  double force;
  char hand;
  osrf_msgs::JointCommands command_msg;
public:
  graspper(std::string in_hand){
    hand=in_hand.at(0);
    if (hand=='l'){
    command_msg.name.push_back("atlas::left_f0_j0");
    command_msg.name.push_back("atlas::left_f0_j1");
    command_msg.name.push_back("atlas::left_f0_j2");
    command_msg.name.push_back("atlas::left_f1_j0");
    command_msg.name.push_back("atlas::left_f1_j1");
    command_msg.name.push_back("atlas::left_f1_j2");
    command_msg.name.push_back("atlas::left_f2_j0");
    command_msg.name.push_back("atlas::left_f2_j1");
    command_msg.name.push_back("atlas::left_f2_j2");
    command_msg.name.push_back("atlas::left_f3_j0");
    command_msg.name.push_back("atlas::left_f3_j1");
    command_msg.name.push_back("atlas::left_f3_j2");
    }else{
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
    }
      /*  unsigned int n = command_msg.name.size();
    }
    command_msg.position.resize(n);
    command_msg.velocity.resize(n);
    command_msg.effort.resize(n);
    command_msg.kp_position.resize(n);
    command_msg.ki_position.resize(n);
    command_msg.kd_position.resize(n);
    command_msg.kp_velocity.resize(n);
    command_msg.i_effort_min.resize(n);
    command_msg.i_effort_max.resize(n);*/
    if (hand=='l')
    cmd_pub = nh_.advertise<osrf_msgs::JointCommands>("/sandia_hands/l_hand/joint_commands",1,true);
    else cmd_pub = nh_.advertise<osrf_msgs::JointCommands>("/sandia_hands/r_hand/joint_commands",1,true);
    command_msg.position.resize(12);
/*    for (unsigned int i = 0; i < n; i++)
    {
      std::vector<std::string> pieces;
      boost::split(pieces, command_msg.name[i], boost::is_any_of(":"));

      nh_.getParam("atlas_controller/gains/" + pieces[2] + "/p",
        command_msg.kp_position[i]);

      nh_.getParam("atlas_controller/gains/" + pieces[2] + "/i",
        command_msg.ki_position[i]);

      nh_.getParam("atlas_controller/gains/" + pieces[2] + "/d",
        command_msg.kd_position[i]);

      nh_.getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp",
        command_msg.i_effort_min[i]);
      command_msg.i_effort_min[i] = -command_msg.i_effort_min[i];

      nh_.getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp",
        command_msg.i_effort_max[i]);

      command_msg.velocity[i]     = 0;
      command_msg.effort[i]       = 0;
      command_msg.kp_velocity[i]  = 0;
    }*/

  sub_ = nh_.subscribe("/atlas/force_torque_sensors",10,&graspper::force_CB,this);
  grap_sub_ = nh_.subscribe(std::string("hand_grasp_")+in_hand,2,&graspper::grasp_CB,this);
  }
  ~graspper(){}

void force_CB(const atlas_msgs::ForceTorqueSensorsConstPtr &ForceTorque){
  if (hand=='l') force = sqrt(pow(ForceTorque->l_hand.force.x,2)+pow(ForceTorque->l_hand.force.y,2)+pow(ForceTorque->l_hand.force.z,2)
                 +pow(ForceTorque->l_hand.torque.x,2)+pow(ForceTorque->l_hand.torque.y,2)+pow(ForceTorque->l_hand.torque.z,2));
  else force = sqrt(pow(ForceTorque->r_hand.force.x,2)+pow(ForceTorque->r_hand.force.y,2)+pow(ForceTorque->r_hand.force.z,2)
                 +pow(ForceTorque->r_hand.torque.x,2)+pow(ForceTorque->r_hand.torque.y,2)+pow(ForceTorque->r_hand.torque.z,2));
}

void grasp_CB(const hand_grasp::graspConstPtr &msg){
  double cmd_force=0;
/*  if(msg->strength <=0){
    //TODO:open hand
  }
  else*/
    double x=0;
    while(cmd_force<=msg->strength&&(ros::ok())&&(x<=1)){
      ros::spinOnce();
      cmd_force=force;
        command_msg.position[0] = x*0;
        command_msg.position[1] = x*1.5;
        command_msg.position[2] = x*1.7;
        command_msg.position[3] = x*0;
        command_msg.position[4] = x*1.5;
        command_msg.position[5] = x*1.7;
        command_msg.position[6] = x*0;
        command_msg.position[7] = x*1.5;
        command_msg.position[8] = x*1.7;
        command_msg.position[9] = x*-0.2;
        command_msg.position[10] = x*0.8;
        command_msg.position[11] = x*1.2;
      x+=0.01;
      cmd_pub.publish(command_msg);
      ros::Duration(0.01).sleep();
    }
}
};

int main(int argc, char **argv) {
  ros::init(argc, argv,std::string("hand_grasp_service_")+argv[1]);
  graspper* hand_grasp =new graspper(argv[1]);
  ROS_INFO("ready");
  ros::spin();
  return 0;
}
