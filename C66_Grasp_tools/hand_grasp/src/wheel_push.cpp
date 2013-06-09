#include <ros/ros.h>
#include <math.h>
#include "move_hand/pelvis_move_hand.h"

int main(int argc, char **argv) {
  ros::init(argc, argv,"point_finger");
  ros::NodeHandle nh_;
  ros::ServiceClient serv = nh_.serviceClient<move_hand::pelvis_move_hand>("pelvis_move_hand");
  move_hand::pelvis_move_hand msg;
  msg.request.quick = true;
  while(ros::ok()){
	  ros::spinOnce();
	  msg.request.PositionDestination_right.x =0.555;
	  msg.request.PositionDestination_right.y =-0.337;
	  msg.request.PositionDestination_right.z =0.103;
	  msg.request.AngleDestination_right.x = -1.153;
	  msg.request.AngleDestination_right.y = 0.113;
	  msg.request.AngleDestination_right.z = 0.126;
	  serv.call(msg);
	  ros::spinOnce();
	  msg.request.PositionDestination_right.x =0.555;
	  msg.request.PositionDestination_right.y =-0.337;
	  msg.request.PositionDestination_right.z =0.26;
	  msg.request.AngleDestination_right.x = -1.153;
	  msg.request.AngleDestination_right.y = 0.113;
	  msg.request.AngleDestination_right.z = 0.126;
	  serv.call(msg);
	  ros::spinOnce();
	  msg.request.PositionDestination_right.x =0.555;
	  msg.request.PositionDestination_right.y =-0.39;
	  msg.request.PositionDestination_right.z =0.26;
	  msg.request.AngleDestination_right.x = -1.153;
	  msg.request.AngleDestination_right.y = 0.113;
	  msg.request.AngleDestination_right.z = 0.126;
	  serv.call(msg);
	  ros::spinOnce();
	  msg.request.PositionDestination_right.x =0.555;
	  msg.request.PositionDestination_right.y =-0.39;
	  msg.request.PositionDestination_right.z =0.103;
	  msg.request.AngleDestination_right.x = -1.153;
	  msg.request.AngleDestination_right.y = 0.113;
	  msg.request.AngleDestination_right.z = 0.126;
	  serv.call(msg);
  }
  return 0;
}
