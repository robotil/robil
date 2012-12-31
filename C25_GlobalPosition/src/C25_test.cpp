#include "ros/ros.h"
#include "C25_GlobalPosition/C25.h"
#include "C25_GlobalPosition/C25C0_ROP.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "C25_test");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<C25_GlobalPosition::C25>("C25/service");
  C25_GlobalPosition::C25 srv;
  /*
   *  at the moment the C21_node has no use for the data input,
   *  once called it will reply a with a global position and an IMU data
   *
  */
	  if (client.call(srv))
	  {
		  cout<<"Imu and position data"<<endl;
		  cout<<"========================="<<endl;
		  cout<<"Imu"<<endl;
		  cout<<"---"<<endl;
		  cout<<srv.response.robotPosition.imu<<endl;
		  cout<<"World pose and orientation"<<endl;
		  cout<<"--------------------------"<<endl;
		  cout<<srv.response.robotPosition.pose<<endl;
	  }
	  else
	  {
		  ROS_ERROR("couldn't get a reply\n");
		return 1;
	  }

  return 0;
}
