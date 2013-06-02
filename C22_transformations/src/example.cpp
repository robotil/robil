#include <ros/ros.h>
#include "trans.hpp"
#include <C22_GroundRecognitionAndMapping/C22.h>

int main(int argc,char**argv){
	 ros::init(argc, argv, "c22_tester");
	 ros::NodeHandle n;
	 ros::ServiceClient client = n.serviceClient<C22_GroundRecognitionAndMapping::C22>("C22");
	 C22_GroundRecognitionAndMapping::C22 srv;
	 /*
	  *  at the moment the C22_node has no use for the data input,
	  *  once called it will reply a with a matrix repenting the terrain status in a 25x25x25 cm^3 resolution
	  *
	  */
		  if (client.call(srv))
		  {
			  C22_transform trans;
			 std::cout<<"Global position in map:\n"<<trans.globalPosToMap(srv.response.drivingPath)<<"\n";
		  }else{
			  std::cout<<"service fail\n";
		  }
		  return 0;
}
