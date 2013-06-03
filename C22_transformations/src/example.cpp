#include <ros/ros.h>
#include <C22_transformations/MapTransformations.h>
#include <C22_GroundRecognitionAndMapping/C22.h>

struct XY{public: double x,y;};

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
			 XY bot;
			 trans.GlobalToMap(srv.response.drivingPath, srv.response.drivingPath.robotPos, bot);
			 std::cout<<"Global position in map: GLOBAL("<<srv.response.drivingPath.robotPos.x<<","<<srv.response.drivingPath.robotPos.y<<") -> MAP("<<bot.x<<","<<bot.y<<")\n";
		  }else{
			  std::cout<<"service fail\n";
		  }
		  return 0;
}
