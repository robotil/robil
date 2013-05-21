/*********************************************************8
 * a test module for the C21_Node
 * please lunch only after the C21_Node is running
 *
 */

#include "ros/ros.h"
#include "C22_GroundRecognitionAndMapping/C22.h"

#include "stdio.h"

using namespace std;

int main(int argc, char **argv)
{
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
		  cout<<"About to print matrix"<<endl;
		   for (unsigned int i=0;i<srv.response.drivingPath.row.size();i++){
		  		if (i==0){
		  			std::cout<<"   ";
		  			for (int j=srv.response.drivingPath.row.size()-1;j>=0;j--){
		  				std::cout<<std::setw(2)<<std::setfill('0')<<j<<" ";
		  			}
		  			std::cout<<endl;
		  		}
		  	}
		   int robotPoseRow=(srv.response.drivingPath.robotPos.x-srv.response.drivingPath.xOffset)*4;
		   int robotPoseColum=(srv.response.drivingPath.robotPos.y-srv.response.drivingPath.yOffset)*4;
		  	for (int i=srv.response.drivingPath.row.size();i>0;i--){
		  		std::cout<<std::setw(2)<<std::setfill('0')<<i<<" ";
		  		for (int j=srv.response.drivingPath.row.size()-1;j>=0;j--){
		  			if(robotPoseColum==j && robotPoseRow==i)
		  				std::cout <<  "\033[0;34m"<< "R  ";
						else if (srv.response.drivingPath.row.at(i-1).column.at(j).status==0)
							std::cout << "\033[0m"<< "A  ";
							else if (srv.response.drivingPath.row.at(i-1).column.at(j).status==1)
								std::cout << "\033[0;33m"<<"B  ";
							else std::cout << "\033[0m"<< "-  ";
		  		}
		  		std::cout <<endl;
		  	}
	  }
	  else
	  {
		  ROS_ERROR("couldn't get a reply\n");
		return 1;
	  }

  return 0;
}
