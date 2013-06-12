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
  ros::init(argc, argv, "c22compact_tester");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<C22_GroundRecognitionAndMapping::C22>("C22");
  C22_GroundRecognitionAndMapping::C22 srv;
  /*
   *  at the moment the C22_node has no use for the data input,
   *  once called it will reply a with a matrix repenting the terrain status in a 25x25x25 cm^3 resolution
   *
   *
  */
  while(1){
	  if (client.call(srv))
	  {
		  cout<<"About to print matrix"<<endl;
		   for (unsigned int i=0;i<srv.response.drivingPath.row.size();i++){
		  		if (i==0){
		  			std::cout<<"   ";
		  			for (unsigned  int j=0;j<srv.response.drivingPath.row.size();j++){
		  				std::cout<<std::setw(2)<<std::setfill('0')<<j<<" ";
		  			}
		  			std::cout<<endl;
		  		}
		  	}
		  	for (int i=srv.response.drivingPath.row.size();i>0;i--){
		  		std::cout <<"\033[0;34m";
		  		std::cout<<std::setw(2)<<std::setfill('0')<<i<<" ";
		  		for (unsigned int j=srv.response.drivingPath.row.size()-1;j>0;j--){
		  			if (srv.response.drivingPath.row.at(i-1).column.at(j).status==C22_GroundRecognitionAndMapping::C22_MAP_SQUARE::AVAILABLE)
		  				std::cout << "\033[0;32m"<< "A  ";
		  				else if (srv.response.drivingPath.row.at(i-1).column.at(j).status==C22_GroundRecognitionAndMapping::C22_MAP_SQUARE::BLOCKED)
							std::cout << "\033[0;33m"<<"B  ";
							else if (srv.response.drivingPath.row.at(i-1).column.at(j).status==C22_GroundRecognitionAndMapping::C22_MAP_SQUARE::ATLAS)
								std::cout << "\033[0;36m"<<"R  ";
								else if (srv.response.drivingPath.row.at(i-1).column.at(j).status==C22_GroundRecognitionAndMapping::C22_MAP_SQUARE::DEBREES)
									std::cout << "\033[0;34m"<<"D  ";
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

	  ros::Rate t(1);
	  t.sleep();
  }

  return 0;
}
