/*********************************************************8
 * a test module for the C21_Node
 * please lunch only after the C21_Node is running
 *
 */

#include "ros/ros.h"
#include "C22_CompactGroundRecognitionAndMapping/C22.h"

#include "stdio.h"

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "c22compact_tester");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<C22_CompactGroundRecognitionAndMapping::C22>("C22compact");
  C22_CompactGroundRecognitionAndMapping::C22 srv;
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
		  			for (unsigned  int j=0;j<srv.response.drivingPath.row.size();j++){
		  				std::cout<<std::setw(2)<<std::setfill('0')<<j<<" ";
		  			}
		  			std::cout<<endl;
		  		}
		  	}
		  	for (int i=srv.response.drivingPath.row.size();i>0;i--){
		  		std::cout<<std::setw(2)<<std::setfill('0')<<i<<" ";
		  		for (unsigned int j=0;j<srv.response.drivingPath.row.size();j++){
		  			if (srv.response.drivingPath.row.at(i-1).column.at(j).status==0)
		  				std::cout << "A  ";
		  				else if (srv.response.drivingPath.row.at(i-1).column.at(j).status==1)
		  					std::cout << "B  ";
		  				else std::cout << "-  ";
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
