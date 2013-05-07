#include "ros/ros.h"
#include "C23_Tasks.cpp"
#include "C23_Detector.hpp"
#include <C0_RobilTask/RobilTask.h>
#include <C0_RobilTask/RobilTaskAction.h>

using namespace C0_RobilTask;

	


int main(int argc, char **argv)
{
  ros::init(argc, argv, "C23_objectReconition");
 
  if(argc!=4){
	 // cout << "usage: C23_module <left camera topic> <right camera topic> <pointcloud topic>" << endl;
	  return 1;
  }
 // startWindowThread();
  C23_Detector detector(argv[1],argv[2],argv[3]);
  C23_SearchObject searchObjectTask(&detector);
  C23_TrackObject trackObjectTask(&detector);
  C23_TakePicture takePictureTask(&detector);
  ros::spin();

  return 0;
}
