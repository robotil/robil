#include "FootPlacementService.h"

// fake function for C22 service
bool FakeC22(C22_GroundRecognitionAndMapping::C22::Request &req,
		C22_GroundRecognitionAndMapping::C22::Response &res) {

	// initiating the map matrix
	res.drivingPath.row.resize(SIZE);
	for (int i = 0; i < SIZE; i++)
		res.drivingPath.row[i].column.resize(SIZE);

	/*********************************************************
	 * fake robot position
	 *********************************************************/
	res.drivingPath.robotPos.x = res.drivingPath.robotPos.y = 0;
	res.drivingPath.robotPos.z = 0.93;
	res.drivingPath.robotOri.x = res.drivingPath.robotOri.z = 0;
	 res.drivingPath.robotOri.y =1;
	/*********************************************************
	 * fake planes
	 *********************************************************/
	for (int i = 0; i < SIZE; i++)
		{
			for (int j = 0; j < SIZE; j++)
			{
				C22_GroundRecognitionAndMapping::C22_PLANE_TYPE *plane;
				res.drivingPath.row[i].column[j].planes.resize(1);
				plane = &(res.drivingPath.row[i].column[j].planes[0]);

				plane->x = 0;
				plane->y = 0;
				plane->z = 1;
				plane->d = 0;
				/*plane->representingPoint.x=(j-3)*0.15;
				plane->representingPoint.y=(i-3)*0.15;
				plane->representingPoint.z=-(plane->x*plane->representingPoint.x
						+plane->y*plane->representingPoint.y+plane->d)/plane->z;*/
			}
		}
	/*********************************************************/
	return true;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "FakeC22");
	ros::NodeHandle nh;

	ros::ServiceServer fakeC22 = nh.advertiseService("C22", FakeC22);
	ROS_INFO("FAKE C22 READY");
	ros::spin();
	return 0;
}


