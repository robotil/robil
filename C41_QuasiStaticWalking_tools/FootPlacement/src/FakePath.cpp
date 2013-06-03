#include "FootPlacementService.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "FakePath");
	ros::NodeHandle nh;
	ros::Publisher pathPub = nh.advertise<C31_PathPlanner::C31_Waypoints>("path", 1, true);
	C31_PathPlanner::C31_Waypoints waypoints;
	C31_PathPlanner::C31_Location location;
	location.y = 0.0;
	for (double i = 0; i < 5; i += 0.5) {
		location.x = i;
		waypoints.points.push_back(location);
	}
	for (int i = 0; i < 3; i++) {
		pathPub.publish(waypoints);
		sleep(1);
	}
}
