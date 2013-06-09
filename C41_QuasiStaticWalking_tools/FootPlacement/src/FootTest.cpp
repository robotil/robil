#include "FootPlacementService.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "FootPlacementTest");
	ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient
			<FootPlacement::FootPlacement_Service>("foot_placement");
	FootPlacement::FootPlacement_Service srv;

	/***********************************************************
	 * setting feet positions on request
	 ***********************************************************/
	srv.request.start_pose.foot_index = LEFT;
	srv.request.start_pose.pose.position.x = 0.0;
	srv.request.start_pose.pose.position.y = 0.0;
	srv.request.start_pose.pose.position.z = 0.0;
	srv.request.start_pose.pose.ang_euler.x = 0.0;
	srv.request.start_pose.pose.ang_euler.y = 0.0;
	srv.request.start_pose.pose.ang_euler.z = 0.0;
	srv.request.other_foot_pose.foot_index = RIGHT;
	srv.request.other_foot_pose.pose.position.x = 1.0;
	srv.request.other_foot_pose.pose.position.y = 1.0;
	srv.request.other_foot_pose.pose.position.z = 1.0;
	srv.request.other_foot_pose.pose.ang_euler.x = 0.0;
	srv.request.other_foot_pose.pose.ang_euler.y = 0.0;
	srv.request.other_foot_pose.pose.ang_euler.z = 0.0;
	/***********************************************************/
	if (client.call(srv)) {
		printf("list size: %lu\n", srv.response.foot_placement_path.size());
		for (unsigned long i = 0; i < srv.response.foot_placement_path.size(); i++) {
			printf("step %lu: position: %lf %lf %lf\n", i,
					srv.response.foot_placement_path[i].pose.position.x,
					srv.response.foot_placement_path[i].pose.position.y,
					srv.response.foot_placement_path[i].pose.position.z);
		}
	}
	return 0;
}
