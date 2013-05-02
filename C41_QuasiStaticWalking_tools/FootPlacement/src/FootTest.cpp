#include "FootPlacementService.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "FootPlacementTest");
	ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient
			<FootPlacement::FootPlacement>("foot_placement");
	FootPlacement::FootPlacement srv;

	/***********************************************************
	 * requested direction X, Y, and which foot to move, and
	 * whether to use C22 or not
	 ***********************************************************/
	srv.request.dirX = 0;
	srv.request.dirY = 1;
	srv.request.foot = LEFT;
	srv.request.useC22 = 0;
	/***********************************************************/
	if (client.call(srv)) {
		printf("list size: %lu\n", srv.response.positions.size());
		for (unsigned long i = 0; i < srv.response.positions.size(); i++) {
			printf("cost %lu: %lf, point: %lf %lf %lf\n", i,
					srv.response.positions[i].cost,
					srv.response.positions[i].point.x,
					srv.response.positions[i].point.y,
					srv.response.positions[i].point.z);
		}
	}
	return 0;
}
