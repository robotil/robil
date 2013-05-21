#include "FootPlacementService.h"

/*
 * subscribe to service C22
 * advertise service foot_placement
 */
FootPlacementService::FootPlacementService() {
	mapClient = nh.serviceClient
			<C22_CompactGroundRecognitionAndMapping::C22>("C22compact");
	footServer = nh.advertiseService
			("foot_placement", &FootPlacementService::callback, this);
}

/*
 * callback function when foot_placement service is called
 */
bool FootPlacementService::callback(FootPlacement::FootPlacement::Request &req,
		FootPlacement::FootPlacement::Response &res) {
	// requesting for map recognition
	if (mapClient.call(mapService)) {

		geometry_msgs::Point pelvis = mapService.response.drivingPath.robotPos;
		geometry_msgs::Point leftFootPos;
		geometry_msgs::Point rightFootPos;

		// find feet positions
		tf::TransformListener listener;
	    tf::StampedTransform tfLeftFoot;
	    tf::StampedTransform tfRightFoot;
	    bool flag = true;
	    while (flag) {
			try {
				listener.waitForTransform("/pelvis", "/l_foot" ,ros::Time(0), ros::Duration(0.2));
				listener.lookupTransform("/pelvis", "/l_foot" ,ros::Time(0), tfLeftFoot);
				listener.waitForTransform("/pelvis", "/r_foot" ,ros::Time(0), ros::Duration(0.2));
				listener.lookupTransform("/pelvis", "/r_foot" ,ros::Time(0), tfRightFoot);
				flag = false;
			} catch (tf::TransformException &ex) {
				ROS_ERROR("%s",ex.what());
			}
	    }
	    leftFootPos.x = pelvis.x + tfLeftFoot.getOrigin().getX();
	    leftFootPos.y = pelvis.y + tfLeftFoot.getOrigin().getY();
	    leftFootPos.z = pelvis.z + tfLeftFoot.getOrigin().getZ();
	    rightFootPos.x = pelvis.x + tfRightFoot.getOrigin().getX();
	    rightFootPos.y = pelvis.y + tfRightFoot.getOrigin().getY();
	    rightFootPos.z = pelvis.z + tfRightFoot.getOrigin().getZ();

	    // calculating the costs list
	    calcFootMatrix(
				res.positions,
				req.useC22, req.foot,
				mapService.response.drivingPath,
				leftFootPos, rightFootPos,
				req.dirX, req.dirY,
				SLOPE_WEIGHT, DISTANCE_WEIGHT, HEIGHT_WEIGHT, DIRECTION_WEIGHT);
	}
	return true;
}

/*
 * initiating the node and the service
 */
int main(int argc, char **argv) {
	ros::init(argc, argv, "FootPlacementNode");
	FootPlacementService footPlacementService;
	ROS_INFO("FOOTPLACEMENT SERVICE READY");
	ros::spin();
	return 0;
}
