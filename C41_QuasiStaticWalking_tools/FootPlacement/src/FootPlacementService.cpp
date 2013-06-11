#include "FootPlacementService.h"

/*
 * subscribe to service C22
 * advertise service foot_placement
 * subscribe to path topic
 */
FootPlacementService::FootPlacementService() {
	mapClient = nh.serviceClient
			<C22_CompactGroundRecognitionAndMapping::C22>("C22compact");
	footServer = nh.advertiseService
			("foot_placement", &FootPlacementService::callback, this);
	ros::SubscribeOptions pathSo =
			ros::SubscribeOptions::create<C31_PathPlanner::C31_Waypoints>(
			"path", 1,
			boost::bind( &FootPlacementService::getPath,this,_1),
		    ros::VoidPtr(), nh.getCallbackQueue());
	pathSub = nh.subscribe(pathSo);
	footPlacementPathPublisher = nh.advertise<FootPlacement::Foot_Placement_path>("foot_placement", 1, true);
	walkNotificationPublisher = nh.advertise<C42_WalkType::extreme_slope>("walk_notification/extreme_slope", 1, true);
}

/*
 * callback function when foot_placement service is called
 */
bool FootPlacementService::callback(FootPlacement::FootPlacement_Service::Request &req,
		FootPlacement::FootPlacement_Service::Response &res) {
	// requesting for map recognition
	if (mapClient.call(mapService)) {

		geometry_msgs::Point pelvis = mapService.response.drivingPath.robotPos;

		/****************************************************************************************
		 * code to find feet position. not necessary anymore since we get it at service request *
		 ****************************************************************************************
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
		****************************************************************************************/

	    // calculating the costs list
	    calcFootMatrix(
				res.foot_placement_path,
				USE_C22,
				mapService.response.drivingPath,
				req.start_pose, req.other_foot_pose,
				pathPoints->points,
				SLOPE_WEIGHT, DISTANCE_WEIGHT, HEIGHT_WEIGHT, DIRECTION_WEIGHT);

        // FIXME: Now needs to transform points to global coordinates.


	}
	else {
		printf("ERROR! NO MAP SERVICE!\n");
		return false;
	}
	FootPlacement::Foot_Placement_path footPlacementPath;
	footPlacementPath.foot_placement_path = res.foot_placement_path;

	footPlacementPathPublisher.publish(footPlacementPath);
	return true;
}

/*
 * updating the path points
 */
void FootPlacementService::getPath(const C31_PathPlanner::C31_Waypoints::ConstPtr& points) {
	pathPoints = points;
	printf("Received path:\n");
	for (unsigned long i = 0; i < points->points.size(); i++)
		printf("  point %lu: %lf %lf\n", i, points->points[i].x, points->points[i].y);
}

/*
 * publishing an empty extreme slope message
 */
void FootPlacementService::publishExtremeSlopeMsg() {
	C42_WalkType::extreme_slope emptyMsg;
	walkNotificationPublisher.publish(emptyMsg);
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
