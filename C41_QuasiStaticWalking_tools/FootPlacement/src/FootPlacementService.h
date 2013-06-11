#ifndef FOOTPLACEMENTSERVICE_H_
#define FOOTPLACEMENTSERVICE_H_

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "C22_CompactGroundRecognitionAndMapping/C22.h"
#include "C31_PathPlanner/C31_Waypoints.h"
//#include <C22_transformations/MapTransformations.h>
#include "FootPlacement/FootPlacement_Service.h"
#include "FootPlacement/Foot_Placement_path.h"
#include "C42_WalkType/extreme_slope.h"
#include "tf/transform_listener.h"

#define SQUARE_SIZE 0.05
#define C22_SIZE 40
#define SIZE 36
#define STEPS 5

#define MAX_STEP_SIZE 0.3
#define MIN_STEP_SIZE 0.1
#define MAX_STEP_HEIGHT 0.1

#define SLOPE_WEIGHT 5
#define DISTANCE_WEIGHT 5
#define HEIGHT_WEIGHT 5
#define DIRECTION_WEIGHT 5
#define GROUND_TYPE_WEIGHT 5

#define LEFT 0
#define RIGHT 1
#define NORMALIZER 100
#define EXTREME_SLOPE 0.174532925
#define EXTREME_SLOPE_COR 0.5

#define MIN_DIST_FROM_TARGET 1

#define USE_C22 0

#define PI 3.14159265
#define MAX_ORIENTATION_CHANGE PI/4


class FootPlacementService {
private:
	ros::NodeHandle nh;
	ros::ServiceClient mapClient;
	ros::ServiceServer footServer;
	ros::Subscriber pathSub;
	ros::Publisher footPlacementPathPublisher;
	ros::Publisher walkNotificationPublisher;
	C22_CompactGroundRecognitionAndMapping::C22 mapService;
	C31_PathPlanner::C31_Waypoints::ConstPtr pathPoints;

	//callback function when service is requested
	bool callback(FootPlacement::FootPlacement_Service::Request &req,
			FootPlacement::FootPlacement_Service::Response &res);

	//callback function when path has been received
	void getPath(const C31_PathPlanner::C31_Waypoints::ConstPtr& points);

	//kinamtic possibility
	int possible(int i, int j);

	void createAvgMatrix(
			C22_CompactGroundRecognitionAndMapping::C22_PLANE_TYPE avgMap[SIZE][SIZE],
			const C22_CompactGroundRecognitionAndMapping::C22C0_PATH& path);

	//calculate point in the place i,j on plane
	geometry_msgs::Point calcPoint(const int &i, const int &j,
			const C22_CompactGroundRecognitionAndMapping::C22_PLANE_TYPE &plane,
			const geometry_msgs::Point &robotPos,
			const geometry_msgs::Point &robotOri);

	//calculate the angle between the plane (a,b,c) and the plane XY [(0,0,1)]
	double calcSlope(const double &a,const double &b,const double &c);

	//calculate the distance between (x1,y1) to (x2,y2)
	double calcDistance(const double &x1,const double &y1,
			const double &x2,const double &y2);

	//calculate the angle between (x1,y1) to (x2,y2)
	double calcAngle(const double &x1,const double &y1,
			const double &x2,const double &y2);

	//calculate the weight of single cell
	double singleCellWeight(const double &legDistance,
			const double &slope,const double &distance,
			const double &height,const double &direction,const double &slopeWeight,
			const double &distanceWeight,const double &heightWeight,
			const double &directionWeight);


	void copyFoot(FootPlacement::Foot_Placement_data &a, FootPlacement::Foot_Placement_data &b);

	void calcFootMatrix(
			std::vector<FootPlacement::Foot_Placement_data>& foot_placement_path,
			const int &useC22,
			const C22_CompactGroundRecognitionAndMapping::C22C0_PATH& map,
			const FootPlacement::Foot_Placement_data& start_pose,
			const FootPlacement::Foot_Placement_data& other_foot_pose,
			const std::vector<C31_PathPlanner::C31_Location>& points,
			const double &slopeWeight,
			const double &distanceWeight,
			const double &heightWeight,
			const double &directionWeight);




	void publishExtremeSlopeMsg();

public:
	FootPlacementService();
};

#endif
