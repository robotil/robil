#ifndef FOOTPLACEMENTSERVICE_H_
#define FOOTPLACEMENTSERVICE_H_

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "C22_CompactGroundRecognitionAndMapping/C22.h"
#include "FootPlacement/FootPlacement.h"
#include "tf/transform_listener.h"

#define SQUARE_SIZE 0.25
#define SIZE 19
#define C22_SIZE 100

#define SLOPE_WEIGHT 0
#define DISTANCE_WEIGHT 5
#define HEIGHT_WEIGHT 0
#define DIRECTION_WEIGHT 5

#define LEFT 0
#define RIGHT 1
#define NORMALIZER 100

class FootPlacementService {
private:
	ros::NodeHandle nh;
	ros::ServiceClient mapClient;
	ros::ServiceServer footServer;
	C22_CompactGroundRecognitionAndMapping::C22 mapService;

	//callback function when service is requested
	bool callback(FootPlacement::FootPlacement::Request &req,
			FootPlacement::FootPlacement::Response &res);

	//kinamtic possibility
	int possible(int i, int j);

	void createMatrix25(int map[SIZE][SIZE],
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
	double singleCellWeight(const double &slope,const double &distance,
			const double &height,const double &direction,const double &slopeWeight,
			const double &distanceWeight,const double &heightWeight,
			const double &directionWeight);

	void calcFootMatrix(
			std::vector<FootPlacement::Pos>& positions,
			const int &useC22,
			const int &leg,
			const C22_CompactGroundRecognitionAndMapping::C22C0_PATH& path,
			const geometry_msgs::Point& robotLeftLegPos,
			const geometry_msgs::Point& robotRightLegPos,
			const double &dirX, const double &dirY,
			const double &slopeWeight,
			const double &distanceWeight,
			const double &heightWeight,
			const double &directionWeight);

public:
	FootPlacementService();
};

#endif
