#include "FootPlacementService.h"

int FootPlacementService::possible(int i, int j)
{
	if(0==i && SIZE/2==j)
		return 0;
	return 1;
}

void FootPlacementService::createAvgMatrix(
		C22_CompactGroundRecognitionAndMapping::C22_PLANE_TYPE avgMap[SIZE][SIZE],
		const C22_CompactGroundRecognitionAndMapping::C22C0_PATH& path)
{

	for (int i=0; i<SIZE-5+1; i++)
	{
		for(int j=0; j<SIZE-5+1; j++)
		{
			printf("%d %d\n",i,j);
			C22_CompactGroundRecognitionAndMapping::C22_PLANE_TYPE plane;
			plane.x=plane.y=plane.z=plane.d=0;
			plane.repPoint.x=plane.repPoint.y=plane.repPoint.z=0;
			int size=0;
			for(int k=0; k<5; k++)
			{
				for(int l=0; l<5; l++)
				{
					for(unsigned m=0; m<path.row[i].column[j].planes.size(); m++)
					{
						if(2==path.row[i+k].column[j+l].status)
						{
							continue;
						}
						plane.x+=path.row[i+k].column[j+l].planes[m].x;
						plane.y+=path.row[i+k].column[j+l].planes[m].y;
						plane.z+=path.row[i+k].column[j+l].planes[m].z;
						plane.d+=path.row[i+k].column[j+l].planes[m].d;
						plane.repPoint.x+=path.row[i+k].column[j+l].planes[m].repPoint.x;
						plane.repPoint.y+=path.row[i+k].column[j+l].planes[m].repPoint.y;
						plane.repPoint.z+=path.row[i+k].column[j+l].planes[m].repPoint.z;
						size++;
						//ground type handle
					}
				}
			}
			avgMap[i][j]=plane;
		}
	}

	/*
	for (int i=0; i<SIZE; i++)
		{
			for(int j=0; j<SIZE; j++)
			{
				printf("%d ",map[i][j]);
			}
			printf("\n");
		}
		printf("\n");
	*/
}

geometry_msgs::Point FootPlacementService::calcPoint(const int &i, const int &j,
		const C22_CompactGroundRecognitionAndMapping::C22_PLANE_TYPE &plane,
		const geometry_msgs::Point &robotPos, const geometry_msgs::Point &robotOri)
{
	//static int flag=1;
	geometry_msgs::Point point,xStep,yStep;
	xStep.z=yStep.z=0;

	yStep.x = SQUARE_SIZE* sin( robotOri.z);
	yStep.y = SQUARE_SIZE* cos( robotOri.z);


	/*
	yStep.y=1*SQUARE_SIZE;
	yStep.x=0;
	*/

	//xstep is 90deg clockwise from ystep
	xStep.x= yStep.y;
	xStep.y = -yStep.x;
	/*	
	if(flag)
	{
		printf("x: %lf %lf\ny: %lf %lf\n", xStep.x,xStep.y, yStep.x, yStep.y);
		flag=0;
	}	

	*/
	
	point.x =-(C22_SIZE-2)*xStep.x+ robotPos.x+ (j)*xStep.x+(i)*yStep.x;
	point.y= -(C22_SIZE-2)*xStep.y+ robotPos.y+(j)*xStep.y+(i)*yStep.y;
	point.z=-(plane.x*point.x+plane.y*point.y+plane.d)/plane.z;
	return point;
}


//calculate the angle between the plane (a,b,c) and the plane XY [(0,0,1)]
double FootPlacementService::calcSlope(const double &a,const double &b,const double &c)
{
	return fabs(asin(c/sqrt(a*a+b*b+c*c)));
}

//calculate the distance between (x1,y1) to (x2,y2)
double FootPlacementService::calcDistance(const double &x1,const double &y1,
		const double &x2,const double &y2)
{
	double dx = (x1-x2);
	double dy = (y1-y2);
	return sqrt( dx*dx+dy*dy );
}

//calculate the angle between (x1,y1) to (x2,y2)
double FootPlacementService::calcAngle(const double &x1,const double &y1,
		const double &x2,const double &y2)
{
	double  norm1 = calcDistance(0,0,x1,y1);
	double 	norm2 = calcDistance(0,0,x2,y2);
	return 	fabs(acos((x1*x2+y1*y2) / (norm1*norm2)));
}

//calculate the weight of single cell
double FootPlacementService::singleCellWeight(const double &slope,const double &distance,
		const double &height,const double &direction,const double &slopeWeight,
		const double &distanceWeight,const double &heightWeight,
		const double &directionWeight)
{
	return slope*slopeWeight+distance*distanceWeight+height*heightWeight+
			direction*directionWeight/NORMALIZER;
}

/*
double FootPlacementService::physicalWeight(const double &slope,
		const double &height, int groundType)
{
	return slope*SLOPE_WEIGHT+height*HEGIHT_WEIGHT+groundType*GROUND_TYPE_WEIGHT;
}
*/


void FootPlacementService::calcFootMatrix(
		std::vector<FootPlacement::Foot_Placement_data>& foot_placement_path,
		const int &useC22,
		const C22_CompactGroundRecognitionAndMapping::C22C0_PATH& map,
		const FootPlacement::Foot_Placement_data& start_pose,
		const FootPlacement::Foot_Placement_data& other_foot_pose,
		const std::vector<C31_PathPlanner::C31_Location>& points,
		const double &slopeWeight,
		const double &distanceWeight,
		const double &heightWeight,
		const double &directionWeight)
{
	printf("start\n");
	FootPlacement::Foot_Placement_data startPose=start_pose;
	FootPlacement::Foot_Placement_data otherFootPose=other_foot_pose;
	C22_CompactGroundRecognitionAndMapping::C22_PLANE_TYPE avgMap[SIZE][SIZE];
	const geometry_msgs::Point &robotPos = map.robotPos;
	const geometry_msgs::Point &robotOri = map.robotOri;
	FootPlacementService::createAvgMatrix(avgMap,map);
	unsigned curPoint=0;
	double minCost=std::numeric_limits<double>::infinity();
	geometry_msgs::Point minCostPoint;

	printf("pos: %lf %lf %lf \n",robotPos.x, robotPos.y, robotPos.z);
	printf("startPose: %lf %lf %lf \n",startPose.pose.position.x
			, startPose.pose.position.y, startPose.pose.position.z);
	printf("otherFootPose: %lf %lf %lf \n",otherFootPose.pose.position.x
			, otherFootPose.pose.position.y, otherFootPose.pose.position.z);

	
	for(int k=0; k<STEPS; k++)
	{
		while(0.1>calcDistance(startPose.pose.position.x,
				startPose.pose.position.y,
				points[curPoint].x,points[curPoint].y))
		{
			curPoint++;
		}
		for (int i=0; i<SIZE; i++)
		{
			for (int j=0; j<SIZE; j++)
			{
				if( !this->possible(i,j))
				{
					continue ;
				}

				C22_CompactGroundRecognitionAndMapping::C22_PLANE_TYPE plane=
						avgMap[i][j];


				geometry_msgs::Point squarePoint = calcPoint(i,j,plane,
						robotPos,robotOri);

				double slope=calcSlope(plane.x,plane.y,plane.z);
				double direction=calcAngle(points[curPoint].x-robotPos.x,
						points[curPoint].y-robotPos.y,
						squarePoint.x-robotPos.x,squarePoint.y-robotPos.y);

				double distance=calcDistance(startPose.pose.position.x,
						startPose.pose.position.y,
						squarePoint.x,squarePoint.y);

				double height=fabs(startPose.pose.position.z-plane.repPoint.z);

				double cost=singleCellWeight(slope,distance,height,direction,
						slopeWeight,distanceWeight,heightWeight,directionWeight);


				if(minCost>cost)
				{

					minCost=cost;
					minCostPoint=squarePoint;

				}

			} //j
		} //i

		double stepDistance=calcDistance(startPose.pose.position.x,
								startPose.pose.position.y,
								minCostPoint.x,minCostPoint.y);
		double targetDistance=calcDistance(points[curPoint].x,
				points[curPoint].y,
				minCostPoint.x,minCostPoint.y);

		double angle=0;
		if(points.size()>curPoint+1)
		{
			//direction to new point
			angle=atan2(points[curPoint].y-points[curPoint+1].y,
				points[curPoint].x-points[curPoint+1].x);
			//in relation to current direction
			angle-=startPose.pose.ang_euler.z;
			//normalize
			if(angle> PI)
			{
				angle-=PI;
			}
			else if(angle<-PI)
			{
				angle+=PI;
			}
			//move the proportional part of distance from the angle
			angle*=stepDistance/targetDistance;
			if(fabs(angle)>=MAX_ORIENTATION_CHANGE)
			{
				angle*= MAX_ORIENTATION_CHANGE /angle;
			}
		}



		FootPlacement::Foot_Placement_data data;
		data.foot_index = startPose.foot_index;
		data.pose.position.x=minCostPoint.x-robotPos.x;
		data.pose.position.y=minCostPoint.y-robotPos.y;
		data.pose.position.z=minCostPoint.z-robotPos.z;
		data.pose.ang_euler.x = 0.0;
		data.pose.ang_euler.y = 0.0;

		data.pose.ang_euler.z = angle;

		data.clearance_height = 0.0;
		foot_placement_path.push_back(data);


		startPose=otherFootPose;
		otherFootPose=data;

	} //k

} //function



