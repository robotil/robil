#include "FootPlacementService.h"

int FootPlacementService::possible(int i, int j)
{
	if(0==i && SIZE/2==j) // Locationh of robot is impossible to step on.
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
			//printf("%d %d\n",i,j);
			C22_CompactGroundRecognitionAndMapping::C22_PLANE_TYPE plane;
			plane.x=plane.y=plane.z=plane.d=0;
			plane.repPoint.x=plane.repPoint.y=plane.repPoint.z=0;
			int size=0;
			for(int k=0; k<5; k++)
			{
				for(int l=0; l<5; l++)
				{
					if(2==path.row[i+k].column[j+l].status)
					{
						if(k>=1 && k<=3 && l>=1 && l<=3)
						{
							size=-100;
							break;
						}
						else
						{
							continue;
						}

					}
					for(unsigned m=0; m<path.row[i].column[j].planes.size(); m++)
					{

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
			if(size>20)
			{
				plane.x/=size;
				plane.y/=size;
				plane.z/=size;
				plane.d/=size;
				plane.repPoint.x/=size;
				plane.repPoint.y/=size;
				plane.repPoint.z/=size;
				avgMap[i][j]=plane;
			}
			else
			{
				plane.x=0;
				plane.y=0;
				plane.z=0;
				plane.d=0;
				plane.repPoint.x=1000;
				plane.repPoint.y=1000;
				plane.repPoint.z=1000;
				avgMap[i][j]=plane;
			}


		}
	}


	/*
	for (int i=0; i<20; i++)
		{
			for(int j=16; j<20; j++)
			{
				printf("%lf,%lf %lf \n",avgMap[i][j].x,avgMap[i][j].y,avgMap[i][j].z);
			}

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
	double slope=fabs(asin(c/sqrt(a*a+b*b+c*c)));
	if(1000==a)
		return 1000;
// 	if(slope<EXTREME_SLOPE)
// 		publishExtremeSlopeMsg();
	return slope;
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
double FootPlacementService::singleCellWeight(const double &legDistance,
		const double &slope,const double &distance,
		const double &height,const double &direction,const double &slopeWeight,
		const double &distanceWeight,const double &heightWeight,
		const double &directionWeight)
{
    if(legDistance<MIN_STEP_SIZE)
		return 100;
    if(legDistance>MAX_STEP_SIZE)
		return 1000;
    if(height>MAX_STEP_HEIGHT)
		return 10000;
	return (/*slope*slopeWeight+*/ distance*distanceWeight+height*heightWeight+
			direction*directionWeight)/NORMALIZER;
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
	double minCost;
	geometry_msgs::Point minCostPoint;

	double a=robotOri.z;
	double cosa=cos(a), sina=sin(a);

	printf("a cos sin: %lf %lf %lf \n",a, cosa, sina);

	printf("pos: %lf %lf %lf \n",robotPos.x, robotPos.y, robotPos.z);
	printf("startPose: %lf %lf %lf \n",startPose.pose.position.x
			, startPose.pose.position.y, startPose.pose.position.z);
	printf("otherFootPose: %lf %lf %lf \n",otherFootPose.pose.position.x
			, otherFootPose.pose.position.y, otherFootPose.pose.position.z);

	
	for(int k=0; k<STEPS; k++)
	{
		while(MIN_DIST_FROM_TARGET>calcDistance(startPose.pose.position.x,
				startPose.pose.position.y,
				points[curPoint].x,points[curPoint].y) && curPoint<points.size())
		//FIXME: If we run out of points, and the last point is close to startPose, we will access points[] above range!
		{
			curPoint++;
		}

		if(curPoint==points.size())
		{
			printf("Need more points\n");
			break;
		}
		else
		{
			printf("step %d point: %lf %lf\n",k,points[curPoint].x,points[curPoint].y);
		}
		minCost = std::numeric_limits<double>::infinity();
		for (int i=0; i<SIZE; i++)
		{
			for (int j=0; j<SIZE; j++)
			{
				if( !this->possible(i,j))
				{
					//FIXME: If this become true for all, for some reason, will generate same minCost point as previous step!
					continue ;
				}

				C22_CompactGroundRecognitionAndMapping::C22_PLANE_TYPE plane=
						avgMap[i][j];

				if(plane.x==plane.y && plane.y==plane.z && plane.z==plane.d
						&& 0==plane.d)
				{
					continue;
				}


				//geometry_msgs::Point squarePoint = calcPoint(i,j,plane,
				//		robotPos,robotOri);


				geometry_msgs::Point squarePoint = plane.repPoint;

				squarePoint.x=robotPos.x+plane.repPoint.x*cosa+plane.repPoint.y*(-sina);
				squarePoint.y=robotPos.y+plane.repPoint.x*(sina)+plane.repPoint.y*cosa;




				//printf("%d %d %lf %lf %lf %lf %lf", i, j, squarePoint.x, squarePoint.y, squarePoint.z,
				//		plane.repPoint.x, plane.repPoint.y);
				
				double slope=calcSlope(plane.x,plane.y,plane.z);
				
				if( fabs(20.0*0.05-j*0.05) < EXTREME_SLOPE_COR )
					if(slope>EXTREME_SLOPE)
						publishExtremeSlopeMsg();
				
				double direction=calcAngle(points[curPoint].x,
						points[curPoint].y,
						squarePoint.x,squarePoint.y);

				double distance1=calcDistance(startPose.pose.position.x,
						startPose.pose.position.y,
						squarePoint.x,squarePoint.y);

				double distance2=calcDistance(points[curPoint].x,
						points[curPoint].y,
						squarePoint.x,squarePoint.y);

				double height=fabs(robotPos.z+plane.repPoint.z);

				double legDist= calcDistance(otherFootPose.pose.position.x,
								otherFootPose.pose.position.y,
										squarePoint.x,squarePoint.y);

				double cost=singleCellWeight(legDist,slope,distance2,height,direction,
						slopeWeight,distanceWeight,heightWeight,directionWeight);

				//printf("%d %d %lf %lf %lf %lf\n", i, j,legDist, distance1, distance2, cost);

				if(minCost>cost)
				{
                    //printf("Changing minCost point for step %d at point %d, %d\n", k,i,j);
					//printf("%d %d %lf %lf %lf %lf\n", i, j,legDist, distance1, distance2, cost);

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


		// FIXME:  This is wrong, because it never exchanges foot_index and possibly other information about the startpos. It only changes point in space.






		FootPlacement::Foot_Placement_data data;
		data.foot_index = startPose.foot_index; 
		data.pose.position.x=minCostPoint.x;
		data.pose.position.y=minCostPoint.y;
		data.pose.position.z=minCostPoint.z+robotPos.z;
		data.pose.ang_euler.x = 0.0;
		data.pose.ang_euler.y = 0.0;

		data.pose.ang_euler.z = angle;

		data.clearance_height = 0.0;
		foot_placement_path.push_back(data);

		/*
		printf("data1 step %d point: %lf %lf %lf\n",k,data.pose.position.x,
								data.pose.position.y,data.pose.position.z);
		printf("start1 step %d point: %lf %lf %lf\n",k,startPose.pose.position.x,
						startPose.pose.position.y,startPose.pose.position.z);

		printf("other1 step %d point: %lf %lf %lf\n",k,otherFootPose.pose.position.x,
						otherFootPose.pose.position.y,otherFootPose.pose.position.z);
		*/
		startPose.foot_index = otherFootPose.foot_index;
		startPose.pose.position.x=otherFootPose.pose.position.x;
		startPose.pose.position.y=otherFootPose.pose.position.y;
		startPose.pose.position.z=otherFootPose.pose.position.z;
		//startPose.pose.position.x=otherFootPose.pose.ang_euler.x;
		//startPose.pose.position.y=otherFootPose.pose.ang_euler.y;
		//startPose.pose.position.z=otherFootPose.pose.ang_euler.z;
		//startPose.clearance_height = otherFootPose.clearance_height;


		otherFootPose.foot_index = data.foot_index;
		otherFootPose.pose.position.x=data.pose.position.x;
		otherFootPose.pose.position.y=data.pose.position.y;
		otherFootPose.pose.position.z=data.pose.position.z;
		//otherFootPose.pose.position.x=data.pose.ang_euler.x;
		//otherFootPose.pose.position.y=data.pose.ang_euler.y;
		//otherFootPose.pose.position.z=data.pose.ang_euler.z;
		//otherFootPose.clearance_height = data.clearance_height;

		//copyFoot(otherFootPose,data);

		/*
		printf("data2 step %d point: %lf %lf %lf\n",k,data.pose.position.x,
										data.pose.position.y,data.pose.position.z);

		printf("start2 step %d point: %lf %lf %lf\n",k,startPose.pose.position.x,
				startPose.pose.position.y,startPose.pose.position.z);

		printf("other2 step %d point: %lf %lf %lf\n",k,otherFootPose.pose.position.x,
				otherFootPose.pose.position.y,otherFootPose.pose.position.z);
		*/
	} //k (step counter)

} //function



void FootPlacementService::copyFoot(FootPlacement::Foot_Placement_data &a,
		FootPlacement::Foot_Placement_data &b)
{
	a.foot_index = b.foot_index;
	a.pose.position.x=b.pose.position.x;
	a.pose.position.y=b.pose.position.y;
	a.pose.position.z=b.pose.position.z;
	a.pose.position.x=b.pose.ang_euler.x;
	a.pose.position.y=b.pose.ang_euler.y;
	a.pose.position.z=b.pose.ang_euler.z;
	a.clearance_height = b.clearance_height;
}













