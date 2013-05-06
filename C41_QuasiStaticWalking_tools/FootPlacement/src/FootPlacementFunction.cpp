#include "FootPlacementService.h"

int FootPlacementService::possible(int i, int j)
{
	if(i==SIZE/2 && j==SIZE/2)
		return 0;
	return 1;
}

geometry_msgs::Point FootPlacementService::calcPoint(const int &i, const int &j,
		const C22_CompactGroundRecognitionAndMapping::C22_PLANE_TYPE &plane,
		const geometry_msgs::Point &robotPos, const geometry_msgs::Point &robotOri)
{
	geometry_msgs::Point point,xStep,yStep;
	xStep.z=yStep.z=0;

	yStep.x = SQUARE_SIZE* cos( robotOri.z);
	yStep.y = SQUARE_SIZE* sin( robotOri.z);


	/*
	yStep.y=1*SQUARE_SIZE;
	yStep.x=0;
	*/

	//xstep is 90deg clockwise from ystep
	xStep.x= yStep.y;
	xStep.y = -yStep.x;

	point.x = robotPos.x+ (j-SIZE/2)*xStep.x+(SIZE/2-i)*yStep.x;
	point.y=robotPos.y+(j-SIZE/2)*xStep.y+(SIZE/2-i)*yStep.y;
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
			direction*directionWeight;
}

void FootPlacementService::calcFootMatrix(
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
		const double &directionWeight)
{
	const geometry_msgs::Point &robotPos = path.robotPos;
	const geometry_msgs::Point &robotOri = path.robotOri;

	const geometry_msgs::Point robotLegPos = (LEFT==leg)? robotLeftLegPos:robotRightLegPos;

	for (int i=0; i<SIZE; i++)
	{
		for (int j=0; j<SIZE; j++)
		{
			if(!this->possible(i,j))
			{
				continue ;
			}
			else if(useC22)
			{
				int len=path.row[0].column[0].planes.size();
				double minCost=std::numeric_limits<double>::infinity();
				geometry_msgs::Point minCostPoint;
				C22_CompactGroundRecognitionAndMapping::C22_PLANE_TYPE minCostPlane;


				if(0==len)
				{
					continue ;
				}

				for(int k=0; k<len; k++)
				{
					C22_CompactGroundRecognitionAndMapping::C22_PLANE_TYPE plane=
							path.row[i].column[j].planes[k];

					geometry_msgs::Point repPoint = calcPoint(i,j,plane,
							robotPos,robotOri);

					double slope=calcSlope(plane.x,plane.y,plane.z);
					double direction=calcAngle(dirX-robotPos.x,dirY-robotPos.y
											,repPoint.x-robotPos.x,repPoint.y-robotPos.y);

					double distance=calcDistance(robotLegPos.x,robotLegPos.y,
														repPoint.x,repPoint.y);

					double height=fabs(robotLegPos.z-repPoint.z);

					double cost=singleCellWeight(slope,distance,height,direction,
							slopeWeight,distanceWeight,heightWeight,directionWeight);


					if(minCost>cost)
					{

						minCost=cost;
						minCostPlane=plane;
						minCostPoint=repPoint;

					}

				}

				minCost/=NORMALIZER;

				FootPlacement::Pos pos;
				pos.plane = minCostPlane;
				pos.point = minCostPoint;
				pos.cost = minCost;

				for(std::vector<FootPlacement::Pos>::iterator p=positions.begin(); p!=positions.end(); p++)
				{
					if(p->cost > minCost)
					{
						positions.insert(p,pos);
						break ;
					}
				}


				if(positions.empty())
				{
					positions.push_back(pos);
				}

			}

			else
			{

				C22_CompactGroundRecognitionAndMapping::C22_PLANE_TYPE plane;
				plane.x=0;
				plane.y=0;
				plane.z=1;
				plane.d=-robotLegPos.z;

				geometry_msgs::Point repPoint = calcPoint(i,j,plane,
						robotPos,robotOri);

				double slope=calcSlope(plane.x,plane.y,plane.z);
				double direction=calcAngle(dirX-robotPos.x,dirY-robotPos.y
										,repPoint.x-robotPos.x,repPoint.y-robotPos.y);

				double distance=calcDistance(robotLegPos.x,robotLegPos.y,
													repPoint.x,repPoint.y);

				double height=fabs(robotLegPos.z-repPoint.z);

				double cost=singleCellWeight(slope,distance,height,direction,
						slopeWeight,distanceWeight,heightWeight,directionWeight);




				cost/=NORMALIZER;

				FootPlacement::Pos pos;
				pos.plane = plane;
				pos.point.x = repPoint.x-robotPos.x;
				pos.point.y = repPoint.y-robotPos.y;
				pos.point.z = repPoint.z-robotPos.z;
				pos.cost = cost;

				for(std::vector<FootPlacement::Pos>::iterator p=positions.begin(); p!=positions.end(); p++)
				{
					if(p->cost > cost)
					{
						positions.insert(p,pos);
						break ;
					}
				}


				if(positions.empty())
				{
					positions.push_back(pos);
				}

			}
		}
	}

}
