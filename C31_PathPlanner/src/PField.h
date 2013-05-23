#ifndef ___COGNI_POTENTIAL_FIELD_H___
#define ___COGNI_POTENTIAL_FIELD_H___

#include "Vec2d.hpp"
#include "cogniteam_pathplanning.h"
using namespace std;


class PField{
//------------------ types
public:
	typedef ObsMap Map;
	typedef vector<Vec2d> Points;
	
	enum RepulsorType{RT_ERROR, RT_R1};
	enum AttractorType{AT_ERROR, AT_A1};

	struct SmoothingParameters{
		//define ellipse of attractors/repulsors search local window
		double viewRadiusForward;
		double viewRadiusSide;
		//max iteration numbers for prevent endless search of simulation
		double maxIterationNumber;
		//rate of simulation step
		double stepRate;
		//virtual attractor power for push smoothing alg. forward
		double inertia;
		//distance between points in resulted path
		double distanceBetweenPoints;
		double maxAngleWhileReducing; // not used in RDP_MODE3
		//type of repulsors and attractors potential calculation
		RepulsorType repulsorType;
		AttractorType attractorType;
		
		SmoothingParameters():
			viewRadiusForward(0),viewRadiusSide(0),
			maxIterationNumber(0),stepRate(0),inertia(0),
			distanceBetweenPoints(0), maxAngleWhileReducing(0),
			repulsorType(RT_ERROR),attractorType(AT_ERROR)
		{}
		bool notDefined()const{
			if(
				viewRadiusForward==0 || viewRadiusSide==0 ||
				maxIterationNumber<0 || stepRate == 0 ||
				distanceBetweenPoints==0 || maxAngleWhileReducing==0 ||
				repulsorType==RT_ERROR || attractorType==AT_ERROR
			){
				cout<<"SmoothingParameters::notDefined=true: "
					<<(viewRadiusForward==0)<<(viewRadiusSide==0)
					<<(maxIterationNumber==0)<<(stepRate==0)
					<<(distanceBetweenPoints==0)<<(maxAngleWhileReducing==0)
					<<(repulsorType==RT_ERROR)<<(attractorType==AT_ERROR)
				<<endl;
				return true;
			}
			return false;
		}
	};
	
//------------------ members
private:

	const Map&  map;
	const Path& opath;

//------------------ methods
public:
	PField(const Map& map, const Path& path):
	map(map), opath(path)
	{
		
	}
	
	Points smooth(const SmoothingParameters& params)const;
	Path smoothWaypoints(const SmoothingParameters& params)const;
	Path castPath(const Points& points)const;

private:
public://<- for testing only
	Points simulate(const SmoothingParameters& params) const;
	Points reducePath(const Points& path, const SmoothingParameters& params) const;
	Points addPointsToPath(const Points& path, const SmoothingParameters& params) const;
	Points addPointsToPath(const Points& path, double distBtwPoints) const;
};

#endif

