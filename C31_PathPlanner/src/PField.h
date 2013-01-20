#ifndef ___COGNI_POTENTIAL_FIELD_H___
#define ___COGNI_POTENTIAL_FIELD_H___

#include "Vec2d.hpp"
#include "cogniteam_pathplanning.h"
using namespace std;

class PField{
//------------------ types
public:
	enum RepulsorType{RT_R1};
	enum AttractorType{AT_A1};

	struct SmoothingParameters{
		double viewRadiusForward;
		double viewRadiusSide;
		double maxIterationNumber;
		double stepRate;
		double inertia;
		double distanceBetweenPoints;
		double maxAngleWhileReducing;
		RepulsorType repulsorType;
		AttractorType attractorType;
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
	
	Path smooth(const SmoothingParameters& params)const;

private:
	
	typedef vector<Vec2d> Points;
	Points simulate(const SmoothingParameters& params) const;
	Points reducePath(const Points& path, const SmoothingParameters& params) const;

};

#endif

