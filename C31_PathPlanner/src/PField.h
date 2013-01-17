#ifndef ___COGNI_POTENTIAL_FIELD_H___
#define ___COGNI_POTENTIAL_FIELD_H___

#include "Vec2d.hpp"
#include "cogniteam_pathplanning.h"
using namespace std;

class PField{
//------------------ types
public:
	
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
	
	Path smooth()const;

private:
	
	typedef vector<Vec2d> Points;
	enum RepulsorType{RT_R1};
	enum AttractorType{AT_A1};
	Points simulate(double step, double viewRF, double viewRS, RepulsorType rt, AttractorType at) const;

};

#endif

