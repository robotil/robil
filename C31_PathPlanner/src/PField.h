#ifndef ___COGNI_POTENTIAL_FIELD_H___
#define ___COGNI_POTENTIAL_FIELD_H___

#include "Vec2d.hpp"
#include "cogniteam_pathplanning.h"

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

};

#endif

