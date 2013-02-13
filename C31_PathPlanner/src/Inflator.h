#ifndef _COGNITEAM_PATH_PLANNER_INFLATOR_H_
#define _COGNITEAM_PATH_PLANNER_INFLATOR_H_

#include "Map.h"
using namespace std;


class Inflator{
public:

	Inflator(size_t radius, char cbv);
	Map inflate(const Map& source)const;

private:
	size_t radius;
	char cellBlockValue;
	vector<int> tx;
	vector<int> ty;

};

#endif //_COGNITEAM_PATH_PLANNER_INFLATOR_H_
