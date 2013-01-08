#ifndef _COGNITEAM_PATH_PLANNER_INFLATOR_H_
#define _COGNITEAM_PATH_PLANNER_INFLATOR_H_

#include "Map.h"
using namespace std;


class Inflator{
public:

	Inflator(size_t radius, char cbv);
	Map inflate(const Map& source)const;
	Map coloring(const Map& source, size_t x, size_t y, char av, char bl)const;

private:
	void coloring(const Map& source, size_t x, size_t y, char c, char av, char bl, Map& visited, Map& res)const;

private:
	size_t radius;
	char cellBlockValue;
	vector<size_t> tx;
	vector<size_t> ty;

};

#endif //_COGNITEAM_PATH_PLANNER_INFLATOR_H_
