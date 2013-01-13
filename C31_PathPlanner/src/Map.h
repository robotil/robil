/*
 * Map.h
 *
 *  Created on: Jan 6, 2013
 *      Author: dan
 */

#ifndef COGNITEAM_PATHPLANNING_MAP_H_
#define COGNITEAM_PATHPLANNING_MAP_H_

#include <iostream>
#include <vector>
#include <queue>
#include <set>
#include <map>
#include <string.h>
#include <sstream>
#include <math.h>
using namespace std;

//typedef vector<vector<unsigned int> > RAWMAP;


class Map{
public:
	enum STATUS{ST_AVAILABLE=0,ST_BLOCKED,ST_UNCHARTED};


	Map(int w, int h);
	Map(int w, int h, char* cmap);
	Map(const Map& map);

	class MapCreator{
	public:
		vector<char> data;
		size_t w, h;
		Map map()const{ Map m(w, h); m._data = data; return m; }
	};

	size_t index(int x, int y)const{ return y*w()+x; }
	char operator()(int x, int y)const{
		return getByIndex(index(x,y));
	}
	char& operator()(int x, int y){
		return getByIndex(index(x,y));
	}

	size_t w()const{ return _w; }
	size_t h()const{ return _h; }

	char str(int x, int y)const{
		char c = (*this)(x,y);
		if(c==ST_AVAILABLE) return '.';
		if(c==ST_BLOCKED) return 'B';
		if(c==ST_UNCHARTED) return '-';
		return '?';
	}

	bool inRange(long x, long y)const;
	void approximate(long& x, long& y)const;

private:
	void approximate(long& x, long& y, char ctype)const;

	char getByIndex(size_t ix)const{ return _data[ix]; }
	char& getByIndex(size_t ix){ return _data[ix]; }

	size_t _w, _h;
	vector<char> _data;
};

ostream& operator<<(ostream& out, const Map& m);


#endif /* COGNITEAM_PATHPLANNING_MAP_H_ */
