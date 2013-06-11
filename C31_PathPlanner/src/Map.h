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

#define MM_GRID 0
#define MM_ALTS 1
#define MAP_MODE MM_ALTS


class BaseMap{
protected:
	static int map_id_counter;
	int map_id;
	BaseMap(int w, int h):map_id(map_id_counter++), _w(w),_h(h){}
	BaseMap(const BaseMap& map):map_id(map_id_counter++), _w(map._w),_h(map._h){}

	size_t _w, _h;

public:

	size_t w()const{ return _w; }
	size_t h()const{ return _h; }
	size_t index(int x, int y)const{ return y*w()+x; }

	bool inRange(long x, long y)const;

};

template <typename Item>
class MapT:public BaseMap{
public:


	MapT(int w, int h):BaseMap(w,h){
		_data.resize(_w*_h);
		for(size_t i=0;i<_data.size();i++) _data[i]=0;
	}
	MapT(int w, int h, Item* cmap):BaseMap(w,h){
		_data.resize(_w*_h);
		for(size_t i=0;i<_data.size();i++) _data[i]=cmap[i];
	}
	MapT(const MapT<Item>& map):BaseMap(map){
		_data.resize(_w*_h);
		for(size_t i=0;i<_data.size();i++) _data[i]=map._data[i];
	}
	class MapCreator{
	public:
		vector<Item> data;
		size_t w, h;
		MapT<Item> map()const{
			MapT<Item> m(w, h);
			for(size_t i=0;i<m._data.size();i++){ m._data[i] = data[i]; }
			return m;
		}
	};


	Item operator()(int x, int y)const{
		return getByIndex(index(x,y));
	}
	Item& operator()(int x, int y){
		return getByIndex(index(x,y));
	}

	const MapT<Item>& operator=(const MapT<Item>& other){
		_w = other._w; _h = other._h;
		if(_data.size()!=other._data.size()){
			_data.resize(_w*_h);
		}
		for(size_t i=0;i<_data.size();i++) _data[i]=other._data[i];
		return *this;
	}

//	char str(int x, int y)const{
//		char c = (*this)(x,y);
//		if(c==ST_AVAILABLE) return '.';
//		if(c==ST_BLOCKED) return 'B';
//		if(c==ST_UNCHARTED) return '-';
//		if(c>=32) return c;
//		return '?';
//	}

//	bool inRange(long x, long y)const;
//	void approximate(const long cx, const long cy, long& x, long& y)const;

	inline void clear(){
		_w=_h=0;
		_data = vector<Item>();
	}

private:
//	double approximate(const long cx, const long cy, long& x, long& y, char ctype)const;

	Item getByIndex(size_t ix)const{ return _data[ix]; }
	Item& getByIndex(size_t ix){ return _data[ix]; }


	vector<Item> _data;
};

#define EXTENDS(ITEM, ME)  	\
	typedef MapT<ITEM> Supper;\
public:\
	ME(int w, int h):Supper(w,h){	}\
	ME(int w, int h, ITEM* cmap):Supper(w, h, cmap){ }\
	ME(const ME& map):Supper(map){ }\
	ME(const Supper& map):Supper(map){ }


class ObsMap:public MapT<char>{

	EXTENDS(char, ObsMap)

	enum STATUS{
		ST_AVAILABLE=0,
		ST_BLOCKED=1,
		ST_UNCHARTED=2,
		ST_ATLAS=3,
		ST_MUD=4,
		ST_DEBRIS=5,
	};



	char str(int x, int y)const{
		char c = (*this)(x,y);
		if(c==ST_AVAILABLE) return '.';
		if(c==ST_BLOCKED) return 'B';
		if(c==ST_UNCHARTED) return '-';
		if(c==ST_ATLAS) return 'A';
		if(c==ST_MUD) return 'M';
		if(c==ST_DEBRIS) return 'D';
		if(c>=32) return c;
		return '?';
	}

	static STATUS filterObstacles(int value){
		switch((STATUS)value){
		case ST_AVAILABLE: return ST_AVAILABLE;
		case ST_UNCHARTED: return ST_UNCHARTED;
		case ST_BLOCKED: return ST_BLOCKED;
		case ST_ATLAS: return ST_AVAILABLE;
		case ST_MUD: return ST_AVAILABLE;
		case ST_DEBRIS: return ST_AVAILABLE;
		}
	}
	static STATUS filterTerrain(int value){
		switch((STATUS)value){
		case ST_AVAILABLE: return ST_UNCHARTED;
		case ST_UNCHARTED: return ST_UNCHARTED;
		case ST_BLOCKED: return ST_UNCHARTED;
		case ST_ATLAS: return ST_ATLAS;
		case ST_MUD: return ST_MUD;
		case ST_DEBRIS: return ST_DEBRIS;
		}
	}

	void approximate(const long cx, const long cy, long& x, long& y)const;

protected:
	double approximate(const long cx, const long cy, long& x, long& y, char ctype)const;

};
ostream& operator<<(ostream& out, const ObsMap& m);

class AltMap:public MapT<double>{
	EXTENDS(double, AltMap)

};
ostream& operator<<(ostream& out, const AltMap& m);


class MapEditor{
public:

	MapEditor(){};
	ObsMap coloring(const ObsMap& source, size_t x, size_t y, char av, char bl)const;
	ObsMap replace(const ObsMap& source, const char from, const char to)const;
	ObsMap cut(const ObsMap& source, const char main_color, const char other_color)const;

	enum MergeOperator{
		OR, AND, XOR, SUB
	};
	ObsMap merge(const ObsMap& m1, const ObsMap& m2, MergeOperator op)const;

private:
	void coloring(const ObsMap& source, size_t x, size_t y, char c, char av, char bl, ObsMap& visited, ObsMap& res)const;

};

class World{
public:
	ObsMap grid;
	ObsMap terrain;
	AltMap altitudes;
	ObsMap walls;
	AltMap slops;
	AltMap costs;
	World():grid(0,0),terrain(0,0),altitudes(0,0),walls(0,0),slops(0,0),costs(0,0){}
	void update(const ObsMap& grid, const AltMap& alts, const ObsMap& terrain);
#if MAP_MODE == MM_ALTS
	size_t w()const{ return altitudes.w(); }
	size_t h()const{ return altitudes.h(); }
	bool inRange(long x, long y)const{ return altitudes.inRange(x,y); }
	void approximate(const long cx, const long cy, long& x, long& y)const{ walls.approximate(cx, cy, x, y); }
#else
	size_t w()const{ return grid.w(); }
	size_t h()const{ return grid.h(); }
	bool inRange(long x, long y)const{ return grid.inRange(x,y); }
	void approximate(const long cx, const long cy, long& x, long& y)const{ grid.approximate(cx, cy, x, y); }
#endif
};


#endif /* COGNITEAM_PATHPLANNING_MAP_H_ */
