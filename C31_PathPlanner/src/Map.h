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


template <typename Item>
class MapT{
public:
	enum STATUS{ST_AVAILABLE=0,ST_BLOCKED,ST_UNCHARTED};


	MapT(int w, int h):map_id(map_id_counter++), _w(w),_h(h){
		_data.resize(_w*_h);
		for(size_t i=0;i<_data.size();i++) _data[i]=ST_UNCHARTED;
	}
	MapT(int w, int h, Item* cmap):map_id(map_id_counter++), _w(w),_h(h){
		_data.resize(_w*_h);
		for(size_t i=0;i<_data.size();i++) _data[i]=cmap[i];
	}
	MapT(const MapT<Item>& map):map_id(map_id_counter++), _w(map._w),_h(map._h){
		_data.resize(_w*_h);
		for(size_t i=0;i<_data.size();i++) _data[i]=map._data[i];
	}
	class MapCreator{
	public:
		vector<Item> data;
		size_t w, h;
		const MapT<Item>& map()const{
			MapT<Item> m(w, h);
			for(size_t i=0;i<m._data.size();i++){ m._data[i] = data[i]; }
			return m;
		}
	};

	size_t index(int x, int y)const{ return y*w()+x; }
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

	size_t w()const{ return _w; }
	size_t h()const{ return _h; }

	char str(int x, int y)const{
		char c = (*this)(x,y);
		if(c==ST_AVAILABLE) return '.';
		if(c==ST_BLOCKED) return 'B';
		if(c==ST_UNCHARTED) return '-';
		if(c>=32) return c;
		return '?';
	}

	bool inRange(long x, long y)const;
	void approximate(const long cx, const long cy, long& x, long& y)const;

	inline void clear(){
		_w=_h=0;
		_data = vector<Item>();
	}

private:
	double approximate(const long cx, const long cy, long& x, long& y, char ctype)const;

	Item getByIndex(size_t ix)const{ return _data[ix]; }
	Item& getByIndex(size_t ix){ return _data[ix]; }

	static int map_id_counter;
	int map_id;

	size_t _w, _h;
	vector<Item> _data;
};

#define EXTENDS(ITEM, ME)  	\
	typedef MapT<ITEM> Supper;\
public:\
	ME(int w, int h):Supper(w,h){	}\
	ME(int w, int h, ITEM* cmap):Supper(w, h, cmap){ }\
	ME(const ME& map):Supper(map){ }\
	ME(const Supper& map):Supper(map){ }


class Map:public MapT<char>{
//	typedef MapT<char> Supper;
//public:
//	Map(int w, int h):Supper(w,h){	}
//	Map(int w, int h, char* cmap):Supper(w, h, cmap){ }
//	Map(const Map& map):Supper(map){ }
//	Map(const Supper& map):Supper(map){ }
	EXTENDS(char, Map)

	enum STATUS{ST_AVAILABLE=0,ST_BLOCKED,ST_UNCHARTED};



	char str(int x, int y)const{
		char c = (*this)(x,y);
		if(c==ST_AVAILABLE) return '.';
		if(c==ST_BLOCKED) return 'B';
		if(c==ST_UNCHARTED) return '-';
		if(c>=32) return c;
		return '?';
	}

	bool inRange(long x, long y)const;
	void approximate(const long cx, const long cy, long& x, long& y)const;

private:
	double approximate(const long cx, const long cy, long& x, long& y, char ctype)const;

};
ostream& operator<<(ostream& out, const Map& m);

class AltMap:public MapT<double>{
//	typedef MapT<double> Supper;
//public:
//	AltMap(int w, int h):Supper(w,h){	}
//	AltMap(int w, int h, double* cmap):Supper(w, h, cmap){ }
//	AltMap(const AltMap& map):Supper(map){ }
//	AltMap(const Supper& map):Supper(map){ }
	EXTENDS(double, AltMap)

	bool inRange(long x, long y)const;

};
//ostream& operator<<(ostream& out, const Map& m);


class MapEditor{
public:

	MapEditor(){};
	Map coloring(const Map& source, size_t x, size_t y, char av, char bl)const;
	Map replace(const Map& source, const char from, const char to)const;

private:
	void coloring(const Map& source, size_t x, size_t y, char c, char av, char bl, Map& visited, Map& res)const;

};

class World{
public:
	Map grid;
	AltMap altitudes;
	World(const Map& m, const AltMap& a):grid(m), altitudes(a){}
	World(const World& w):grid(w.grid), altitudes(w.altitudes){}
	const World& operator=(const World& w){grid=(w.grid); altitudes=(w.altitudes); return *this;}
};


#endif /* COGNITEAM_PATHPLANNING_MAP_H_ */
