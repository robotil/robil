/*
 * Map.cpp
 *
 *  Created on: May 22, 2013
 *      Author: dan
 */

#include "Map.h"
typedef ObsMap Map;
// -------------------------- MAP ---------------------------------------------


ostream& operator<<(ostream& out, const Map& m){
	out<<"  "; for(size_t x=0;x<10;x++){cout<<' '<<x<<' ';}for(size_t x=10;x<m.w();x++){cout<<x<<' ';} out<<endl;
	for(long y=(long)m.h()-1;y>=0;y--){
//	for(size_t y=0;y<m.h();y++){
		if(y<10) out<<' '; out<<y;
		for(size_t x=0;x<m.w();x++){
			out<<' '<<m.str(x,y)<<' ';
		}
		out<<endl;
	}
	return out;
}

bool Map::inRange(long x, long y)const{
	if(x<0||y<0) return false;
	if(x>=(long)w()||y>=(long)h()) return false;
	return true;
}
bool AltMap::inRange(long x, long y)const{
	if(x<0||y<0) return false;
	if(x>=(long)w()||y>=(long)h()) return false;
	return true;
}
double Map::approximate(const long cx, const long cy, long& tx, long& ty, char ctype)const{
// 	cout<<"tx="<<tx<<", ty="<<ty<<", ctype="<<ctype<<", cx="<<cx<<", cy="<<cy<<endl;
	if(inRange(tx, ty)) return 0;
	long x = 0;long y = 0;
	#define DIST(x,y, xx, yy) ::hypot(double((xx-cx)-(x-cx)), double((yy-cy)-(y-cy)))

	double min_dis = DIST(tx,ty,  x,y);
	long minX(-1), minY(-1);
	const Map& me = *this;

//	#define proc if(me(x,y) == ctype){ double dis = DIST(tx,ty,  x,y); cout<<"x,y="<<x<<","<<y<<"="<<dis; if(min_dis>dis || minX<0){ min_dis=dis; minX=x; minY=y; cout<<" set as min: "<<minX<<","<<minY<<"="<<min_dis;} cout<<endl;}
	#define proc if(me(x,y) == ctype){ double dis = DIST(tx,ty,  x,y); if(min_dis>dis || minX<0){ min_dis=dis; minX=x; minY=y;} }

	for(; x< (long)w()	; x++){ proc }	x--; y++;
	for(; y< (long)h()	; y++){ proc }	x--; y--;
	for(; x>=0  		; x--){ proc }	x++; y++;
	for(; y>=0  		; y--){ proc }

	#undef proc
	#undef DIST
	tx = minX; ty=minY;
// 	cout<<"----"<<endl;
	return min_dis;
}
void Map::approximate(const long cx, const long cy, long& tx, long& ty)const{
	long x(tx), y(ty);
	if(inRange(tx, ty)){ /*cout<<"original x,y in range"<<endl;*/ return; }
	double dis_av = approximate(cx, cy, tx, ty, Map::ST_AVAILABLE);
	long x_av(tx), y_av(ty);

	//if(inRange(tx, ty)){ cout<<"x,y on available celles in range : "<<tx<<","<<ty<<endl; return; }
	tx=x; ty=y;
	double dis_un = approximate(cx, cy, tx, ty, Map::ST_UNCHARTED);
	long x_un(tx), y_un(ty);

	if(dis_av<=dis_un){ tx=x_av; ty=y_av; } else { tx=x_un; ty=y_un; }

	if(inRange(tx, ty)){ /*cout<<"x,y on uncharted celles in range"<<endl;*/ return; }
	approximate(cx, cy, tx, ty, Map::ST_BLOCKED);
	//cout<<"x,y on blocked celles"<<endl;
}

