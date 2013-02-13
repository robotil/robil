//============================================================================
// Name        : PP.cpp
// Author      : danerde@gmail.com
// Version     :
// Copyright   : Cogniteam
// Description : Hello World in C++, Ansi-style
//============================================================================

#include "Map.h"
#include "QTNode.h"
#include "AStar.h"
#include "Inflator.h"
#include "math.h"
#include "Vec2d.hpp"
#include "PField.h"

#include "cogniteam_pathplanning.h"

#define PRINT_AS_VECTORS 1

using namespace std;

// -------------------------- MAP ---------------------------------------------

Map::Map(int w, int h):_w(w),_h(h){
	_data.resize(_w*_h);
	for(size_t i=0;i<_data.size();i++) _data[i]=ST_UNCHARTED;
}
Map::Map(int w, int h, char* cmap):_w(w),_h(h){
	_data.resize(_w*_h);
	for(size_t i=0;i<_data.size();i++) _data[i]=cmap[i];
}
Map::Map(const Map& map):_w(map._w),_h(map._h){
	_data.resize(_w*_h);
	for(size_t i=0;i<_data.size();i++) _data[i]=map._data[i];
}

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
// -------------------------- QTNode ---------------------------------------------

QTNode::QTNode(size_t x1, size_t x2, size_t y1, size_t y2, const Map& map, QTNode* supper):
	_x1(x1),_x2(x2),_y1(y1),_y2(y2),_supper(supper),_map(map){

	state = _x1==_x2 && _y1==_y2 ? QTNode::st_sing : QTNode::st_mult;

	::memset(subs, 0, 4*sizeof(QTNode*));
	if(state!=st_sing){
		size_t x11=x1, xn=(x2-x1)/2, x12=x1+xn, x21=x12+1, x22=x2;
		size_t y11=y1, yn=(y2-y1)/2, y12=y1+yn, y21=y12+1, y22=y2;

		#define CNODE(x11, x12, y11, y12, N) if(x11<=x12 && y11<=y12) subs[N]=new QTNode(x11, x12, y11, y12, map, this);
		CNODE(x11, x12, y11, y12, 0)
		CNODE(x21, x22, y11, y12, 1)
		CNODE(x11, x12, y21, y22, 2)
		CNODE(x21, x22, y21, y22, 3)
		#undef CNODE
	}
}
QTNode::~QTNode(){
	for(size_t i=0;i<4;i++){
		if(subs[i]) delete subs[i]; subs[i]=0;
	}
}
#define FOR_ALL for(size_t i=0;i<4;i++)if(subs[i])

const QTNode* QTNode::findEmpty(size_t x, size_t y)const{
	if(!inRange(x,y)) return NULL;
	if(state == st_bl) return NULL;
	if(state == st_av) return this;
	if(state == st_sing){
		if(isEmpty()) return this;
		return NULL;
	}
	FOR_ALL{
		const QTNode* res = subs[i]->findEmpty(x,y);
		if(res) return res;
	}
	return NULL;
}

bool QTNode::isAvailable(size_t x, size_t y)const{
	if(!inRange(x,y)) return true;
	if(isOneCell()){
		return _map(_x1,_y1) == Map::ST_AVAILABLE;
	}
	if(state == st_bl) return false;
	if(state == st_av) return true;
	FOR_ALL{
		if(subs[i]->inRange(x,y))
			return subs[i]->isAvailable(x,y);
	}
	return true;
}

int QTNode::folding(){
	if(isOneCell()){
		return _map(_x1,_y1) == Map::ST_AVAILABLE ? 1 : -1;
	}else{
		int ca=0, cb=0;
		FOR_ALL{
			int res = subs[i]->folding();
			if( res==1 ){
				ca++;
			}else if(res == -1){
				cb++;
			}
		}else{ca++;cb++;}
		if(ca==4 || cb==4){
			FOR_ALL{delete subs[i]; subs[i]=0;}
			if(ca==4){
				state = st_av;
				return 1;
			}
			state = st_bl;
			return -1;
		}
		return 0;
	}
}

vector<QTNode::XY> QTNode::getBorder()const{
	vector<XY> res;
	#define add(x,y) if(x>=0 && y>=0 && x<(long)_map.w() && y<(long)_map.h()){XY xy={x,y};res.push_back(xy);}
	long x = _x1-1;long y = _y1-1;
	for(; x<=(long)_x2; x++) add(x,y)
	for(; y<=(long)_y2; y++) add(x,y)
	for(; x>=(long)  _x1; x--) add(x,y)
	for(; y>=(long)  _y1; y--) add(x,y)
	return res;
	#undef add
}
set<const QTNode*> QTNode::getNeighbors()const{
	if(state == st_bl) return set<const QTNode*>();
	if(state == st_mult){
		set<const QTNode*> s;
		#define UNION(A, B) {set<const QTNode*> b = B; for(set<const QTNode*>::const_iterator n=b.begin();n!=b.end();n++) A.insert(*n); }
		FOR_ALL{ UNION(s, subs[i]->getNeighbors()); }
		#undef UNION
		return s;
	}
	vector<XY> border = getBorder();
	set<const QTNode*> s;
	for(size_t i=0;i<border.size();i++){
		const QTNode* e = getRoot()->findEmpty(border[i].x, border[i].y);
		if(e) s.insert(e);
	}
	return s;
}
size_t QTNode::linesIntersectCenter(size_t a1, size_t a2, size_t b1, size_t b2)const{
	if(a1<=b1 && b2<=a2) return (b2+b1)/2;
	if(b1<=a1 && a2<=b2) return (a2+a1)/2;
	if(b2<a1 || a2<b1) return -1;
	if(a1<=b1 && a2<=b2) return (a2+b1)/2;
	if(b1<=a1 && b2<=a2) return (b2+a1)/2;
	return -1;
}
vector<QTNode::XY> QTNode::corridorX(size_t x11, size_t x12, size_t x21, size_t x22, size_t y)const{
	vector<XY> res;
	if(x11>x22){ XY p1={x11,y}, p2={x22,y}; res.push_back(p1); res.push_back(p2); }
	if(x21>x12){ XY p1={x12,y}, p2={x21,y}; res.push_back(p1); res.push_back(p2); }
	return res;
}
vector<QTNode::XY> QTNode::corridorY(size_t y11, size_t y12, size_t y21, size_t y22, size_t x)const{
	vector<XY> res;
	if(y11>y22){ XY p1={x,y11}, p2={x,y22}; res.push_back(p1); res.push_back(p2); }
	if(y21>y12){ XY p1={x,y12}, p2={x,y21}; res.push_back(p1); res.push_back(p2); }
	return res;
}
vector<QTNode::XY> QTNode::corridorDiagonal(size_t x11, size_t x12, size_t x21, size_t x22,   size_t y11, size_t y12, size_t y21, size_t y22)const{
	vector<XY> res;
	if(y11>y22){
		if(x11>x22){ XY p1={x11,y11}, p2={x22,y22}; res.push_back(p1); res.push_back(p2); }
		if(x21>x12){ XY p1={x12,y11}, p2={x21,y22}; res.push_back(p1); res.push_back(p2); }
	}
	if(y21>y12){
		if(x11>x22){ XY p1={x11,y12}, p2={x22,y21}; res.push_back(p1); res.push_back(p2); }
		if(x21>x12){ XY p1={x12,y12}, p2={x21,y21}; res.push_back(p1); res.push_back(p2); }
	}
	return res;
}

vector<QTNode::XY> QTNode::getCorridor(const QTNode* target)const{
	int y = linesIntersectCenter(_y1, _y2, target->_y1, target->_y2);
	int x = linesIntersectCenter(_x1, _x2, target->_x1, target->_x2);
	if(x<0 && y>=0) return corridorX(_x1, _x2, target->_x1, target->_x2, y);
	if(x>=0 && y<0) return corridorY(_y1, _y2, target->_y1, target->_y2, x);
	return corridorDiagonal(_x1, _x2, target->_x1, target->_x2,   _y1, _y2, target->_y1, target->_y2);
}

#undef FOR_ALL

// -------------------------- AStar ---------------------------------------------

namespace {
	Vec2d vec2d(const AStar::QT& c){
		return Vec2d(c->getCenterX(),c->getCenterY());
	}
}

double AStar::heuristic_cost_estimate(size_t sx, size_t sy, size_t gx, size_t gy){
		return Vec2d::distance(Vec2d(sx, sy), Vec2d(gx, gy));
	}
double AStar::dist_between(QT current, QT neighbor){
		return Vec2d::distance( vec2d(current), vec2d(neighbor) );
	}

AStar::Path AStar::search(size_t sx, size_t sy, size_t gx, size_t gy, AStar::QT qtRoot){
	 Path path;
	 QT start = qtRoot->findEmpty(sx,sy);
	 QT goal = qtRoot->findEmpty(gx,gy);
	 if(start==NULL || goal==NULL) return path;

	 set<QT> closedset;    								// The set of nodes already evaluated.
	 QTComparison comparison(*this);
	 priority_queue<QT,vector<QT>, QTComparison> openset(comparison);
	 map<QT,QT> came_from;    							// The map of navigated nodes.

	 map<QT,double> g_score;    // Cost from start along best known path.
	 f_score = map<QT,double>();

	 g_score[start]=0;
	 // Estimated total cost from start to goal .
	 f_score[start] = g_score[start] + heuristic_cost_estimate(sx,sy, gx, gy);
	 openset.push(start);    							// The set of tentative nodes to be evaluated, initially containing the start node

	 while( openset.empty()==false ){
		 QT current  =  openset.top(); // the node in openset having the lowest f_score[] value
		 if( current == goal ){
			 return reconstruct_path(came_from, goal);
		 }

		 openset.pop(); // remove current from openset
		 closedset.insert(current); // add current to closedset
		 QTNode::NEIGHBORS neighbor_nodes = current->getNeighbors();
		 for(QTNode::NEIGHBORS::const_iterator i_neighbor=neighbor_nodes.begin();i_neighbor!=neighbor_nodes.end();i_neighbor++){
			 QT neighbor = *i_neighbor;
			 if( closedset.find(neighbor)!=closedset.end() ){
				 continue;
			 }
			 double tentative_g_score = g_score[current] + dist_between(current,neighbor);

			 bool not_in_openset = f_score.find(neighbor)==f_score.end();
			 if( not_in_openset || tentative_g_score <= g_score[neighbor] ){
				 came_from[neighbor] = current;
				 g_score[neighbor] = tentative_g_score;
				 // Estimated total cost from start to goal through neighbor.
				 f_score[neighbor] = g_score[neighbor] + heuristic_cost_estimate(neighbor->getCenterX(), neighbor->getCenterY(), gx, gy);
				 if(not_in_openset) openset.push(neighbor);
			 }
		 }
	 }
	 return Path();
}

vector<QTNode::XY> QTPath::extractPoints(size_t sx, size_t sy, size_t gx, size_t gy, bool centers)const{
	typedef const QTNode* QT;
	const vector<QT>& path = _path;
	vector<QTNode::XY> points;
	if(path.size()>0){
		QTNode::XY p = {sx,sy};
		points.push_back(p);
		if(path.size()>2){
			const QTNode* prev = path[0];
			for(size_t i=1;i<path.size()-1;i++) {
				vector<QTNode::XY> corr = prev->getCorridor(path[i]);
				points.push_back(corr[0]);
				points.push_back(corr[1]);
				if(centers){
					QTNode::XY p = {path[i]->getCenterX(),path[i]->getCenterY()};
					points.push_back(p);
				}
				prev = path[i];
			}
			{
				vector<QTNode::XY> corr = prev->getCorridor(path[path.size()-1]);
				points.push_back(corr[0]);
				points.push_back(corr[1]);
			}
		}
		QTNode::XY e = {gx,gy};
		points.push_back(e);
	}
	return points;
}

AStar::Path AStar::reconstruct_path(map<AStar::QT,AStar::QT>& came_from, AStar::QT current_node){
	 if( came_from.find(current_node) != came_from.end() ){
		 Path p = reconstruct_path(came_from, came_from[current_node]);
		 p.push_back(current_node);
		 return p;
	 }else{
		 Path p; p.push_back(current_node);
		 return p;
	 }
}

// -------------------------------- Inflator -------------------------------------


Inflator::Inflator(size_t radius, char sbv)
: radius(radius), cellBlockValue(sbv)
{
	//TODO: MUST UNIQUE ON (tx,ty) => INCREACE SPEED OF INFLATION
	//cout<<"Inflator("<<radius<<","<<(int)sbv<<")"<<endl;
	int rr = radius*radius;
	for(int x = -(int)radius; x<=(int)radius; x++){
		int y = (int)round(sqrt( rr - x*x ));
		tx.push_back(x); ty.push_back(y);
		tx.push_back(x); ty.push_back(-y);
		tx.push_back(y); ty.push_back(x);
		tx.push_back(-y); ty.push_back(x);
	}
	//for( size_t i=0;i<tx.size();i++){cout<<"x,y="<<tx[i]<<","<<ty[i]<<endl;}
	//cout<<"Inflator: end"<<endl;
}
Map Inflator::inflate(const Map& source)const{
	Map m(source);
	for( size_t y=0; y<source.h(); y++){
		for( size_t x=0; x<source.w(); x++){
			if(source(x,y)==cellBlockValue){
				for( size_t i=0;i<tx.size();i++){
					size_t xx, yy;
					#define setInRage(val, mi, ma, v) { if( (int)(v) < (int)(mi) ) val=(mi); else if( (int)(v) > (int)(ma) ) val=(ma); else val=(v); }
					setInRage(xx, 0, m.w()-1, x+tx[i])
					setInRage(yy, 0, m.h()-1, y+ty[i])
					#undef setInRage
					m(xx,yy) = cellBlockValue;
					//cout<<"x,y="<<x<<","<<y<<"  "<<"xx,yy="<<xx<<","<<yy<<endl;
				}
			}
		}
	}
	return m;
}

Map MapEditor::replace(const Map& source, const char from, const char to)const{
	Map res ( source );
	for( size_t y=0; y<source.h(); y++){for( size_t x=0; x<source.w(); x++){
		if(res(x,y)==from) res(x,y)=to;
	}}
	return res;
}

Map MapEditor::coloring(const Map& source, size_t x, size_t y, const char av, const char bl)const{
	Map visited(source.w(), source.h());
	Map res(source.w(), source.h());
	//cout<<": "<<(int)source(x,y)<<": "<<(int)av<<": "<<(int)bl<<endl;
	coloring(source, x, y, source(x,y), av, bl, visited, res);
	for( size_t y=0; y<source.h(); y++){for( size_t x=0; x<source.w(); x++){
		if(visited(x,y)==Map::ST_UNCHARTED) res(x,y)=bl;
	}}
	return res;
}
void MapEditor::coloring(const Map& source, size_t x, size_t y, const char c, const char av, const char bl, Map& visited, Map& res)const{
	if(visited(x,y)==Map::ST_BLOCKED) return;
	visited(x,y) = Map::ST_BLOCKED;
	if(c == source(x,y)){
		res(x,y)=av;
		if(x>0)coloring(source, x-1, y, c, av, bl, visited, res);
		if(x<source.w()-1)coloring(source, x+1, y, c, av, bl, visited, res);
		if(y>0)coloring(source, x, y-1, c, av, bl, visited, res);
		if(y<source.h()-1)coloring(source, x, y+1, c, av, bl, visited, res);
	}
	else res(x,y)=bl;
}

// -------------------------------------------------------------------------------

int cogniteam_pathplanning_test_map_inflation(int argc, char** argv) {
	cout << "START" << endl; // prints PP

	char cmap[]={
			0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
			0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
			0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,

			0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
			0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
			0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,

			0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,1,
			0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,0,1,1,1,1,1,
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,0,0,
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,

			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
	};
	char cmap1[]={
			0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,

			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
			0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,

			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,

			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
	};
	size_t w=21,h=16;
	Map map(w, h, cmap1);
	cout<<"map"<<endl<<map<<endl;

	Inflator i(3, Map::ST_BLOCKED);
	MapEditor e;
	cout<<"inflated:"<<endl<<e.coloring(i.inflate(map), 11,7, 0,1)<<endl;

	return 0;
}


#include <cstdlib>
#include <cstdio>
int cogniteam_pathplanning_test(int argc, char** argv) {
	cout << "START" << endl; // prints PP

	char cmap_1[]={
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,0,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1,2,2,2,2,1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1,1,2,2,2,2,2,2,2,2,2,2,2,1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1,1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1,0,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,0,0,2,2,2,0,0,0,0,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,1,1,1,0,2,2,2,2,2,2,2,2,2,2,2,2,2,0,0,2,2,1,1,1,1,1,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,1,1,1,1,1,1,1,2,1,1,1,1,2,2,0,2,1,1,1,1,0,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,2,2,2,
		2,2,2,2,2,2,2,2,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,
		2,2,2,2,2,2,2,2,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,2,2,
		2,2,2,2,2,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,
		2,2,2,2,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		2,2,2,2,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		2,2,2,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		2,2,2,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		2,2,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		2,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		2,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
	};
	
	clock_t start_time = clock();
	
	char* cmap = cmap_1;
	size_t w=48,h=48;
	
	Map map(w, h, cmap);
	cout<<"map source"<<endl<<map<<endl;
	
	long _sx=atoi(argv[1]), _sy=atoi(argv[2]);
	long _ex=atoi(argv[3]), _ey=atoi(argv[4]);
	
	#define START_P _sx,_sy 
	#define END_P   _ex,_ey
	
	#define POINTS START_P, END_P
	
	printf("Plan path : from %i,%i to %i,%i \n", POINTS);
	
	if(map.inRange(END_P)==false){
		map.approximate(START_P, END_P);
	}
	
#if PRINT_AS_VECTORS == 1
	cout<<"\nPRINT MAP - STR"<<endl;
	for(size_t yy=0;yy<map.h();yy++){
		for(size_t xx=0;xx<map.w();xx++){
			if(map(xx,yy)==Map::ST_BLOCKED){
				cout<<xx<<'\t'<<yy<<endl;
			}
		}	
	}
	cout<<"PRINT MAP - END\n"<<endl;
#endif
	
	clock_t time_inflator = clock();
	
	Inflator i( 1 , Map::ST_BLOCKED);
	MapEditor e;
	
	map = 
	e.coloring( 
		e.replace(
			i.inflate(map), 
			Map::ST_UNCHARTED, Map::ST_AVAILABLE), 
		START_P, Map::ST_AVAILABLE,Map::ST_BLOCKED
	);
	
	map = MapEditor().replace(map, Map::ST_UNCHARTED,Map::ST_AVAILABLE);
	
	time_inflator = clock()-time_inflator;
	
	Map input_map = map;

	clock_t time_qt = clock();
	
	QTNode qt(0,w-1, 0, h-1, map);
	qt.folding();
	
	time_qt = clock() - time_qt;
	
	cout<<"QT:"<<endl<<qt<<endl;

#define PRINT_NODE(X,Y){\
	const QTNode* node = qt.findEmpty(X,Y);\
	if(node){\
		cout<<"FOUND:  "<<X<<":"<<Y<<endl;\
		cout<<"FE: "<<endl<<(*node);\
		QTNode::BORDER bord = node->getBorder();\
		for(size_t i=0;i<bord.size();i++) cout<<bord[i].x<<","<<bord[i].y<<"  "; cout<<endl;\
		QTNode::NEIGHBORS nei = node->getNeighbors();\
		cout<<"NEIGHBORS: "<<nei.size()<<endl;\
		for(QTNode::NEIGHBORS::const_iterator i=nei.begin();i!=nei.end();i++) cout<<(**i);\
	}else{\
		cout<<"NOT FOUND : "<<X<<":"<<Y<<endl;\
	}cout<<"-------"<<endl;}

	PRINT_NODE(19,13)
	PRINT_NODE(20,14)
	PRINT_NODE(20,15)
	PRINT_NODE(5,5)
	

#if PRINT_AS_VECTORS == 1
	cout<<"\nPRINT INFLATED MAP - STR"<<endl;
	for(size_t yy=0;yy<map.h();yy++){
		for(size_t xx=0;xx<map.w();xx++){
			if(map(xx,yy)==Map::ST_BLOCKED){
				cout<<xx<<'\t'<<yy<<endl;
			}
		}	
	}
	cout<<"PRINT INFLATED - END\n"<<endl;
#endif

	AStar a_star;
	size_t sx(2),sy(13),gx(15),gy(3);
	
	clock_t time_astar = clock();
	
	AStar::Path path = a_star.search(POINTS , &qt);
	
	time_astar = clock() - time_astar;

	cout<<"A* ("<<sx<<","<<sy<<":"<<gx<<","<<gy<<") result : path length = "<<path.size()<<endl;
	cout<<"path (by nodes): "<<endl;
	for(size_t i=0; i<path.size();i++){
		cout<<*(path[i]);
		map(path[i]->getCenterX(), path[i]->getCenterY())='@';
	}
	cout << "------------"<< endl;
	Path res_path;
	
	clock_t time_ep = clock();
	
	vector<QTNode::XY> points = QTPath(path).extractPoints(POINTS);
	
	time_ep = clock() - time_ep; time_astar+=time_ep;
	
	cout<<"path by points: ";
	for( size_t i=0;i<points.size(); i++){
		cout<<"("<<points[i].x<<","<<points[i].y<<") ";
		map(points[i].x, points[i].y)='+';
		res_path.push_back(Waypoint(points[i].x,points[i].y));
	}
#if PRINT_AS_VECTORS == 1
	cout<<"\nPRINT PATH AFTER A* - STR"<<endl;
	for( size_t i=0;i<points.size(); i++){
		cout<<points[i].x<<'\t'<<points[i].y<<endl;
	}
	cout<<"PRINT PATH AFTER A* - END\n"<<endl;
#endif

	cout<<endl;
	cout<<"map with path"<<endl<<map<<endl;
	
	PField pf(input_map, res_path);
	PField::SmoothingParameters pf_params;
	pf_params.viewRadiusForward = 5;
	pf_params.viewRadiusSide = 2;
	pf_params.stepRate=0.6;
	pf_params.inertia=pow(1/pf_params.viewRadiusForward*0.5,2);
	pf_params.distanceBetweenPoints = 2;
	pf_params.maxAngleWhileReducing = Vec2d::d2r(10);

	clock_t time_smoothing = clock();
	
	PField::Points smoothed_points= pf.smooth(pf_params);
	
	time_smoothing = clock() - time_smoothing;
	
	Path smoothed = pf.smoothWaypoints(pf_params);
	
	cout<<"smoothed path: ";
	for( size_t i=0;i<smoothed.size(); i++){
		cout<<"("<<smoothed[i].x<<","<<smoothed[i].y<<") ";
		map(smoothed[i].x, smoothed[i].y)='o';
	}
	
	cout<<endl;
	cout<<"map with smoothed path"<<endl<<map<<endl;
	
	start_time = clock()-start_time;
	
#if PRINT_AS_VECTORS == 1
	cout<<"\nPRINT PATH AFTER PFIELD - STR"<<endl;
	for( size_t i=0;i<smoothed_points.size(); i++){
		cout<<smoothed_points[i].x<<'\t'<<smoothed_points[i].y<<endl;
	}
	cout<<"PRINT PATH AFTER PFIELD - END\n"<<endl;
#endif
//#define STR_TIME(x) ((double)x)/(double)CLOCKS_PER_SEC/1000.0<<" msec"
#define STR_TIME(x) x<<" clocks"
	cout<<"TIMES:"<<endl;
	cout<<"   total : "<<STR_TIME(start_time)<<endl;
	cout<<"   inflation : "<<STR_TIME(time_inflator)<<endl;
	cout<<"   quad tree : "<<STR_TIME(time_qt)<<endl;
	cout<<"   A* : "<<STR_TIME(time_astar)<<endl;
	cout<<"   smoothing : "<<STR_TIME(time_smoothing)<<endl;
	
	cout << endl << "END" << endl; // prints PP
	return 0;
}

namespace {

	struct Pair{ Waypoint s, g;
		Pair(const Waypoint& ss, const Waypoint& ee):s(ss.x,ss.y),g(ee.x,ee.y){}
		Pair(const TransitWaypoint& ss, const TransitWaypoint& ee):s(ss.x,ss.y),g(ee.x,ee.y){}
		Pair(const Waypoint& ss, const TransitWaypoint& ee):s(ss.x,ss.y),g(ee.x,ee.y){}
		Pair(const TransitWaypoint& ee, const Waypoint& ss):s(ss.x,ss.y),g(ee.x,ee.y){}
	};

}

PField::Points searchPath(const Map& source_map, const Waypoint& start, const Waypoint& finish, const Constraints& constraints){
	using namespace std;

	//cout<<"searchPath: "<<"Input map:"<<endl<<source_map<<endl;
	
	//TODO: PROCESS CONSTRAINTS PATH BEFORE INFLATION

	if( source_map(start.x, start.y)==Map::ST_BLOCKED || source_map(finish.x, finish.y)==Map::ST_BLOCKED ){
		cout<<"searchPath: "<<"some of interesting points are not available (before inflation)"<<endl;
		return PField::Points();
	}
	
	Inflator i( constraints.dimentions.radius , Map::ST_BLOCKED);
	MapEditor e;
	
// 	Map map =  i.inflate(source_map);
	
// 	Map map =  
// 		e.replace(
// 			i.inflate(source_map), 
// 			Map::ST_UNCHARTED, Map::ST_AVAILABLE)
// 	;

	Map map = 
		e.coloring( 
			e.replace(
				i.inflate(source_map), 
				Map::ST_UNCHARTED, Map::ST_AVAILABLE), 
			start.x, start.y, Map::ST_AVAILABLE,Map::ST_BLOCKED
		);
	
	//cout<<"searchPath: "<<"Inflated map:"<<endl<<map<<endl;

	QTNode qt(0,map.w()-1, 0, map.h()-1, map);
	qt.folding();

	// CHECK IF ALL INTERESTING POINTS (start, stop and transits) ARE AVAILABLE

	if( !qt.findEmpty(start.x, start.y) || !qt.findEmpty(finish.x, finish.y) ){
		cout<<"searchPath: "<<"some of interesting points are not available"<<endl;
		return PField::Points();
	}
	if(constraints.transits.size()!=0){
		for( size_t i=0 ; i<constraints.transits.size(); i++ ){
			if( !qt.findEmpty(constraints.transits[i].x, constraints.transits[i].y) ){
				cout<<"searchPath: "<<"some of transit points are not available"<<endl;
				return PField::Points();
			}
		}
	}

	// CREATE SEGMENTS

	vector<Pair> segments;
	if(constraints.transits.size()==0){
		segments.push_back(Pair(start, finish));
	}else{
		size_t i=0;
		segments.push_back(Pair(start, constraints.transits[i]));
		for(; i<constraints.transits.size()-1; i++)
			segments.push_back(Pair(constraints.transits[i], constraints.transits[i+1]));
		segments.push_back(Pair(constraints.transits[i], finish));
	}

	// CREATE PATH

	Path path;
	AStar a_star;
	for( size_t i=0; i<segments.size(); i++){
		#define SEGMENT segments[i].s.x, segments[i].s.y,  segments[i].g.x,segments[i].g.y
		AStar::Path qt_path = a_star.search( SEGMENT , &qt);
		vector<QTNode::XY> points;
		if( qt_path.size()>0 ) points = QTPath(qt_path).extractPoints( SEGMENT );
		for( size_t p=0; p<points.size(); p++ ){
			path.push_back(Waypoint(points[p].x,points[p].y));
		}
		#undef SEGMENT
	}

	PField::SmoothingParameters pf_params;
	pf_params.viewRadiusForward = 5;
	pf_params.viewRadiusSide = 2;
	pf_params.stepRate=0.6;
	pf_params.inertia=pow(1/pf_params.viewRadiusForward*0.5,2);
	pf_params.distanceBetweenPoints = 2;
	pf_params.maxAngleWhileReducing = Vec2d::d2r(10); // unused

	PField pf(map, path);
	PField::Points smoothed_path = pf.smooth(pf_params);
	
	return smoothed_path;
}
