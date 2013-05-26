/*
 * QTNode.cpp
 *
 *  Created on: May 22, 2013
 *      Author: dan
 */
#include "QTNode.h"

typedef ObsMap Map;

// -------------------------- QTNode ---------------------------------------------

BWQTNode::BWQTNode(size_t x1, size_t x2, size_t y1, size_t y2, const Map& map, BWQTNode* supper):
	Bound(x1, y1, x2, y2),_supper(supper),_map(map){

	state = _x1==_x2 && _y1==_y2 ? BWQTNode::st_sing : BWQTNode::st_mult;

	::memset(subs, 0, 4*sizeof(BWQTNode*));
	if(state!=st_sing){
		size_t x11=x1, xn=(x2-x1)/2, x12=x1+xn, x21=x12+1, x22=x2;
		size_t y11=y1, yn=(y2-y1)/2, y12=y1+yn, y21=y12+1, y22=y2;

		#define CNODE(x11, x12, y11, y12, N) if(x11<=x12 && y11<=y12) subs[N]=new BWQTNode(x11, x12, y11, y12, map, this);
		CNODE(x11, x12, y11, y12, 0)
		CNODE(x21, x22, y11, y12, 1)
		CNODE(x11, x12, y21, y22, 2)
		CNODE(x21, x22, y21, y22, 3)
		#undef CNODE
	}
}
BWQTNode::~BWQTNode(){
	for(size_t i=0;i<4;i++){
		if(subs[i]) delete subs[i]; subs[i]=0;
	}
}
#define FOR_ALL for(size_t i=0;i<4;i++)if(subs[i])

const BWQTNode* BWQTNode::findEmpty(size_t x, size_t y)const{
	if(!inRange(x,y)) return NULL;
	if(state == st_bl) return NULL;
	if(state == st_av) return this;
	if(state == st_sing){
		if(isEmpty()) return this;
		return NULL;
	}
	FOR_ALL{
		const BWQTNode* res = subs[i]->findEmpty(x,y);
		if(res) return res;
	}
	return NULL;
}

bool BWQTNode::isAvailable(size_t x, size_t y)const{
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

int BWQTNode::folding(){
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

vector<Bound::XY> Bound::getBorder(long mapw , long maph)const{
	vector<XY> res;
	#define add(x,y) if(x>=0 && y>=0 && x<mapw && y<maph){XY xy={x,y};res.push_back(xy);}
	long x = _x1-1;long y = _y1-1;
	for(; x<=(long)_x2; x++) add(x,y)
	for(; y<=(long)_y2; y++) add(x,y)
	for(; x>=(long)  _x1; x--) add(x,y)
	for(; y>=(long)  _y1; y--) add(x,y)
	return res;
	#undef add
}

vector<Bound::XY> Bound::getOnBorderPoints()const{
	POINTS res;
	size_t x=_x1, y=_y1;
	for(; x<_x2; x++) {XY xy={x,y};res.push_back(xy);}
	for(; y<_y2; y++) {XY xy={x,y};res.push_back(xy);}
	for(; x>_x1; x--) {XY xy={x,y};res.push_back(xy);}
	for(; y>_y1; y--) {XY xy={x,y};res.push_back(xy);}
	return res;
}
vector<Bound::XY> Bound::getBorderNeighbors(const XY& a, long mapw , long maph)const{
	vector<XY> res;
	//cout<<"w h = "<<mapw<<" "<<maph<<".  a="<<a.x<<","<<a.y<<";  "<<_x1<<","<<_y1<<","<<_x2<<","<<_y2<<endl;
	#define add(x,y) /*cout<<"!@"<<x<<","<<y<< " not in rage "<<(!inRange(x,y)?"true":"false")<<endl; */if(long(x)>=0 && long(y)>=0 && long(x)<mapw && long(y)<maph && !inRange(x,y)){XY xy={x,y};res.push_back(xy); /*cout<<"!!! "<<x<<","<<y<<endl;*/}
	add(a.x-1, a.y-1); add(a.x, a.y-1); add(a.x+1, a.y-1);
	add(a.x-1, a.y); 				    add(a.x+1, a.y);
	add(a.x-1, a.y+1); add(a.x, a.y+1); add(a.x+1, a.y+1);
	return res;
	#undef add
}

Bound::POINTS Bound::removeSameSide(const POINTS& points, const XY& a)const{
	if(a.x==_x1&&a.y==_y1) return removePoints(points, a, SameXorY());
	if(a.x==_x2&&a.y==_y1) return removePoints(points, a, SameXorY());
	if(a.x==_x2&&a.y==_y2) return removePoints(points, a, SameXorY());
	if(a.x==_x1&&a.y==_y2) return removePoints(points, a, SameXorY());
	if(a.x==_x1) return removePoints(points, a, SameX());
	if(a.x==_x2) return removePoints(points, a, SameX());
	if(a.y==_y1) return removePoints(points, a, SameY());
	if(a.y==_y2) return removePoints(points, a, SameY());
	return points;
}

set<const BWQTNode*> BWQTNode::getNeighbors()const{
	if(state == st_bl) return set<const BWQTNode*>();
	if(state == st_mult){
		set<const BWQTNode*> s;
		#define UNION(A, B) {set<const BWQTNode*> b = B; for(set<const BWQTNode*>::const_iterator n=b.begin();n!=b.end();n++) A.insert(*n); }
		FOR_ALL{ UNION(s, subs[i]->getNeighbors()); }
		#undef UNION
		return s;
	}
	vector<XY> border = getBorder();
	set<const BWQTNode*> s;
	for(size_t i=0;i<border.size();i++){
		const BWQTNode* e = getRoot()->findEmpty(border[i].x, border[i].y);
		if(e) s.insert(e);
	}
	return s;
}
size_t Bound::linesIntersectCenter(size_t a1, size_t a2, size_t b1, size_t b2)const{
	if(a1<=b1 && b2<=a2) return (b2+b1)/2;
	if(b1<=a1 && a2<=b2) return (a2+a1)/2;
	if(b2<a1 || a2<b1) return -1;
	if(a1<=b1 && a2<=b2) return (a2+b1)/2;
	if(b1<=a1 && b2<=a2) return (b2+a1)/2;
	return -1;
}
vector<Bound::XY> Bound::corridorX(size_t x11, size_t x12, size_t x21, size_t x22, size_t y)const{
	vector<XY> res;
	if(x11>x22){ XY p1={x11,y}, p2={x22,y}; res.push_back(p1); res.push_back(p2); }
	if(x21>x12){ XY p1={x12,y}, p2={x21,y}; res.push_back(p1); res.push_back(p2); }
	return res;
}
vector<Bound::XY> Bound::corridorY(size_t y11, size_t y12, size_t y21, size_t y22, size_t x)const{
	vector<XY> res;
	if(y11>y22){ XY p1={x,y11}, p2={x,y22}; res.push_back(p1); res.push_back(p2); }
	if(y21>y12){ XY p1={x,y12}, p2={x,y21}; res.push_back(p1); res.push_back(p2); }
	return res;
}
vector<Bound::XY> Bound::corridorDiagonal(size_t x11, size_t x12, size_t x21, size_t x22,   size_t y11, size_t y12, size_t y21, size_t y22)const{
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

vector<Bound::XY> Bound::getCorridor(const Bound& target)const{
	int y = linesIntersectCenter(_y1, _y2, target._y1, target._y2);
	int x = linesIntersectCenter(_x1, _x2, target._x1, target._x2);
	if(x<0 && y>=0) return corridorX(_x1, _x2, target._x1, target._x2, y);
	if(x>=0 && y<0) return corridorY(_y1, _y2, target._y1, target._y2, x);
	return corridorDiagonal(_x1, _x2, target._x1, target._x2,   _y1, _y2, target._y1, target._y2);
}

vector<BWQTNode::XY> BWQTNode::getCorridor(const BWQTNode* target)const{
	return Bound::getCorridor(*target);
}


//===============================================================================
//====================  QTNODE ==================================================
//===============================================================================

QTNode::QTNode(size_t x1, size_t x2, size_t y1, size_t y2, const AltMap& map, QTNode* supper):
	Bound(x1, y1, x2, y2),_supper(supper),_map(map){

	state() = _x1==_x2 && _y1==_y2 ? QTNode::st_singl : QTNode::st_mult;

	::memset(subs, 0, 4*sizeof(QTNode*));
	if(state()!=st_singl){
		size_t x11=x1, xn=(x2-x1)/2, x12=x1+xn, x21=x12+1, x22=x2;
		size_t y11=y1, yn=(y2-y1)/2, y12=y1+yn, y21=y12+1, y22=y2;

		#define CNODE(x11, x12, y11, y12, N) if(x11<=x12 && y11<=y12) subs[N]=new QTNode(x11, x12, y11, y12, map, this);
		CNODE(x11, x12, y11, y12, 0)
		CNODE(x21, x22, y11, y12, 1)
		CNODE(x11, x12, y21, y22, 2)
		CNODE(x21, x22, y21, y22, 3)
		#undef CNODE
	}else{
		value() = map(_x1,_y1) * 100;
	}
}
QTNode::~QTNode(){
	for(size_t i=0;i<4;i++){
		if(subs[i]) delete subs[i]; subs[i]=0;
	}
}

const QTNode* QTNode::find(size_t x, size_t y)const{
	if(!inRange(x,y)) return NULL;
	if(isHomogeny()) return this;
	FOR_ALL{
		const QTNode* res = subs[i]->find(x,y);
		if(res) return res;
	}
	return NULL;
}

QTNode::Value QTNode::value(size_t x, size_t y)const{
	const QTNode* node = find(x,y);
	if(node) return node->value();
	return ValueUnknown;
}

int QTNode::folding(){
	if(isHomogeny()){
		return 1;
	}else{
		int hom_count=0;
		int sub_count=0;
		int last_sub_index=-1;
		FOR_ALL{
			last_sub_index = i;
			sub_count++;
			hom_count += subs[i]->folding();
		}
		if(sub_count!=hom_count) return 0;
		Value v = subs[last_sub_index]->value();
		FOR_ALL{
			if( v != subs[i]->value() ) return 0;
		}
		state()=st_folded;
		value()=v;
		FOR_ALL{
			delete subs[i]; subs[i]=0;
		}
		return 1;
	}
}

QTNode::NEIGHBORS QTNode::getNeighbors()const{
	typedef NEIGHBORS SET;
	typedef SET::const_iterator ITER;
	if(state() == st_mult){
		SET s;
		#define UNION(A, B) {SET b = B; for(ITER n=b.begin();n!=b.end();n++) A.insert(*n); }
		FOR_ALL{ UNION(s, subs[i]->getNeighbors()); }
		#undef UNION
		return s;
	}
	vector<XY> border = getBorder();
	SET s;
	for(size_t i=0;i<border.size();i++){
		const QTNode* e = getRoot()->find(border[i].x, border[i].y);
		if(e) s.insert(e);
	}
	return s;
}

namespace {
	vector<QTNode::XY> UNION(const vector<QTNode::XY>& a, const vector<QTNode::XY>& b){
		vector<QTNode::XY> points;
		for(size_t i=0;i<a.size();i++)points.push_back(a[i]);
		for(size_t i=0;i<b.size();i++)points.push_back(b[i]);
		return points;
	}
}

vector<QTNode::XY> QTNode::getNeighborsPoints(const QTNode::XY& xy)const{
	vector<QTNode::XY> points;
	const QTNode* root = getRoot();
	const QTNode* node = root->find(xy.x, xy.y);
	if(!node) return points;
	vector<QTNode::XY> ins, outs;
	ins = removeSameSide(node->getOnBorderPoints(), xy);
	outs = node->getBorderNeighbors(xy, _map.w(), _map.h());
	return UNION(ins, outs);
}

#undef FOR_ALL

