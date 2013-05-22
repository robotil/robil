/*
 * QTNode.h
 *
 *  Created on: Jan 6, 2013
 *      Author: dan
 */

#ifndef COGNITEAM_PATHPLANNING_QTNODE_H_
#define COGNITEAM_PATHPLANNING_QTNODE_H_

#include "Map.h"

#include <iostream>
#include <vector>
#include <iostream>
#include <vector>

typedef ObsMap Map;

class Bound{
public:
	struct XY{ size_t x, y; };
	Bound(size_t x1, size_t y1, size_t x2, size_t y2)
	:_x1(x1), _x2(x2), _y1(y1), _y2(y2){}

	bool inRange(size_t x, size_t y)const{
		return (_x1<=x && x<=_x2) && (_y1<=y && y<=_y2);
	}

	typedef vector<XY> BORDER;
	vector<XY> getBorder(long mapw , long maph)const;

private:
	size_t linesIntersectCenter(size_t a1, size_t a2, size_t b1, size_t b2)const;
	vector<XY> corridorX(size_t x11, size_t x12, size_t x21, size_t x22, size_t y)const;
	vector<XY> corridorY(size_t y11, size_t y12, size_t y21, size_t y22, size_t x)const;
	vector<XY> corridorDiagonal(size_t x11, size_t x12, size_t x21, size_t x22,   size_t y11, size_t y12, size_t y21, size_t y22)const;

public:
	size_t getCenterX()const{
		return (_x2+_x1)/2;
	}
	size_t getCenterY()const{
		return (_y2+_y1)/2;
	}
	vector<XY> getCorridor(const Bound& target)const;

protected:
	size_t _x1,_x2,_y1,_y2;



	typedef vector<Bound::XY> POINTS;

	POINTS getOnBorderPoints()const;
	struct SameXorY{bool operator()(const XY& a, const XY& c, const Bound& b)const{ return a.x==c.x || a.y==c.y;}};
	struct SameX{bool operator()(const XY& a, const XY& c, const Bound& b)const{ return a.x==c.x;}};
	struct SameY{bool operator()(const XY& a, const XY& c, const Bound& b)const{ return a.y==c.y;}};
	struct SameNode{bool operator()(const XY& a, const XY& c, const Bound& b)const{ return b.inRange(c.x,c.y);}};
	template <class COND> POINTS removePoints(const POINTS& points, const XY& a, const COND& cond)const{
		POINTS n;
		for(size_t i=0;i<points.size();i++) if(!cond(a, points[i], *this)) n.push_back(points[i]);
		return n;
	}

	POINTS removeSameSide(const POINTS& points, const XY& a)const;
	POINTS getBorderNeighbors(const XY& a, long mapw , long maph)const;
};
static void print(ostream & o, vector<Bound::XY>& v){
	for(size_t i=0;i<v.size();i++){
		o<<"{"<<v[i].x<<","<<v[i].y<<"} ";
	}
	o<<endl;
}

class BWQTNode: public Bound{
public:
	enum State{ st_mult, st_sing, st_av, st_bl };
#define FOR_ALL for(size_t i=0;i<4;i++)if(subs[i])


	BWQTNode(size_t x1, size_t x2, size_t y1, size_t y2, const Map& map, BWQTNode* supper=0);
	virtual ~BWQTNode();
	bool isOneCell()const{
		return state == BWQTNode::st_sing;
	}
	bool isEmpty()const{
		if(isOneCell()){
			return _map(_x1,_y1) == Map::ST_AVAILABLE;
		}
		if(state != st_mult){
			return state == st_av ? true: false;
		}
		return false;
	}

	const BWQTNode* findEmpty(size_t x, size_t y)const;

	bool isAvailable(size_t x, size_t y)const;

	int folding();

	typedef set<const BWQTNode*> NEIGHBORS;
	set<const BWQTNode*> getNeighbors()const;

	vector<XY> getBorder()const{
		return Bound::getBorder((long)_map.w(), (long)_map.h());
	}

	vector<XY> getCorridor(const BWQTNode* target)const;

private:
	const BWQTNode* getRoot()const{
		if(_supper==0) return this;
		return _supper->getRoot();
	}

private:

	BWQTNode* subs[4];
	BWQTNode* _supper;
	const Map& _map;
	State state;

#undef FOR_ALL

	friend int print(ostream& out, const BWQTNode* n, string tab);
};

inline char BWQTNodeStr(int s){
	if(s==0) return '*';
	if(s==1) return '1';
	if(s==2) return '+';
	if(s==3) return '-';
	return '?';
}

inline int print(ostream& out, const BWQTNode* n, string tab=""){
	if(n->state==BWQTNode::st_sing){
		out<<tab<</*n<<": "<<*/BWQTNodeStr(n->state)<<" "<<(n->isEmpty()?"A":"B")<<" ("<<n->_x1<<","<<n->_y1<<")"/*<<" p="<<n->_supper*/<<endl;
	}else{
		out<<tab<</*n<<": "<<*/BWQTNodeStr(n->state)<<" "<<(n->isEmpty()?"A":"B")<<" ("<<n->_x1<<","<<n->_y1<<":"<<n->_x2<<","<<n->_y2<<")"/*<<" p="<<n->_supper*/<<endl;
	}
	for(int i=0;i<4;i++){
		if(n->subs[i]) print(out, n->subs[i], tab+"   ");
	}
	return 0;
}
inline ostream& operator<<(ostream& out, const BWQTNode& n){
	print(out,&n);
	return out;
}

class QTPath{
	typedef const BWQTNode* QT;
	std::vector<QT> _path;
public:
	QTPath(const std::vector<const BWQTNode*>& p):_path(p){}
	vector<BWQTNode::XY> extractPoints(size_t sx, size_t sy, size_t gx, size_t gy, bool centers = false)const;
};


//===============================================================================
//====================  QTNODE ==================================================
//===============================================================================

class QTNode:public Bound{
public:
	enum State{ st_mult, st_singl, st_folded };
	typedef long Value;
	static const Value ValueUnknown = 1000000000;

#define FOR_ALL for(size_t i=0;i<4;i++)if(subs[i])

	QTNode(size_t x1, size_t x2, size_t y1, size_t y2, const AltMap& map, QTNode* supper=0);
	virtual ~QTNode();

	bool isHomogeny()const{
		return state() == QTNode::st_singl || state() == QTNode::st_folded;
	}

	Value value()const{return _value;}
	Value& value(){return _value;}
	State state()const{return _state;}
	State& state(){return _state;}

	Value value(size_t x, size_t y)const;

	const QTNode* find(size_t x, size_t y)const;

	int folding();

	typedef set<const QTNode*> NEIGHBORS;
	NEIGHBORS getNeighbors()const;

	vector<XY> getBorder()const{
		return Bound::getBorder((long)_map.w(), (long)_map.h());
	}

	vector<XY> getCorridor(const QTNode* target)const{
		return Bound::getCorridor(*target);
	}

	vector<XY> getNeighborsPoints(const QTNode::XY& xy)const;

	const AltMap* mapPtr()const{ return &_map; }

private:
	const QTNode* getRoot()const{
		if(_supper==0) return this;
		return _supper->getRoot();
	}

private:
	QTNode* subs[4];
	QTNode* _supper;
	const AltMap& _map;
	State _state;
	Value _value;

#undef FOR_ALL

	friend int print(ostream& out, const QTNode* n, string tab);
};

inline char QTNodeStr(int s){
	if(s==0) return '*';
	if(s==1) return '1';
	if(s==2) return '+';
	return '?';
}

inline int print(ostream& out, const QTNode* n, string tab=""){
	if(n->state()==QTNode::st_singl){
		out<<tab<</*n<<": "<<*/QTNodeStr(n->state())<<" "<<(n->value())<<" ("<<n->_x1<<","<<n->_y1<<")"/*<<" p="<<n->_supper*/<<endl;
	}else{
		out<<tab<</*n<<": "<<*/QTNodeStr(n->state())<<" "<<(n->value())<<" ("<<n->_x1<<","<<n->_y1<<":"<<n->_x2<<","<<n->_y2<<")"/*<<" p="<<n->_supper*/<<endl;
	}
	for(int i=0;i<4;i++){
		if(n->subs[i]) print(out, n->subs[i], tab+"   ");
	}
	return 0;
}
inline ostream& operator<<(ostream& out, const QTNode& n){
	print(out,&n);
	return out;
}

#endif /* COGNITEAM_PATHPLANNING_QTNODE_H_ */
