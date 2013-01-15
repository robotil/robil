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

class QTNode{
public:
	enum State{ st_mult, st_sing, st_av, st_bl };
#define FOR_ALL for(size_t i=0;i<4;i++)if(subs[i])

	struct XY{ size_t x, y; };

	QTNode(size_t x1, size_t x2, size_t y1, size_t y2, const Map& map, QTNode* supper=0);
	virtual ~QTNode();
	bool isOneCell()const{
		return state == QTNode::st_sing;
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
	bool inRange(size_t x, size_t y)const{
		return (_x1<=x && x<=_x2) && (_y1<=y && y<=_y2);
	}

	const QTNode* findEmpty(size_t x, size_t y)const;

	bool isAvailable(size_t x, size_t y)const;

	int folding();

	typedef vector<XY> BORDER;
	vector<XY> getBorder()const;
	typedef set<const QTNode*> NEIGHBORS;
	set<const QTNode*> getNeighbors()const;

	size_t getCenterX()const{
		return (_x2+_x1)/2;
	}
	size_t getCenterY()const{
		return (_y2+_y1)/2;
	}

private:
	size_t linesIntersectCenter(size_t a1, size_t a2, size_t b1, size_t b2)const;
	vector<XY> corridorX(size_t x11, size_t x12, size_t x21, size_t x22, size_t y)const;
	vector<XY> corridorY(size_t y11, size_t y12, size_t y21, size_t y22, size_t x)const;
	vector<XY> corridorDiagonal(size_t x11, size_t x12, size_t x21, size_t x22,   size_t y11, size_t y12, size_t y21, size_t y22)const;
public:
	vector<XY> getCorridor(const QTNode* target)const;

private:
	const QTNode* getRoot()const{
		if(_supper==0) return this;
		return _supper->getRoot();
	}

private:
	size_t _x1,_x2,_y1,_y2;
	QTNode* subs[4];
	QTNode* _supper;
	const Map& _map;
	State state;

#undef FOR_ALL

	friend int print(ostream& out, const QTNode* n, string tab);
};

inline char QTNodeStr(int s){
	if(s==0) return '*';
	if(s==1) return '1';
	if(s==2) return '+';
	if(s==3) return '-';
	return '?';
}

inline int print(ostream& out, const QTNode* n, string tab=""){
	if(n->state==QTNode::st_sing){
		out<<tab<</*n<<": "<<*/QTNodeStr(n->state)<<" "<<(n->isEmpty()?"A":"B")<<" ("<<n->_x1<<","<<n->_y1<<")"/*<<" p="<<n->_supper*/<<endl;
	}else{
		out<<tab<</*n<<": "<<*/QTNodeStr(n->state)<<" "<<(n->isEmpty()?"A":"B")<<" ("<<n->_x1<<","<<n->_y1<<":"<<n->_x2<<","<<n->_y2<<")"/*<<" p="<<n->_supper*/<<endl;
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

class QTPath{
	typedef const QTNode* QT;
	std::vector<QT> _path;
public:
	QTPath(const std::vector<const QTNode*>& p):_path(p){}
	vector<QTNode::XY> extractPoints(size_t sx, size_t sy, size_t gx, size_t gy, bool centers = false)const;
};


#endif /* COGNITEAM_PATHPLANNING_QTNODE_H_ */
