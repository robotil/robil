/*
 * AStart.h
 *
 *  Created on: Jan 6, 2013
 *      Author: dan
 */

#ifndef COGNITEAM_PATHPLANNING_ASTAR_H_
#define COGNITEAM_PATHPLANNING_ASTAR_H_

#include "Map.h"
#include "QTNode.h"

#include <vector>
#include <queue>
#include <set>
#include <map>
#include <math.h>

#include "cogniteam_pathplanner_parameters.h"

class AStar{
public:

	typedef const BWQTNode* QT;
	typedef vector<QT> Path;

private:

	double heuristic_cost_estimate(size_t sx, size_t sy, size_t gx, size_t gy);
	double dist_between(QT current, QT neighbor);

	class QTComparison{
		AStar& astar;
	public:
		QTComparison(AStar& astar):astar(astar){}
		bool operator() (const QT & a, const QT & b) const{
			return astar.f_score[a] > astar.f_score[b];
		}
	};

	map<QT,double> f_score;

public:
	Path search(size_t sx, size_t sy, size_t gx, size_t gy, QT qtRoot);

	//vector<QTNode::XY> extractPoints(size_t sx, size_t sy, size_t gx, size_t gy, const Path& path, bool centers = false)const;
private:
	Path reconstruct_path(map<QT,QT>& came_from, QT current_node);
};


static bool operator<(const QTNode::XY& __x, const QTNode::XY& __y){return __x.x!=__y.x?__x.x < __y.x:__x.y<__y.y;};
static bool operator==(const QTNode::XY& __x, const QTNode::XY& __y){ return __x.x==__y.x && __x.y==__y.y; }

class BStarParameters{
public:
	double w_distance;
	double w_slop;
	double w_alt_delta;
	BStarParameters(){
		SET_BS_PARAMETERS((*this));
	}
};

class BStar{
public:

	typedef const QTNode* QT;
	typedef QTNode::XY Point;
	typedef vector<Point> Path;

private:

	double heuristic_cost_estimate(const Point& start, const Point& goal)const;
	double cost_estimate(const Point& current, const Point& neighbor)const;

	struct lowest_first {
		bool operator()(const BStar::Point& __x, const BStar::Point& __y) const {
			return __x.x!=__y.x?__x.x < __y.x:__x.y<__y.y;
		}
	};

	class QTComparison{
		BStar& eng;
	public:
		QTComparison(BStar& eng):eng(eng){}
		bool operator() (const Point & a, const Point & b) const{
			return eng.f_score[a] > eng.f_score[b];
		}
	};

	map<Point,double> f_score;

public:

	BStar(QT qtRoot, const AltMap& alts, const AltMap& slops, const ObsMap& walls, const BStarParameters& params):
		_qtRoot(qtRoot), _alts(alts), _slops(slops), _walls(walls), _params(params)
	{

	}

	Path search(size_t sx, size_t sy, size_t gx, size_t gy);

private:
	Path reconstruct_path(map<Point,Point>& came_from, const Point& current_node);

	QT _qtRoot;
	const AltMap& _alts;
	const AltMap& _slops;
	const ObsMap& _walls;
	const BStarParameters& _params;
};



#endif /* COGNITEAM_PATHPLANNING_ASTAR_H_ */
