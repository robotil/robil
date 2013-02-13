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


class AStar{
public:

	typedef const QTNode* QT;
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




#endif /* COGNITEAM_PATHPLANNING_ASTAR_H_ */
