/*
 * AStar.cpp
 *
 *  Created on: May 22, 2013
 *      Author: dan
 */

#include "AStar.h"
typedef ObsMap Map;

#include "Vec2d.hpp"

#define PRINT_SEARCH_INFO 0

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
		 BWQTNode::NEIGHBORS neighbor_nodes = current->getNeighbors();
		 for(BWQTNode::NEIGHBORS::const_iterator i_neighbor=neighbor_nodes.begin();i_neighbor!=neighbor_nodes.end();i_neighbor++){
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

vector<BWQTNode::XY> QTPath::extractPoints(size_t sx, size_t sy, size_t gx, size_t gy, bool centers)const{
	typedef const BWQTNode* QT;
	const vector<QT>& path = _path;
	vector<BWQTNode::XY> points;
	if(path.size()>0){
		BWQTNode::XY p = {sx,sy};
		points.push_back(p);
		if(path.size()>=2){
			const BWQTNode* prev = path[0];
			for(size_t i=1;i<path.size()-1;i++) {
				vector<BWQTNode::XY> corr = prev->getCorridor(path[i]);
				points.push_back(corr[0]);
				points.push_back(corr[1]);
				if(centers){
					BWQTNode::XY p = {path[i]->getCenterX(),path[i]->getCenterY()};
					points.push_back(p);
				}
				prev = path[i];
			}
			{
				vector<BWQTNode::XY> corr = prev->getCorridor(path[path.size()-1]);
				points.push_back(corr[0]);
				points.push_back(corr[1]);
			}
		}
		BWQTNode::XY e = {gx,gy};
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



// -------------------------- BStar ---------------------------------------------

namespace {
	Vec2d vec2d(const BStar::Point& c){
		return Vec2d(c.x, c.y);
	}
}

double BStar::heuristic_cost_estimate(const Point& start, const Point& goal)const{

		double slop = _slops(start.x, start.y);
		double distance = Vec2d::distance(vec2d(start), vec2d(goal));
		double obsticle = _walls(start.x, start.y)==ObsMap::ST_BLOCKED ? 100000 : 0;
		double alts_delta = fabs( _alts(start.x, start.y) - _alts(goal.x, goal.y) );

		return obsticle
			  + _params.w_distance	*	distance
			  + _params.w_slop		*	slop
			  + _params.w_alt_delta	*	alts_delta
		;
	}
double BStar::cost_estimate(const Point& current, const Point& neighbor)const{
		double slop = _slops(neighbor.x, neighbor.y);
		double distance = Vec2d::distance(vec2d(current), vec2d(neighbor));
		double obsticle = _walls(neighbor.x, neighbor.y)==ObsMap::ST_BLOCKED ? 100000 : 0;

		return obsticle
			  + _params.w_distance	*	distance
			  + _params.w_slop		*	slop
		;
	}

namespace{
	int find(const BStar::Path& p, const BStar::Point& a){
		for(size_t i=0; i<p.size(); i++){
			if(p[i].x==a.x && p[i].y==a.y) return (int)i;
		}
		return -1;
	}
}

namespace{
	void print(string text, string tag="INFO"){
		cout<<"B*: ["<<tag<<"] "<<text<<endl;
	}
#if ( PRINT_SEARCH_INFO==1 )
	#define P(TEXT){ stringstream sss; sss<<TEXT; print(sss.str()); }
#else
	#define P(TEXT)
#endif

	ostream& operator<<(ostream& o, const QTNode::XY& v){
		return o<<"{"<<v.x<<","<<v.y<<"}";
	}
	ostream& operator<<(ostream& o, const vector<QTNode::XY>& v){
		o<<"points ("<<v.size()<<"): ";
		for(size_t i=0;i<v.size();i++) o<<v[i]<<" ";
		return o;
	}
}

BStar::Path BStar::search(size_t sx, size_t sy, size_t gx, size_t gy){
	 BStar::QT qtRoot = _qtRoot;
	 Path path;
	 Point start = {sx,sy};
	 Point goal = {gx,gy};
	 vector<Point> goal_neigbors = qtRoot->getNeighborsPoints(goal);
	 QT qtStart = qtRoot->find(start.x, start.y);
	 QT qtGoal = qtRoot->find(goal.x, goal.y);
	 if(qtStart == qtGoal){
		 path.push_back(start); path.push_back(goal);
		 return path;
	 }
	 set<Point, lowest_first> closedset;    					// The set of nodes already evaluated.
	 QTComparison comparison(*this);
	 priority_queue<Point,vector<Point>, QTComparison> openset(comparison);
	 map<Point,Point> came_from;    							// The map of navigated nodes.

	 map<Point,double> g_score;    								// Cost from start along best known path.
	 f_score = map<Point,double>();

	 g_score[start]=0;
	 // Estimated total cost from start to goal .
	 f_score[start] = g_score[start] + heuristic_cost_estimate(start, goal);
	 openset.push(start);    									// The set of tentative nodes to be evaluated, initially containing the start node
	 P("openset <- "<<start)

	 while( openset.empty()==false ){
		 Point current  =  openset.top(); 						// the node in openset having the lowest f_score[] value
		 P("openset -> "<<current)
		 if( current == goal ){
			 P("reconstruct_path")
			 return reconstruct_path(came_from, goal);
		 }

		 openset.pop(); 										// remove current from openset
		 closedset.insert(current); 							// add current to closedset
		 P("openset -> closedset")

		 vector<Point> neighbor_nodes= qtRoot->getNeighborsPoints(current);
		 P("nei_nodes of "<<current<<": "<<neighbor_nodes)
		 if(find(goal_neigbors, current)>=0){
			 P("select goal as neighbor")
			 neighbor_nodes.push_back(goal);
		 }
		 for(vector<Point>::const_iterator i_neighbor=neighbor_nodes.begin();i_neighbor!=neighbor_nodes.end();i_neighbor++){
			 Point neighbor = *i_neighbor;
			 if( closedset.find(neighbor)!=closedset.end() ){	// if node is closed, skip it.
				 continue;
			 }
			 double tentative_g_score = g_score[current] + cost_estimate(current,neighbor);
			 P("tentative score = g("<<current<<")="<<g_score[current]<<" + cost("<<current<<","<<neighbor<<")="<<cost_estimate(current,neighbor)<<" = "<<tentative_g_score)

			 bool not_in_openset = f_score.find(neighbor)==f_score.end();
			 if( not_in_openset || tentative_g_score <= g_score[neighbor] ){
				 came_from[neighbor] = current;
				 g_score[neighbor] = tentative_g_score;
				 // Estimated total cost from start to goal through neighbor.
				 f_score[neighbor] = g_score[neighbor] + heuristic_cost_estimate(neighbor, goal);
				 P("F score = g("<<neighbor<<")="<<g_score[neighbor]<<" + H("<<neighbor<<","<<goal<<")="<<heuristic_cost_estimate(neighbor, goal)<<" = "<<f_score[neighbor])
				 if(not_in_openset){
					 openset.push(neighbor);
					 P("openset <- "<<neighbor)
				 }
			 }
		 }
	 }
	 return Path();
}


BStar::Path BStar::reconstruct_path(map<Point,Point>& came_from, const Point& current_node){
	 if( came_from.find(current_node) != came_from.end() ){
		 Path p = reconstruct_path(came_from, came_from[current_node]);
		 p.push_back(current_node);
		 return p;
	 }else{
		 Path p; p.push_back(current_node);
		 return p;
	 }
}

