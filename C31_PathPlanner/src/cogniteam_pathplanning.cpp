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

#include "cogniteam_pathplanner_parameters.h"

using namespace std;

typedef ObsMap Map;

int BaseMap::map_id_counter = 0;




// -------------------------------- Inflator -------------------------------------

namespace __Inaflator_utils{
	using namespace std;
	bool contains( const vector<int>& tx, const vector<int>& ty, int x, int y ){
		for(size_t i=0;i<tx.size();i++){
			if(tx[i]==x && ty[i]==y) return true;
		}
		return false;
	}
	void addTemplatePoint( vector<int>& tx, vector<int>& ty, int x, int y ){
		if(!contains(tx, ty, x, y)){
			tx.push_back(x); ty.push_back(y);
		}
	}
}
Inflator::Inflator(size_t radius, char sbv)
: radius(radius), cellBlockValue(sbv)
{
	using namespace __Inaflator_utils;
	
	//TODO: MUST UNIQUE ON (tx,ty) => INCREACE SPEED OF INFLATION
	//cout<<"Inflator("<<radius<<","<<(int)sbv<<")"<<endl;
	int rr = radius*radius;
	for(int x = -(int)radius; x<=(int)radius; x++){
		int y = (int)round(sqrt( rr - x*x ));
		addTemplatePoint(tx,ty,  x, y);
		addTemplatePoint(tx,ty,  x,-y);
		addTemplatePoint(tx,ty,  y, x);
		addTemplatePoint(tx,ty, -y, x);
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

Map MapEditor::cut(const Map& source, const char s, const char o)const{
	Map res ( source );
	for( size_t y=0; y<source.h(); y++){for( size_t x=0; x<source.w(); x++){
		if(res(x,y)==s) res(x,y)=s; else res(x,y)=o;
	}}
	return res;
}

Map MapEditor::replace(const Map& source, const char from, const char to)const{
	Map res ( source );
	for( size_t y=0; y<source.h(); y++){for( size_t x=0; x<source.w(); x++){
		if(res(x,y)==from) res(x,y)=to;
	}}
	return res;
}

ObsMap MapEditor::	merge(const ObsMap& m1, const ObsMap& m2, MergeOperator op)const{
	if(m1.w()!=m2.w() || m1.h()!=m2.h()){
		cout<<"ERROR: MapEditor::merge: dimensions of maps are different"<<endl;
		return Map(m1);
	}
	Map res ( m1 );
	for( size_t y=0; y<m1.h(); y++){for( size_t x=0; x<m1.w(); x++){
		if(
			( m1(x,y)==Map::ST_UNCHARTED && m2(x,y)!=Map::ST_UNCHARTED ) ||
			( m2(x,y)==Map::ST_UNCHARTED && m1(x,y)!=Map::ST_UNCHARTED ) ||
			( m2(x,y)==Map::ST_UNCHARTED && m1(x,y)==Map::ST_UNCHARTED )
		){
			res(x,y) = m2(x,y)==Map::ST_UNCHARTED? m1(x,y) : m2(x,y);
		}else{
			switch(op){
			case OR:
				res(x,y) = m1(x,y)==Map::ST_BLOCKED || m2(x,y)==Map::ST_BLOCKED ? Map::ST_BLOCKED : Map::ST_AVAILABLE;
				break;
			case AND:
				res(x,y) = m1(x,y)==Map::ST_BLOCKED && m2(x,y)==Map::ST_BLOCKED ? Map::ST_BLOCKED : Map::ST_AVAILABLE;
				break;
			case XOR:
				res(x,y) = m1(x,y)!=Map::ST_BLOCKED != m2(x,y)!=Map::ST_BLOCKED ? Map::ST_BLOCKED : Map::ST_AVAILABLE;
				break;
			}
		}
	}}
	return res;
}

Map MapEditor::coloring(const Map& source, size_t x, size_t y, const char av, const char bl)const{
	//ALLOCATE MEMORY FOR COLORED MAP AND FOR VISITED/UNVISITED FLAGES
	Map visited(source.w(), source.h());
	Map res(source.w(), source.h());
	//START COLORING PROCESS
	coloring(source, x, y, source(x,y), av, bl, visited, res);
	//COLOR ALL UNVISITED CELLS IN RESULT MAP AS BLOCKED
	for( size_t y=0; y<source.h(); y++){for( size_t x=0; x<source.w(); x++){
		if(visited(x,y)==Map::ST_UNCHARTED) res(x,y)=bl;
	}}
	return res;
}
void MapEditor::coloring(const Map& source, size_t x, size_t y, const char c, const char av, const char bl, Map& visited, Map& res)const{
	//IF CURRENT CELL IS VISITED RETURN
	if(visited(x,y)==Map::ST_BLOCKED) return;
	//IF CURRETN CELL IS UNVISITED => SELECT IT AS VISITED
	visited(x,y) = Map::ST_BLOCKED;
	//IF CURRENT CELL HAS RIGHT COLOR (COLOR OF FIRST CELL, LIKELY AVALIABLE) DO
	if(c == source(x,y)){
		//IN RESULT MAP COLOR THE CELL AS AVALIABLE
		res(x,y)=av;
		//CONTINUE SPREAD TO ALL CONRNERS OF THE EARTH
		if(x>0)				coloring(source, x-1, y, c, av, bl, visited, res);
		if(x<source.w()-1)	coloring(source, x+1, y, c, av, bl, visited, res);
		if(y>0)				coloring(source, x, y-1, c, av, bl, visited, res);
		if(y<source.h()-1)	coloring(source, x, y+1, c, av, bl, visited, res);
	}
	//IF CURRENT CELL HAS NOT RIGHT COLOR (COLOR OF FIRST CELL, LIKELY AVALIABLE) DO
	else{
		//IN RESULT MAP COLOR THE CELL AS BLOCKED
		res(x,y)=bl;
	}
}

// -------------------------------------------------------------------------------


namespace {

	struct Pair{ Waypoint s, g;
		Pair(const Waypoint& ss, const Waypoint& ee):s(ss.x,ss.y),g(ee.x,ee.y){}
		Pair(const TransitWaypoint& ss, const TransitWaypoint& ee):s(ss.x,ss.y),g(ee.x,ee.y){}
		Pair(const Waypoint& ss, const TransitWaypoint& ee):s(ss.x,ss.y),g(ee.x,ee.y){}
		Pair(const TransitWaypoint& ss, const Waypoint& ee):s(ss.x,ss.y),g(ee.x,ee.y){}
	};

}

PField::Points searchPath(const Map& source_map, const Waypoint& start, const Waypoint& finish, const Constraints& constraints){
	using namespace std;
	PRINT_VERSION

	//cout<<"searchPath: "<<"Input map:"<<endl<<source_map<<endl;
	
	//TODO: PROCESS CONSTRAINTS PATH BEFORE INFLATION

	if( source_map(start.x, start.y)==Map::ST_BLOCKED || source_map(finish.x, finish.y)==Map::ST_BLOCKED ){
		cout<<"searchPath: "<<"some of interesting points are not available (before inflation)"<<endl;
		return PField::Points();
	}
	
	size_t rr = constraints.dimentions.radius;
	if( rr<constraints.dimentions.radius) rr++;
	
	Inflator i( rr , Map::ST_BLOCKED);
	MapEditor e;

	Map inflated_map = e.replace(
			i.inflate(source_map),
			Map::ST_UNCHARTED, Map::ST_AVAILABLE
		);

	if( inflated_map(start.x, start.y)==Map::ST_BLOCKED || inflated_map(finish.x, finish.y)==Map::ST_BLOCKED ){
		cout<<"searchPath: "<<"some of interesting points are not available (after inflation)"<<endl;
		return PField::Points();
	}

	Map map = 
		e.coloring( 
			inflated_map,
			start.x, start.y, Map::ST_AVAILABLE,Map::ST_BLOCKED
		);
	
	//cout<<"searchPath: "<<"Inflated map:"<<endl<<map<<endl;

	BWQTNode qt(0,map.w()-1, 0, map.h()-1, map);
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
		vector<BWQTNode::XY> points;
		if( qt_path.size()>0 ) points = QTPath(qt_path).extractPoints( SEGMENT );
		for( size_t p=0; p<points.size(); p++ ){
			path.push_back(Waypoint(points[p].x,points[p].y));
		}
		#undef SEGMENT
	}

	PField::SmoothingParameters pf_params;
	SET_PF_PARAMETERS(pf_params)

	PField pf(map, path);
	PField::Points smoothed_path = pf.smooth(pf_params);
	
	return smoothed_path;
}

PField::Points searchPath_transitAccurate(const Map& source_map, const Waypoint& start, const Waypoint& finish, const Constraints& constraints){
	using namespace std;
	PRINT_VERSION

	//TODO: PROCESS CONSTRAINTS PATH BEFORE INFLATION

	if( source_map(start.x, start.y)==Map::ST_BLOCKED || source_map(finish.x, finish.y)==Map::ST_BLOCKED ){
		cout<<"searchPath: "<<"some of interesting points are not available (before inflation)"<<endl;
		return PField::Points();
	}

	size_t rr = constraints.dimentions.radius;
	if( rr<constraints.dimentions.radius) rr++;

	Inflator i( rr , Map::ST_BLOCKED);
	MapEditor e;

	Map inflated_map = e.replace(
			i.inflate(source_map),
			Map::ST_UNCHARTED, Map::ST_AVAILABLE
		);

	if( inflated_map(start.x, start.y)==Map::ST_BLOCKED || inflated_map(finish.x, finish.y)==Map::ST_BLOCKED ){
		cout<<"searchPath: "<<"some of interesting points are not available (after inflation)"<<endl;
		return PField::Points();
	}

	Map map =
		e.coloring(
			inflated_map,
			start.x, start.y, Map::ST_AVAILABLE,Map::ST_BLOCKED
		);

	BWQTNode qt(0,map.w()-1, 0, map.h()-1, map);
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

	PField::Points g_smoothed_path;
	AStar a_star;
	for( size_t i=0; i<segments.size(); i++){
		Path path;
		#define SEGMENT segments[i].s.x, segments[i].s.y,  segments[i].g.x,segments[i].g.y
		AStar::Path qt_path = a_star.search( SEGMENT , &qt);
		vector<BWQTNode::XY> points;
		if( qt_path.size()>0 ) points = QTPath(qt_path).extractPoints( SEGMENT );
		for( size_t p=0; p<points.size(); p++ ){
			path.push_back(Waypoint(points[p].x,points[p].y));
		}
		#undef SEGMENT

		PField::SmoothingParameters pf_params;
		SET_PF_PARAMETERS(pf_params)

		PField pf(map, path);
		PField::Points smoothed_path = pf.smooth(pf_params);

		append(g_smoothed_path, smoothed_path);
	}

	return g_smoothed_path;
}

#define EmptyPath PField::Points()

PField::Points searchPath(
		const AltMap& alts, const AltMap& slops, const AltMap& costs, const Map& s_walls,
		const Waypoint& start, const Waypoint& finish, const Constraints& constraints
){
	using namespace std;
	PRINT_VERSION

	//TODO: PROCESS CONSTRAINTS PATH BEFORE INFLATION

	if( s_walls(start.x, start.y)==Map::ST_BLOCKED || s_walls(finish.x, finish.y)==Map::ST_BLOCKED ){
		cout<<"searchPath: "<<"some of interesting points are unattainable (before inflation)"<<endl;
		return EmptyPath;
	}

	size_t rr = constraints.dimentions.radius;
	if( rr<constraints.dimentions.radius) rr++;

	Inflator i( rr , Map::ST_BLOCKED);
	MapEditor e;

	Map inflated_map = e.replace(
			i.inflate(s_walls),
			Map::ST_UNCHARTED, Map::ST_AVAILABLE
		);

	if( inflated_map(start.x, start.y)==Map::ST_BLOCKED || inflated_map(finish.x, finish.y)==Map::ST_BLOCKED ){
		cout<<"searchPath: "<<"some of interesting points are unattainable (after inflation)"<<endl;
		return EmptyPath;
	}

	Map walls =
		e.coloring(
			inflated_map,
			start.x, start.y, Map::ST_AVAILABLE,Map::ST_BLOCKED
		);

	AltMap map = costs;

	QTNode qt(0,map.w()-1, 0, map.h()-1, map);
	qt.folding();

	// CHECK IF ALL INTERESTING POINTS (start, stop and transits) ARE AVAILABLE

	if( walls(start.x, start.y)==ObsMap::ST_BLOCKED || walls(finish.x, finish.y)==ObsMap::ST_BLOCKED ){
		cout<<"searchPath: "<<"some of interesting points are unattainable"<<endl;
		return EmptyPath;
	}
	if(constraints.transits.size()!=0){
		for( size_t i=0 ; i<constraints.transits.size(); i++ ){
			if( walls(constraints.transits[i].x, constraints.transits[i].y)==ObsMap::ST_BLOCKED ){
				cout<<"searchPath: "<<"some of transit points are unattainable"<<endl;
				return EmptyPath;
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
		for(; i<constraints.transits.size()-1; i++){
			segments.push_back(Pair(constraints.transits[i], constraints.transits[i+1]));
		}
		segments.push_back(Pair(constraints.transits[i], finish));
	}

	// CREATE PATH

	Path path;
	BStarParameters bstarparams;
	BStar b_star(&qt, alts, slops, walls, bstarparams);
	for( size_t i=0; i<segments.size(); i++){
		#define SEGMENT segments[i].s.x, segments[i].s.y,  segments[i].g.x,segments[i].g.y

		BStar::Path qt_path = b_star.search( SEGMENT );
		vector<QTNode::XY> points = qt_path;
		for( size_t p=0; p<points.size(); p++ ){
			path.push_back(Waypoint(points[p].x,points[p].y));
		}
		#undef SEGMENT
	}
#ifdef DO_SMOOTHING
	PField::SmoothingParameters pf_params;
	SET_PF_PARAMETERS(pf_params)

	PField pf(s_walls, path);
	PField::Points smoothed_path = pf.smooth(pf_params);

	return smoothed_path;
#else
	PField::Points smoothed_path;
	for( size_t p=0; p<path.size(); p++ )
		smoothed_path.push_back(Vec2d(path[p].x,path[p].y));
	return smoothed_path;
#endif
}

PField::Points searchPath_transitAccurate(
		const AltMap& alts, const AltMap& slops, const AltMap& costs, const Map& s_walls, const Map& s_obstacles, const Map& s_terrain,
		const Waypoint& start, const Waypoint& finish, const Constraints& constraints, Map& o_obstacles
){
	using namespace std;
	PRINT_VERSION

	//TODO: PROCESS CONSTRAINTS PATH BEFORE INFLATION

	if( s_walls(start.x, start.y)==Map::ST_BLOCKED || s_walls(finish.x, finish.y)==Map::ST_BLOCKED ){
		cout<<"searchPath: "<<"some of interesting points are unattainable (before inflation)"<<endl;
		return EmptyPath;
	}

	size_t rr = constraints.dimentions.radius;
	if( rr<constraints.dimentions.radius) rr++; // Because radius is float and rr is int

	Inflator i( rr, Map::ST_BLOCKED);
	Inflator ii( rr*0.5, Map::ST_DEBREES);
	MapEditor e;

	Map walls = e.merge(s_walls, s_obstacles, MapEditor::OR);

	Map inflated_map = e.replace(
			i.inflate(walls),
			Map::ST_UNCHARTED, Map::ST_AVAILABLE
		);
	Map inflated_terrain = e.replace(e.cut(
			ii.inflate(s_terrain),
			Map::ST_DEBREES, Map::ST_AVAILABLE), Map::ST_DEBREES, Map::ST_BLOCKED
		);

	inflated_map = e.merge(inflated_map, inflated_terrain, MapEditor::OR);

	o_obstacles = inflated_map;

	if( inflated_map(start.x, start.y)==Map::ST_BLOCKED || inflated_map(finish.x, finish.y)==Map::ST_BLOCKED ){
		cout<<"searchPath: "<<"some of interesting points are unattainable (after inflation)"<<endl;
		return EmptyPath;
	}

	walls =
		e.coloring(
			inflated_map,
			start.x, start.y, Map::ST_AVAILABLE,Map::ST_BLOCKED
		);

	AltMap map = costs;

	QTNode qt(0,map.w()-1, 0, map.h()-1, map);
	qt.folding();

	// CHECK IF ALL INTERESTING POINTS (start, stop and transits) ARE AVAILABLE

	if( walls(start.x, start.y)==ObsMap::ST_BLOCKED || walls(finish.x, finish.y)==ObsMap::ST_BLOCKED ){
		cout<<"searchPath: "<<"some of interesting points are unattainable"<<endl;
		return EmptyPath;
	}
	if(constraints.transits.size()!=0){
		for( size_t i=0 ; i<constraints.transits.size(); i++ ){
			if( walls(constraints.transits[i].x, constraints.transits[i].y)==ObsMap::ST_BLOCKED ){
				cout<<"searchPath: "<<"some of transit points are unattainable"<<endl;
				return EmptyPath;
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
		for(; i<constraints.transits.size()-1; i++){
			segments.push_back(Pair(constraints.transits[i], constraints.transits[i+1]));
		}
		segments.push_back(Pair(constraints.transits[i], finish));
	}

	// CREATE PATH

	PField::Points g_smoothed_path;
	Path g_smoothed;
	BStarParameters bstarparams;
	BStar b_star(&qt, alts, slops, walls, bstarparams);
	for( size_t i=0; i<segments.size(); i++){
		Path path;
		#define SEGMENT segments[i].s.x, segments[i].s.y,  segments[i].g.x,segments[i].g.y

		BStar::Path qt_path = b_star.search( SEGMENT );

		vector<QTNode::XY> points = qt_path;
		for( size_t p=0; p<points.size(); p++ ){
			path.push_back(Waypoint(points[p].x,points[p].y));
		}

		#undef SEGMENT

#ifdef DO_SMOOTHING
		PField::SmoothingParameters pf_params;
		SET_PF_PARAMETERS(pf_params)

		PField pf(s_walls, path);
		PField::Points smoothed_path = pf.smooth(pf_params);

#else
		PField::Points smoothed_path;
		for( size_t p=0; p<path.size(); p++ )
			smoothed_path.push_back(Vec2d(path[p].x,path[p].y));
#endif
		append(g_smoothed_path, smoothed_path);
	}

	return g_smoothed_path;
}

#undef EmptyPath






#include "cogniteam_pathplanning_tests.cpp"

