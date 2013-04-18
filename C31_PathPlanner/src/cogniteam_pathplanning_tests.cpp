/*####
#
#   THIS FILE INCLIDED IN cogniteam_pathplanning.cpp
#
####*/

int cogniteam_pathplanning_test_map_inflation(int argc, char** argv) {
	cout << "START" << endl; // prints PP

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
#include "MapFileReader.hpp"
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
	
	
	vector<char> map_from_file;
	if(argc>5){
		cout<<"map from file: "<<argv[5]<<endl;
		map_from_file = map_file_reader::readMap(argv[5], w, h);
		cmap = map_from_file.data();
	}
	
	
	Map map(w, h, cmap);
	cout<<"map source"<<endl<<map<<endl;
	
	long _sx=atoi(argv[1]), _sy=atoi(argv[2]);
	long _ex=atoi(argv[3]), _ey=atoi(argv[4]);
	long _rr=atoi(argv[6]);
	
	#define START_P _sx,_sy 
	#define END_P   _ex,_ey
	
	#define POINTS START_P, END_P
	
	printf("Robot radius : %i celles \n", _rr);
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
	
	Inflator i( _rr , Map::ST_BLOCKED);
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
	SET_PARAMETERS(pf_params)
	
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
// 	cout<<"TIMES:"<<endl;
// 	cout<<"   total : "<<STR_TIME(start_time)<<endl;
// 	cout<<"   inflation : "<<STR_TIME(time_inflator)<<endl;
// 	cout<<"   quad tree : "<<STR_TIME(time_qt)<<endl;
// 	cout<<"   A* : "<<STR_TIME(time_astar)<<endl;
// 	cout<<"   smoothing : "<<STR_TIME(time_smoothing)<<endl;
	
	cout << endl << "END" << endl; // prints PP
	return 0;
}


//THIS FUNCTION IS EXACT COPY OF searchPath function in cogniteam_pathplanning.cpp WITH ADDITIONAL DEBUG INFORMATION OUTPUT
Path test_searchPath(const Map& source_map, const Waypoint& start, const Waypoint& finish, const Constraints& constraints){
	using namespace std;

	//TODO: PROCESS CONSTRAINTS PATH BEFORE INFLATION

	if( source_map(start.x, start.y)==Map::ST_BLOCKED || source_map(finish.x, finish.y)==Map::ST_BLOCKED ){
		cout<<"searchPath: "<<"some of interesting points are not available (before inflation)"<<endl;
		return Path();
	}

	size_t rr = constraints.dimentions.radius;
	if( rr<constraints.dimentions.radius) rr++;

	Inflator i( rr , Map::ST_BLOCKED);
	MapEditor e;

	Map map =
		e.coloring(
			e.replace(
				i.inflate(source_map),
				Map::ST_UNCHARTED, Map::ST_AVAILABLE
			),
			start.x, start.y, Map::ST_AVAILABLE,Map::ST_BLOCKED
		);

	QTNode qt(0,map.w()-1, 0, map.h()-1, map);
	qt.folding();

	// CHECK IF ALL INTERESTING POINTS (start, stop and transits) ARE AVAILABLE

	if( !qt.findEmpty(start.x, start.y) || !qt.findEmpty(finish.x, finish.y) ){
		cout<<"searchPath: "<<"some of interesting points are not available"<<endl;
		return Path();
	}
	if(constraints.transits.size()!=0){
		for( size_t i=0 ; i<constraints.transits.size(); i++ ){
			if( !qt.findEmpty(constraints.transits[i].x, constraints.transits[i].y) ){
				cout<<"searchPath: "<<"some of transit points are not available"<<endl;
				return Path();
			}
		}
	}

	// CREATE SEGMENTS

	cout<<"CREATE SEGMENTS: "<<endl;
	vector<Pair> segments;
	if(constraints.transits.size()==0){
		segments.push_back(Pair(start, finish));
		cout<<"   "<<start.x<<","<<start.y<<" -- "<<finish.x<<","<<finish.y<<endl;
	}else{
		size_t i=0;
		segments.push_back(Pair(start, constraints.transits[i]));
		cout<<"   "<<start.x<<","<<start.y<<" -- "<<constraints.transits[i].x<<","<<constraints.transits[i].y<<endl;
		for(; i<constraints.transits.size()-1; i++){
			segments.push_back(Pair(constraints.transits[i], constraints.transits[i+1]));
			cout<<"   "<<constraints.transits[i].x<<","<<constraints.transits[i].y<<" -- "<<constraints.transits[i+1].x<<","<<constraints.transits[i+1].y<<endl;
		}
		segments.push_back(Pair(constraints.transits[i], finish));
		cout<<"   "<<constraints.transits[i].x<<","<<constraints.transits[i].y<<" -- "<<finish.x<<","<<finish.y<<endl;
	}

	// CREATE PATH

	Path path;
	AStar a_star;
	cout<<"Search A* paths for segments: "<<endl;
	for( size_t i=0; i<segments.size(); i++){
		#define SEGMENT segments[i].s.x, segments[i].s.y,  segments[i].g.x,segments[i].g.y
		cout<<"    "<<segments[i].s.x<<","<<segments[i].s.y<<" -- "<<segments[i].g.x<<","<<segments[i].g.y<<endl<<"    " "    ";
		AStar::Path qt_path = a_star.search( SEGMENT , &qt);
		vector<QTNode::XY> points;
		if( qt_path.size()>0 ) points = QTPath(qt_path).extractPoints( SEGMENT );
		for( size_t p=0; p<points.size(); p++ ){
			path.push_back(Waypoint(points[p].x,points[p].y));
			cout<<"("<<points[p].x<<","<<points[p].y<<") ";
		}
		cout<<endl;
		#undef SEGMENT
	}

	PField::SmoothingParameters pf_params;
	SET_PARAMETERS(pf_params)

	PField pf(map, path);
	PField::Points smoothed_path = pf.smooth(pf_params);

	cout<<"smoothed path: ";
	for( size_t i=0;i<smoothed_path.size(); i++){
		cout<<"("<<smoothed_path[i].x<<","<<smoothed_path[i].y<<") ";
	}
	cout<<endl;

	Path smoothed = pf.smoothWaypoints(pf_params);
	cout<<"smoothed path (rounded for grid): ";
	for( size_t i=0;i<smoothed.size(); i++){
		cout<<"("<<smoothed[i].x<<","<<smoothed[i].y<<") ";
	}
	cout<<endl;


	return smoothed;
}

Path test_searchPath_transitAccurate(const Map& source_map, const Waypoint& start, const Waypoint& finish, const Constraints& constraints){
	using namespace std;

	//TODO: PROCESS CONSTRAINTS PATH BEFORE INFLATION

	if( source_map(start.x, start.y)==Map::ST_BLOCKED || source_map(finish.x, finish.y)==Map::ST_BLOCKED ){
		cout<<"searchPath: "<<"some of interesting points are not available (before inflation)"<<endl;
		return Path();
	}

	size_t rr = constraints.dimentions.radius;
	if( rr<constraints.dimentions.radius) rr++;

	Inflator i( rr , Map::ST_BLOCKED);
	MapEditor e;

	Map map =
		e.coloring(
			e.replace(
				i.inflate(source_map),
				Map::ST_UNCHARTED, Map::ST_AVAILABLE
			),
			start.x, start.y, Map::ST_AVAILABLE,Map::ST_BLOCKED
		);

	cout<<"Inflated map: "<<endl<<map<<endl;

	QTNode qt(0,map.w()-1, 0, map.h()-1, map);
	qt.folding();
	cout<<"QT: "<<qt<<endl;

	// CHECK IF ALL INTERESTING POINTS (start, stop and transits) ARE AVAILABLE

	if( !qt.findEmpty(start.x, start.y) || !qt.findEmpty(finish.x, finish.y) ){
		cout<<"searchPath: "<<"some of interesting points are not available"<<endl;
		return Path();
	}
	if(constraints.transits.size()!=0){
		for( size_t i=0 ; i<constraints.transits.size(); i++ ){
			if( !qt.findEmpty(constraints.transits[i].x, constraints.transits[i].y) ){
				cout<<"searchPath: "<<"some of transit points are not available"<<endl;
				return Path();
			}
		}
	}

	// CREATE SEGMENTS

	cout<<"CREATE SEGMENTS: "<<endl;
	vector<Pair> segments;
	if(constraints.transits.size()==0){
		segments.push_back(Pair(start, finish));
		cout<<"   "<<start.x<<","<<start.y<<" -- "<<finish.x<<","<<finish.y<<endl;
	}else{
		size_t i=0;
		segments.push_back(Pair(start, constraints.transits[i]));
		cout<<"   "<<start.x<<","<<start.y<<" -- "<<constraints.transits[i].x<<","<<constraints.transits[i].y<<endl;
		for(; i<constraints.transits.size()-1; i++){
			segments.push_back(Pair(constraints.transits[i], constraints.transits[i+1]));
			cout<<"   "<<constraints.transits[i].x<<","<<constraints.transits[i].y<<" -- "<<constraints.transits[i+1].x<<","<<constraints.transits[i+1].y<<endl;
		}
		segments.push_back(Pair(constraints.transits[i], finish));
		cout<<"   "<<constraints.transits[i].x<<","<<constraints.transits[i].y<<" -- "<<finish.x<<","<<finish.y<<endl;
	}

	// CREATE PATH

	PField::Points g_smoothed_path;
	Path g_smoothed;
	AStar a_star;
	cout<<"Search A* paths for segments("<<segments.size()<<"): "<<endl;
	for( size_t i=0; i<segments.size(); i++){
		Path path;
		#define SEGMENT segments[i].s.x, segments[i].s.y,  segments[i].g.x,segments[i].g.y
		cout<<"    "<<segments[i].s.x<<","<<segments[i].s.y<<" -- "<<segments[i].g.x<<","<<segments[i].g.y<<endl;
		AStar::Path qt_path = a_star.search( SEGMENT , &qt);

		cout<<"... QT path: ";
		for( size_t p=0; p<qt_path.size(); p++ ){
			cout<<"("<<*(qt_path[p])<<") ";
		}cout<<endl;

		vector<QTNode::XY> points;
		if( qt_path.size()>0 ){
			points = QTPath(qt_path).extractPoints( SEGMENT );
		}

		cout<<"... extracted path: ";
		for( size_t p=0; p<points.size(); p++ ){
			path.push_back(Waypoint(points[p].x,points[p].y));
			cout<<"("<<points[p].x<<","<<points[p].y<<") ";
		}cout<<endl;

		#undef SEGMENT

		PField::SmoothingParameters pf_params;
		SET_PARAMETERS(pf_params)

		PField pf(map, path);
		PField::Points smoothed_path = pf.smooth(pf_params);

		cout<<"    " "smoothed path: ";
		for( size_t i=0;i<smoothed_path.size(); i++){
			cout<<"("<<smoothed_path[i].x<<","<<smoothed_path[i].y<<") ";
			g_smoothed_path.push_back(smoothed_path[i]);
		}
		cout<<endl;

		Path smoothed = pf.smoothWaypoints(pf_params);
		cout<<"    " "smoothed path (rounded for grid): ";
		for( size_t i=0;i<smoothed.size(); i++){
			cout<<"("<<smoothed[i].x<<","<<smoothed[i].y<<") ";
			g_smoothed.push_back(smoothed[i]);
		}
		cout<<endl;


	}

	cout<<"G smoothed path: ";
	for( size_t i=0;i<g_smoothed_path.size(); i++){
		cout<<"("<<g_smoothed_path[i].x<<","<<g_smoothed_path[i].y<<") ";
	}
	cout<<endl;

	cout<<"G smoothed path (rounded for grid): ";
	for( size_t i=0;i<g_smoothed.size(); i++){
		cout<<"("<<g_smoothed[i].x<<","<<g_smoothed[i].y<<") ";
	}
	cout<<endl;


	return g_smoothed;
}

int cogniteam_pathplanning_test_transits(int argc, char** argv) {
	cout << "START: cogniteam_pathplanning_test_transits" << endl; // prints PP

	char* cmap = 0;
	size_t w=0,h=0;

	vector<char> map_from_file;
	if(argc>5){
		cout<<"map from file: "<<argv[5]<<endl;
		map_from_file = map_file_reader::readMap(argv[5], w, h);
		cmap = map_from_file.data();
	}

	Map map(w, h, cmap);
	cout<<"map source"<<endl<<map<<endl;

	long _sx=atoi(argv[1]), _sy=atoi(argv[2]);
	long _ex=atoi(argv[3]), _ey=atoi(argv[4]);
	long _rr=atoi(argv[6]);

	#define START_P _sx,_sy
	#define END_P   _ex,_ey

	#define POINTS START_P, END_P

	printf("Robot radius : %i celles \n", _rr);
	printf("Plan path : from %i,%i to %i,%i \n", POINTS);

	if(map.inRange(END_P)==false){
		map.approximate(START_P, END_P);
	}

	Waypoint start(START_P), finish(END_P);
	RobotDimentions dimentions; dimentions.radius = _rr;
	Transits transits;
		TransitWaypoint wp1={36,48}; cout<<"add transit: "<<wp1.x<<","<<wp1.y<<endl;
		TransitWaypoint wp2={10,53}; cout<<"add transit: "<<wp2.x<<","<<wp2.y<<endl;
		transits.push_back(wp1);
		transits.push_back(wp2);
	Attractors attractors;
	Constraints con(dimentions, transits, attractors);


	Path smoothed = test_searchPath_transitAccurate(map, start, finish, con);

	for( size_t i=0;i<smoothed.size(); i++){
		map(smoothed[i].x, smoothed[i].y)='o';
	}
	for( size_t i=0;i<transits.size(); i++){
		if(map(transits[i].x, transits[i].y)=='o')
			map(transits[i].x, transits[i].y)='T';
		else
			map(transits[i].x, transits[i].y)='t';
	}
	{
		if(map(start.x, start.y)=='o')
			map(start.x, start.y)='S';
		else
			map(start.x, start.y)='s';
	}
	{
		if(map(finish.x, finish.y)=='o')
			map(finish.x, finish.y)='G';
		else
			map(finish.x, finish.y)='g';
	}

	cout<<endl;
	cout<<"map with smoothed path"<<endl<<map<<endl;

}


