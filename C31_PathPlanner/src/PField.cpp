

#include "PField.h"

using namespace std;
typedef vector<Vec2d> Points;

#define DISPLAY_VIEW_FRAME 0
#define DISPLAY_ATTRACTORS 0
#define DISPLAY_REPULSORS  0
#define DISPLAY_FORCES  0

namespace {
	
	
	inline bool inMapRange(const Vec2d& v, const Map& map){
		long x = round(v.x); if(x<0) return false; if(x>=map.w())return false;
		long y = round(v.y); if(y<0) return false; if(y>=map.h())return false;
		return true;
	}

	inline Waypoint convert(const Vec2d& v, const Map& map){
		long x = round(v.x); if(x<0) x=0; if(x>=map.w())x=map.w()-1;
		long y = round(v.y); if(y<0) y=0; if(y>=map.h())y=map.h()-1;
		return Waypoint(x, y);
	}
	inline Vec2d convert(const Waypoint& w, const Map& map){
		return Vec2d(w.x, w.y);
	}
	
	Path convert( const vector<Vec2d>& points , const Map& map){
		Path path;
		
		for(size_t i=0;i<points.size();i++){
			path.push_back(convert(points[i], map));
		}
		
		return path;
	}
	bool equals(const Waypoint& w, const Waypoint& q){ return w.x==q.x && w.y==q.y; }
	vector<Vec2d> convert( const Path& path , const Map& map){
		vector<Vec2d> points;
		int j=-1;
		
		for(size_t i=0;i<path.size();i++){
			if(!(i<0 || equals(path[j],path[i])==false)) continue;
			points.push_back(convert(path[i], map));
			j=i;
		}
		
		return points;
	}
}

Path PField::smooth(const SmoothingParameters& in_params)const{
	SmoothingParameters params = in_params;
	params.repulsorType = RT_R1;
	params.attractorType = AT_A1;
	params.maxIterationNumber = (long)round(opath.size()/params.stepRate);
	
	Points points = simulate(params);
	
	cout<<"smoothed "<<endl;
	for(size_t i=0;i<points.size();i++){
		cout<<points[i].x<<"\t"<<points[i].y<<endl;
	}
	
	Points reduced = reducePath(points,params);
	cout<<"reduced "<<endl;
	for(size_t i=0;i<reduced.size();i++){
		cout<<reduced[i].x<<"\t"<<reduced[i].y<<endl;
	}
	
	return convert(reduced, map);
}

struct Position{
	Vec2d loc;
	double heading;
	Position(Vec2d loc, double v):loc(loc),heading(v){}
	Vec2d delta(double ang, double dist){ return Vec2d::poliar(heading+ang+PI05,dist); }
	void move(double ang, double dist){
		loc=loc+delta(ang,dist);
		heading += ang;
	}
};

namespace {
	inline Vec2d ellipse(double a, double F, double S){
		return Vec2d(S*cos(a), F*cos(a));
	}
	inline double MOD(double a, double b){
		double e = a/b;
		double f = (long)e;
		return e*(e-f);
	}
	Points frontiers( const Points& points, const Vec2d& loc ){
		const int k = 180;
		const double block = PI2/k;
		Vec2d blocks[k];
		for(size_t i=0;i<k;i++) blocks[i] = Vec2d::Null();
		for(size_t i=0;i<points.size();i++){
			Vec2d local_i = points[i]-loc;
			double a = local_i.ang()+PI;
			size_t aa = (long)(a/block);
			if(blocks[aa].isNull || blocks[aa].len()>local_i.len()){
				blocks[aa]=local_i; 
			}
		}
		Points res;
		for(size_t i=0;i<k;i++){
			if(blocks[i].isNull)continue;
			blocks[i] = blocks[i]+loc;
			res.push_back(blocks[i]);
		}
		return res;
	}
	
	bool isFrontier(const Map& map, const Waypoint& wp){
		if(wp.x==0 || wp.y==0 || wp.x==map.w()-1 || wp.y==map.h()-1) return true;
		if( map(wp.x+1, wp.y)==Map::ST_AVAILABLE ) return true;
		if( map(wp.x-1, wp.y)==Map::ST_AVAILABLE ) return true;
		if( map(wp.x, wp.y+1)==Map::ST_AVAILABLE ) return true;
		if( map(wp.x, wp.y-1)==Map::ST_AVAILABLE ) return true;
		return false;
	}
	
	void searchIP(
		double viewRF, double viewRS, 
		const Position& pos, const Map& map, const Points& path, size_t cWP,
		Points& repulsors, Points& attractors
	)
	{
		Points repulsors_tmp;
		if(DISPLAY_VIEW_FRAME) cout<<"view frame"<<endl;
		for(long y=0;y<=viewRF;y++){
			for(long x=-viewRS;x<=viewRS;x++){
				Vec2d xy = Vec2d(x,y).rotate(pos.heading)+pos.loc;
				if(DISPLAY_VIEW_FRAME) cout<<xy.x<<'\t'<<xy.y<<endl;
				if(inMapRange(xy,map)==false) continue;
				Waypoint wp_xy = convert(xy,map);
				if(map(wp_xy.x, wp_xy.y)==Map::ST_BLOCKED && isFrontier(map, wp_xy)){
					repulsors_tmp.push_back(xy);
				}
			}
		}
		size_t i=cWP;
		while(i<path.size()-1){
			//cout<<"ignore path["<<i<<"] dist to pos = "<<Vec2d::distance(path[i],pos.loc)<<endl;
			if(Vec2d::distance(path[i],pos.loc)<1){ i++; continue; }
			break;
		}
		//cout<<"start from path["<<i<<"]  pos = "<<pos.loc<<","<<Vec2d::r2d(pos.heading)<<"deg"<<endl;
		Vec2d vwp = (path[i]-pos.loc).rotate(-pos.heading);
		//cout<<"vwp = "<<vwp.x<<'\t'<<vwp.y<<endl;
		const double E = 0.5;
		#define WAYPOINT_OUT_OF_VIEW ( fabs(vwp.x)<=viewRS+E && (vwp.y>-E && vwp.y<=viewRF+E) )
		#define NOT_END_OF_PATH (i<path.size())
		#define ATTRACTORS_EMPTY attractors.size()==0
		while( NOT_END_OF_PATH  && ( WAYPOINT_OUT_OF_VIEW || ATTRACTORS_EMPTY ) ){
			//cout<<"att added "<<path[i]<<endl;
			attractors.push_back(path[i]);
			i++;
			vwp = (path[i]-pos.loc).rotate(-pos.heading);
			//cout<<"vwp = "<<vwp.x<<'\t'<<vwp.y<<endl;
		}
		//cout<<"finished: "<<(i<path.size())<<", "<<(fabs(vwp.x)<=viewRS)<<", "<<(vwp.y>-0.5)<<(vwp.y<=viewRF)<<endl;
		
		repulsors = frontiers(repulsors_tmp, pos.loc);
		#undef WAYPOINT_OUT_OF_VIEW
		#undef NOT_END_OF_PATH
		#undef ATTRACTORS_EMPTY
	}
	
	size_t searchOnPathPosition(const Position& pos, const Points& path){
		size_t nearest = 0;
		double min_dist = Vec2d::distance(pos.loc, path[0]);
		for(size_t i=0;i<path.size();i++){
			double dist = Vec2d::distance(pos.loc, path[i]);
			if(dist <= min_dist){
				//cout<<"searchOnPathPosition: mid dist="<<min_dist<<", dist="<<dist<<", wp="<<path[i]<<", loc="<<pos.loc<<endl;
				min_dist = dist;
				nearest = i;
			}
		}
		if(nearest == path.size()-1) return nearest;
		Vec2d A = pos.loc, B = path[nearest], C = path[nearest+1];
		Vec2d b = B-C, c = C-A, a = B-A;
		//http://en.wikipedia.org/wiki/Law_of_cosines
		double G = acos( (B-C).dot(A-C)/(b.len()*c.len()) );
		if( G<PI ) nearest++;
		return nearest;
	}
}

PField::Points PField::simulate(const SmoothingParameters& params) const{

	const double viewRF = params.viewRadiusForward;
	const double viewRS = params.viewRadiusSide;
	const RepulsorType rt = params.repulsorType;
	const AttractorType at = params.attractorType;
	
	const double stepF = params.stepRate;
	const int stepNumbers = params.maxIterationNumber*2;
	const double inertia = params.inertia;
	
	Points opath_points = convert(opath, map);
	Points smoothed_path;
	cout<<"path size reduced from "<<opath.size()<<" to "<<opath_points.size()<<endl;
	if(opath.size()<2) return opath_points;
	
	Vec2d last = opath_points[opath_points.size()-1];
	Vec2d prevlast = opath_points[opath_points.size()-2];
	Vec2d virtLast = prevlast + (last-prevlast).addLen(0.6);
	opath_points.push_back(virtLast);
	
	size_t start_pos=0;
	
	cout<<"start pos = "<<start_pos<<" = "<<opath_points[start_pos]<<endl;
	
	Position pos ( opath_points[start_pos], 
				   (opath_points[start_pos+1]-opath_points[start_pos]).angY()
			 );

	size_t iteration = 0;
	Points trac;
	while(true){iteration++;
		
		if(DISPLAY_FORCES){ 
			cout<<"iteration = "<<iteration<<endl;
			cout<<"POS "<<pos.loc.x<<'\t'<<pos.loc.y<<'\t'<<pos.loc.heading<<'\t'<<start_pos<<endl;
		}
		
		smoothed_path.push_back(pos.loc);
		
		if(iteration==stepNumbers || start_pos == opath_points.size()-1){
			break;
		}

		
		Points repulsors;
		Points attractors;
		searchIP(viewRF, viewRS, pos, map, opath_points, start_pos, repulsors, attractors);
		
		if(DISPLAY_ATTRACTORS){
			cout<<"attractors"<<endl;
			for(size_t i=0;i<attractors.size();i++){
				Vec2d xy = attractors[i];
				cout<<xy.x<<'\t'<<xy.y<<endl;
			}
		}
		if(DISPLAY_REPULSORS){
			cout<<"repulsors"<<endl;
			for(size_t i=0;i<repulsors.size();i++){
				Vec2d xy = repulsors[i];
				cout<<xy.x<<'\t'<<xy.y<<endl;
			}
		}

		
		Vec2d force(0,0.1);
		for(size_t i=0;i<attractors.size();i++){
			Vec2d v = attractors[i] - pos.loc;
			if(at==AT_A1){
				Vec2d f = Vec2d::poliar(v.ang(), 1/pow(v.len()/viewRF,2));
				if(DISPLAY_FORCES){ 
					cout<<"att="<<attractors[i]<<",v="<<v<<", f="<<f<<", ang="<<Vec2d::r2d(v.ang())<<"deg, len="<<v.len()<<", viewRF="<<viewRF<<", len/FR="<<(v.len()/viewRF)<<", f.a="<<Vec2d::r2d(f.ang())<<"deg, pow="<<f.len()<<endl;
				}
				force = force + f;
			}
		}
		for(size_t i=0;i<repulsors.size();i++){
			Vec2d v = repulsors[i] - pos.loc;
			Vec2d ell = ellipse(v.ang(), viewRF, viewRS).rotate(pos.heading);
			if(rt==RT_R1){
				Vec2d f = Vec2d::poliar(PI+v.ang(), 1/pow(v.len()/ell.len(),2));
				if(DISPLAY_FORCES){ 
					cout<<"rep="<<repulsors[i]<<",v="<<v<<", f="<<f<<", ang="<<Vec2d::r2d(v.ang())<<"deg, len="<<v.len()<<", viewRF="<<viewRF<<", |ell|="<<ell.len()<<", len/|ell|="<<(v.len()/ell.len())<<", f.a="<<Vec2d::r2d(f.ang())<<"deg pow="<<f.len()<<endl;
				}
				force = force + f;
			}
		}
		
		force = force + pos.delta(0,inertia);
		
		double delta_heading = force.rotate(-pos.heading).angY();
		double power = 1;//force.len();
		
		if(DISPLAY_FORCES){
			cout<<"force = "<<force<<", ang="<<Vec2d::r2d(force.ang())<<"deg, len="<<force.len()<<", dh="<<Vec2d::r2d(delta_heading)<<"deg, power="<<power<<", cur.heading="<<Vec2d::r2d(pos.heading)<<"deg"<<endl;
		}
		
		if(DISPLAY_FORCES){ cout<<"move: "<<pos.loc<<":"<<Vec2d::r2d(pos.heading)<<"d -> "; }
		pos.move(delta_heading*stepF, power*stepF);
		if(DISPLAY_FORCES){ cout<<pos.loc<<":"<<Vec2d::r2d(pos.heading)<<"d"<<endl;}
		
		start_pos = searchOnPathPosition(pos, opath_points);
	}
	
	smoothed_path.push_back(opath_points[opath_points.size()-1]);
	
	cout<<"END OF SMOOTHING"<<endl;
	return smoothed_path;
	
}

PField::Points PField::reducePath(const Points& path, const SmoothingParameters& params) const{
	if(path.size()<2) return path;
	Points rpath;
	double sum=0;
	double ang=0;
	double heading=(path[1]-path[0]).angY();
	rpath.push_back(path[0]);
	size_t lastAdded = 0;
	for(size_t i=1;i<path.size();i++){
		double dist = (path[i]-path[lastAdded]).len();
		double angle = (path[i]-path[i-1]).angY() - heading;
		heading=(path[i]-path[i-1]).angY();
		sum+=dist;
		ang+=angle;
		if(sum > params.distanceBetweenPoints || ang > params.maxAngleWhileReducing){
			lastAdded = i-1;
			rpath.push_back(path[lastAdded]);
			sum=dist;
			ang=angle;
		}
	}
	if(path.size()-1 != lastAdded) rpath.push_back(path[path.size()-1]);
	
	return rpath;
}






