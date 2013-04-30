

#include "PField.h"

using namespace std;
typedef vector<Vec2d> Points;

#define DISPLAY_VIEW_FRAME 0
#define DISPLAY_ATTRACTORS 0
#define DISPLAY_REPULSORS  0
#define DISPLAY_FORCES  0
#define DISPLAY_ORIGINAL_EXTENDED_PATH 0
#define DISPLAY_SMOOTHED_PATH 0
#define DISPLAY_SMOOTHED_REDUCED_PATH 0
#define DISPLAY_SMOOTHED_REDUCEDFIX_PATH 0
#define DISPLAY_INFO_FROM_SIMULATE 0
#define DISPLAY_INFO_FROM_SEARCHIP 0
#define DISPLAY_INFO_FROM_REDUCEPATH 0
#define DISPLAY_INFO_FROM_ADDPOINTS 0

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

Path PField::smoothWaypoints(const SmoothingParameters& in_params)const{
	return convert(smooth(in_params), map);
}
Points PField::smooth(const SmoothingParameters& in_params)const{
	SmoothingParameters params = in_params;
	if(opath.size()==0){
		cout<<"WARNING: path size is zero."<<endl;

		return convert(opath, map);
	}

	if(params.repulsorType==RT_ERROR)  params.repulsorType = RT_R1;
	if(params.attractorType==AT_ERROR) params.attractorType = AT_A1;
	//if(params.maxIterationNumber==0) params.maxIterationNumber = (long)round(2*opath.size()/params.stepRate);
	
	if(params.notDefined()){
		cout<<"WARNING: Some of Smoothing Parameters are not defined"<<endl;
		
		return convert(opath, map);
	}
	
	Points points = simulate(params);
	
	#if DISPLAY_SMOOTHED_PATH == 1
		cout<<"smoothed "<<endl;
		for(size_t i=0;i<points.size();i++){
			cout<<points[i].x<<"\t"<<points[i].y<<endl;
		}
	#endif
	
	Points reduced = reducePath(points, params);
	reduced = reducePath(reduced, params);
	
	#if DISPLAY_SMOOTHED_REDUCED_PATH == 1
		cout<<"reduced "<<endl;
		for(size_t i=0;i<reduced.size();i++){
			cout<<reduced[i].x<<"\t"<<reduced[i].y<<endl;
		}
	#endif

	Points reduced_fixed = addPointsToPath(reduced, params);

	#if DISPLAY_SMOOTHED_REDUCEDFIX_PATH == 1
		cout<<"reduced fixed "<<endl;
		for(size_t i=0;i<reduced_fixed.size();i++){
			cout<<reduced_fixed[i].x<<"\t"<<reduced_fixed[i].y<<endl;
		}
	#endif
	
	return reduced_fixed;
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
		#if DISPLAY_VIEW_FRAME == 1
			cout<<"view frame"<<endl;
		#endif
		for(long y=0;y<=viewRF;y++){
			for(long x=-viewRS;x<=viewRS;x++){
				Vec2d xy = Vec2d(x,y).rotate(pos.heading)+pos.loc;
				#if DISPLAY_VIEW_FRAME ==1
					cout<<xy.x<<'\t'<<xy.y<<endl;
				#endif
				if(inMapRange(xy,map)==false) continue;
				Waypoint wp_xy = convert(xy,map);
				if(map(wp_xy.x, wp_xy.y)==Map::ST_BLOCKED && isFrontier(map, wp_xy)){
					repulsors_tmp.push_back(xy);
				}
			}
		}
		size_t i=cWP;
		while(i<path.size()-1){
			#if DISPLAY_INFO_FROM_SEARCHIP == 1
						cout<<"ignore path["<<i<<"] dist to pos = "<<Vec2d::distance(path[i],pos.loc)<<endl;
			#endif
			if(Vec2d::distance(path[i],pos.loc)<1){ i++; continue; }
			break;
		}
		#if DISPLAY_INFO_FROM_SEARCHIP == 1
				cout<<"start from path["<<i<<"]  pos = "<<pos.loc<<","<<Vec2d::r2d(pos.heading)<<"deg"<<endl;
		#endif
		Vec2d vwp = (path[i]-pos.loc).rotate(-pos.heading);
		#if DISPLAY_INFO_FROM_SEARCHIP == 1
				cout<<"vwp = "<<vwp.x<<'\t'<<vwp.y<<endl;
		#endif
		const double E = 0.5;

		#define WAYPOINT_OUT_OF_VIEW ( fabs(vwp.x)<=viewRS+E && (vwp.y>-E && vwp.y<=viewRF+E) )
		#define NOT_END_OF_PATH (i<path.size())
		#define ATTRACTORS_EMPTY attractors.size()==0

		while( NOT_END_OF_PATH  && ( WAYPOINT_OUT_OF_VIEW || ATTRACTORS_EMPTY ) ){
			#if DISPLAY_INFO_FROM_SEARCHIP == 1
						cout<<"att added "<<path[i]<<endl;
			#endif
			attractors.push_back(path[i]);
			i++;
			vwp = (path[i]-pos.loc).rotate(-pos.heading);
			#if DISPLAY_INFO_FROM_SEARCHIP == 1
						//cout<<"vwp = "<<vwp.x<<'\t'<<vwp.y<<endl;
			#endif
		}
		#if DISPLAY_INFO_FROM_SEARCHIP == 1
				cout<<"finished: "<<(i<path.size())<<", "<<(fabs(vwp.x)<=viewRS)<<", "<<(vwp.y>-0.5)<<(vwp.y<=viewRF)<<endl;
		#endif
		
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
		if(nearest == path.size()-1 && nearest == 0){
			return nearest;
		}

		if(nearest == path.size()-1){
			//cout<<"... nearest == path.size()-1"<<endl;
			Vec2d A = pos.loc, C = path[nearest], B = path[nearest-1];
			Vec2d b = B-C, c = C-A, a = B-A;
			//http://en.wikipedia.org/wiki/Law_of_cosines
			double G = acos( (B-C).dot(A-C)/(b.len()*c.len()) );
			//cout<<"... G="<<Vec2d::r2d(G)<<"deg  A="<<A<<" B="<<B<<" C="<<C<<endl;
			if( G>PI*0.5 ){
				//cout<<"... ... G="<<Vec2d::r2d(G)<<"deg < PI/2 => nearest = "<<(nearest+1)<<endl;
				nearest++;
			}

			return nearest;
		}
		Vec2d A = pos.loc, C = path[nearest], B = path[nearest+1];
		Vec2d b = B-C, c = C-A, a = B-A;
		//http://en.wikipedia.org/wiki/Law_of_cosines
		double G = acos( (B-C).dot(A-C)/(b.len()*c.len()) );
		//cout<<"... G="<<Vec2d::r2d(G)<<"deg  A="<<A<<" B="<<B<<" C="<<C<<endl;
		if( G<PI*0.5 ){
			//cout<<"... ... G="<<Vec2d::r2d(G)<<"deg < PI/2 => nearest = "<<(nearest+1)<<endl;
			nearest++;
		}
	}
}

PField::Points PField::simulate(const SmoothingParameters& params) const{

	const double viewRF = params.viewRadiusForward;
	const double viewRS = params.viewRadiusSide;
	const RepulsorType rt = params.repulsorType;
	const AttractorType at = params.attractorType;
	
	const double stepF = params.stepRate;
	const double inertia = params.inertia;
	
	Points opath_points = convert(opath, map);
	size_t reduced_opath_size = opath_points.size();

	opath_points = addPointsToPath(opath_points, 1);

	#if DISPLAY_ORIGINAL_EXTENDED_PATH == 1
		cout<<"originaly extended path "<<endl;
		for(size_t i=0;i<opath_points.size();i++){
			cout<<opath_points[i].x<<"\t"<<opath_points[i].y<<endl;
		}
	#endif

	const int stepNumbers = params.maxIterationNumber>0 ? params.maxIterationNumber : opath_points.size()*2.0/params.stepRate;
	#if DISPLAY_INFO_FROM_SIMULATE == 1
		cout<<"iterations : "<<stepNumbers<<endl;
	#endif

	Points smoothed_path;
	#if DISPLAY_INFO_FROM_SIMULATE == 1
		cout<<"path size reduced from "<<opath.size()<<" to "<<reduced_opath_size<<" and extended to "<<opath_points.size()<<endl;
	#endif
	if(opath_points.size()<2) return opath_points;
	
	Vec2d last = opath_points[opath_points.size()-1];
	Vec2d prevlast = opath_points[opath_points.size()-2];
	Vec2d virtLast = prevlast + (last-prevlast).addLen(0.6);
	opath_points.push_back(virtLast);
	
	size_t start_pos=0;
	
	#if DISPLAY_INFO_FROM_SIMULATE == 1
		cout<<"start pos = "<<start_pos<<" = "<<opath_points[start_pos]<<endl;
	#endif
	
	Position pos ( opath_points[start_pos], 
				   (opath_points[start_pos+1]-opath_points[start_pos]).angY()
			 );

	size_t iteration = 0;
	Points trac;
	while(true){iteration++;
		
		#if DISPLAY_FORCES == 1 
			cout<<"iteration = "<<iteration<<endl;
			cout<<"POS "<<pos.loc.x<<'\t'<<pos.loc.y<<'\t'<<pos.loc.heading<<'\t'<<start_pos<<endl;
		#endif
		
		smoothed_path.push_back(pos.loc);
		
		if(iteration==stepNumbers || start_pos == opath_points.size()-1){
			break;
		}

		
		Points repulsors;
		Points attractors;
		searchIP(viewRF, viewRS, pos, map, opath_points, start_pos, repulsors, attractors);
		
		#if DISPLAY_ATTRACTORS == 1
			cout<<"attractors"<<endl;
			for(size_t i=0;i<attractors.size();i++){
				Vec2d xy = attractors[i];
				cout<<xy.x<<'\t'<<xy.y<<endl;
			}
		#endif
		#if DISPLAY_REPULSORS == 1
			cout<<"repulsors"<<endl;
			for(size_t i=0;i<repulsors.size();i++){
				Vec2d xy = repulsors[i];
				cout<<xy.x<<'\t'<<xy.y<<endl;
			}
		#endif
		
		Vec2d force(0,0.1);
		for(size_t i=0;i<attractors.size();i++){
			Vec2d v = attractors[i] - pos.loc;
			if(at==AT_A1){
				Vec2d f = Vec2d::poliar(v.ang(), 1/pow(v.len()/viewRF,2));
				#if DISPLAY_FORCES == 1
					cout<<"att="<<attractors[i]<<",v="<<v<<", f="<<f<<", ang="<<Vec2d::r2d(v.ang())<<"deg, len="<<v.len()<<", viewRF="<<viewRF<<", len/FR="<<(v.len()/viewRF)<<", f.a="<<Vec2d::r2d(f.ang())<<"deg, pow="<<f.len()<<endl;
				#endif
				force = force + f;
			}
		}
		for(size_t i=0;i<repulsors.size();i++){
			Vec2d v = repulsors[i] - pos.loc;
			Vec2d ell = ellipse(v.ang(), viewRF, viewRS).rotate(pos.heading);
			if(rt==RT_R1){
				Vec2d f = Vec2d::poliar(PI+v.ang(), 1/pow(v.len()/ell.len(),2));
				#if DISPLAY_FORCES == 1
					cout<<"rep="<<repulsors[i]<<",v="<<v<<", f="<<f<<", ang="<<Vec2d::r2d(v.ang())<<"deg, len="<<v.len()<<", viewRF="<<viewRF<<", |ell|="<<ell.len()<<", len/|ell|="<<(v.len()/ell.len())<<", f.a="<<Vec2d::r2d(f.ang())<<"deg pow="<<f.len()<<endl;
				#endif
				force = force + f;
			}
		}
		
		force = force + pos.delta(0,inertia);
		
		double delta_heading = force.rotate(-pos.heading).angY();
		double power = 1;//force.len();
		
		#if DISPLAY_FORCES == 1
			cout<<"force = "<<force<<", ang="<<Vec2d::r2d(force.ang())<<"deg, len="<<force.len()<<", dh="<<Vec2d::r2d(delta_heading)<<"deg, power="<<power<<", cur.heading="<<Vec2d::r2d(pos.heading)<<"deg"<<endl;
		#endif
		
		#if DISPLAY_FORCES == 1
			cout<<"move: "<<pos.loc<<":"<<Vec2d::r2d(pos.heading)<<"d -> "; 
		#endif
		pos.move(delta_heading*stepF, power*stepF);
		#if DISPLAY_FORCES == 1
			cout<<pos.loc<<":"<<Vec2d::r2d(pos.heading)<<"d"<<endl;
		#endif
		
		start_pos = searchOnPathPosition(pos, opath_points);
	}
	
	smoothed_path.push_back(opath_points[opath_points.size()-1]);
	
	cout<<"END OF SMOOTHING"<<endl;
	return smoothed_path;
	
}

// #define RDP_MODE1
// #define RDP_MODE2
#define RDP_MODE3
PField::Points PField::reducePath(const Points& path, const SmoothingParameters& params) const{
	if(path.size()<2) return path;
	Points rpath;
	double sum=0;
	double ang=0;
	double heading=(path[1]-path[0]).angY();
	rpath.push_back(path[0]);
	size_t lastAdded = 0;
	for(size_t i=1;i<path.size();i++){
		double dist = (path[i]-path[i-1]).len();
		double angle = (path[i]-path[i-1]).angY() - heading;
		#if DISPLAY_INFO_FROM_REDUCEPATH == 1
				cout<<">>> path["<<i<<"]="<<path[i]<<" path["<<i-1<<"]="<<path[i-1]<<" dist:ang="<<dist<<":"<<Vec2d::r2d(angle)<<"d ";
		#endif
		heading=(path[i]-path[i-1]).angY();
		#if DISPLAY_INFO_FROM_REDUCEPATH == 1
				cout<<"heading="<<Vec2d::r2d(heading)<<"d ";
		#endif
		sum+=dist;
		double prev_ang=ang;
		ang+=angle;
		#if DISPLAY_INFO_FROM_REDUCEPATH == 1
				cout<<"sum="<<sum<<", sum_ang="<<Vec2d::r2d(ang)<<"d "<<endl;
		#endif
#if defined( RDP_MODE1 )
		// REMOVE POINTS UP TO travel distance > TH or travel turn angles > TH
		if(sum > params.distanceBetweenPoints || ang > params.maxAngleWhileReducing){
			lastAdded = i-1;
			rpath.push_back(path[lastAdded]);
			sum=dist;
			ang=angle;
			#if DISPLAY_INFO_FROM_REDUCEPATH == 1
					cout<<"get "<<lastAdded<<", sum="<<sum<<", sum_ang="<<Vec2d::r2d(ang)<<"d "<<endl;
			#endif
		}
#elif defined( RDP_MODE2 )
		// REMOVE POINTS UP TO travel distance > TH, THEN RETURN POINTS UP TO travel turn angle < TH
		if(sum > params.distanceBetweenPoints){
			size_t j=i;
			for(;j>lastAdded && ang>params.maxAngleWhileReducing;j--){
				double angle = (path[j]-path[j-1]).angY() - ((path[j-1]-path[j-2]).angY());
				ang-=angle;
			}
			i=j;
			lastAdded = i-1;
			rpath.push_back(path[lastAdded]);
			sum=0;
			ang=0;
			#if DISPLAY_INFO_FROM_REDUCEPATH == 1
					cout<<"get "<<lastAdded<<", sum="<<sum<<", sum_ang="<<Vec2d::r2d(ang)<<"d "<<endl;
			#endif
		}
#elif defined( RDP_MODE3 )
		// REMOVE POINTS UP TO travel distance > 2*TH, THEN insert transit point for create same (50%) travel turn angle.
		if(sum > params.distanceBetweenPoints*2){
			//if(ang > params.maxAngleWhileReducing)
			{
				#define ANG_FIX -( PI05-(PI-prev_ang)/2 )
				Vec2d bb = (path[i-1]-path[lastAdded]).scale(0.5);
				double fix_ang = ANG_FIX / 2;
				Vec2d b = bb.changeLen(bb.len()/cos(fix_ang)).rotate(fix_ang) + path[lastAdded];
				rpath.push_back(b);
			}
			lastAdded = i-1;
			rpath.push_back(path[lastAdded]);
			sum=0;
			ang=0;
			#if DISPLAY_INFO_FROM_REDUCEPATH == 1
					cout<<"get "<<lastAdded<<", sum="<<sum<<", sum_ang="<<Vec2d::r2d(ang)<<"d "<<endl;
			#endif
		}
#endif
	}
	if(path.size()-1 != lastAdded) rpath.push_back(path[path.size()-1]);
	
	return rpath;
}


PField::Points PField::addPointsToPath(const Points& path, const SmoothingParameters& params) const{
	return addPointsToPath(path, params.distanceBetweenPoints);
}

PField::Points PField::addPointsToPath(const Points& path, double distBtwPoints) const{
	if(path.size()<2) return path;
	Points rpath;
	rpath.push_back(path[0]);
	for(size_t i=0,j=1; j<path.size(); i++,j++){
		double d = (path[j]-path[i]).len();
		double r = distBtwPoints;
		if(d > distBtwPoints*1.4){
			double n = d / distBtwPoints;
			int in1 = (int)n;
			int in2 = in1+1;
			double dd1 = ::fabs( d - in1*distBtwPoints);
			double dd2 = ::fabs( d - in2*distBtwPoints);
			bool dd1dd2 = dd1<dd2;
			double x=0;
			if(dd1dd2){
				x = dd1/in1;
				r = r+x;
			}else{
				x = dd2/in2;
				r = r-x;
			}

			int nn = d/r;

			#if DISPLAY_INFO_FROM_ADDPOINTS == 1
				#define N(x) ", "#x"="<<x
				cout<<"adding points: d="<<d<<N(n)<<N(in1)<<N(in2)<<N(dd1)<<N(dd2)<<N(dd1dd2)<<N(x)<<N(r)<<N(nn)<<endl;
				#undef N
			#endif

			for(int e=1; e<nn; e++)
				rpath.push_back(path[i] + (path[j]-path[i]).scale((double)e/(double)n));
		}
		rpath.push_back(path[j]);

	}
	return rpath;
}

Path PField::castPath(const Points& points)const{
	return convert(points, map);
}





