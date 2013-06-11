/*
 * AltTransforms.cpp
 *
 *  Created on: May 19, 2013
 *      Author: dan
 */

#include "AltTransforms.h"

ObsMap AltTransforms::walls()const{
	ObsMap grid(alts.w(), alts.h());
	std::cout<<"Grid created: "<< grid.w() <<'x'<< grid.h()<< std::endl;
	size_t f_x=0, l_x=alts.w()-1;
	size_t f_y=0, l_y=alts.h()-1;
	for(size_t y=f_y+1; y<l_y; y++)	for(size_t x=f_x+1; x<l_x; x++) grid(x,y)=filter(x,y);
	for(size_t y=f_y+1; y<l_y; y++) { grid(f_x,y)=filterBorder(f_x,y); grid(l_x,y)=filterBorder(l_x,y); }
	for(size_t x=f_x+1; x<l_x; x++) { grid(x,f_y)=filterBorder(x,f_y); grid(x,l_y)=filterBorder(x,l_y); }
	return grid;
}
ObsMap AltTransforms::walls(const ObsMap& f)const{
	ObsMap grid(alts.w(), alts.h());
	std::cout<<"Grid created: "<< grid.w() <<'x'<< grid.h()<< std::endl;
	size_t f_x=0, l_x=alts.w()-1;
	size_t f_y=0, l_y=alts.h()-1;
	for(size_t y=f_y+1; y<l_y; y++)	for(size_t x=f_x+1; x<l_x; x++) grid(x,y)=filter(x,y, f);
	for(size_t y=f_y+1; y<l_y; y++) { grid(f_x,y)=filterBorder(f_x,y, f); grid(l_x,y)=filterBorder(l_x,y, f); }
	for(size_t x=f_x+1; x<l_x; x++) { grid(x,f_y)=filterBorder(x,f_y, f); grid(x,l_y)=filterBorder(x,l_y, f); }
	return grid;
}

char AltTransforms::filter(size_t x, size_t y)const{
	double alt = alts(x,y)-params.max_alt;
	if(
		alt > alts(x-1,y) || alt > alts(x+1,y) || alt > alts(x,y-1) || alt > alts(x,y+1)
	) return ObsMap::ST_BLOCKED;
	return ObsMap::ST_AVAILABLE;
}

char AltTransforms::filterBorder(size_t x, size_t y)const{
	double alt = alts(x,y)-params.max_alt;
	if( x>0 			&& alt > alts(x-1,y)) return ObsMap::ST_BLOCKED;
	if( x<alts.w()-1 	&& alt > alts(x+1,y)) return ObsMap::ST_BLOCKED;
	if( y>0 			&& alt > alts(x,y-1)) return ObsMap::ST_BLOCKED;
	if( y<alts.h()-1 	&& alt > alts(x,y+1)) return ObsMap::ST_BLOCKED;
	return ObsMap::ST_AVAILABLE;
}

#define KN(x,y) (f((x),(y)) != ObsMap::ST_UNCHARTED)

char AltTransforms::filter(size_t x, size_t y, const ObsMap& f)const{
	double alt = alts(x,y)-params.max_alt;
	if(
		(KN(x-1,y) && alt > alts(x-1,y)) || (KN(x+1,y) && alt > alts(x+1,y)) || (KN(x,y-1) && alt > alts(x,y-1)) || (KN(x,y+1) && alt > alts(x,y+1))
	) return ObsMap::ST_BLOCKED;
	return ObsMap::ST_AVAILABLE;
}

char AltTransforms::filterBorder(size_t x, size_t y, const ObsMap& f)const{
	double alt = alts(x,y)-params.max_alt;
	if( x>0 			&& (KN(x-1,y) && alt > alts(x-1,y))) return ObsMap::ST_BLOCKED;
	if( x<alts.w()-1 	&& (KN(x+1,y) && alt > alts(x+1,y))) return ObsMap::ST_BLOCKED;
	if( y>0 			&& (KN(x,y-1) && alt > alts(x,y-1))) return ObsMap::ST_BLOCKED;
	if( y<alts.h()-1 	&& (KN(x,y+1) && alt > alts(x,y+1))) return ObsMap::ST_BLOCKED;
	return ObsMap::ST_AVAILABLE;
}



AltMap AltTransforms::slops()const{
	AltMap _slops(alts.w(), alts.h());
	std::cout<<"Slops created: "<< _slops.w() <<'x'<< _slops.h()<< std::endl;
	size_t f_x=0, l_x=alts.w()-1;
	size_t f_y=0, l_y=alts.h()-1;
	for(size_t y=f_y+1; y<l_y; y++)	for(size_t x=f_x+1; x<l_x; x++) _slops(x,y)=slop(x,y);
	for(size_t y=f_y+1; y<l_y; y++) { _slops(f_x,y)=slopBorder(f_x,y); _slops(l_x,y)=slopBorder(l_x,y); }
	for(size_t x=f_x+1; x<l_x; x++) { _slops(x,f_y)=slopBorder(x,f_y); _slops(x,l_y)=slopBorder(x,l_y); }
	return _slops;
}
AltMap AltTransforms::slops(const ObsMap& f)const{
	AltMap _slops(alts.w(), alts.h());
	std::cout<<"Slops created: "<< _slops.w() <<'x'<< _slops.h()<< std::endl;
	size_t f_x=0, l_x=alts.w()-1;
	size_t f_y=0, l_y=alts.h()-1;
	for(size_t y=f_y+1; y<l_y; y++)	for(size_t x=f_x+1; x<l_x; x++) _slops(x,y)=slop(x,y, f);
	for(size_t y=f_y+1; y<l_y; y++) { _slops(f_x,y)=slopBorder(f_x,y, f); _slops(l_x,y)=slopBorder(l_x,y, f); }
	for(size_t x=f_x+1; x<l_x; x++) { _slops(x,f_y)=slopBorder(x,f_y, f); _slops(x,l_y)=slopBorder(x,l_y, f); }
	return _slops;
}

AltMap AltTransforms::costs()const{
	AltMap _slops(alts.w(), alts.h());
	std::cout<<"Costs created: "<< _slops.w() <<'x'<< _slops.h()<< std::endl;
	size_t f_x=0, l_x=alts.w()-1;
	size_t f_y=0, l_y=alts.h()-1;
	for(size_t y=f_y+0; y<=l_y; y++) for(size_t x=f_x+0; x<=l_x; x++) _slops(x,y)=cost(x,y);
	return _slops;
}
AltMap AltTransforms::costs(const ObsMap& f)const{
	AltMap _slops(alts.w(), alts.h());
	std::cout<<"Costs created: "<< _slops.w() <<'x'<< _slops.h()<< std::endl;
	size_t f_x=0, l_x=alts.w()-1;
	size_t f_y=0, l_y=alts.h()-1;
	for(size_t y=f_y+0; y<=l_y; y++) for(size_t x=f_x+0; x<=l_x; x++) _slops(x,y)=cost(x,y, f);
	return _slops;
}

namespace {
	inline double max(double b, double v[], int n){
		double s=0;
		for(int i=0;i<n;i++){
			s=fmax(s, fabs(b-v[i]));
		}
		return s;
	}
	inline double max(double b, double v[], bool f[], int n){
		double s=0;
		for(int i=0;i<n;i++){ if(f[i])
			s=fmax(s, fabs(b-v[i]));
		}
		return s;
	}
}
double AltTransforms::slop(size_t x, size_t y)const{
	double alt = alts(x,y);
	double nei[]={alts(x-1,y),alts(x+1,y),alts(x,y-1),alts(x,y+1)};
	return max(alt, nei, 4);
}
double AltTransforms::slopBorder(size_t x, size_t y)const{
	double alt = alts(x,y);
	double slop=0;
	#define C(X,Y)  slop=fmax(slop, fabs( alt - alts(X,Y)))
	if( x>0 			) C(x-1,y);
	if( x<alts.w()-1 	) C(x+1,y);
	if( y>0 			) C(x,y-1);
	if( y<alts.h()-1 	) C(x,y+1);
	return slop;
	#undef C
}

double AltTransforms::cost(size_t x, size_t y)const{
	double f = 1.0/(params.costTL-params.costLL);
	double blocks = round((alts(x,y)-params.costLL)*f)/f;
	//std::cout<<""<<blocks<<" = "<<"round(("<<alts(x,y)<<"-"<<params.costLL<<")*"<<f<<")/"<<f<<""<<endl;
	return blocks;
}

double AltTransforms::slop(size_t x, size_t y, const ObsMap& f)const{
	double alt = alts(x,y);
	double nei[]={alts(x-1,y),alts(x+1,y),alts(x,y-1),alts(x,y+1)};
	double kn[]={KN(x-1,y),KN(x+1,y),KN(x,y-1),KN(x,y+1)};
	return max(alt, kn, 4);
}
double AltTransforms::slopBorder(size_t x, size_t y, const ObsMap& f)const{
	double alt = alts(x,y);
	double slop=0;
	#define C(X,Y)  if(KN(X,Y)) slop=fmax(slop, fabs( alt - alts(X,Y)))
	if( x>0 			) C(x-1,y);
	if( x<alts.w()-1 	) C(x+1,y);
	if( y>0 			) C(x,y-1);
	if( y<alts.h()-1 	) C(x,y+1);
	return slop;
	#undef C
}

double AltTransforms::cost(size_t x, size_t y, const ObsMap& ff)const{
	double f = 1.0/(params.costTL-params.costLL);
	double blocks = round((alts(x,y)-params.costLL)*f)/f;
	//std::cout<<""<<blocks<<" = "<<"round(("<<alts(x,y)<<"-"<<params.costLL<<")*"<<f<<")/"<<f<<""<<endl;
	return blocks;
}

