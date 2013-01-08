#ifndef _CONVERTER_C22_H_
#define _CONVERTER_C22_H_

#include "C22_GroundRecognitionAndMapping/C22.h"
#include "Map.h"


Map convertToMap(C22_GroundRecognitionAndMapping::C22::Response &res){
	Map::MapCreator m;
	size_t h = res.drivingPath.row.size();
	bool size_ok = false;
	if(h){
		size_t w = res.drivingPath.row.at(0).column.size();
		if(w){
			size_ok = true;
			m.w=w; m.h=h;
			for(size_t y=0;y<h;y++)
				for(size_t x=0;x<w;x++)
					m.data.push_back(res.drivingPath.row.at(y).column.at(x).status);
		}
	}
	if(size_ok==false){
		return Map(0,0);
	}
	return m.map();
};







#endif
