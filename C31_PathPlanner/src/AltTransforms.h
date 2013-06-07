/*
 * AltTransforms.h
 *
 *  Created on: May 19, 2013
 *      Author: dan
 */

#ifndef AltTransforms_H_
#define AltTransforms_H_

#include "Map.h"
#include "cogniteam_pathplanner_parameters.h"

class AltTransformsParameters{
public:
	double max_alt;
	double costTL,costLL;
	AltTransformsParameters(double ma):max_alt(0),costTL(0),costLL(0){
		AltTransformsParameters& wp = *this;
		SET_WD_PARAMETERS(wp);
		wp.max_alt = ma;
	}
	AltTransformsParameters():max_alt(0),costTL(0),costLL(0){
		AltTransformsParameters& wp = *this;
		SET_WD_PARAMETERS(wp);
	}
};

class AltTransforms {
public:
	const AltMap& alts;
	const AltTransformsParameters& params;
	AltTransforms(const AltMap& m, const AltTransformsParameters& wdp):alts(m), params(wdp){}
	ObsMap walls()const;
	AltMap slops()const;
	AltMap costs()const;
private:
	char filter(size_t x, size_t y)const;
	char filterBorder(size_t x, size_t y)const;

	double slop(size_t x, size_t y)const;
	double slopBorder(size_t x, size_t y)const;

	double cost(size_t x, size_t y)const;
};

#endif /* AltTransforms_H_ */
