/*
 * PathPlannerParameters.h
 *
 *  Created on: Apr 22, 2013
 *      Author: dan
 */

#ifndef PATHPLANNERPARAMETERS_H_
#define PATHPLANNERPARAMETERS_H_

// #define SET_PARAMETERS(pf_params)\
// 	pf_params.viewRadiusForward = 5;\
// 	pf_params.viewRadiusSide = 2;\
// 	pf_params.stepRate=0.6;\
// 	pf_params.inertia=pow(1/pf_params.viewRadiusForward*0.5,2);\
// 	pf_params.distanceBetweenPoints = 2;\
// 	pf_params.maxAngleWhileReducing = Vec2d::d2r(10);
#define SET_PARAMETERS(pf_params)\
	pf_params.viewRadiusForward = 15;\
	pf_params.viewRadiusSide = 4;\
	pf_params.stepRate=0.3;\
	pf_params.inertia=pow(1/(pf_params.viewRadiusForward*0.5),2);\
	pf_params.distanceBetweenPoints = 2;\
	pf_params.maxAngleWhileReducing = Vec2d::d2r(10);


#endif /* PATHPLANNERPARAMETERS_H_ */
