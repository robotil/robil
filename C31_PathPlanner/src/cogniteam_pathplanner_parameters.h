/*
 * PathPlannerParameters.h
 *
 *  Created on: Apr 22, 2013
 *      Author: dan
 */

#ifndef PATHPLANNERPARAMETERS_H_
#define PATHPLANNERPARAMETERS_H_

#define PLANNER_VERSION "2013_05_23_12_20"

/*
#define SET_PF_PARAMETERS(potential_field)\
 	(potential_field).viewRadiusForward = 5;\
 	(potential_field).viewRadiusSide = 2;\
 	(potential_field).stepRate=0.6;\
 	(potential_field).inertia=pow(1/pf_params.viewRadiusForward*0.5,2);\
 	(potential_field).distanceBetweenPoints = 2;\
 	(potential_field).maxAngleWhileReducing = Vec2d::d2r(10);
*/
#define SET_PF_PARAMETERS(potential_field)\
	(potential_field).viewRadiusForward = 15;\
	(potential_field).viewRadiusSide = 4;\
	(potential_field).stepRate=0.3;\
	(potential_field).inertia=pow(1/(pf_params.viewRadiusForward*0.5),2);\
	(potential_field).distanceBetweenPoints = 2;\
	(potential_field).maxAngleWhileReducing = Vec2d::d2r(10);

#define SET_WD_PARAMETERS(walls_detector)\
	(walls_detector).max_alt = 0.45;\
	(walls_detector).costTL = 0.05;\
	(walls_detector).costLL = 0.0;

#define SET_BS_PARAMETERS(AStarSearch)\
	(AStarSearch).w_distance = 1;\
	(AStarSearch).w_slop = 1;\
	(AStarSearch).w_alt_delta = 1;

#define DO_SMOOTHING

#endif /* PATHPLANNERPARAMETERS_H_ */
