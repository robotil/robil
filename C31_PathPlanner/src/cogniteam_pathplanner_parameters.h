/*
 * PathPlannerParameters.h
 *
 *  Created on: Apr 22, 2013
 *      Author: dan
 */

#ifndef PATHPLANNERPARAMETERS_H_
#define PATHPLANNERPARAMETERS_H_

#define PLANNER_VERSION "2013_06_12_17_30"

#define PARAMS_FROM_ROS 1
#define PARAMS_FROM_CODE 2
#define PARAMS_SOURCE PARAMS_FROM_ROS

#if PARAMS_SOURCE==PARAMS_FROM_CODE
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

#endif

#if PARAMS_SOURCE == PARAMS_FROM_ROS
#include <ros/ros.h>

#define SET_PF_PARAMETERS(potential_field)\
	ros::param::param<double>("/C31_GlobalPathPlanner/potential_field/viewRadiusForward", (potential_field).viewRadiusForward, 15);\
	ros::param::param<double>("/C31_GlobalPathPlanner/potential_field/viewRadiusSide", (potential_field).viewRadiusSide, 4);\
	ros::param::param<double>("/C31_GlobalPathPlanner/potential_field/stepRate", (potential_field).stepRate, 0.3);\
	ros::param::param<double>("/C31_GlobalPathPlanner/potential_field/distanceBetweenPoints", (potential_field).distanceBetweenPoints, 2);\
	ros::param::param<double>("/C31_GlobalPathPlanner/potential_field/maxAngleWhileReducing", (potential_field).maxAngleWhileReducing, Vec2d::d2r(10));\
	(potential_field).inertia=pow(1/(pf_params.viewRadiusForward*0.5),2);


#define SET_WD_PARAMETERS(walls_detector)\
	ros::param::param<double>("/C31_GlobalPathPlanner/walls_detector/max_alt", (walls_detector).max_alt, 0.45);\
	ros::param::param<double>("/C31_GlobalPathPlanner/walls_detector/costTL", (walls_detector).costTL, 0.05);\
	ros::param::param<double>("/C31_GlobalPathPlanner/walls_detector/costLL", (walls_detector).costLL, 0.0);


#define SET_BS_PARAMETERS(AStarSearch)\
	ros::param::param<double>("/C31_GlobalPathPlanner/a_star/w_distance", (AStarSearch).w_distance, 1);\
	ros::param::param<double>("/C31_GlobalPathPlanner/a_star/w_slop", (AStarSearch).w_slop, 1);\
	ros::param::param<double>("/C31_GlobalPathPlanner/a_star/w_alt_delta", (AStarSearch).w_alt_delta, 1);


#define SET_MD_PARAMETERS(mud_detector)\
	ros::param::param<int>("/C31_GlobalPathPlanner/mud_detector/wp_number", (mud_detector).wp_number, 5);
#define SET_DD_PARAMETERS(debrees_detector)\
	ros::param::param<int>("/C31_GlobalPathPlanner/debrees_detector/wp_number", (debrees_detector).wp_number, 5);\
	ros::param::param<int>("/C31_GlobalPathPlanner/debrees_detector/wp_radius", (debrees_detector).wp_number, 3);

#endif

#define DO_SMOOTHING

#endif /* PATHPLANNERPARAMETERS_H_ */
