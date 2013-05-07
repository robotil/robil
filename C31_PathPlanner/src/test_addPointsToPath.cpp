#include "PField.h"

using namespace std;
typedef vector<Vec2d> Points;

#include <stdlib.h>
class Tester{
public:
#define PRINT_FOR_CALC
	void printPath(std::string t, Points& path){
		std::cout<<t<<endl;
		for(size_t i=0;i<path.size();i++){
			cout<<""<<path[i].x<<"\t"<<path[i].y<<endl;
		}
		cout<<"----"<<endl;
	}

	void test(){
		#define SET_PARAMETERS(pf_params)\
			pf_params.viewRadiusForward = 15;\
			pf_params.viewRadiusSide = 4;\
			pf_params.stepRate=0.3;\
			pf_params.inertia=pow(1/pf_params.viewRadiusForward*0.5,2);\
			pf_params.distanceBetweenPoints = 2;\
			pf_params.maxAngleWhileReducing = Vec2d::d2r(10);
		PField::SmoothingParameters pf_params;
		SET_PARAMETERS(pf_params)

		Points path;
		Points path_gps;
//		path.push_back(Vec2d(36,35c-3.000000,-38.250000));
//		path.push_back(Vec2d(36,36));	path_gps.push_back(Vec2d(-2.947547,-37.765373));
//		path.push_back(Vec2d(36,38));	path_gps.push_back(Vec2d(-2.914125,-37.279060));
//		path.push_back(Vec2d(36,40));	path_gps.push_back(Vec2d(-2.881521,-36.830246));
//		path.push_back(Vec2d(36,42));	path_gps.push_back(Vec2d(-2.847428,-36.381538));
//		path.push_back(Vec2d(48,99));	path_gps.push_back(Vec2d(0.027643,-22.102570));

		path.push_back(Vec2d(32,36));	path_gps.push_back(Vec2d(0.000000,0.000000));
		path.push_back(Vec2d(32,37));	path_gps.push_back(Vec2d(0.228735,0.429826));
		path.push_back(Vec2d(33,39));	path_gps.push_back(Vec2d(0.392075,0.888510));
		path.push_back(Vec2d(34,41));	path_gps.push_back(Vec2d(0.551447,1.309332));
		path.push_back(Vec2d(34,42));	path_gps.push_back(Vec2d(0.717862,1.727420));
		path.push_back(Vec2d(72,76));	path_gps.push_back(Vec2d(10.106066,10.106066));

//		path.push_back(Vec2d(35,36));	path_gps.push_back(Vec2d(-0.250000,0.000000));
//		path.push_back(Vec2d(35,37));	path_gps.push_back(Vec2d(-0.240994,0.487417));
//		path.push_back(Vec2d(35,39));	path_gps.push_back(Vec2d(-0.233427,0.974858));
//		path.push_back(Vec2d(35,41));	path_gps.push_back(Vec2d(-0.226012,1.424796));
//		path.push_back(Vec2d(35,43));	path_gps.push_back(Vec2d(-0.218230,1.874729));
//		path.push_back(Vec2d(36,76));	path_gps.push_back(Vec2d(0.003749,10.149953));

		path = path;//_gps;

#ifndef PRINT_FOR_CALC
		std::cout<<"Input path: "<<path<<std::endl;
#else
		printPath("Input path: ", path);
#endif
		std::cout<<"distanceBetweenPoints" << pf_params.distanceBetweenPoints <<std::endl;

		PField pf(Map(10,10), Waypoints());
		Points reduced = pf.reducePath(path, pf_params);
		reduced = pf.reducePath(reduced, pf_params);
		reduced = pf.reducePath(reduced, pf_params);
		Points result = pf.addPointsToPath(reduced, pf_params);

#ifndef PRINT_FOR_CALC
		std::cout<<"Result  path: "<<result<<std::endl;
#else
		printPath("Result  path: ", result);
#endif

	}
};
int main(int a, char** b){
	Tester tester;
	tester.test();
	return 0;
}
