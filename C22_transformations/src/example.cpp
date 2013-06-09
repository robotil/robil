#include <ros/ros.h>
#include <C22_transformations/MapTransformations.h>
#include <C22_GroundRecognitionAndMapping/C22.h>

struct XY{public: double x,y;};

int main(int argc,char**argv){

	 C22_transform trans;
	 C22_transform::MapProperties prop(Vec2d(-10,-12) , 0.25, 0, true);
//	 Vec2d A(2,-2), B(10,10), C(0,0);
	 Vec2d A(-0.498,0), B(6,0), C(5.5,0);
	 Vec2d TM(0,0), TW(0,0);

#define TEST(A) trans.GlobalToMap(prop, A, TM); trans.MapToGlobal(prop, TM, TW); std::cout<<" "<< #A <<"  W:"<< A << " --> M:"<< TM << " --> W:"<< TW << std::endl;

	 TEST(A)
	 TEST(B)
	 TEST(C)

	 return 0;
}
