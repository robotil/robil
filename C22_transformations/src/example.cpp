#include <ros/ros.h>
#include <C22_transformations/MapTransformations.h>
#include <C22_GroundRecognitionAndMapping/C22.h>

struct XY{public: double x,y;};

int main(int argc,char**argv){

	 C22_transform trans;
	 C22_transform::MapProperties prop(Vec2d(0.75,-0.75) , 0.25, 0, true);
	 Vec2d A(1.75,-0.25), B(1,0.75);
	 Vec2d TM(0,0), TW(0,0);

#define TEST(A) trans.GlobalToMap(prop, A, TM); trans.MapToGlobal(prop, TM, TW); std::cout<<" "<< #A <<"  W:"<< A << " --> M:"<< TM << " --> W:"<< TW << std::endl;

	 TEST(A)
	 TEST(B)

	 return 0;
}
