//Path.h
#ifndef PATH_H
#define PATH_H
#define N 100
#define _r 0
#define _l 1

class PassPoints{
public:
	IkSolution Array[2][N];
	//PassPoints(){}
	PassPoints(IkSolution R0,IkSolution L0,IkSolution R1,IkSolution L1);
};



#endif
