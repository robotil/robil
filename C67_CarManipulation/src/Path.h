//Path.h
#ifndef PATH_H
#define PATH_H
#define N 100
#define _r 0
#define _l 1

class PathPoints{
public:
	IkSolution Array[2][N];
	//PassPoints(){}
	PathPoints(IkSolution R0,IkSolution L0,IkSolution R1,IkSolution L1);
};

class sPathPoints{
public:
	IkSolution Array[N];
	sPathPoints(IkSolution R0,IkSolution R1);
};

class pPathPoints{
public:
	int size;
	IkSolution *pArray;
	pPathPoints(IkSolution R0, IkSolution R1, int m_size);
	~pPathPoints(){ delete [] pArray; }
};
#endif
