#ifndef IFIGURE_H
#define IFIGURE_H

#include "cline.h"

class IFigure
{
public:
	virtual void OnLineMove(CLine* line, QPointF point) = 0;
};

#endif // IFIGURE_H