#ifndef C11_NODE_SUBSCRIBER_H
#define C11_NODE_SUBSCRIBER_H

#include <QChar>
#include <QImage>
#include "structs.h"

class IC11_Node_Subscriber
{
	public:
//		virtual void OnImgReceived(QImage image)=0;
		virtual void OnImgReceived(std::string fileName)=0;
//		virtual void OnOccupancyGridReceived(int grid[100][100], StructPoint robotPos, int xOffset, int yOffset,double orient)=0;
//		virtual void OnPathReceived(std::vector<StructPoint> points)=0;
//		virtual void OnHMIResponseReceived()=0;
		virtual void OnWaitResponseFinished()=0;
//		virtual void OnExecutionStatusUpdate(int status)=0;
};

#endif // C11_NODE_SUBSCRIBER_H
