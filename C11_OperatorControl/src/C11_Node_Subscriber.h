#ifndef C11_NODE_SUBSCRIBER_H
#define C11_NODE_SUBSCRIBER_H

#include <QChar>
#include <QImage>


class IC11_Node_Subscriber
{
	public:
		virtual void OnImgReceived(QImage image)=0;
		virtual void OnImgReceived(std::string fileName)=0;
		virtual void OnOccupancyGridReceived(int grid[48][48])=0;
};

#endif // C11_NODE_SUBSCRIBER_H
