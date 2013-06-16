#ifndef ROBOTDATAITEM_H
#define ROBOTDATAITEM_H

#include <QGraphicsItem>
#include <QGraphicsScene>
//#include "traingleItem.h"

class CRobotDataItem : public QGraphicsItem
{
public:
	CRobotDataItem(QGraphicsScene* scene, QString str);
        ~CRobotDataItem();
        QRectF boundingRect() const;
        void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
        void SetText(QString str);

private:
        QString Text;
};

#endif // ROBOTDATAITEM_H
