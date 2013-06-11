#ifndef SCALEITEM_H
#define SCALEITEM_H

#include <QGraphicsItem>
#include <QGraphicsScene>
//#include "traingleItem.h"

class CScaleItem : public QGraphicsItem
{
public:
        CScaleItem(QGraphicsScene* scene);
        ~CScaleItem();
        QRectF boundingRect() const;
        void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);


private:
};

#endif // SCALEITEM_H
