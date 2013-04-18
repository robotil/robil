#ifndef ARCITEM_H
#define ARCITEM_H

#include <QGraphicsEllipseItem>
#include "pointitem.h"

class CArcItem : public QGraphicsEllipseItem
{
public:
	CArcItem(QPointF p,QGraphicsScene* scene);
	~CArcItem();
	QPainterPath shape()  const ;
	QRectF boundingRect()  const;
	bool IsArcContains(QPointF p);
	void MoveArcPoint(QPointF p);
	void paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget = 0 );

private:
	QPointF centerPos;
	double radius;
	CPointItem *RightPoint;
	CPointItem *LeftPoint;
	void checkMousePos(QPointF new_point);
	CPointItem *PressedPoint;
	QPointF GetRightPoint();
	QPointF GetLeftPoint();
	void updatePie_RightPressed(QPointF p);
	void updatePie_LeftPressed(QPointF p);
	bool isPoly;
};

#endif // ARCITEM_H
