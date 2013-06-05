#ifndef LINEITEM_H
#define LINEITEM_H

#include <QGraphicsLineItem>
#include <QGraphicsScene>
#include "pointitem.h"

class CLineItem : public QGraphicsLineItem
{
public:
	CLineItem(QPointF p1,QPointF p2,QGraphicsScene* scene,QColor c=Qt::darkBlue);
	CLineItem(QGraphicsScene* scene,QColor c=Qt::darkBlue);
	~CLineItem();
	QPainterPath shape()  const ;
	QRectF boundingRect() const;
	CPointItem* getPointItem1();
	CPointItem* getPointItem2();
	void setPointItem1(QPointF p1);
	void setPointItem2(QPointF p2);
	QPointF getPoint1();
	QPointF getPoint2();
	bool isContain(QPointF p);
	CPointItem* getPressedPoint();
	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
	void setRadius(int r);
	void MovePoint1To(QPointF p);
	void MovePoint2To(QPointF p);
	bool isPointOnEdge(QPointF p);
	void setEndLine(bool b);
	bool getEndLine();
	
private:
	//QGraphicsScene* pScene;
	CPointItem pointItem1;
	CPointItem pointItem2;
	CPointItem *pPressedPoint;
	QPointF points[2];
	bool bEndLine;
	QColor penColor;
};

#endif // LINEITEM_H
