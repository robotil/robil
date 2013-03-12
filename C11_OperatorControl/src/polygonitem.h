#ifndef POLYGONITEM_H
#define POLYGONITEM_H

#include <QGraphicsItem>
#include <QGraphicsScene>
#include "pointitem.h"

class CPolygonItem :public QGraphicsItem
{

public:
	CPolygonItem(QPointF p[40],int numOf,QGraphicsScene* scene);
	~CPolygonItem();
	QRectF boundingRect() const;
	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

private:
	CPointItem *pointItem[40];
	int NumOfPoints;

	
};


//
//class CLineItem : public QGraphicsLineItem
//{
//public:
//	CLineItem(QPointF p1,QPointF p2,QGraphicsScene* scene);
//	CLineItem(QGraphicsScene* scene);
//	~CLineItem();
//	QPainterPath shape()  const ;
//	QRectF boundingRect() const;
//	CPointItem* getPointItem1();
//	CPointItem* getPointItem2();
//	void setPointItem1(QPointF p1);
//	void setPointItem2(QPointF p2);
//	QPointF getPoint1();
//	QPointF getPoint2();
//	bool isContain(QPointF p);
//	CPointItem* getPressedPoint();
//	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
//	void setRadius(int r);
//	void MovePoint1To(QPointF p);
//	void MovePoint2To(QPointF p);
//	
//private:
//	//QGraphicsScene* pScene;
//	CPointItem pointItem1;
//	CPointItem pointItem2;
//	CPointItem *pPressedPoint;
//};

#endif // LINEITEM_H
