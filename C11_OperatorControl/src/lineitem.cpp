#include "lineitem.h"
#include <QPainter>
#include <QGraphicsScene>

CLineItem::CLineItem(QPointF p1,QPointF p2,QGraphicsScene* scene,QColor c)
	:QGraphicsLineItem(p1.x(),p1.y(),p2.x(),p2.y()),pointItem1(p1,scene),pointItem2(p2,scene)
{
	QGraphicsScene* pScene;
	pScene = scene;
	pScene->addItem(&pointItem1);
	points[0] = pointItem1.getPoint();
	points[1] = pointItem2.getPoint();
	pPressedPoint = NULL;
	update();
	pScene->update();
	bEndLine = false;
	penColor = c;
}
CLineItem::CLineItem(QGraphicsScene* scene,QColor c)
	:pointItem1(scene),pointItem2(scene)
{
	QGraphicsScene* pScene;
	pScene = scene;
	penColor = c;
	pScene->addItem(&pointItem1);
	points[0] = pointItem1.getPoint();
	points[1] = pointItem2.getPoint();
	pPressedPoint = NULL;
	update();
	pScene->update();
}
void CLineItem::setEndLine(bool b)
{
	bEndLine = b;
}
bool CLineItem::getEndLine()
{
	return bEndLine;
}
CLineItem::~CLineItem()
{
	
}
QPainterPath CLineItem::shape()  const 
{
	QPainterPath path;
	//path.moveTo(pos().x(),pos().y());
	//path.lineTo(pos().x(),pos().y());
	/*path.moveTo(pointItem1.x(),pointItem1.y());
	path.lineTo(pointItem2.x(),pointItem2.y());*/
	path.moveTo(points[0]);
	path.lineTo(points[1]);
	return path;
}
bool CLineItem::isPointOnEdge(QPointF p)
{
	bool b = false;
	CPointItem *other;
	other=new CPointItem(p,scene());
	QLineF l;
	l = line();
	b = collidesWithItem(other);
	if(b)
		return true;
	else
		return false;
}

QRectF CLineItem::boundingRect()  const
{
	//return QRectF(pointItem1.x(),pointItem1.y(),pointItem2.x()-pointItem1.x(),pointItem2.y()-pointItem1.y());
	//return QRectF(pnts[0],pnts[1]);
	//return QRectF(0,0,950,1000);
	return QGraphicsLineItem::boundingRect();
}
void CLineItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
	QLineF line(pointItem1.getPoint(), pointItem2.getPoint());
	//painter->setPen(QPen(Qt::darkBlue, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));	
	painter->setPen(QPen(/*Qt::darkYellow*/penColor, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));	
	painter->drawLine(line);
}
CPointItem* CLineItem::getPointItem1()
{
	return &pointItem1;
}
void CLineItem::MovePoint1To(QPointF p)
{
	pointItem1.setPointParam(p);
	points[0] = pointItem1.getPoint();
	points[1] = pointItem2.getPoint();
	setLine(pointItem1.getPoint().x(),pointItem1.getPoint().y(), pointItem2.getPoint().x(),pointItem2.getPoint().y());
}
void CLineItem::MovePoint2To(QPointF p)
{
	pointItem2.setPointParam(p);
	points[0] = pointItem1.getPoint();
	points[1] = pointItem2.getPoint();
	setLine(pointItem1.getPoint().x(),pointItem1.getPoint().y(), pointItem2.getPoint().x(),pointItem2.getPoint().y());
}
CPointItem* CLineItem::getPointItem2()
{
	return &pointItem2;
}
QPointF CLineItem::getPoint1()
{
	return pointItem1.getPoint();
}
QPointF CLineItem::getPoint2()
{
	return pointItem2.getPoint();
}
void CLineItem::setPointItem1(QPointF p1)
{
	pointItem1.setPointParam(p1);
	points[0] = pointItem1.getPoint();
	points[1] = pointItem2.getPoint();
	setLine(pointItem1.getPoint().x(),pointItem1.getPoint().y(), pointItem2.getPoint().x(),pointItem2.getPoint().y());
	update();
	//pScene->update();
}
void CLineItem::setPointItem2(QPointF p2)
{
	pointItem2.setPointParam(p2);
	points[0] = pointItem1.getPoint();
	points[1] = pointItem2.getPoint();
	setLine(pointItem1.getPoint().x(),pointItem1.getPoint().y(), pointItem2.getPoint().x(),pointItem2.getPoint().y());
	//pScene->addItem(this);
	update();
	//pScene->update();
}
bool CLineItem::isContain(QPointF p)
{
	bool b;
	b = pointItem1.isContainPoint(p);
	if(b)
		pPressedPoint = &pointItem1;
	else
		pPressedPoint = NULL;

	return b;
}
CPointItem* CLineItem::getPressedPoint()
{
	return pPressedPoint;
}
void CLineItem::setRadius(int r)
{
	pointItem1.setRadius(r);
}
