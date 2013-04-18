#include "traingleitem.h"
#include <QPainter>

QPointF pnts[3];
	
CtraingleItem::CtraingleItem(QGraphicsScene* scene,QPointF pTraing,QPointF pArc)
	: QGraphicsPolygonItem(),arcItem(pArc,scene)
{
	posX = pTraing.x();
	posY = pTraing.y();
	pnts[0]=QPointF(posX,posY);
	pnts[1]=QPointF(posX-12,posY+24);
	pnts[2]=QPointF(posX+12,posY+24);
	scene->addItem(&arcItem);
}
CtraingleItem::~CtraingleItem()
{

}
QPainterPath CtraingleItem::shape()  const 
{
	QPainterPath path;
	path.moveTo(pnts[0]);
	path.lineTo(pnts[1]);
	path.lineTo(pnts[2]);
	return path;
}
QRectF CtraingleItem::boundingRect()  const
{
	return QRectF(posX,posY,pnts[1].x()-posX,posY-pnts[1].y());
}
void CtraingleItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
	painter->setPen(QPen(Qt::green));
	painter->setBrush(QBrush(Qt::green));
	painter->drawPolygon(pnts,3);
}
bool CtraingleItem::selectArc(QPointF p)
{
	return(arcItem.IsArcContains(p));
}
void CtraingleItem::MoveArc(QPointF p)
{
	arcItem.MoveArcPoint(p);
}