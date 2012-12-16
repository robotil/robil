#include <QPen>
#include <QPainter>
#include <QGraphicsSceneMouseEvent>
#include "cnode.h"
#include "cline.h"
#include "IFigure.h"

CLine::CLine(IFigure* ifigure)
{
	IsMoving = false;
	IsFirstMove = false;
	pIFigure = ifigure;
}

CLine::CLine(IFigure* ifigure, CNode *node1, CNode *node2) : QGraphicsLineItem(node1->x(),node1->y(),node2->x(),node2->y())
{
	pNode1 = node1;
	pNode2 = node2;
	IsMoving = false;
	pIFigure = ifigure;
}

CLine::~CLine()
{

}

QRectF CLine::boundingRect()  const
{
//	qreal adjust = 2;
//	double length = sqrt(pow(pNode2->GetPos().x()-pNode1->GetPos().x(),2)+pow(pNode2->GetPos().y()-pNode1->GetPos().y(),2));
//    return QRectF( -10, -10,length, length);
	return QGraphicsLineItem::boundingRect();
}
    
QPainterPath CLine::shape()  const 
{
	QPainterPath path;
	path.moveTo(pNode1->GetPos());
	path.lineTo(pNode2->GetPos());
	return path;
}

void CLine::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
	QLineF line(pNode1->GetPos(), pNode2->GetPos());
	painter->setPen(QPen(Qt::black, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
	painter->drawLine(line);
}

void CLine::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
	IsMoving = true;
	StartPos = event->pos();
	IsFirstMove = true;
}

void CLine::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
	IsMoving = false;
	IsFirstMove = false;
	update();
    QGraphicsItem::mouseReleaseEvent(event);
}

void CLine::mouseMoveEvent( QGraphicsSceneMouseEvent * event )
{
	if(IsMoving)
	{
		if(IsFirstMove)
		{
			IsFirstMove = false;
			QPointF startPoint = pos();
			QPointF eventPoint = event->pos();
			startPoint.setX(startPoint.x() + eventPoint.x());
			startPoint.setY(startPoint.y() + eventPoint.y());
			pIFigure->OnLineMove(this,startPoint);
//			emit SigOnLineMove(this,startPoint);
		}
	}
}