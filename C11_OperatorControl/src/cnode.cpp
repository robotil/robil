#include <QPainter>
#include <QStyleOptionGraphicsItem>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsScene>
#include "cnode.h"

CNode::CNode(QGraphicsScene* scene)
{
	pScene = scene;
	IsMoving = false;
}

CNode::CNode(int x, int y, QGraphicsScene* scene)
{
	pScene = scene;
	setPos(x,y);
	IsMoving = false;
}

CNode::~CNode()
{

}

QRectF CNode::boundingRect()  const
{
    return QRectF( -5, -5,10, 10);
}
    
QPainterPath CNode::shape()  const 
{
	QPainterPath path;
	path.addEllipse(-5, -5, 10, 10);
	return path;
}
 
void CNode::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
	/*painter->setPen(Qt::NoPen);
     painter->setBrush(Qt::darkGray);
     painter->drawEllipse(-7, -7, 20, 20);*/

     QRadialGradient gradient(-3, -3, 10);
     if (option->state & QStyle::State_Sunken) {
         gradient.setCenter(3, 3);
         gradient.setFocalPoint(3, 3);
         gradient.setColorAt(1, QColor(Qt::yellow).light(120));
         gradient.setColorAt(0, QColor(Qt::darkYellow).light(120));
     } else {
         gradient.setColorAt(0, Qt::yellow);
         gradient.setColorAt(1, Qt::darkYellow);
     }
     painter->setBrush(gradient);

     painter->setPen(QPen(Qt::black, 0));
     painter->drawEllipse(-5, -5, 10, 10);
}

QPointF CNode::GetPos()
{
	return pos();
}

void CNode::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
	IsMoving = true;
	StartPos = event->pos();
//	update();
//    QGraphicsItem::mousePressEvent(event);
}
 
void CNode::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
	IsMoving = false;
	update();
    QGraphicsItem::mouseReleaseEvent(event);
}

void CNode::mouseMoveEvent( QGraphicsSceneMouseEvent * event )
{
	if(IsMoving)
	{
		QPointF startPoint = pos();
		QPointF eventPoint = event->pos();
		startPoint.setX(startPoint.x() + eventPoint.x());
		startPoint.setY(startPoint.y() + eventPoint.y());
		setPos(startPoint);
	}
	update();
	pScene->update();
    QGraphicsItem::mouseMoveEvent(event);
}