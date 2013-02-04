#include "griditem.h"
#include <QPainter>
#include <QGraphicsSceneMouseEvent>

CGridItem::CGridItem(QGraphicsScene* scene)
	: QGraphicsItem()
{
	pScene = scene;
	startX = 175;
	startY = 400;
	pos[2];
	p_i=startX;
	p_j=startY;
}

CGridItem::~CGridItem()
{
	
}
void CGridItem::AddPix()
{
	
}
QRectF CGridItem::boundingRect()  const
{
    return QRectF(175,400,600,600);
}

void CGridItem::SltOnPixClick()
{
	//int r=0;
}
void CGridItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
	/*painter->setPen(QPen(Qt::white, 0));
	painter->setBrush(QBrush());
	painter->drawRect(boundingRect());	*/
}

void CGridItem::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
	/*QPainter pPaint;
	pPaint.setPen(QPen(Qt::white, 0));
	pPaint.drawRect(0.0,0.0,event->pos().x(),event->pos().y());
	update();*/
}

void CGridItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
	int r=0;
}

void CGridItem::mouseMoveEvent( QGraphicsSceneMouseEvent * event )
{
	int r=0;
}
