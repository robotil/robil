#include "polygonitem.h"
#include <QPainter>
#include <QGraphicsScene>

QPointF points[40];

CPolygonItem::CPolygonItem(QPointF p[40],int NumOf,QGraphicsScene* scene)
{
	QGraphicsScene* pScene;
	pScene = scene;
	NumOfPoints = NumOf;
	
	for(int i=0; i<NumOfPoints; i++)
	{
		pointItem[i] = new CPointItem(p[i],pScene);
		points[i] = p[i];
		pScene->addItem(pointItem[i]);
	}
}

CPolygonItem::~CPolygonItem()
{

}

QRectF CPolygonItem::boundingRect()  const
{
	//return QRectF(pointItem1.x(),pointItem1.y(),pointItem2.x(),pointItem2.y());
	return QRectF(0,0,950,1000);
}

void CPolygonItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
	painter->setPen(QPen(Qt::darkYellow, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
	painter->setBrush(QBrush(QColor(255,255,0,100)));
	painter->drawPolygon(points,NumOfPoints);
}
