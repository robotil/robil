#ifndef TRAINGLEITEM_H
#define TRAINGLEITEM_H

#include "arcitem.h"
#include <QGraphicsScene>
#include <QGraphicsPolygonItem>

class CtraingleItem : public QGraphicsPolygonItem
{
public:
	CtraingleItem(QGraphicsScene* scene,QPointF pTraing,QPointF pArc);
	~CtraingleItem();
	QPainterPath shape()  const ;
	QRectF boundingRect()  const;
	void paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget = 0 );
	bool selectArc(QPointF p);
	void MoveArc(QPointF p);

private:
	//QGraphicsScene* pScene;
	int posX;
	int posY;
	CArcItem arcItem;
};

#endif // TRAINGLEITEM_H
