#ifndef GRIDITEM_H
#define GRIDITEM_H

#include <QGraphicsItem>
#include <QGraphicsRectItem>
#include <QGraphicsScene>
#include "pixitem.h"
#include <QPainter>
#include <QGraphicsSceneMouseEvent>

class CGridItem : public QGraphicsItem
{

public:
	CGridItem(QGraphicsScene* scene);
	~CGridItem();
	void AddPix();
	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

private:
	int pos[2];
	//int i;
	//int j;
	int startX,startY;
	float p_i;
	float p_j;
	QGraphicsScene* pScene;
	CPixItem *pPixItem[48][48];
	void SltOnPixClick();
	QRectF boundingRect()  const;


protected:
	void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
	void mouseMoveEvent( QGraphicsSceneMouseEvent * event );
	
};

#endif // GRIDITEM_H
