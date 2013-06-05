#ifndef PIXITEM_H
#define PIXITEM_H

#include <QGraphicsItem>
#include <QGraphicsScene>
//#include "traingleItem.h"

class CPixItem : public QGraphicsItem
{	
public:
	CPixItem(int color,QGraphicsScene* scene,int px,int py);
	~CPixItem();
	QRectF boundingRect() const;
	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
	//bool isContain(QPointF p);
	void SetColor(int color);
	
private:
	//QGraphicsScene* pScene;
	QBrush TransEnumToBrush();
//	void ChangColor();
	int posX;
	int posY;
	int PixColor;
	QPointF PressPoint;
	//CtraingleItem *traingle;
};

#endif // PIXITEM_H
