#ifndef PIXITEM_H
#define PIXITEM_H

#include <QGraphicsItem>
#include <QGraphicsRectItem>
#include <QGraphicsScene>
#include <QGraphicsItem>
//#include <QGraphicsSceneMouseEvent>

class CPixItem : public QGraphicsItem
{	
public:
	CPixItem(int color,QGraphicsScene* scene,int px,int py);
	~CPixItem();
	QRectF boundingRect() const;
	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
	void SetColor(int color);
	//void mousePress(QPointF p);
	//bool isContain(QPointF p);
	static void drawLineTo(const QPointF &endPoint);
	
private:
	QGraphicsScene* pScene;
	QBrush TransEnumToBrush();
//	void ChangColor();
	/*QGraphicsRectItem *pPix;*/
	int posX;
	int posY;
	int PixColor;
	QPointF PressPoint;
	
//protected:
	/*void mousePressEvent(QGraphicsSceneMouseEvent *event);
	void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
	void mouseMoveEvent( QGraphicsSceneMouseEvent * event );*/
};

#endif // PIXITEM_H
