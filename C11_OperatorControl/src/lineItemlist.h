#ifndef LINEITEMLIST_H
#define LINEITEMLIST_H

#include <QGraphicsItem>
#include <QGraphicsScene>
#include "lineitem.h"

class CLineItemList : public QGraphicsItem
{
public:
	CLineItemList(QPointF p,bool ShowLines,QGraphicsScene* scene,QColor c = Qt::darkBlue);
	~CLineItemList();
	void addPointItem1(QPointF p);
	QRectF boundingRect() const;
	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
	CLineItemList* getNextLineItem();
	void setNextLineItem(CLineItemList *nextLintItem);
	void setLineItem(CLineItem *lintItem);
	CLineItem* getLineItem();
	bool isContain(QPointF p);
	CLineItem* getPressedPoint();
	void LastPoint(QPointF p);
	void removeLastPoint();
	bool isPointOnEdge(QPointF p);

private:
	CLineItem LineItem;
	QGraphicsScene* pScene;
	CLineItemList *pNextLineItemList;
	CLineItem *pPointPressedInLine;
	bool b_ShowLines;
	QColor penColor;
};

#endif // LINELIST_H
