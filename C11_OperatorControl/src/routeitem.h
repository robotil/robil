#ifndef ROUTEITEM_H
#define ROUTEITEM_H

#include <QGraphicsItem>
#include <QGraphicsScene>
#include "lineItemlist.h"

class CRouteItem : public QGraphicsItem
{
public:
	CRouteItem(QGraphicsScene* scene);
	~CRouteItem();
	void addPointToLine(QPointF p,bool ShowLines);
	QRectF boundingRect() const;
	QPainterPath shape()  const ;
	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
	bool isContain(CLineItemList *ItmList,QPointF p);
	void MovePointTo(QPointF p);
	void ConnectLastPoint(QPointF p);
	void endPath(QPointF p);
	bool selectPoint(QPointF p);
	void ReleasePoint();
	bool selectEdge(QPointF p);
	void MoveLineTo(QPointF p);

private:
	CLineItemList *pFirstLineItemList;
	int numOfPoints;
	QGraphicsScene* pScene;
	CLineItemList *pPressedPointInLineList;
	bool isLastPointInPath(CLineItemList *LnLst);
	void updatePolygonPoints(QPointF p,QPointF pNew);
	bool isPointOnEdge(CLineItemList *ItmList,QPointF p);
	void updatePoints();
	bool isPoly;
	CLineItemList* GoToLastLine();
	void UpdatetLastPoint();
};

#endif // ROUTEITEM_H
