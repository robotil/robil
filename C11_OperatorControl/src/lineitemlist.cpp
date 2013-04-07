#include "lineItemlist.h"
#include <QPainter>
#include <QGraphicsScene>

CLineItemList::CLineItemList(QPointF p,bool ShowLines,QGraphicsScene* scene, QColor c)
	:LineItem(p,p,scene,c)
{
	pNextLineItemList = NULL;
	pScene = scene;
	b_ShowLines = ShowLines;
	if(b_ShowLines)
		pScene->addItem(&LineItem);

	//this->setPos(p);
	update();
	pScene->update();
	pPointPressedInLine = NULL;
	penColor = c;
}
CLineItemList::~CLineItemList()
{
	//delete pPointPressedInLine;
}
QRectF CLineItemList::boundingRect()  const
{
    return QRectF(0,0,950,1000);
	//return QRectF(LineItem.x(),LineItem.y(), LineItem.x(),LineItem.y());
}
void CLineItemList::addPointItem1(QPointF p)
{
	LineItem.setPointItem2(p);
	if(b_ShowLines)
		pScene->addItem(&LineItem);

	if(pNextLineItemList==NULL)
	{
		pNextLineItemList = new CLineItemList(p,b_ShowLines,pScene,penColor);
		if(b_ShowLines)
			pScene->addItem(pNextLineItemList);

	}
	else
	{
		pNextLineItemList->addPointItem1(p);//addPoint2 to point1 in nextLine
	}
	update();
	pScene->update();
}
void CLineItemList::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
}
CLineItemList* CLineItemList::getNextLineItem()
{
	return pNextLineItemList;
}
void CLineItemList::setNextLineItem(CLineItemList *nextLintItem)
{
	pNextLineItemList  = nextLintItem;
	update();
	pScene->update();
}
void CLineItemList::setLineItem(CLineItem *lintItem)
{
	LineItem.setPointItem1(lintItem->getPointItem1()->getPoint());
	LineItem.setPointItem2(lintItem->getPointItem2()->getPoint());
	update();
	pScene->update();
}
CLineItem* CLineItemList::getLineItem()
{
	return &LineItem;
}
bool CLineItemList::isPointOnEdge(QPointF p)
{
	return LineItem.isPointOnEdge(p);
}
bool CLineItemList::isContain(QPointF p)
{
	bool b;
	b = LineItem.isContain(p);
	if(b)
		pPointPressedInLine = &LineItem;
	else
		pPointPressedInLine = NULL;

	return b;
}
CLineItem* CLineItemList::getPressedPoint()
{
	return pPointPressedInLine;
}

void CLineItemList::LastPoint(QPointF p)
{
	LineItem.setPointItem2(p);
}
void CLineItemList::removeLastPoint()
{
	pScene->removeItem(pNextLineItemList);
	delete pNextLineItemList;
	pNextLineItemList = NULL;
	pScene->update();
}
